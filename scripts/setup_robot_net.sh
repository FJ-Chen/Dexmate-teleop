#!/usr/bin/env bash
# 把真机相关的三个网段地址固化进 NetworkManager(写盘,重启/拔插网线后自动恢复),
# 然后做全设备连通性自检。重复执行无害。
#
#     sudo bash scripts/setup_robot_net.sh            # 固化 + 自检
#     bash scripts/setup_robot_net.sh --check         # 只自检,不改配置(免 sudo)
#
# 为什么需要:enp49s0 的 NM 配置里原本只有主地址,机器人/Sharpa 手这些网段的
# 第二地址若用 `ip addr add` 手工加,只改内核运行时状态、不落盘,断链即丢 ——
# 表现为「mDNS 能发现却连不上」/「两只手都 ping 不通」,2026-08-10 现场一天
# 栽了两次。固化进 NM 连接配置后,每次链路 up 由 NM 自动配全。
# Wuji 手套走另一块 USB 网卡(enx*,本机应有 192.168.1.10/24),不在固化范围,
# 但下面的自检会一并检查。
set -u

IF=enp49s0
ADDRS=(
    "192.168.50.100/24"    # Dexmate 机器人(zenoh 在 192.168.50.20:7447)
    "192.168.10.240/24"    # Sharpa 手(左 .10 / 右 .20)
    "192.168.5.100/24"     # 其它设备(SOP 网络表)
)
# 自检清单:设备名|IP
DEVICES=(
    "Dexmate 机器人|192.168.50.20"
    "Sharpa 左手  |192.168.10.10"
    "Sharpa 右手  |192.168.10.20"
    "Wuji 左手套  |192.168.1.100"
    "Wuji 右手套  |192.168.1.101"
)

# ---------------------------------------------------------------- 固化 --
if [ "${1:-}" != "--check" ]; then
    CON=$(nmcli -g GENERAL.CONNECTION device show "$IF" 2>/dev/null || true)
    if [ -z "$CON" ] || [ "$CON" = "--" ]; then
        echo "警告: $IF 没有活动的 NetworkManager 连接(网线没插?)。" >&2
        echo "      退回临时方案 ip addr add(断链后会丢)。" >&2
        for a in "${ADDRS[@]}"; do
            ip -4 addr show "$IF" | grep -q "${a%/*}/" && echo "已存在  $a" \
                || { ip addr add "$a" dev "$IF" 2>/dev/null && echo "已添加(临时) $a" \
                     || echo "添加失败 $a(需要 sudo?)" >&2; }
        done
    else
        echo "NetworkManager 连接配置: $CON"
        have=$(nmcli -g ipv4.addresses con show "$CON" | tr -d ' ')
        changed=0
        for a in "${ADDRS[@]}"; do
            if [[ ",$have," == *",$a,"* ]]; then
                echo "已固化  $a"
            elif nmcli con mod "$CON" +ipv4.addresses "$a" 2>/dev/null; then
                echo "已写入  $a"
                changed=1
            else
                echo "写入失败 $a(需要 sudo?)" >&2
                exit 1
            fi
        done
        # 注意不设 ipv4.gateway:这些网段都是交换机直连,乱加网关会抢默认路由
        if [ "$changed" -eq 1 ]; then
            nmcli con up "$CON" >/dev/null && echo "已重新激活 $CON(地址即刻生效)"
        fi
    fi
    echo
    echo "当前 $IF 的地址:"
    ip -4 addr show "$IF" | grep inet || echo "  (无)"
    echo
fi

# ---------------------------------------------------------------- 自检 --
# 先按网段判断本机能不能到,再并行 ping:能区分「本机没配网段」和「设备没上电」,
# 且不通的设备不再一个一个各等一秒。
host_segs=$(ip -4 -o addr show | awk '{split($4,a,"/"); sub(/\.[0-9]+$/,"",a[1]); print a[1]}')

echo "连通性:"
tmp=$(mktemp -d); trap 'rm -rf "$tmp"' EXIT
to_check=()
for d in "${DEVICES[@]}"; do
    name=${d%%|*}; addr=${d##*|}; seg=${addr%.*}
    if ! grep -qx "$seg" <<<"$host_segs"; then
        case "$seg" in
            192.168.1) hint="Wuji 手套的 USB 网卡(enx*)未接或没配 192.168.1.10/24" ;;
            *)         hint="本机缺 $seg.x 地址,重跑 sudo bash scripts/setup_robot_net.sh" ;;
        esac
        echo "  跳过  $name $addr —— $hint"
        continue
    fi
    to_check+=("$d")
    ( ping -c1 -W1 "$addr" >/dev/null 2>&1 && touch "$tmp/$addr" ) &
done
wait
fail=0
for d in ${to_check[@]+"${to_check[@]}"}; do
    name=${d%%|*}; addr=${d##*|}
    if [ -e "$tmp/$addr" ]; then
        echo "  通    $name $addr"
    else
        echo "  不通  $name $addr(设备未上电,或网线/交换机问题)"
        fail=1
    fi
done
# 机器人通了再确认 zenoh 端口开着(ping 通≠服务起来了)
if [ -e "$tmp/192.168.50.20" ]; then
    if timeout 2 bash -c 'echo > /dev/tcp/192.168.50.20/7447' 2>/dev/null; then
        echo "  通    机器人 zenoh 端口 192.168.50.20:7447"
    else
        echo "  不通  机器人 zenoh 端口 7447(本体服务未就绪,刚上电可等十几秒重试)"
        fail=1
    fi
fi
exit "$fail"
