#!/bin/bash
# Mac 从机：连接机载 ROS master（11341），本机运行 RViz 读取飞机话题。
# 用法：bash tools/remote_rviz/mac_rviz.sh [飞机IP]               # 打开 EGO 实时地图（ego_live.rviz）
#       bash tools/remote_rviz/mac_rviz.sh [飞机IP] rostopic list # 同一环境下执行任意命令
# 不填 IP 时沿用上次连通的地址；飞机 IP 变了就填 1 号终端打印的新地址。
# 机载节点按主机名公布地址；首次或 IP 变化时本脚本用 sudo 更新 /etc/hosts 中
# 带 "# airseafly-drone" 标记的那一行，其余内容不动。
set -eu
remote_project=$(CDPATH= cd -P -- "$(dirname -- "$0")/../.." && pwd)
state_file=$HOME/.config/airseafly/drone_ip
conda_base=${AIRSEAFLY_CONDA_BASE:-$HOME/miniforge3}
ros_env=${AIRSEAFLY_ROS_ENV:-ros_noetic}
ipv4='^[0-9]{1,3}(\.[0-9]{1,3}){3}$'

drone_ip=${AIRSEAFLY_DRONE_IP:-}
if [ $# -gt 0 ] && [[ $1 =~ $ipv4 ]]; then
  drone_ip=$1
  shift
fi
if [ -z "$drone_ip" ] && [ -r "$state_file" ]; then
  drone_ip=$(head -1 "$state_file")
fi
drone_ip=${drone_ip:-172.18.9.122}
if ! [[ $drone_ip =~ $ipv4 ]]; then
  echo "飞机 IP 格式不对：$drone_ip" >&2
  exit 2
fi

# Advertise the address of the interface that actually routes to the drone.
mac_iface=$(route -n get "$drone_ip" 2>/dev/null | awk '/interface:/{print $2}')
mac_ip=$(ipconfig getifaddr "${mac_iface:-en0}" 2>/dev/null || true)
if [ -z "$mac_ip" ]; then
  echo "找不到通往 $drone_ip 的本机网卡地址，请确认与飞机在同一网络。" >&2
  exit 2
fi

set +u
. "$conda_base/etc/profile.d/conda.sh"
conda activate "$ros_env"
set -u
export ROS_MASTER_URI=http://$drone_ip:11341
export ROS_IP=$mac_ip
unset ROS_HOSTNAME

# The master reports the host name onboard nodes advertise.
drone_host=$(python - "$ROS_MASTER_URI" <<'PY' || true
import socket, sys, xmlrpc.client
from urllib.parse import urlparse
socket.setdefaulttimeout(3)
try:
    code, _, uri = xmlrpc.client.ServerProxy(sys.argv[1]).getUri('/airseafly_mac_rviz')
except Exception:
    sys.exit(1)
print(urlparse(uri).netloc.rsplit(':', 1)[0] if code == 1 else '')
PY
)
if [ -z "$drone_host" ]; then
  echo "连不上 ${ROS_MASTER_URI}：确认飞机 1 号终端已启动；IP 变了就运行" >&2
  echo "  bash tools/remote_rviz/mac_rviz.sh <1号终端打印的IP>" >&2
  exit 2
fi
mkdir -p "$(dirname "$state_file")"
echo "$drone_ip" > "$state_file"

case "$drone_host" in
  localhost|127.*)
    echo "飞机节点只对本机公布地址（${drone_host}），Mac 收不到数据；机载不要设置 AIRSEAFLY_ROS_IP=127.0.0.1。" >&2
    exit 2 ;;
esac
if [[ ! $drone_host =~ $ipv4 ]]; then
  if ! [[ $drone_host =~ ^[A-Za-z0-9][A-Za-z0-9.-]*$ ]]; then
    echo "飞机公布的主机名异常：$drone_host" >&2
    exit 2
  fi
  mapped=$(dscacheutil -q host -a name "$drone_host" 2>/dev/null | awk '/^ip_address:/{print $2; exit}')
  if [ "$mapped" != "$drone_ip" ]; then
    echo "把 $drone_host 指向 ${drone_ip}（写入 /etc/hosts，需要本机密码）" >&2
    sudo /bin/sh -c '
      tmp=$(mktemp) &&
      grep -v "# airseafly-drone$" /etc/hosts > "$tmp" &&
      printf "%s\t%s\t# airseafly-drone\n" "$1" "$2" >> "$tmp" &&
      cat "$tmp" > /etc/hosts && rm -f "$tmp" &&
      dscacheutil -flushcache && killall -HUP mDNSResponder' sh "$drone_ip" "$drone_host"
  fi
fi
echo "ROS_MASTER_URI=$ROS_MASTER_URI  ROS_IP=$ROS_IP  飞机=$drone_host" >&2

if [ $# -gt 0 ]; then
  exec "$@"
fi
# RoboStack's librospack (loaded by rviz plugins) references Python symbols
# without linking libpython; preload it for rviz only. The conda python is
# statically linked, so preloading into rostopic etc. would double the runtime.
export DYLD_INSERT_LIBRARIES=$CONDA_PREFIX/lib/libpython3.12.dylib
exec rviz -d "${AIRSEAFLY_RVIZ_CONFIG:-$remote_project/tools/remote_rviz/ego_live.rviz}"
