#!/bin/bash
set -e
drone_ip=${1:-172.18.9.122}
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://$drone_ip:11341
export ROS_IP=$(ip -4 route get "$drone_ip" | awk '{for(i=1;i<NF;i++) if($i=="src"){print $(i+1); exit}}')
unset ROS_HOSTNAME
host=drone24-NUC14MNK-B2
if ! getent ahostsv4 "$host" | awk '{print $1}' | grep -qx "$drone_ip"; then
 echo "请在 WSL 中先执行："
 echo "echo '$drone_ip $host # airseafly-drone' | sudo tee -a /etc/hosts"
 exit 1
fi
config_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
exec rviz -d "$config_dir/wsl_ego_live.rviz"
