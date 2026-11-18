# Sourced by every onboard ROS entry; mirrors airseafly_real400_mission/ros_network.py.
# Advertise this machine by hostname: /etc/hosts maps it to 127.0.1.1 here, so
# onboard links stay on loopback and survive WiFi drops or IP changes, while a
# remote RViz maps the same name to the current LAN address
# (tools/remote_rviz/mac_rviz.sh). AIRSEAFLY_ROS_IP pins an explicit address,
# e.g. AIRSEAFLY_ROS_IP=127.0.0.1 keeps the graph local-only.
unset ROS_IP ROS_HOSTNAME
if [ -n "${AIRSEAFLY_ROS_IP:-}" ]; then
    export ROS_IP="$AIRSEAFLY_ROS_IP"
else
    airseafly_ros_host=$(hostname 2>/dev/null || true)
    case "$(getent ahostsv4 "$airseafly_ros_host" 2>/dev/null | awk 'NR==1{print $1}')" in
        127.*) export ROS_HOSTNAME="$airseafly_ros_host" ;;
        *) export ROS_IP=127.0.0.1 ;;
    esac
    unset airseafly_ros_host
fi
