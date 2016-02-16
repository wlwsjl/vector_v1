
# Set this to whatever physical port you are using to communicate externally 
# (eg. eth0, eth1, wlan0,...etc)
export ROBOT_NETWORK=eth0
export ROS_IP=$(ip -4 address show $ROBOT_NETWORK | grep 'inet' | sed 's/.*inet \([0-9\.]\+\).*/\1/')
export ROS_MASTER_URI=http://$ROS_IP:11311/
