
# Set this to whatever physical port you are using to communicate externally 
# (eg. eth0, eth1, wlan0,...etc)

# Uncomment if vector2
#export ROBOT_NETWORK=eth0

#Uncomment if vector1
export ROBOT_NETWORK=br0

#Uncomment if simulation system
<<<<<<< Updated upstream
#export ROBOT_NETWORK=wlan0
export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311
=======
<<<<<<< HEAD
#export ROBOT_NETWORK=wlan0
=======
export ROBOT_NETWORK=wlan0
>>>>>>> parent of 0474e34... converted the Jaco wrist joint to a revoulte joint instead of a continuous joint. Screwing motion enabled
>>>>>>> Stashed changes

#ROS IP needs to be set no matter what PC
#export ROS_IP=$(ip -4 address show $ROBOT_NETWORK | grep 'inet' | sed 's/.*inet \([0-9\.]\+\).*/\1/')

# Uncomment if vector2
#export ROS_MASTER_IPADD=$(gethostip vector1 | awk '{print $2}')
#export ROS_MASTER_URI=http://$ROS_MASTER_IPADD:11311/

#Uncomment if vector1
export ROS_MASTER_URI=http://$ROS_IP:11311/

#Uncomment if simulation system
#export ROS_MATER_URI=http://vector1:11311/
