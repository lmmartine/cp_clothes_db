
cd /home/lmartinez/ros_ws/src/cp_clothes_db/IMG_OUT && mkdir $1


roslaunch cp_clothes_db cp_clothes_db_xtion1.launch directory_name:=$1 & roslaunch cp_clothes_db cp_clothes_db_xtion2.launch directory_name:=$1 &
sleep 10
rosservice call /xtion1_mask/active "select: true" & rosservice call /xtion2_mask/active "select: true" 
sleep 5
cd /home/lmartinez/bags
rosbag play tf_static.bag 
sleep 5
rosbag play $1