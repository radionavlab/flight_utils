#!/bin/bash

# Transforms the PVA_Ref messages into geometry point messages for visualization
# Runs in background
rosrun topic_tools transform $1 $2 geometry_msgs/PointStamped 'geometry_msgs.msg.PointStamped(header=std_msgs.msg.Header(seq=0, frame_id="fcu"), point=geometry_msgs.msg.Point(m.Pos.x, m.Pos.y, m.Pos.z))' --import geometry_msgs std_msgs & export job_id_1=$!

# Transforms the local_odom messages into geometry point messages for visualization
# Runs in background
rosrun topic_tools transform $3 $4 geometry_msgs/PointStamped 'geometry_msgs.msg.PointStamped(header=m.header, point=geometry_msgs.msg.Point(m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z))' --import geometry_msgs & export job_id_2=$!

echo `pwd`/$5

rviz -d $5

kill $job_id_1 $job_id_2

