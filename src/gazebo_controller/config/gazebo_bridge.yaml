# Bridge configuration for ROS2 and Gazebo

topics:
 # Velocity command topic from ROS to Gazebo
 - ros_topic_name: "cmd_vel"
   gz_topic_name: "/cmd_vel"
   ros_type_name: "geometry_msgs/msg/Twist"
   gz_type_name: "gz.msgs.Twist"
   direction: "ROS_TO_GZ"

 # Odometry topic from Gazebo to ROS
 - ros_topic_name: "odom"
   gz_topic_name: "/odom"
   ros_type_name: "nav_msgs/msg/Odometry"
   gz_type_name: "gz.msgs.Odometry"
   direction: "GZ_TO_ROS"

 # Ground truth pose from Gazebo to ROS (remapped to /tf in launch file)
 - ros_topic_name: "model/vehicle_blue/pose"
   gz_topic_name: "/model/vehicle_blue/pose"
   ros_type_name: "tf2_msgs/msg/TFMessage"
   gz_type_name: "gz.msgs.Pose_V"
   direction: "GZ_TO_ROS"

 # Lidar scan topic from Gazebo to ROS
 - ros_topic_name: "lidar"
   gz_topic_name: "/lidar"
   ros_type_name: "sensor_msgs/msg/LaserScan"
   gz_type_name: "gz.msgs.LaserScan"
   direction: "GZ_TO_ROS"
   qos:
     reliability: "BEST_EFFORT"
     durability: "VOLATILE"
     history: "KEEP_LAST"
     depth: 10

 # TF topic from Gazebo to ROS
 - ros_topic_name: "tf"
   gz_topic_name: "/tf"
   ros_type_name: "tf2_msgs/msg/TFMessage"
   gz_type_name: "gz.msgs.Pose_V"
   direction: "GZ_TO_ROS"
   qos:
     reliability: "RELIABLE"
     history: "KEEP_LAST"
     depth: 10

 - ros_topic_name: "tf_static"
   gz_topic_name: "/tf_static"
   ros_type_name: "tf2_msgs/msg/TFMessage"
   gz_type_name: "gz.msgs.Pose_V"
   direction: "GZ_TO_ROS"
   qos:
     durability: "TRANSIENT_LOCAL"

 # Clock topic from Gazebo to ROS
 - ros_topic_name: "clock"
   gz_topic_name: "/clock"
   ros_type_name: "rosgraph_msgs/msg/Clock"
   gz_type_name: "gz.msgs.Clock"
   direction: "GZ_TO_ROS"
 