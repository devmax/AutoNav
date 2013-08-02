clear
read -p "Please enter file name you would like to log to: " name
<<<<<<< HEAD
rosbag record /ardrone/image_raw /ardrone/navdata /ardrone/predictedPose /ardrone/imu /cmd_vel /log_circleControl /log_obs_IMU_XYZ /log_obs_IMU_RPY /log_obs_tag /log_predictInternal /log_predictUpTo /ar_pose_marker /quatTOrpy /log_tags /tf -O $name.bag
=======
rosbag record /ardrone/image_raw /ardrone/navdata /ardrone/predictedPose /ardrone/imu /cmd_vel /log_circleControl /log_obs_IMU_XYZ /log_obs_IMU_RPY /log_obs_PTAM /log_predictInternal /log_predictUpTo /ar_pose_marker /AutoNav/commands /ardrone/variances /vslam/info /tf /vslam/pose -O $name.bag
>>>>>>> 0339d1aef307420878e1cce4833a127c3740ab07
