clear
read -p "Please enter file name you would like to log to: " name
rosbag record /ardrone/image_raw /ardrone/navdata /ardrone/predictedPose /ardrone/imu /cmd_vel /log_circleControl /log_obs_IMU_XYZ /log_obs_IMU_RPY /log_obs_PTAM /log_predictInternal /log_predictUpTo /ar_pose_marker /AutoNav/commands /ardrone/variances /vslam/info /tf /vslam/pose -O $name.bag