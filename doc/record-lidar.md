ros2 bag record -o lio_test_$(date +%m%d_%H%M) \
  /livox/lidar /livox/imu /tf_static \
  --compression-mode file --compression-format zstd