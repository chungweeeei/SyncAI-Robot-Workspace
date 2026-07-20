ros2 bag record \
  -o livox_mapping_test \
  --compression-mode file \
  --compression-format zstd \
  --max-bag-size 2000000000 \
  /livox/lidar /livox/imu
--max-bag-size（bytes，此例約 2 GB）到達後會 split 並壓縮該段，避免單檔過大。