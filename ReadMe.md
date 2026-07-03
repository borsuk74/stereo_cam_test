## run broadcast process on host, relaying data to docker which has linux and ros1, explanation is below:

  - host_imu_stream.py (now at host_imu_stream.py) — reuses
  imu_visual.py's IMU class directly (same proven ICM-20948 driver + Mahony AHRS), reads
  at 50 Hz, and streams fixed-size 88-byte samples (timestamp, quaternion, angular
  velocity rad/s, linear acceleration m/s²) over TCP on port 6002 to any connected client.
  - imu_tcp_relay.py in the camera_driver package — connects to 172.17.0.1:6002, decodes
  samples, publishes sensor_msgs/Imu on /imu.
  - dual_cam_tcp.launch now brings up all three: cam0_relay, cam1_relay, and imu_relay.

 ## To run:

  Host (must run in the same directory as imu_visual.py since it imports it):
 .../stereo_cam_test
  python3 host_dual_cam_stream.py &
  python3 host_imu_stream.py &

  Container:
  source /catkin_ws/devel/setup.bash
  roslaunch camera_driver dual_cam_tcp.launch

  Then check rostopic hz /imu and rostopic echo /imu alongside the camera topics. Once all
  three look good, you can record the calibration bag with rosbag record -O
  calibration.bag /cam0/image_raw /cam1/image_raw /imu.

## Based on the setup (topics /cam0/image_raw, /cam1/image_raw, /imu), run this
  inside the container after the launch file is up:

  rosbag record -O /root/bags/calibration.bag /cam0/image_raw /cam1/image_raw
  /imu

  Since /root/bags is mounted from the host's bags/ directory (via
  run_container.sh), calibration.bag will land at
  /home/aleks/Documents/gitclone/stereo_cam_test/bags/calibration.bag and
  survive when the container exits.

  Before recording, verify topics are flowing: rostopic hz /cam0/image_raw 
  /cam1/image_raw /imu

## to check rosbag file:
  rosbag info calibration.bag