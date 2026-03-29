# PX4 side requirements for single-camera OpenVINS input

This document lists what must be configured or verified in PX4 for ROS2 VIO input from OpenVINS.

## 1. PX4 estimator settings

Set EKF2 to accept vision position and yaw fusion.

Recommended parameters:

- EKF2_AID_MASK: include vision position and vision yaw bits
- EKF2_EV_CTRL: enable vision position and vision yaw fusion (newer PX4 uses EV_CTRL)
- EKF2_HGT_REF: keep baro or set according to your test strategy
- EKF2_EV_DELAY: start with 0, then tune based on timestamp offset
- EKF2_EV_POS_X / EKF2_EV_POS_Y / EKF2_EV_POS_Z: camera-to-IMU position offset in meters

Note: exact parameter names can differ by PX4 version. Use `param show EKF2_*EV*` and apply equivalent fields.

## 2. Topics and DDS bridge

Ensure PX4 receives ROS2 input topic:

- ROS2 publish topic: /fmu/in/vehicle_visual_odometry
- ROS2 type in this workspace: px4_msgs/msg/VehicleOdometry
- PX4 uORB target: vehicle_visual_odometry

Ensure PX4 outputs IMU topic used by bridge:

- ROS2 subscribe topic: /fmu/out/sensor_combined

## 3. Time synchronization

- Keep ROS2 and simulator clock coherent.
- If simulation is used, publish /clock and set `use_sim_time=true` in ROS2 nodes.
- Large EV timestamp lag causes EKF rejection. Tune EKF2_EV_DELAY if needed.

## 4. Frame convention checks

Input sent from ROS2 to PX4 must be:

- position in NED
- orientation as FRD body to NED world quaternion
- velocity frame consistent with message field (this implementation uses BODY_FRD)

If EV innovation spikes or heading jumps, frame transform is likely wrong.

## 5. Verification checklist on PX4 side

1. In PX4 shell, verify uORB update:
   - `listener vehicle_visual_odometry 1`
2. Verify estimator accepted EV fusion:
   - check estimator status and innovation values
3. Arm in controlled test mode and check stable local position behavior
4. If rejected:
   - verify topic rate (20 to 50 Hz recommended)
   - verify quaternion order and frame
   - verify EKF2 vision fusion parameters

## 6. Suggested initial test order

1. Keep vehicle disarmed, stream VIO only
2. Confirm `vehicle_visual_odometry` updates continuously
3. Confirm EKF starts fusing EV without large innovation
4. Perform short hover test with offboard only after estimator is stable
