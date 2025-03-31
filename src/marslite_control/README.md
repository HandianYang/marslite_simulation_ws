# `marslite_control` Package

## Tests

### `test_default_pose_planning`

This test drives the TM5-700 robotic arm to several given poses. The order of these poses is: `HOME` -> `DEFAULT1` -> `DEFAULT2` -> `DEFAULT3` -> `DEFAULT4` -> `HOME`. For details of these poses, please refer to `./include/model_predictive_control/Pose.h`.

```Shell
roslaunch marslite_control test_default_pose_planning.launch
```

### `test_kinematics`

This test verifies the functionality of `tm_kinematics.h`, which calculates forward and inverse kinematics of the TM5-series robotic arm. For the derivation of the inverse kinematics, please refer to `./include/tm_kinematics/README.md`.

```Shell
roslaunch marslite_control test_kinematics.launch
```

### `test_gripper_moving`

This test sends control commands to the gripper, making it move toward 6 directions in a given distance. The purpose of this test is to verify the functionality of `tm_kinematics.h` and `model_predictive_control.h`.

```Shell
roslaunch marslite_control test_gripper_moving.launch
```

### `test_joystick_teleoperation`

This test implements the teleoperation of the robotic arm, driven by the pose of the VR joysticks, which is recognized as some ROS topic in `PoseStamped` type. In the test, the pose of joysticks is passed by the `/unity/joy_pose/left` topic.

```Shell
roslaunch marslite_control test_joystick_teleoperation.launch
```


### `test_cartesian_control` (real robot ONLY)

Bringup MARS and cartesian controllers:
```bash
(under kyyoung)
$ source /home/kyyoung/josh/detect_patient_ws/devel/setup.bash
$ roslaunch robot_control mars_lite_bringup.launch
```

Launch ROS#:
```bash
(under root)
$ sd
$ roslaunch file_server ros_sharp_communication.launch
```

Test joystick teleoperation using cartesian controllers:
```bash
(under root)
$ sd
$ rosrun marslite_control test_cartesian_control
```

**[Note] DO NOT launch default `mars_lite_bringup.launch` for this test**

## References

### `marslite::mpc::ModelPredictiveControl`

#### Member Functions

### `marslite::control::marslite_control`

#### Member Functions
