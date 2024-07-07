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

## References

### `marslite_mpc`

```c++
namespace marslite {
    namespace mpc {
        class ModelPredictiveControl;
    }
}
```

#### Public Member Functions

#### Private Member Functions

### `marslite_control`

```c++
namespace marslite {
    namespace control {
        class MarsliteControlScheme;
    }
}
```

#### Public Member Functions

#### Private Member Functions

