# `marslite_control` Package

## Tests

### `test_default_pose_planning`

This test drives the TM5-700 robotic arm to several given poses. The order of these poses are: `HOME` -> `DEFAULT1` -> `DEFAULT2` -> `DEFAULT3` -> `DEFAULT4` -> `HOME`.

```Shell
roslaunch marslite_control test_default_pose_planning.launch
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

