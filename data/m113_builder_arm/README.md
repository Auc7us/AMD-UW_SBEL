# M113 Builder Arm Assets

This directory is the M113 tracked builder's project-local copy of the
manipulator model.

The source OBJ geometry is copied without modification from the existing AMD-UW
arm model. At runtime, the C++ construction matches
`lunar-manip/model/arm_model.py::_apply_scale` and the tracked-builder reference:

- chassis-frame offset: `(-2.5, 0.0, 0.4)` metres
- relative mounting yaw: `180` degrees
- arm geometry and joint-frame scale: `2.0`
- mass and inertia: unchanged from the 1x export
- fingers and finger contact pads: 1x, translated to the scaled gripper tip

The matching ROS IK model is the project-local
`amd_uw_ros2.inverse_kinematics` copy of
`lunar-manip/model/inverseKin.py`. Instantiate it with `scale=2.0` for this
builder; its reference link lengths are
`(0.32516, 1.27, 1.143, 0.3577) * scale`.

The builder currently constructs this arm directly in C++, so it does not run
the Python model during simulation. The Python copy is retained for independent
asset ownership and future importer-based tooling.
