# AMD-UW SynChrono Demo

## TODO

1. [ ] Add ROS controller integration.
2. [ ] Stop at rock.
3. [ ] Move to SCM terrain.
4. [ ] Explore a PyChrono wrapper for SynChrono.
5. [ ] Integrate with Harry.
6. [ ] Scale to many vehicles and rocks.

## Refactoring Note

- [x] Keep the current demo single-file while it remains one compact executable.
- [x] Split robot rig setup and per-step robot updates into their own module.
- [x] Split rock field generation into its own module.
- [x] Split custom SynChrono agents into their own module.
- [ ] Continue refactoring along domain boundaries when scale-up requires it: terrain/world setup, ROS controller drivers, robot arm, and per-robot sensors.

## Side Quest

- [ ] Explore texture support in Chrono.
