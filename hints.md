# Observations & HINTS

- read carefully both the `README.md` and the project description - they don't have the same information
- `previous_path_x` and `previous_path_y` are always the future `n` steps of your car, i.e. they don't include `x` and `y`
- your car's `x` and `y` are always one of the points you've sent to the simulator in the previous cycle
- some of the other cars don't appear immediately on the road and will have strange coordinates, e.g. `d = -200`
- the provided coordinate conversion functions are not smooth at all and you need to make your own
- you can use `spline` for `Frenet` -> `Cartesian` XY conversion, where you fit `x` and `y` to `s`
- your conversion will never be completely accurate (expect displacements of 0 to 0.5 per axis, 0.1 on average)
- always be careful when handling `s` - remember that the track is a circuit and `s` values around `0` and `6945.554` are fiddly to work with  
- `Cartesian` XY -> `Frenet` is very inaccurate and complicated to improve, also not very useful
- the time between two cycles is ~2-3 steps, i.e. `0.04s-0.06s`
- it is very useful to always include >3 previous path values to your next values for avoiding crazy behaviour of your car
- generate your trajectory from your car's position in a couple of steps from now (maybe 10?)
- when getting your car coordinates, don't rely on the provided `s`, `d` or `speed`
- keep your previous trajectory in `Frenet` and `Cartesian` and use it for calculating `s`, `d`, `s_dot` and `d_dot` from your car's `x` and `y` from the simulator
- all cars are roughly ~5m long - use that knowledge for calculating buffer distance
- different simulation delay from 0.02 until 7 s causes higher jerk and acc
