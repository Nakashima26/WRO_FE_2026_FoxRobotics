# Native twin calculations

`sim.py` uses `NativeVehicle` for longitudinal dynamics, steering lag/backlash,
braking, wheel slip and bicycle motion. `World.cast` uses a batched C kernel
for sensor ray/segment intersections. Both kernels share the existing
`pure_pursuit.native` C99 build and loader; no Rust toolchain is required.

The Python implementations remain executable references. Select them with:

```sh
FOX_TWIN_PHYSICS=python FOX_TWIN_CAST=python python -m pure_pursuit.chassis_twin
```

`FOX_NATIVE=0` disables every native kernel, including these. Missing compilers
or failed native builds fall back through the existing native loader.
Vehicle parameters are copied when a run starts and must stay fixed during that
run, matching the simulator's existing cached steering limit.

Native dynamics use double precision, the same operation order and the system
math functions. In particular, separate sin/cos calls and a runtime exponent
prevent compiler substitutions that changed the Python result by one ULP.
Do not enable fast math or floating point contraction.

Validation is in `tests/test_native_vehicle.py` (all state fields after each
substep, including custom steering tables/reverse/sliding) and
`tests/test_native_cast.py` (distances plus exact material/object/normal hits).
Ray distances allow tiny rounding differences from NumPy's dot product;
complete race comparisons must also preserve the resulting trace and metrics.

On the development Mac, 24 seeds x 2 noise seeds at fixed frame cadence
(`uno.sh`, FF=0, FD=0) produced identical trace columns except CPU timing,
and identical non-timing metrics for Python versus both native kernels.
Summed run wall time was 286.624 s versus 278.365 s with four concurrent jobs.
This roughly 3% observation is host/load dependent, not a physical-car speedup.
The isolated vehicle kernel gain did not translate to a full-run improvement
until the batched ray kernel was added.
