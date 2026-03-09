# python_examples

Minimal Python test for the bindings.

## Run

```bash
python3 python_examples/minimal_test.py
```

Optional custom URDF:

```bash
python3 python_examples/minimal_test.py --urdf /absolute/path/to/robot.urdf
```

The script checks that:
- `sai_model_py` imports from `/Users/william/OpenSai/core/sai-model/build/python`
- `sai_primitives_py` imports from `build/python`
- `JointTask`, `MotionForceTask`, and `ComMotionTask` can be constructed and called once

Important: `sai_model_py` and `sai_primitives` must be built for the same Python ABI version.
If imports fail, rebuild `sai_primitives` with your active Python:

```bash
cmake -S . -B build -DBUILD_PYTHON_BINDINGS=ON -DPython_EXECUTABLE=$(which python3)
cmake --build build --target sai_primitives_py
```
