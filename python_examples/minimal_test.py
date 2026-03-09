#!/usr/bin/env python3
"""Minimal binding smoke test for sai_primitives + sai_model_py.

Usage:
  python3 python_examples/minimal_test.py
  python3 python_examples/minimal_test.py --urdf /path/to/robot.urdf
"""

from __future__ import annotations

import argparse
import pathlib
import sys
import glob

import numpy as np

SAI_MODEL_PY_PATH = "/Users/william/OpenSai/core/sai-model/build/python"
DEFAULT_URDF = "/Users/william/OpenSai/core/sai-model/urdf_models/puma/puma.urdf"


def call_first(obj, names, *args):
    for name in names:
        value = getattr(obj, name, None)
        if value is not None:
            if callable(value):
                return value(*args)
            if args:
                raise TypeError(f"Attribute '{name}' is not callable")
            return value
    raise AttributeError(f"None of {names} found on {type(obj)}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", default=DEFAULT_URDF)
    args = parser.parse_args()

    repo_root = pathlib.Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(repo_root / "build" / "python"))
    sys.path.insert(0, SAI_MODEL_PY_PATH)

    try:
        import sai_model_py as sm  # noqa: WPS433
    except ModuleNotFoundError as exc:
        candidates = glob.glob(f"{SAI_MODEL_PY_PATH}/sai_model_py*.so")
        raise RuntimeError(
            "Could not import sai_model_py with this Python interpreter.\n"
            f"Python: {sys.executable}\n"
            f"Available module files: {candidates}"
        ) from exc

    try:
        import sai_primitives_py as sp  # noqa: WPS433
    except ModuleNotFoundError as exc:
        candidates = glob.glob(
            str(repo_root / "build" / "python" / "sai_primitives_py*.so")
        )
        raise RuntimeError(
            "Could not import sai_primitives_py with this Python interpreter.\n"
            f"Python: {sys.executable}\n"
            f"Available module files: {candidates}\n"
            "Rebuild sai_primitives_py with the same interpreter used for sai_model_py, e.g.:\n"
            "  cmake -S . -B build -DBUILD_PYTHON_BINDINGS=ON "
            "-DPython_EXECUTABLE=$(which python3)\n"
            "  cmake --build build --target sai_primitives_py"
        ) from exc

    urdf_path = pathlib.Path(args.urdf)
    if not urdf_path.exists():
        raise FileNotFoundError(f"URDF not found: {urdf_path}")

    robot = sm.SaiModel(str(urdf_path), False)
    call_first(robot, ["update_model", "updateModel"])

    dof = call_first(robot, ["dof"])
    n_prec = [[1.0 if i == j else 0.0 for j in range(dof)] for i in range(dof)]

    joint_task = sp.JointTask(robot)
    joint_task.updateTaskModel(n_prec)
    joint_task.setGoalPosition(np.ones(6))
    joint_tau = joint_task.computeTorques()

    motion_task = sp.MotionForceTask(robot, "end-effector")
    motion_task.updateTaskModel(n_prec)
    motion_task.setGoalPosition(np.ones(3))
    motion_tau = motion_task.computeTorques()

    com_task = sp.ComMotionTask(robot, "end-effector")
    com_task.updateTaskModel(n_prec)
    com_task.setGoalPosition(np.ones(3))
    com_tau = com_task.computeTorques()

    print("sai_primitives binding smoke test passed")
    print(f"dof={dof}")
    print(f"JointTask torques length: {len(joint_tau)}")
    print(f"MotionForceTask torques length: {len(motion_tau)}")
    print(f"ComMotionTask torques length: {len(com_tau)}")
    print(joint_tau)
    print(motion_tau)
    print(com_tau)

if __name__ == "__main__":
    main()
