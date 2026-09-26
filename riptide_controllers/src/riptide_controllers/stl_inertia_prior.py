#! /usr/bin/env python3
#
# Compute an inertia-tensor PRIOR for the vehicle from its STL mesh.
#
# Uniform density scaled to the MEASURED total mass (the mass in the vehicle yaml comes from
# weighing the real robot). The geometry of the CAD is trusted; the mass *distribution* is not --
# so this is a prior/seed only: model_based_designer.py uses it to sanity-bound the effective
# inertia it identifies experimentally, never as the design value itself.
#
# The mesh is already in the body frame (vehicle long axis = y), so no rotation is applied.
#
# Run (no ROS needed):
#   ros2 run riptide_controllers2 stl_inertia_prior.py            # or plain python3
#   python3 stl_inertia_prior.py --robot talos --mesh-name Talos3.stl
#
# Writes ~/osu-uwrt/stl_inertia_prior.yaml, which model_based_designer.py picks up automatically.

import argparse
import datetime
import os
import sys

import numpy as np
import yaml


def find_source_root(start):
    """Walk up until we find a directory containing src/riptide_core (the workspace root)."""
    d = os.path.abspath(start)
    while d != os.sep:
        if os.path.isdir(os.path.join(d, "src", "riptide_core")):
            return d
        d = os.path.dirname(d)
    return None


def resolve_descriptions_path(sub):
    """Prefer the source tree; fall back to the installed riptide_descriptions2 share."""
    root = find_source_root(os.path.dirname(__file__)) or find_source_root(os.getcwd()) \
        or os.path.expanduser("~/osu-uwrt/release")
    for cand in (os.path.join(root, "src", "riptide_core", "riptide_descriptions", sub),
                 os.path.join(root, "src", "riptide_descriptions", sub)):
        if os.path.exists(cand):
            return cand
    try:
        from ament_index_python import get_package_share_directory
        return os.path.join(get_package_share_directory("riptide_descriptions2"), sub)
    except Exception:
        return cand


def main():
    ap = argparse.ArgumentParser(description="STL -> uniform-density inertia prior")
    ap.add_argument("--robot", default="talos")
    ap.add_argument("--mesh", default="", help="explicit STL path (overrides --mesh-name)")
    ap.add_argument("--mesh-name", default="Talos3.stl")
    ap.add_argument("--config", default="", help="explicit vehicle yaml (for mass + comparison)")
    ap.add_argument("--mass", type=float, default=0.0, help="override measured mass [kg]")
    ap.add_argument("--out", default=os.path.expanduser("~/osu-uwrt/stl_inertia_prior.yaml"))
    args = ap.parse_args()

    import trimesh   # heavy import; keep it after argparse so --help stays fast

    config_path = args.config or resolve_descriptions_path(os.path.join("config", args.robot + ".yaml"))
    mesh_path = args.mesh or resolve_descriptions_path(os.path.join("meshes", args.mesh_name))
    for p, what in ((config_path, "vehicle config"), (mesh_path, "mesh")):
        if not os.path.exists(p):
            sys.exit(f"ERROR: {what} not found: {p}")

    with open(config_path) as f:
        config = yaml.safe_load(f)
    mass = args.mass or float(config["mass"])

    print(f"Loading {mesh_path} (this can take a while for a big mesh)...")
    mesh = trimesh.load(mesh_path, force="mesh")
    volume = float(mesh.volume)
    if volume <= 0.0:
        sys.exit("ERROR: mesh volume is non-positive; the STL is unusable for mass properties")
    if not mesh.is_watertight:
        print("WARNING: mesh is not watertight -- volume/inertia are approximate")

    density = mass / volume
    mesh.density = density
    com = np.asarray(mesh.center_mass, dtype=float)
    inertia = np.asarray(mesh.moment_inertia, dtype=float)   # about COM, body-frame axes
    diag = [float(inertia[i, i]) for i in range(3)]

    config_inertia = [float(v) for v in config.get("inertia", [0.0, 0.0, 0.0])]
    config_com = [float(v) for v in config.get("com", [0.0, 0.0, 0.0])]

    sim_diag = None
    sim_path = os.path.join(os.path.dirname(config_path), "simulator.yaml")
    if os.path.exists(sim_path):
        with open(sim_path) as f:
            sim = yaml.safe_load(f)
        m = sim.get("vehicle_properties", {}).get("inertia3x3")
        if m:
            sim_diag = [float(m[0]), float(m[4]), float(m[8])]

    print()
    print(f"mesh          : {mesh_path}")
    print(f"watertight    : {mesh.is_watertight}   faces: {len(mesh.faces)}")
    print(f"extents [m]   : {np.round(mesh.extents, 3).tolist()}")
    print(f"volume [m^3]  : {volume:.6f}   measured mass [kg]: {mass:.3f}   density: {density:.1f}")
    print(f"COM (STL)     : {np.round(com, 3).tolist()}    (vehicle yaml com: {config_com})")
    print(f"I diag (STL)  : {[round(v, 4) for v in diag]}")
    print(f"I diag (yaml) : {config_inertia}")
    if sim_diag:
        print(f"I diag (sim)  : {sim_diag}   (simulator.yaml inertia3x3)")
    if int(np.argmin(diag)) != int(np.argmin(config_inertia)):
        print("WARNING: smallest-inertia axis differs between STL and vehicle yaml -- the yaml's")
        print("         'inertia' entry may be AXIS-SWAPPED (vehicle is long in y, so Iyy should")
        print("         be the smallest). The simulator's inertia3x3 agrees with the STL.")

    out = {
        "robot": args.robot,
        "generated": datetime.datetime.now().isoformat(timespec="seconds"),
        "mesh": mesh_path,
        "mass": mass,
        "volume": volume,
        "density": density,
        "watertight": bool(mesh.is_watertight),
        "com": [float(v) for v in com],
        "inertia_diag": diag,
        "inertia_3x3": [[float(inertia[i, j]) for j in range(3)] for i in range(3)],
        "vehicle_yaml_inertia": config_inertia,
        "simulator_inertia_diag": sim_diag,
    }
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, "w") as f:
        yaml.safe_dump(out, f, sort_keys=False)
    print(f"\nWrote {args.out}")


if __name__ == "__main__":
    main()
