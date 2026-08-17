#!/usr/bin/env python3
"""Pre-generates the RAN connection matrix and caches it to a .npz file.

Building the connection matrix M takes ~20s (it's an O(N^2) geodesic-distance
computation over every pair of sphere nodes). Run this once, ahead of time,
and the node can just load the result from disk on startup instead of paying
that cost every run.

n_sub, v and the output path are read from the same ran_params.yaml that
spherical_RAN_server.py reads, which means the generator and the server can't
end up disagreeing. That disagreement is what the server's cache check refuses
to start on ("cached kernel parameters do not match current parameters"), and it
used to be easy to cause, because the values were hardcoded in both files and
had to be kept in step by hand.

There are deliberately no command-line overrides for n_sub or v. If you want to
change them, edit ran_params.yaml. That's the file the server reads, so editing
it is the only way to change both halves at once.

Usage:
    python3 generate_kernel_cache.py                     # uses ran_params.yaml
    python3 generate_kernel_cache.py --params-file X.yaml
    python3 generate_kernel_cache.py --out /tmp/trial.npz  # don't clobber the live cache
"""

import argparse
import os

import numpy as np
import pyvista as pv
import yaml

# Index constants for a polar point [r, theta, phi] — copied from
# mean_field_model_3d.ipynb so geodesic_distance/cartesian_to_polar_3D below
# match the notebook's convention exactly (M generated here must be
# byte-for-byte equivalent to what the notebook would produce).
X = MAG = 0
Y = PHI = 1     # phi = xy angle
Z = THETA = 2   # theta = z axis angle
QUALITY = 3


def cartesian_to_polar_3D(points):
    cpy = points.copy()
    for i in range(len(cpy)):
        x, y, z = cpy[i]
        r = np.sqrt(x**2 + y**2 + z**2)
        theta = np.arctan2(y, x)
        if r != 0.0:
            phi = np.arccos(z / r)
        else:
            phi = 0.0
        cpy[i] = r, theta, phi
    return cpy


def geodesic_distance(point1, point2):
    return np.arccos(np.clip(
        np.cos(point1[THETA]) * np.cos(point2[THETA])
        + np.sin(point1[THETA]) * np.sin(point2[THETA]) * np.cos(point1[PHI] - point2[PHI]),
        -1, 1))


def genetate_connection_matrix(sphere_points, v_val):
    num_nodes = len(sphere_points)
    M = np.zeros((num_nodes, num_nodes))
    alphas = np.zeros((num_nodes, num_nodes))

    for i in range(num_nodes):
        for j in range(num_nodes):
            alphas[i, j] = geodesic_distance(sphere_points[i], sphere_points[j])
            M[i, j] = np.cos(np.pi * np.power((alphas[i, j] / np.pi), v_val)) * (1 / num_nodes)

    return alphas, M


def default_params_file():
    """Find the ran_params.yaml that the server actually loads.

    This prefers the installed copy, which with --symlink-install points back at
    the source tree anyway. If that isn't found it falls back to the source tree
    directly, so the script still works in a shell that hasn't sourced the
    workspace.
    """
    try:
        from ament_index_python.packages import get_package_share_directory
        installed = os.path.join(
            get_package_share_directory('crazyswarm_bringup'), 'config', 'ran_params.yaml')
        if os.path.isfile(installed):
            return installed
    except Exception:
        pass

    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(
        here, '..', '..', 'crazyswarm_bringup', 'config', 'ran_params.yaml'))


def load_ran_params(path):
    """Flatten a ROS 2 params file into a plain dict.

    This merges the `ros__parameters` of every top-level section, in the order
    they appear in the file. That means a node-specific block placed after the
    `/**` wildcard overrides it, which is the same precedence ROS itself applies.
    """
    with open(path, 'r') as f:
        content = yaml.safe_load(f)

    params = {}
    for section in content.values():
        if isinstance(section, dict) and 'ros__parameters' in section:
            params.update(section['ros__parameters'])

    if not params:
        raise RuntimeError(
            f'{path} has no `ros__parameters` block -- is it a ROS 2 params file?')
    return params


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--params-file', default=default_params_file(),
                        help='ROS params yaml to read n_sub, v and kernel_cache from')
    parser.add_argument('--out', default=None,
                        help='override the output path (default: the yaml\'s kernel_cache)')
    # Using parse_known_args means a ROS-style invocation with a stray
    # --ros-args on it won't abort the script.
    args, _ = parser.parse_known_args()

    params = load_ran_params(args.params_file)
    # These are the parameters the generated kernel depends on. If you change any
    # of them, the cached M is no longer valid for the new configuration. That's
    # why they get saved alongside M below, so the server can check them.
    try:
        n_sub = int(params['n_sub'])
        v = float(params['v'])
        yaml_out = str(params['kernel_cache'])
    except KeyError as e:
        raise RuntimeError(
            f'{args.params_file} is missing {e}, which the kernel depends on.') from e

    print(f'Read from {args.params_file}: n_sub={n_sub}, v={v}')

    out_path = args.out or yaml_out
    if args.out and args.out != yaml_out:
        print(f'NOTE: writing to {out_path}, but the server loads {yaml_out}. '
              'It will not see this cache until you point kernel_cache at it.')

    # Step 1: build the sphere's nodes (same icosphere setup as the notebook).
    icosphere = pv.Icosphere(radius=1.0, nsub=n_sub)
    nodes = cartesian_to_polar_3D(icosphere.points.copy())

    # Step 2: the slow part — pairwise geodesic distances + connection strengths
    # for every node pair. This is the ~20s computation we're caching.
    print(f'Generating connection matrix for {len(nodes)} nodes (n_sub={n_sub})...')
    alphas, M = genetate_connection_matrix(nodes, v)

    # Step 3: save everything needed both to reconstruct and to validate this
    # kernel later. np.savez bundles the named arrays into one .npz file. The
    # yaml's kernel_cache path is relative, so it resolves against whichever
    # directory you run this script from (normally the repo root,
    # /home/noah/biodrone).
    np.savez(out_path,
             nodes=nodes, alphas=alphas, M=M,
             n_sub=n_sub, v=v)
    print(f'Saved kernel cache to {out_path}')


if __name__ == '__main__':
    main()
