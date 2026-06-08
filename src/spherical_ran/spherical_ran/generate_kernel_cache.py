#!/usr/bin/env python3
"""Pre-generates the RAN connection matrix and caches it to a .npz file.

Building the connection matrix M takes ~20s (it's an O(N^2) geodesic-distance
computation over every pair of sphere nodes). Run this once, ahead of time,
and the node can just load the result from disk on startup instead of paying
that cost every run.

Usage:
    python3 generate_kernel_cache.py
"""

import numpy as np
import pyvista as pv

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


def genetate_connection_matrix(sphere_points, J0, J, v_val):
    num_nodes = len(sphere_points)
    M = np.zeros((num_nodes, num_nodes))
    alphas = np.zeros((num_nodes, num_nodes))

    for i in range(num_nodes):
        for j in range(num_nodes):
            alphas[i, j] = geodesic_distance(sphere_points[i], sphere_points[j])
            M[i, j] = J0 + J * np.cos(np.pi * np.power((alphas[i, j] / np.pi), v_val)) * (1 / num_nodes)

    return alphas, M


def main():
    # These are the parameters the generated kernel depends on. If you change
    # any of these, the cached M is no longer valid for the new configuration —
    # that's why they're saved alongside M below, so a loader can check them.
    n_sub = 3
    J0 = 0
    J = 5.0
    v = 0.5

    # Step 1: build the sphere's nodes (same icosphere setup as the notebook).
    icosphere = pv.Icosphere(radius=1.0, nsub=n_sub)
    nodes = cartesian_to_polar_3D(icosphere.points.copy())

    # Step 2: the slow part — pairwise geodesic distances + connection strengths
    # for every node pair. This is the ~20s computation we're caching.
    print(f'Generating connection matrix for {len(nodes)} nodes (n_sub={n_sub})...')
    alphas, M = genetate_connection_matrix(nodes, J0, J, v)

    # Step 3: save everything needed to reconstruct AND validate this kernel
    # later — np.savez bundles multiple named arrays into one .npz file.
    # Relative to the directory you run this script from (e.g. run it from
    # the repo root: /home/noah/biodrone).
    out_path = 'src/spherical_ran/spherical_ran/kernel_cache.npz'
    np.savez(out_path,
             nodes=nodes, alphas=alphas, M=M,
             n_sub=n_sub, J0=J0, J=J, v=v)
    print(f'Saved kernel cache to {out_path}')


if __name__ == '__main__':
    main()
