#!/usr/bin/env python3
"""Pre-generates the RAN connection matrix and caches it to a .npz file --
Fibonacci-sphere variant of generate_kernel_cache.py (which uses an icosphere).

Cloned from generate_kernel_cache.py; the only difference is the node
generation step (Fibonacci golden-angle spiral instead of pv.Icosphere).
See spherical_RAN_server_fibonacci.py for the matching server clone.

Building the connection matrix M takes ~20s (it's an O(N^2) geodesic-distance
computation over every pair of sphere nodes). Run this once, ahead of time,
and the node can just load the result from disk on startup instead of paying
that cost every run.

Usage:
    python3 generate_kernel_cache_fibonacci.py
"""

import numpy as np

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


def generate_fibonacci_sphere_points(num_nodes):
    # Golden-angle spiral (see the "Storing 3D RAN Nodes" markdown in
    # mean_field_model_3d.ipynb, and the Fibonacci-sphere cell added there
    # right after the icosphere generation cell). No dedicated PyPI package
    # implements this -- it's short enough that none exists -- so it's
    # plain numpy, same as everything else here.
    i = np.arange(num_nodes)
    z = 1 - 2 * i / (num_nodes - 1)
    r_xy = np.sqrt(1 - z**2)
    golden_ratio = (1 + np.sqrt(5)) / 2
    golden_angle = 2 * np.pi / golden_ratio**2
    theta = i * golden_angle

    return np.stack([
        r_xy * np.cos(theta),
        r_xy * np.sin(theta),
        z,
    ], axis=1)


def main():
    # These are the parameters the generated kernel depends on. If you change
    # any of these, the cached M is no longer valid for the new configuration —
    # that's why they're saved alongside M below, so a loader can check them.
    # n_sub is kept only to derive a node count matching the icosphere's
    # N = 10*4^n_sub + 2 at the same resolution, for an apples-to-apples
    # comparison against kernel_cache.npz -- the Fibonacci sphere itself has
    # no notion of subdivision.
    n_sub = 3
    v = 0.3
    num_nodes = 10 * 4**n_sub + 2

    # Step 1: build the sphere's nodes (Fibonacci golden-angle spiral).
    fib_points = generate_fibonacci_sphere_points(num_nodes)
    nodes = cartesian_to_polar_3D(fib_points)

    # Step 2: the slow part — pairwise geodesic distances + connection strengths
    # for every node pair. This is the ~20s computation we're caching.
    print(f'Generating connection matrix for {len(nodes)} nodes (Fibonacci sphere, n_sub={n_sub} equivalent)...')
    alphas, M = genetate_connection_matrix(nodes, v)

    # Step 3: save everything needed to reconstruct AND validate this kernel
    # later — np.savez bundles multiple named arrays into one .npz file.
    # Relative to the directory you run this script from (e.g. run it from
    # the repo root: /home/noah/biodrone).
    out_path = 'src/spherical_ran/spherical_ran/kernel_cache_fibonacci.npz'
    np.savez(out_path,
             nodes=nodes, alphas=alphas, M=M,
             n_sub=n_sub, v=v)
    print(f'Saved kernel cache to {out_path}')


if __name__ == '__main__':
    main()
