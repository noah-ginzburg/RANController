#!/usr/bin/env python3
"""Pre-generates the RAN connection matrix and caches it to a .npz file --
Lloyd-relaxation variant of generate_kernel_cache.py (which uses a plain
icosphere).

Cloned from generate_kernel_cache.py; the only difference is the node
generation step (icosphere, then relaxed via spherical Lloyd's algorithm /
Centroidal Voronoi Tessellation, instead of a bare icosphere). This fixes the
icosphere's 12 fixed valence-5 vertices, which are ~16% denser than the rest
of the mesh -- see the Lloyd-relaxation cell in mean_field_model_3d.ipynb
(same algorithm, kept in sync manually) and the -18deg rotation workaround in
crazyflies.yaml, which this should eventually make unnecessary.
See spherical_RAN_server_fibonacci.py for the pattern to follow if/when a
matching server clone is wanted for this variant.

Building the connection matrix M takes ~20s (it's an O(N^2) geodesic-distance
computation over every pair of sphere nodes), plus ~10-20s for the Lloyd
relaxation itself. Run this once, ahead of time, and the node can just load
the result from disk on startup instead of paying that cost every run.

Usage:
    python3 generate_kernel_cache_lloyd.py
"""

import numpy as np
import pyvista as pv
from scipy.spatial import SphericalVoronoi, ConvexHull, cKDTree

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


def spherical_triangle_area(a, b, c):
    # Solid angle subtended by triangle a,b,c as seen from the sphere's
    # center == the triangle's area on the unit sphere (Van Oosterom &
    # Strackee formula) -- much simpler than L'Huilier's theorem.
    numer = np.abs(np.dot(a, np.cross(b, c)))
    denom = 1.0 + np.dot(a, b) + np.dot(b, c) + np.dot(c, a)
    return 2.0 * np.arctan2(numer, denom)


def lloyd_relax_sphere(points, n_iter=30):
    points = points / np.linalg.norm(points, axis=1, keepdims=True)
    for _ in range(n_iter):
        sv = SphericalVoronoi(points, radius=1.0, center=np.zeros(3))
        sv.sort_vertices_of_regions()
        new_points = np.empty_like(points)
        for i, region in enumerate(sv.regions):
            verts = sv.vertices[region]
            gen = points[i]
            # Fan-triangulate the (convex) spherical polygon from its own
            # generator point, area-weight each fan triangle's centroid.
            centroid = np.zeros(3)
            for j in range(len(verts)):
                a, b = verts[j], verts[(j + 1) % len(verts)]
                area = spherical_triangle_area(gen, a, b)
                tri_mid = gen + a + b
                centroid += area * (tri_mid / np.linalg.norm(tri_mid))
            new_points[i] = centroid / np.linalg.norm(centroid)
        points = new_points
    return points


def rotation_matrix(axis, angle):
    axis = axis / np.linalg.norm(axis)
    K = np.array([[0, -axis[2], axis[1]],
                  [axis[2], 0, -axis[0]],
                  [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)


def icosahedral_three_fold_axes():
    """The 10 axes of exact 3-fold rotational symmetry that the Lloyd-relaxed
    mesh inherits from its icosahedron seed -- each one is the (normalized)
    centroid of a pair of the base icosahedron's opposite faces (20 faces ->
    10 antipodal pairs). Spinning the *entire* relaxed node set 120 degrees
    about any one of these axes maps it exactly back onto itself (verified
    below, and in mean_field_model_3d.ipynb, to ~1e-8).

    This doesn't appear to have an established name in the literature --
    the closest match is calling it an "orbit" of the icosahedral rotation
    group's order-3 (C3) subgroup, in the group-theory sense used by
    equivariant bifurcation theory (Golubitsky & Stewart). Call it
    "icosahedral 3-fold orbit placement": any 3 target directions built by
    spinning one direction 120/240 degrees about one of these axes are
    guaranteed to see mathematically identical local node neighborhoods --
    not just similar, but the same set of distances -- so a competitive
    (winner-take-all) network has no structural reason to prefer one over
    the others. See the "Lloyd relaxation" section of
    mean_field_model_3d.ipynb for the full derivation and a worked example.
    """
    phi = (1 + np.sqrt(5)) / 2
    verts = np.array([
        [-1, phi, 0], [1, phi, 0], [-1, -phi, 0], [1, -phi, 0],
        [0, -1, phi], [0, 1, phi], [0, -1, -phi], [0, 1, -phi],
        [phi, 0, -1], [phi, 0, 1], [-phi, 0, -1], [-phi, 0, 1],
    ])
    verts = verts / np.linalg.norm(verts[0])

    hull = ConvexHull(verts)
    axes = []
    seen = set()
    for simplex in hull.simplices:
        axis = verts[simplex].mean(axis=0)
        axis = axis / np.linalg.norm(axis)
        # an axis is a line, not a direction -- collapse a vector and its
        # antipode (the two faces of the pair) to a single canonical sign
        if (axis[0], axis[1], axis[2]) < (0, 0, 0):
            axis = -axis
        key = tuple(np.round(axis, 6))
        if key not in seen:
            seen.add(key)
            axes.append(axis)
    return axes


def symmetric_target_triplet(axis, cone_angle_deg, distance=1.0):
    """3 target positions that are exact 120/240-degree rotations of each
    other about `axis` (one of the axes from icosahedral_three_fold_axes),
    `cone_angle_deg` degrees off that axis, at the given distance. Because
    they're related by the mesh's own exact symmetry, all 3 see identical
    node neighborhoods -- see icosahedral_three_fold_axes' docstring.
    """
    axis = axis / np.linalg.norm(axis)
    theta = np.radians(cone_angle_deg)
    tmp = np.array([1.0, 0.0, 0.0]) if abs(axis[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    e1 = np.cross(axis, tmp)
    e1 /= np.linalg.norm(e1)
    v0 = np.cos(theta) * axis + np.sin(theta) * e1
    v1 = rotation_matrix(axis, 2 * np.pi / 3) @ v0
    v2 = rotation_matrix(axis, 4 * np.pi / 3) @ v0
    return distance * np.array([v0, v1, v2])


def _demo_symmetric_targets(relaxed_points):
    # Sanity-check + usage example: pick the first of the 10 axes, build a
    # target triplet, and confirm all 3 see the same nearest-neighbor
    # distances -- proof the "identical neighborhood" claim actually holds
    # for this cached mesh, not just in theory.
    axes = icosahedral_three_fold_axes()
    axis = axes[0]
    targets = symmetric_target_triplet(axis, cone_angle_deg=39.0)

    tree = cKDTree(relaxed_points)
    print(f'\n{len(axes)} icosahedral 3-fold axes found. Demo target triplet on axis {np.round(axis, 3)}:')
    for i, t in enumerate(targets):
        dists, _ = tree.query(t, k=5)
        print(f'  target {i}: direction={np.round(t, 3)}  nearest-5 node distances (deg)='
              f'{np.round(np.degrees(dists), 3)}')


def main():
    # These are the parameters the generated kernel depends on. If you change
    # any of these, the cached M is no longer valid for the new configuration —
    # that's why they're saved alongside M below, so a loader can check them.
    n_sub = 3
    v = 0.3
    n_lloyd_iter = 30

    # Step 1: build the sphere's nodes (icosphere, then Lloyd-relaxed).
    icosphere = pv.Icosphere(radius=1.0, nsub=n_sub)
    print(f'Relaxing {icosphere.n_points} icosphere points ({n_lloyd_iter} Lloyd iterations)...')
    relaxed_points = lloyd_relax_sphere(icosphere.points.copy(), n_iter=n_lloyd_iter)
    nodes = cartesian_to_polar_3D(relaxed_points)

    _demo_symmetric_targets(relaxed_points)

    # Step 2: the slow part — pairwise geodesic distances + connection strengths
    # for every node pair. This is the ~20s computation we're caching.
    print(f'Generating connection matrix for {len(nodes)} nodes (Lloyd-relaxed icosphere, n_sub={n_sub})...')
    alphas, M = genetate_connection_matrix(nodes, v)

    # Step 3: save everything needed to reconstruct AND validate this kernel
    # later — np.savez bundles multiple named arrays into one .npz file.
    # Relative to the directory you run this script from (e.g. run it from
    # the repo root: /home/noah/biodrone).
    out_path = 'src/spherical_ran/spherical_ran/kernel_cache_lloyd.npz'
    np.savez(out_path,
             nodes=nodes, alphas=alphas, M=M,
             n_sub=n_sub, v=v)
    print(f'Saved kernel cache to {out_path}')


if __name__ == '__main__':
    main()
