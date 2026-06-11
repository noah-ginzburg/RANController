#!/usr/bin/env python3
"""Empirical (beta, u) sweep for the spherical RAN, using the real cached M.

Runs the exact update equation from spherical_RAN_server.py (with leak=1.0,
i.e. no 1.3 modifier) for a grid of (beta, u) and reports, for each combo:

  - spontaneous_bump : does a bump form from noise alone with NO target input?
                       (should be False -- otherwise the drone "wanders" with
                       no targets present)
  - sat_frac         : single-target steady-state peak activation, as a
                       fraction of the theoretical ceiling (1+tanh(beta)).
                       Close to 1.0 = saturated.
  - switch_time_s    : lock onto target A, then abruptly move the input to
                       target B (180 deg away, simulating "passing B").
                       Seconds of sim time until the population-vector heading
                       comes within ~11 deg of B. inf = never switches
                       (this is the "stuck bump" failure mode).
  - two_bump_ratio   : both targets A and B present simultaneously, 90 deg
                       apart. min(z_A, z_B)/max(z_A, z_B) at steady state.
                       Close to 0 = single dominant bump (good).
                       Close to 1 = two simultaneous bumps (bad).

Usage:
    python3 RAN/bifurcation_sweep.py
"""

import numpy as np

X = MAG = 0
Y = PHI = 1     # phi = xy angle (azimuth)
Z = THETA = 2   # theta = z axis angle (polar)
QUALITY = 3

CACHE_PATH = 'src/spherical_ran/spherical_ran/kernel_cache.npz'

# Fixed params (matching current spherical_RAN_server.py defaults, leak=1.0)
LEAK = 1.0
RATE = 12.0
SIGMA = 1.5
KAPPA = 20.0
TARGET_QUALITY = 20.0
DT = 1.0 / 50.0  # RAN_UPDATE_RATE

STEADY_STEPS = 600   # 12s sim time to reach steady state
SWITCH_STEPS = 600   # 12s sim time to observe post-switch behavior
SWITCH_THRESH_RAD = np.deg2rad(11.0)
N_SEEDS = 3


def geodesic_distance(p1, p2):
    return np.arccos(np.clip(
        np.cos(p1[THETA]) * np.cos(p2[THETA])
        + np.sin(p1[THETA]) * np.sin(p2[THETA]) * np.cos(p1[PHI] - p2[PHI]),
        -1, 1))


def generate_sensory_input(nodes, targets, kappa):
    num_nodes = len(nodes)
    b = np.zeros(num_nodes)
    for t in targets:
        target_point = np.array((t[MAG], t[PHI], t[THETA]))
        for i in range(num_nodes):
            alpha = geodesic_distance(nodes[i], target_point)
            b[i] += np.exp(kappa * (np.cos(alpha) - 1.0)) * t[QUALITY]
    b *= 1.0 / np.sqrt(num_nodes)
    return b


def polar_to_cartesian(nodes):
    out = np.zeros((len(nodes), 3))
    for i in range(len(nodes)):
        r, theta, phi = nodes[i]
        out[i, 0] = r * np.sin(phi) * np.cos(theta)
        out[i, 1] = r * np.sin(phi) * np.sin(theta)
        out[i, 2] = r * np.cos(phi)
    return out


def find_vel_avg(cart_nodes, z):
    total = np.sum(z)
    if total == 0:
        return np.zeros(3)
    return (cart_nodes * z[:, None]).sum(axis=0) / total


def step(z, M, b, u, beta, rate, dt, sigma, n, rng):
    noise = rng.normal(0.0, sigma * np.sqrt(dt), n) * (1.0 / np.sqrt(n))
    return (z
            + dt * rate * (-(LEAK * z) + np.tanh(u * (M @ z) + b - beta) - np.tanh(-beta))
            + np.sqrt(rate) * noise)


def run(z, M, b, u, beta, steps, n, rng, rate=RATE):
    for _ in range(steps):
        z = step(z, M, b, u, beta, rate, DT, SIGMA, n, rng)
    return z


def evaluate(M, nodes, cart_nodes, n, u, beta, seed):
    rng = np.random.default_rng(seed)

    # --- spontaneous bump check (no targets) ---
    z0 = rng.normal(0, 0.01, n)
    b_zero = np.zeros(n)
    z_spont = run(z0.copy(), M, b_zero, u, beta, STEADY_STEPS, n, rng)
    spontaneous_bump = np.max(np.abs(z_spont)) > 0.5

    # --- single-target saturation ---
    target_a = [1.0, 0.0, np.pi / 2, TARGET_QUALITY]  # azimuth=0, equator
    b_a = generate_sensory_input(nodes, [target_a], KAPPA)
    z_a = run(np.zeros(n), M, b_a, u, beta, STEADY_STEPS, n, rng)
    ceiling = (1.0 - np.tanh(-beta)) / LEAK
    sat_frac = np.max(z_a) / ceiling

    # --- switch test: A -> B (180 deg away) ---
    target_b = [1.0, np.pi, np.pi / 2, TARGET_QUALITY]  # azimuth=180deg
    b_b = generate_sensory_input(nodes, [target_b], KAPPA)
    target_b_dir = polar_to_cartesian(np.array([[1.0, np.pi, np.pi / 2]]))[0]

    z = z_a.copy()
    switch_time_s = np.inf
    for step_i in range(SWITCH_STEPS):
        z = step(z, M, b_b, u, beta, RATE, DT, SIGMA, n, rng)
        vec = find_vel_avg(cart_nodes, z)
        norm = np.linalg.norm(vec)
        if norm > 1e-9:
            vec = vec / norm
            ang = np.arccos(np.clip(np.dot(vec, target_b_dir), -1, 1))
            if ang < SWITCH_THRESH_RAD:
                switch_time_s = (step_i + 1) * DT
                break

    # --- two simultaneous targets, 90 deg apart ---
    target_c = [1.0, np.pi / 2, np.pi / 2, TARGET_QUALITY]  # azimuth=90deg
    b_ac = generate_sensory_input(nodes, [target_a, target_c], KAPPA)
    z_ac = run(np.zeros(n), M, b_ac, u, beta, STEADY_STEPS, n, rng)

    idx_a = np.argmin([geodesic_distance(node, np.array((target_a[MAG], target_a[PHI], target_a[THETA]))) for node in nodes])
    idx_c = np.argmin([geodesic_distance(node, np.array((target_c[MAG], target_c[PHI], target_c[THETA]))) for node in nodes])
    z_pair = sorted([z_ac[idx_a], z_ac[idx_c]])
    two_bump_ratio = z_pair[0] / z_pair[1] if z_pair[1] > 1e-9 else 0.0

    return spontaneous_bump, sat_frac, switch_time_s, two_bump_ratio


def main():
    data = np.load(CACHE_PATH)
    nodes = data['nodes']
    M = data['M']
    n = len(nodes)
    cart_nodes = polar_to_cartesian(nodes)

    betas = [0.5, 0.75, 1.0, 1.25, 1.5, 2.0, 2.5]
    us = [1.5, 2.0, 2.5, 2.8, 3.0, 3.5, 4.0, 5.0, 6.0, 8.0]

    print(f'leak={LEAK}, rate={RATE}, sigma={SIGMA}, kappa={KAPPA}, target_quality={TARGET_QUALITY}, N={n}')
    print(f'{"beta":>5} {"u":>5} {"spont":>6} {"sat_frac":>9} {"switch_s":>9} {"2bump":>6}')

    results = []
    for beta in betas:
        for u in us:
            spont_votes = []
            sat_fracs = []
            switch_times = []
            ratios = []
            for seed in range(N_SEEDS):
                spont, sat, sw, ratio = evaluate(M, nodes, cart_nodes, n, u, beta, seed=1000 * seed + int(beta * 100) + int(u * 10))
                spont_votes.append(spont)
                sat_fracs.append(sat)
                switch_times.append(sw)
                ratios.append(ratio)

            spont_any = any(spont_votes)
            sat_frac = np.mean(sat_fracs)
            switch_s = np.mean(switch_times)
            ratio = np.mean(ratios)
            results.append((beta, u, spont_any, sat_frac, switch_s, ratio))
            sw_str = f'{switch_s:9.2f}' if np.isfinite(switch_s) else '      inf'
            print(f'{beta:5.2f} {u:5.2f} {str(spont_any):>6} {sat_frac:9.3f} {sw_str} {ratio:6.3f}')

    print()
    print('Candidates: no spontaneous bump, switches within 8s, single dominant bump (ratio<0.3):')
    for beta, u, spont, sat, sw, ratio in results:
        if (not spont) and np.isfinite(sw) and sw < 8.0 and ratio < 0.3:
            print(f'  beta={beta:.2f} u={u:.2f}  sat_frac={sat:.3f} switch_s={sw:.2f} two_bump_ratio={ratio:.3f}')


if __name__ == '__main__':
    main()
