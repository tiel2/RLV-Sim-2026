"""Settled, cylindrical LOX/RP-1 tanks and an equivalent uniform RCS gas reservoir.

Body x points toward the nose. Liquid fills from each aft tank boundary toward +x.
This quasi-static model assumes axial settling; it does not solve free-surface motion.
"""

import numpy as np


def dual_tank_properties(total_mass, rcs_fuel, p):
    """Return CG, principal inertia, liquid masses/heights/centroids from physical geometry."""
    dry = float(p.m_dry)
    gas = np.clip(float(rcs_fuel), 0.0, max(float(total_mass) - dry, 0.0))
    prop = max(float(total_mass) - dry - gas, 0.0)
    ratio = float(p.tank_of_ratio)
    liquids = prop * np.array([ratio, 1.0]) / (ratio + 1.0)
    radii = np.asarray(p.tank_radius)
    heights = liquids / (np.asarray(p.tank_density) * np.pi * radii**2)
    centers = np.asarray(p.tank_bottom) + heights / 2.0
    masses = np.r_[dry, liquids, gas]
    xs = np.r_[float(p.x_cg_dry), centers, float(p.rcs_gas_x)]
    cg = float(masses @ xs / masses.sum())
    inertia = np.array(p.I_dry, copy=True)
    inertia[0] += np.sum(liquids * radii**2 / 2.0)
    transverse = np.sum(liquids * (3.0 * radii**2 + heights**2) / 12.0)
    transverse += np.sum(masses * (xs - cg) ** 2)
    inertia[1:] += transverse
    inertia[0] += gas * float(p.rcs_gas_radius) ** 2 / 2.0
    inertia[1:] += gas * (3.0 * float(p.rcs_gas_radius) ** 2 + float(p.rcs_gas_length) ** 2) / 12.0
    return cg, inertia, liquids, heights, centers
