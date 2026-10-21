"""Monte Carlo worker configuration regressions."""

import multiprocessing as mp
from pathlib import Path

import numpy as np
from conftest import compose_cfg

from scripts.run_montecarlo import _init, _run_episode, _worker_config


def test_file_aero_table_path_is_absolute_before_spawn(tabulated_aero_cfg, monkeypatch):
    cfg = tabulated_aero_cfg
    source = Path(cfg.plant.aero.file.path)
    monkeypatch.chdir(source.parent)
    cfg.plant.aero.file.path = source.name

    worker_cfg = _worker_config(cfg)

    path = Path(worker_cfg["plant"]["aero"]["file"]["path"])
    assert path.is_absolute()
    assert path.samefile(source)
    assert cfg.plant.aero.file.path == source.name


def test_falcon_worker_runs_reproducibly_after_spawn():
    cfg = compose_cfg(["scenario.horizon.t_max=0.1"])
    ctx = mp.get_context("spawn")
    with ctx.Pool(1, initializer=_init, initargs=(_worker_config(cfg),)) as pool:
        first, repeat, other = pool.map(_run_episode, [7, 7, 8])
    assert first == repeat
    assert np.isfinite(first["landing_speed"])
    assert first["landing_position_error"] != other["landing_position_error"]
