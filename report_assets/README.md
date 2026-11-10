# Project 2 report assets

`../report.tex` is the final Project 2 report and produces `../report.pdf`.
`../main.tex` is a separate working document. It is not the final report entry point.

The figures and `metrics.json` use the archived five-case verification campaign in
`results/verification/project2_segments/physics/`, with run IDs dated 2026-11-03.
They do not use later calibrated flight scenarios. The figure-generation script
does not run the simulator or change the archived data.

From the repository root:

```bash
.venv/bin/python report_assets/generate_figures.py
latexmk -pdf report.tex
# Alternative compiler:
tectonic report.tex
```

The script verifies the pinned CRS-11 dataset, computes time-weighted comparison
statistics, cross-checks the landing contact KPIs, and records input SHA-256 hashes
in `metrics.json`. PDF figures are used by LaTeX; PNG copies allow quick inspection.
Archived input folders are required to regenerate figures but not to compile the
report using the included figure PDFs.

## Navigation extension (10 November 2026)

The navigation campaign executes the current simulator and the existing
`synthetic_eskf` implementation. Regenerate it independently of the archived
physics figures. From the repository root, use:

```bash
.venv/bin/python report_assets/generate_navigation.py
.venv/bin/python -m pytest tests/test_synthetic_eskf.py tests/sim/test_timing_contract.py tests/sim/test_multirate.py tests/sim/test_flight_calibration.py -q
tectonic main.tex
tectonic report.tex
```

The generator writes four paired closed-loop runs under `results/report_navigation/`:
identity and ESKF feedback for `project2_landing` and `flight_calibrated_landing`.
Within each pair, only navigation changes. Native CG specific force and inertial angular
rate are recorded at every navigation tick and drive the common-sample replay.

The ESKF uses rotating-frame INS, gravity gradients, a global attitude-error reset,
radial height, and a one-time uncertain alignment observation (1 degree per axis).
Missing initial GNSS uses a declared nominal handoff prior with covariance; no truth
position/velocity fallback or zero substitution is used. The first GNSS initialization
sample is used only once. Mass is still an ideal auxiliary measurement.

`robustness/` holds 10 seeds for each of two landing scenarios and a 3-second startup
outage condition (30 runs total). A native 150-second ascent with its original gimbal
and throttle schedule is replayed for five seeds under nominal aiding, a 5-second startup
outage, and a 20-second high-altitude outage. The fifteen replays save sensor channels,
state-error norms, estimated biases, covariance diagonals, and NEES/NIS diagnostics.
The startup priors have deliberate position/velocity offsets. No guidance or controller
gains are changed in this campaign.

`navigation_metrics.json` records metrics, source/configuration hashes, and package
versions. The navigation figures and generated tables compile without the results folder.
The finite seed counts and covariance diagnostic limitations are stated in the report;
unsuccessful landings are included in the summary and extrema.

## Standalone comparison and optional sensor experiments

The default sensor settings are 100 Hz IMU / 10 Hz GNSS / 20 Hz radial height.
The IMU rate must match the explicit scenario navigation period; both runners reject
mismatches. Default outage/outlier schedules are empty and innovation gates are disabled.

`python scripts/analyze_navigation.py scenario=project2_landing` retains the standalone
comparison and its four diagnostic plots. It uses native force/rate histories and one
common uncertain initialization for IMU-only and ESKF, with no exact truth-state fallback.
`run_single.py` with synthetic navigation also exports this separately seeded replay.
Optional scale factors, sensor-bias random walks and innovation gates are available for
explicit stress experiments. They do not change the report's nominal sensor assumptions.
