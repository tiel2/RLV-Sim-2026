# CRS-11 reference snapshot

Commit-pinned, unmodified upstream data from
[shahar603/Telemetry-Data](https://github.com/shahar603/Telemetry-Data).
See [manifest.json](manifest.json) for source URLs, commit, retrieval date, and SHA-256 hashes,
and [LICENSE](LICENSE) for the upstream Unlicense.

The [flight-reference specification](../../../docs/reference/crs11_flight_reference.md)
defines units, selected phases, comparison roles, quality limitations, and missing inputs.
Use `stage1_raw.json` for primary altitude/speed observations; `analysed.json` contains
derived auxiliary data. These are webcast observations, not onboard sensor logs.

From the repository root, run `python3 scripts/check_flight_reference.py` to verify the
snapshot offline and regenerate `quality_report.json`. Audit thresholds are not measurement
uncertainties or flight-model acceptance criteria.
