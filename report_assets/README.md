# Project 2 report assets

`../report.tex` is the revised report. `../main.tex` is preserved unchanged.

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

The report distinguishes the selected passing campaign from unresolved issues
recorded for a separate November 9 execution. Neither post-processing nor document
compilation qualifies the current simulator source or resolves those execution issues.
