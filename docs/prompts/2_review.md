# Systematic Literature Review & Scoring Prompt

> A reusable prompt for **critically scoring** a paper corpus and producing an interactive,
> filterable/sortable HTML scoreboard with Claude Code.
> **Stage 2 of a two-stage pipeline:** run [`1_survey.md`](1_survey.md) first (it builds the
> survey table and downloads PDFs into `reference/`), then this.
> Field-agnostic — works for any research domain. Hand this entire file to Claude Code.

---

## How to Use

1. Reference this file in a Claude Code conversation (e.g. `@2_review.md`).
2. Make sure **Stage 1 is done**: a survey table of papers exists and their PDFs are in `reference/`.
3. Claude asks a few scoping questions (Phase 0), then scores every paper, writes the ledger and
   per-paper reviews, and builds `review/papers.html`.

> **Running it:** no special orchestration mode is required. Run a normal Claude Code session at
> **high reasoning effort** (e.g. `xhigh`); the agent parallelizes scoring/reviewing across
> subagents on its own. Reserve multi-agent *workflow* orchestration ("ultracode") for when you want
> an extra adversarial verification pass on the scores — it is optional and more token-expensive.

---

## Pre-flight: run with `--dangerously-skip-permissions` (do this FIRST)

This workflow reads many local PDFs, writes the ledger/reviews, and runs a build script. It is
meant to run uninterrupted.

**Step 1 — say this to the user, verbatim, before anything else:**

> This scoring workflow is designed to run with permissions skipped so it isn't interrupted. For
> the smoothest run, please relaunch Claude Code as:
>
> ```
> claude --dangerously-skip-permissions
> ```
>
> It reads the PDFs in `reference/` and writes into `review/`. If you'd rather approve each action,
> you can keep going and approve the prompts as they appear.

**Step 2 — detect the mode and act:** you cannot read the flag directly, so infer it. If your
**first** tool calls trigger permission prompts, you are NOT in skip mode — **stop**, repeat the
message above, and wait for the user to relaunch (or to say "proceed with manual approvals"). Do
not silently push through dozens of prompts.

Do NOT proceed to Phase 0 until this has been communicated.

---

## Phase 0: Scoping (ask the user via AskUserQuestion)

Collect these before scoring. Use the **AskUserQuestion** tool; offer the recommended option first.
Do NOT proceed until answered.

### Q1. Target venue / orientation — calibrates the rubric anchor and weights
The scoring anchor is *"what a reviewer at the target venue would say"*. Ask the user to pick an
orientation (vague is fine) or name specific venue(s):
- **Theory-focused** (e.g. IEEE T-AC, JMLR, Annals of Statistics) → emphasize C1/C2/C4.
- **Application-focused** (e.g. IEEE TASE, RA-L, applied/clinical venues) → emphasize C5/C7.
- **Balanced** (default) → the shipped weights.
- **Specific venue(s)** → use them as the calibration anchor and pick the nearest preset, then
  fine-tune (e.g. a deployment-heavy venue raises C7).

This choice selects a **weight preset** (Phase 1.2) — confirm the resulting weights with the user.

### Q2. C4 field-specific dimension — name it
C4 is the one field-specific criterion. Name it for this domain, reusing the field-specific
**limitation dimension** you defined in `1_survey.md` Phase 3. Examples:
uncertainty/risk-modeling fidelity · clinical-validation rigor · sim-to-real fidelity · scalability ·
sample efficiency · safety-guarantee strength. (If genuinely no field-specific axis applies, you may
fold C4's weight into C1/C3 and mark C4 N/A everywhere — but prefer naming one.)

### Q3. UI / output language
Ask: **English / Korean / Other.** This sets:
- `LANG` in `build_html.py` (`"en"` or `"ko"`; for "Other", add a string set — see Phase 2).
- the language of the review **prose** you write (paper titles and technical terms stay in English).
Korean is a common default for this user base, but ask rather than assume.

---

## Phase 1: Score every paper

### Step 1.1: Group the corpus into thematic threads
Cluster the papers into coherent threads labelled **A, B, C, …** (any number of groups). Each thread
is a methodological or topical family. Give each a one-line description; these become the legend in
the HTML. Keep grouping consistent with the Stage-1 survey sections where possible.

### Step 1.2: The scoring rubric (the scoring contract)

A single reusable rubric so papers are comparable on one scale. Three pillars — **Rigor, Novelty,
Impact**. Calibration anchor = the **Phase-0 target venue**. Scores are **relative across the
corpus** and may be re-calibrated corpus-wide (note the reason + a version tag in the ledger).

**Criteria (each scored 1–5, then weighted):**

| # | Criterion | Pillar | Measures |
|---|---|---|---|
| C1 | Theoretical Rigor & Correctness | Rigor | Formal statements; justified assumptions; complete proofs/derivations; no claimed-vs-implemented gap |
| C2 | Novelty & Originality | Novelty | New theory/method vs incremental; clear, scoped delta from prior art |
| C3 | Methodological Soundness | Rigor | Does the method do what it claims? Modeling choices appropriate, not self-defeating |
| C4 | **‹field-specific dimension›** (Q2) | Rigor | The key technical dimension of *this* field (e.g. uncertainty fidelity, clinical validity, scalability) |
| C5 | Empirical Validation & Baselines | Rigor | Head-to-head vs competitors; statistical rigor (CIs, repeats); realistic scenarios; tests the core claim |
| C6 | Reproducibility | Rigor | Hyperparameters; code/data; ablations; reimplementable |
| C7 | Practical Impact & Deployment | Impact | Real-world feasibility, implementation realism, readiness. Where the subfield is sim/benchmark-dominated, lack of HW/clinical/field deployment is **not** an automatic demerit — note it, don't punish it |
| C8 | Significance & Scientific Influence | Impact | Problem importance; who benefits; field influence/adoption (citations a *secondary* signal only) |

**Weight presets (choose per Phase-0 orientation; columns sum to 100):**

| # | Theory-focused | **Balanced (default)** | Application-focused |
|---|---:|---:|---:|
| C1 | 21 | 17 | 14 |
| C2 | 18 | 15 | 11 |
| C3 | 17 | 16 | 15 |
| C4 | 11 | 9 | 10 |
| C5 | 9 | 14 | 18 |
| C6 | 8 | 8 | 8 |
| C7 | 4 | 9 | 14 |
| C8 | 12 | 12 | 10 |
| **Pillars** | Rigor 66 · Nov 18 · Imp 16 | Rigor 64 · Nov 15 · Imp 21 | Rigor 65 · Nov 11 · Imp 24 |

Confirm the chosen column with the user, then use those weights in the ledger **and** in
`build_html.py` (`CRITERIA`). Fine-tune ±2–3 for a specific named venue if warranted.

**Out-of-scope criterion → N/A + renormalize, NOT 1.** If a criterion is outside a paper's scope,
mark it **`–`** and renormalize the remaining weights to 100% — do not score it 1. Judge a paper on
what it set out to do. Typical case: **C4** for a paper that is *deterministic / generic by design*
(e.g. a pure backbone, solver, or theory paper that makes no field-specific modeling claim). C4 is
still scored normally for any paper that *claims* to address the field-specific dimension. List the
N/A criteria when reporting a score.

**Anchors (1–5):** 5 flagship-tier · 4 strong (solid accept at the target venue) · 3 adequate with
notable gaps · 2 weak · 1 inadequate.

**Total** = (Σ score×weight) on the 1–5 scale, then ×20 → **/100**.

**Verdict bands (/100), at the rigor anchor** (what a target-venue reviewer would say — NOT where it
actually publishes):

| Band | /100 | Verdict |
|---|---|---|
| flagship | 85–100 | accept (often as-is) |
| strong | 75–84 | accept / minor revision |
| good | 65–74 | minor–major revision |
| fair | 55–64 | major revision |
| weak | 45–54 | reject & resubmit |
| poor | <45 | reject |

**Venue prestige ladder (separate axis from paper quality).** Build a short tier table for the
field's venues from the Stage-1 venue list, using SJR / field reputation as an objective prior. Map
each to **T1–T5 / Conf / arXiv** (T1 ≳2.0 SJR · T2 ~1.5–2.0 · T3 ~1.0–1.5 · T4 ~0.8–1.0 ·
T5 ~0.6–0.8 · Conf=conference · arXiv=preprint). **Tier is a venue prior; the band is paper quality.
Do not let one bleed into the other** ("rank venues by score" or "boost score by prestige" both
violate the rubric). Note any field-specific prestige correction (e.g. a small subfield whose top
journals are underweighted by raw SJR).

### Step 1.3: Write the scores ledger → `review/papers.md`

`build_html.py` parses this file. **Match the format exactly** (it is the contract):

````markdown
## Scores ledger

### A. ‹thread description› (‹count›)

| Paper | C1–C8 | /100 | Band | Tier | Note |
|---|---|---|---|---|---|
| Smith·Doe, A Foundational Method for Widgets **J. Widgets 2021** | 5·4·4·–·4·4·4·5 | **88** | flagship | T2 | seminal; ⚠️ direct competitor |
| Lee·Kim, An Incremental Extension **arXiv 2024** | 3·3·3·3·3·2·3·3 | **58** | fair | arXiv | minor delta |

### B. ‹next thread› (‹count›)

| Paper | C1–C8 | /100 | Band | Tier | Note |
|---|---|---|---|---|---|
| … |

## Distribution (optional summary — excluded from the build)
````

Row-format rules (a row that breaks these is silently dropped by the parser):
- **Section header** must be an `##` line containing the word **"ledger"**.
- **Group header**: `### <Letter>. <description> (<count>)` — `<Letter>` is a single A–Z capital.
- **Table**: exactly **6 columns** in this order: `Paper | C1–C8 | /100 | Band | Tier | Note`.
- **Paper cell**: `Authors, Title **Venue Year**`.
  - the venue+year go in `**bold**` at the **end** of the cell, and the bold text must **end with the
    4-digit year**; everything before the first comma is parsed as authors, the rest (up to the bold)
    as the title.
- **C1–C8 cell**: the N scores separated by a middle dot `·` (U+00B7), e.g. `5·4·4·–·4·4·4·5`; use
  `–` for any N/A criterion. The count and order must match `CRITERIA` in `build_html.py`.
- **/100 cell**: the integer in `**bold**`.
- **Band**: lowercase one of `flagship | strong | good | fair | weak | poor` (must match the /100 band).
- **Tier**: `T1 | T2 | T3 | T4 | T5 | Conf | arXiv`.
- **Note**: free text; include **⚠️** to flag a *direct competitor / must-cite*.
- An optional trailing `## Distribution …` H2 is **excluded** from the build (use it for your summary).

### Step 1.4: Write per-paper critical reviews → `review/reviews/<group>.md`

One file per thread: `review/reviews/A.md`, `B.md`, … Each paper is one `##` block. `build_html.py`
matches a review to its ledger row by **group + year + author/title/venue token overlap**, and shows
it in the expandable detail panel. **Match this format:**

````markdown
## (2021, J. Widgets, A Foundational Method for Widgets: Smith and Doe)

### 1. Core idea
Introduces the **widget transform** — an early convex formulation of …

### 2. Method
Solves a small SOCP per step; …

### 3. Critical review · gaps
- No uncertainty handling.
- Single benchmark; no head-to-head baseline.

### 4. Experiments
2 scenarios in sim; 88% success over 100 trials (sim). No real-world test.

### 5. Contributions
- Convex formulation of the widget problem (delta vs prior heuristic solvers).

### 6. Limitations · future work
- Deterministic by design — C4 marked N/A.

**Verdict:** Field-defining but deterministic; a robustness layer is the obvious extension.

---
````

Review-format rules:
- **Header**: `## (<Year>, <Venue>, <Title>: <Authors>)` — parentheses, comma-separated: Year, then
  Venue, then `Title: Authors`. The title may itself contain `": "`; authors are taken after the
  **last** `": "`. Keep venue/title/authors consistent with the ledger row so matching succeeds.
- **Sections**: `### <n>. <heading>` for `n` = 1..6 (the headings are display-only; the **number**
  is what's parsed). Default section labels (set in `build_html.py` `SEC_LABEL`):
  1 Core idea · 2 Method · 3 Critical review · gaps · 4 Experiments · 5 Contributions ·
  6 Limitations · future work. Use `- ` bullets or paragraphs; `**bold**` is rendered.
- **Verdict**: a line `**Verdict:** <one-liner>`. This marker is **required** — a block without it
  is skipped by the builder. (If you change the word "Verdict", change `VERDICT_MARK` to match.)
- Separate papers with a `---` line.
- Keep the scoring math in the ledger only; reviews hold prose + the one-line verdict.

---

## Phase 2: Build the interactive HTML

1. Edit the **PROJECT CONFIG** block at the top of `scripts/paper/build_html.py` to match Phase 0/1:
   - `PROJECT_TITLE` — the scoreboard's H1.
   - `LANG` — `"en"` or `"ko"` (Phase-0 Q3). For "Other", add a new key to the `STRINGS` dict
     (copy the `"en"` set, translate the values) and set `LANG` to it.
   - `CRITERIA` — the 8 `(id, label, weight)` tuples with the **renamed C4** and the **chosen weight
     preset**; `SCORE_SHORT` — the 8 short column labels (rename the "Field" one to match C4).
   - `PILLAR_NOTE` — the pillar split line for the chosen preset.
   - `BANDS`, `SEC_LABEL`, `VERDICT_MARK`, `TIER_NOTE` — adjust only if you changed them above.
   - `GROUP_GLOSS` — optional; leave `{}` to use the group descriptions from the ledger headers.
2. Run it:
   ```
   python scripts/paper/build_html.py
   ```
   → writes `review/papers.html` (a single self-contained file; open in any browser).
3. Sanity-check: the reported counts (papers, groups, reviews matched) look right, and the page has
   no literal `{{…}}` left and no empty detail panels for papers that have reviews.

**Read-state sharing:** checkmarks persist per-browser (localStorage). To share across machines, use
the page's **Export** button → `review/read_status.json`, commit it, and rebuild (or use **Import**).

---

## Objectivity & consistency rules (enforce while scoring)

These mirror `1_survey.md` Phase 3; apply them uniformly so the ledger is defensible.

1. **Honest deltas, not abstracts.** Strip unqualified "first / novel / SOTA / groundbreaking" unless
   backed by comparative data and precisely scoped. C2 rewards a *clear, attributed* delta.
2. **Real numbers, labelled.** Every empirical claim is a specific value with a `(sim)` / `(real)` /
   `(theoretical)` label, a baseline, and a metric. "Improved performance" is not a result.
3. **Apply limitations uniformly.** If paper A is dinged on C4/C5/C6, an equivalent paper B must be
   dinged the same way. No prestige halo — score the work, not the lab.
4. **N/A, not 1, for out-of-scope criteria** (Step 1.2). List the N/A criteria with the score.
5. **Band ≠ Tier.** The band is paper quality; the tier is venue prestige. Never derive one from the
   other.
6. **Calibrate to one anchor.** Re-score the whole corpus against the single Phase-0 anchor; remove
   ad-hoc per-paper fudge and venue offsets. If you re-calibrate, bump the version note in the ledger.
7. **Verify before you score.** Read the PDF in `reference/` (not just the abstract). If a PDF is
   missing/encrypted, flag it rather than scoring from memory.

---

## Anti-patterns to avoid

1. Don't score from the abstract — open the PDF.
2. Don't inflate C2 by restating the method; novelty = the delta over prior art.
3. Don't let a paper's venue prestige move its quality score (or vice-versa).
4. Don't give "all 4s/5s" to incremental or combination papers — deflate C2/C8 honestly.
5. Don't break the ledger/review **format contract** (Phase 1.3/1.4) — the builder will silently drop
   malformed rows or unmatched reviews.
6. Don't forget to mirror the rubric into `build_html.py` (`CRITERIA`/`SCORE_SHORT`/weights/`LANG`),
   or the HTML will disagree with the ledger.
7. Don't confuse "only paper in this thread that does X" (a survey observation) with a paper
   contribution.
