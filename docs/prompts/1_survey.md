# Systematic Survey Construction Prompt

> A reusable prompt template for building comprehensive, critically-reviewed academic surveys with Claude Code.
> Derived from a real survey construction process. Field-agnostic — works for any research domain.
> Hand this entire file to Claude Code along with your topic.

---

## How to Use

1. Reference this file in a Claude Code conversation (e.g. `@1_survey.md`).
2. State your survey topic (e.g., "stochastic MPC for safe robotics", "diffusion models in drug discovery", "federated learning for medical imaging").
3. Claude will ask clarifying questions before starting, then proceed through the phases below.
4. **This is Stage 1** (build the table + download PDFs into `reference/`). When it is done, run **Stage 2** with `@2_review.md` to critically score the corpus and build an interactive HTML scoreboard.

> **Running it:** no special orchestration mode is required. Run a normal Claude Code session at **high reasoning effort** (e.g. `xhigh`); the agent fans out **parallel subagents** itself via the Agent/Task tool (see *Parallelism Strategy* below). Multi-agent *workflow* orchestration ("ultracode") is **optional** and only worth its extra token cost for the heavy verification passes (Phases 2–3).

---

## Pre-flight: run with `--dangerously-skip-permissions` (do this FIRST)

This workflow makes **hundreds of read-only web fetches** (publishers, repositories, metadata APIs, author sites) and **writes many PDFs/files locally**. It is meant to run uninterrupted, so it assumes Claude Code was launched with permissions skipped.

**Step 1 — say this to the user, verbatim, before anything else:**

> This survey workflow is designed to run with permissions skipped so it isn't interrupted on every fetch/download. For the smoothest run, please relaunch Claude Code as:
>
> ```
> claude --dangerously-skip-permissions
> ```
>
> It only does read-only web research and writes PDFs into `reference/`. If you'd rather approve each action, you can keep going and approve the prompts as they appear.

**Step 2 — detect the mode and act on it.** You cannot read the flag directly, so infer it from behavior: if your **first** tool calls trigger permission prompts, you are NOT in skip mode — **stop**, repeat the message above, and wait for the user to relaunch (or to explicitly say "proceed with manual approvals"). Do not silently push through dozens of prompts.

**Institutional access (matters for PDF download in Step 1.2b):** the agent's WebFetch routes through Anthropic's servers and does NOT use your IP, so it cannot reach paywalled PDFs. The download script (`scripts/paper/download.py`) runs locally instead, so on a university IP it pulls paywalled non-Elsevier PDFs directly; Elsevier is the only manual case (details in Step 1.2b).

Do NOT proceed to Phase 0 until this has been communicated.

---

## Phase 0: Scoping (ask the user)

Before any research begins, ask the user the following questions using the AskUserQuestion tool. Do NOT proceed until answers are collected.

### Q1. Topic & Scope
- What is the survey topic? (user already provided this)
- What are the 2-3 intersecting research threads the survey should map?
  - e.g., "stochastic optimization + MPC + safety constraints"
  - e.g., "protein structure prediction + generative models + experimental validation"
  - e.g., "multi-agent RL + communication + emergent behavior"
  - e.g., "neural radiance fields + dynamic scenes + real-time rendering"

### Q2. Time Window
- How far back should the survey go?
  - Options: Last 3 years / Last 5 years / Last 7 years / Custom range
  - Note: "foundational" papers outside the window can be included with justification

### Q3. Source Scope
- Which publication tiers to include?
  - Options:
    - Top-tier venues only (Q1 journals + A* conferences)
    - Top-tier + strong workshops/arXiv with high citations
    - Broad (Q1-Q2 journals + A/B conferences + notable arXiv)
  - Field-specific venue list will be established after initial search

### Q4. Gap Discovery Mode
- How should research gaps be identified?
  - **User-directed**: User specifies known gaps upfront; survey validates/refutes them
  - **Emergent**: Survey discovers gaps organically through capability mapping
  - **Hybrid**: User provides initial hypotheses; survey refines and discovers additional gaps

### Q5. Deliverable Structure
- What sections does the user want? Propose a default based on the topic, but let the user override. Typical structures:

**For methods-heavy topics** (e.g., stochastic MPC, Bayesian optimization, ADMM variants):
```
1. Problem Formulation Variants
2. Solution Methods (by algorithm family)
3. Theoretical Guarantees (convergence, safety, stability)
4. Applications & Benchmarks
5. Software / Implementations
6. Open Problems
```

**For systems-heavy topics** (e.g., surgical robotics, autonomous driving perception, warehouse automation):
```
1-N. Capability categories (by task type or method)
N+1. Foundation models / Large-scale approaches
N+2. Datasets & Benchmarks
N+3. Simulation Platforms
N+4. Hardware
N+5. Open Problems
```

**For intersection topics** (e.g., LLMs + chemistry, diffusion models + protein design, RL + compiler optimization):
```
1. Thread A survey
2. Thread B survey
3. Intersection A+B (the core contribution)
4. Datasets & Benchmarks
5. Open Problems
```

### Q6. Field-Specific Columns
- What metadata matters for this field? Propose defaults and let the user add/remove:
  - **Universal**: Paper, Group, Venue, Year, Code, Weights/Models
  - **Methods** (e.g., optimization, numerical methods): Theoretical guarantees?, Computational complexity, Scalability
  - **Systems** (e.g., robotics, autonomous vehicles): Hardware, Sim platform, Sim2Real, Real-world validated?
  - **ML/AI** (e.g., foundation models, generative AI): Model size, Training data, Inference speed, Open weights?
  - **Control** (e.g., MPC, adaptive control): Stability proof?, Real-time?, Constraint handling
  - **Biomedical** (e.g., drug discovery, medical imaging): Clinical validation?, Dataset size, Patient diversity, Regulatory status
  - **HCI/NLP** (e.g., dialogue systems, accessibility): User study?, Sample size, Languages tested, Ethical review

### Q7. Output Language
- What language should the final deliverables be written in?
  - Options: English / Korean / Chinese / Japanese / Other
  - Note: Paper titles and technical terms remain in English regardless of output language

### Table Formatting Rules (non-negotiable)

These rules apply to ALL tables in survey.md. Enforce strictly.

1. **Column order must be consistent** within each section. Once a table header is defined, every row follows the same column order. Do NOT reorder columns between rows.

2. **Venue column = venue name only.** Do NOT include the year in the Venue column (e.g., write `NeurIPS` not `NeurIPS 2025`). The year is already in the dedicated Year column.

3. **Paper column = hyperlinked title.** Use the paper's DOI or arXiv link as the hyperlink target:
   - Preferred: `[**Paper Name**](https://doi.org/10.xxxx/xxxxx)` (DOI is permanent)
   - Fallback: `[**Paper Name**](https://arxiv.org/abs/XXXX.XXXXX)` (if no DOI)
   - No link: `**Paper Name**` (if neither DOI nor arXiv exists)

4. **All DOIs must be title-verified at the moment of discovery.** A DOI that resolves (HTTP 200/302) is NOT sufficient — many DOIs resolve to the wrong paper (e.g., off-by-one article numbers in Elsevier journals). For every DOI, query the CrossRef API and confirm the returned title matches the intended paper:
   ```
   WebFetch https://api.crossref.org/works/{DOI}
   → Extract "title" field → compare against expected title
   ```
   If the title does not match, the DOI is WRONG — search CrossRef by bibliographic query to find the correct one:
   ```
   WebFetch https://api.crossref.org/works?query.bibliographic={title+keywords}&query.container-title={journal}&rows=3
   ```
   This check must happen at Step 1.2 (when first recording a paper), not deferred to Phase 2. Phase 2 re-verifies as a safety net, but the primary check is at discovery time.

5. **All hyperlinks must be verified.** After building each table, batch-verify all links with HTTP status checks AND CrossRef title checks. Re-verify after any edit pass.

5. **Year column = integer only.** Just the publication year (e.g., `2025`), not `2025a` or `2025 (preprint)`.

---

## Phase 1: Initial Survey Construction

### Step 1.1: Seed Paper Discovery
```
Search strategy (parallel agents):
1. Search "[topic] survey 202X" for existing surveys — extract their reference lists
2. Search "[topic] [key_method]" for each sub-area
3. Search top venues in the field for recent accepted papers
4. Search Google Scholar / Semantic Scholar for highly-cited papers in the time window
5. Check "Awesome-[topic]" GitHub lists if they exist
```

### Step 1.2: Build the Main Table
For each paper found:
- Fetch arXiv/DOI page
- Extract: title, authors, venue, year, and field-specific columns from Q6
- **Immediately verify DOI → title match via CrossRef API** (see Table Formatting Rule 4). Do NOT defer this to Phase 2. A wrong DOI propagates silently through all downstream steps.
- Verify code/data links are accessible (HTTP status check)
- Add to the survey.md main table

### Step 1.2b: Acquire PDFs (automatic — do NOT ask, just download)

As soon as the main table exists, download every paper's PDF into `reference/` automatically (no confirmation prompt — just run it). Build a JSON list of `[year, venue, title, source, url_or_id]` (one entry per paper) and run the downloader once:

```
python scripts/paper/download.py --from-json papers_to_get.json
```

- `source` ∈ `arxiv` (id) · `url` (direct PDF link) · `ieee_doi` (`10.1109/…`, uses Playwright) · `elsevier` (ScienceDirect → auto-detected and listed for manual download).
- Files are saved as `reference/{Year}, {Venue}, {Title}.pdf` and content-validated (page-1 title match); mismatches are kept as `*.suspect` for you to recheck.
- For `url` sources prefer a permanent **direct PDF link** (publisher "content/pdf" URL, arXiv, MLR proceedings, author site).

**Tell the user about institutional access, verbatim:**

> WebFetch routes through Anthropic's servers and does not use your IP, so it can't fetch paywalled PDFs — this script runs locally instead. **On a university IP, all non-Elsevier downloads (arXiv, IEEE, Springer, Wiley, …) succeed automatically. Elsevier (ScienceDirect) blocks scripted access even on a university IP**, so those are listed at the end of the run for you to download manually in a browser, save into `reference/`, then normalize their filenames with `python scripts/paper/rename_by_title.py`.

After the run: if any PDFs are encrypted, run `python scripts/paper/decrypt.py`; to check/clean any manually-added filenames, run `python scripts/paper/rename_by_title.py --dry-run` first, then again without the flag to apply.

### Step 1.3: Create Detailed Entries
For each paper, create an individual .md file under `detailed_survey/sectionN/paper_name.md` containing:

```markdown
# Paper Name

**Full title:** ...
**Authors:** First Author et al. (Affiliation)
**Venue/Year:** ...
**arXiv/DOI:** ...

## Key Methodology
2-3 sentence summary of the core approach. State what is NEW vs prior work.

## [Field-Specific Sections]
(Adapted from Q6 — e.g., Theoretical Guarantees, Hardware, Architecture)

## Main Contributions
- Contribution 1 (state delta over prior work, not method description)
- Contribution 2
- Contribution 3

## Limitations / Gaps
- [Dimension 1 from the consistency checklist — see Phase 3]
- [Dimension 2]
- ...

## Results
| Task/Benchmark | Metric | Value | Context |
|----------------|--------|-------|---------|
| ... | ... | ... | (sim)/(real)/(theoretical) |

## Inference / Deployment (if applicable)
- Inference latency, hardware, real-time capability

## Dataset / Data (if applicable)
- Dataset used, collection method, scale, availability
```

### Step 1.4: Cross-Cutting Observations
For each section, create `_overview.md` with:
- Paper index table
- 5-10 cross-cutting observations identifying patterns, trends, and gaps
- Every observation must be verifiable against individual entries

---

## Phase 2: Link & Fact Verification

### Step 2.1: Link Verification
```
For ALL links in the survey:
1. DOI links: Query CrossRef API (https://api.crossref.org/works/{DOI}) and verify
   the returned title EXACTLY matches the paper title in the survey. HTTP 200 alone
   is NOT sufficient — Elsevier DOIs are frequently off-by-one and resolve to
   completely unrelated papers in the same journal.
2. arXiv links: WebFetch the arXiv abstract page and verify the title matches.
3. GitHub repos: WebFetch each URL, verify repo exists and matches description.
4. HuggingFace / project pages: WebFetch and verify content matches.
5. Report: total links, broken count, TITLE-MISMATCHED count, mismatched descriptions.
```

### Step 2.2: Metadata Verification
For each paper, cross-check against the actual source:
- **DOI → title match confirmed via CrossRef API?** (re-verify even if checked in Phase 1)
- Venue and year correct?
- Author list accurate?
- arXiv ID resolves to the right paper?
- Claimed results reproducible from the paper?

---

## Phase 3: Objectivity & Consistency Review

This is the most critical phase. Run review agents in parallel, one per section group.

### 3.1: Consistency Checklist
Define a set of **limitation dimensions** that EVERY paper in the survey must address. These are field-specific. Examples:

**For robotics/manipulation:**
- Force/impedance awareness (yes/no)
- Dexterous hand support (yes/no/partial)
- Sim-to-real transfer (yes/no/not applicable)
- Real-world validation scope (# objects, # tasks, # trials)
- Code/weights availability

**For control theory (e.g., MPC, adaptive control, robust control):**
- Stability guarantee (yes/no, what type: Lyapunov, ISS, etc.)
- Computational complexity (real-time feasible?)
- Constraint satisfaction (hard/soft/probabilistic)
- Scalability (state/input dimensionality tested)
- Comparison with baselines (which, fair?)

**For ML/AI (e.g., foundation models, generative models, RL):**
- Training data requirements
- Generalization tested? (in-distribution / OOD / zero-shot)
- Computational cost (training + inference)
- Reproducibility (code, data, hyperparams released?)
- Comparison fairness (same data splits, same compute budget?)

**For biomedical/clinical (e.g., drug discovery, imaging, genomics):**
- Clinical validation stage (in silico / in vitro / in vivo / clinical trial)
- Dataset diversity (patient demographics, multi-site?)
- Comparison to clinical standard of care
- Regulatory pathway addressed?
- Ethical approval / IRB status

**For NLP/HCI (e.g., dialogue, translation, accessibility):**
- Languages / domains tested
- User study conducted? (sample size, statistical test)
- Bias/fairness evaluation
- Real-world deployment evidence
- Ethical considerations discussed?

**For physics/simulation (e.g., CFD, molecular dynamics, climate):**
- Physical fidelity (validated against experiment?)
- Computational scaling (strong/weak scaling)
- Resolution / accuracy tradeoff
- Open-source solver?
- Reproducibility (input files, parameters shared?)

Adapt these to your specific field. The key principle: define dimensions BEFORE reviewing, then apply uniformly.

Every paper file MUST address every dimension. If a dimension doesn't apply, explicitly state "Not applicable because [reason]."

### 3.2: Review Criteria (per paper)

**A. Contributions — are they honest deltas?**
- Does each contribution state what is NEW vs prior work?
- Remove: "first", "pioneering", "novel", "groundbreaking", "state-of-the-art" unless:
  - Backed by comparative data, AND
  - Scoped precisely (e.g., "among the first RL-based approaches to X with sim-to-real transfer")
- Replace unqualified "First X" with: "Proposes X" or "Among the first to demonstrate X"
- Contributions must NOT restate the abstract — they must state the delta

**B. Limitations — are real gaps captured?**
- Apply ALL dimensions from the consistency checklist
- Use standardized wording for recurring limitations:
  - `> **Limitation ([dimension]):** [Standard text]`
- Flag if a paper claims generality but tests narrowly
- Flag if a paper claims "real-world" but tests on <5 objects in controlled settings
- Flag computational requirements if unusually high

**C. Results — are numbers real?**
- Every result must be a specific number, not "improved performance" or "strong results"
- If numbers unavailable: "Quantitative results: specific values not reported in the paper"
- Label every result: (sim), (real), (theoretical)
- For comparative results: state the baseline, the metric, and the margin
- Flag success rates presented without: trial count, object count, or success criterion

**D. Paper narrative bias**
- Are contributions taken directly from the paper's abstract without critical evaluation?
- Are "First X" claims verified or at least attributed ("per the authors")?
- Are papers from prestigious labs treated more favorably than equivalent work from smaller groups?

### 3.3: Review Execution
```
Launch parallel review agents (one per 2-3 sections):
- Each reads ALL files in assigned sections
- Checks every criterion above
- Reports: file, section, problematic text, suggested fix, severity
- Severity levels: CRITICAL (factual error / misleading) / MEDIUM (inconsistency) / MINOR (style)
```

### 3.4: Fix Application
```
Launch fix agents to apply all reported issues:
- CRITICAL fixes: apply immediately
- MEDIUM fixes: apply in batch
- MINOR fixes: apply if time permits
```

### 3.5: Final Verification Pass
After fixes, run one more review pass to verify:
- All CRITICAL issues resolved
- No new issues introduced by fixes
- Cross-cutting observations still accurate after individual entry changes
- Counts and categorizations in overview files match individual entries

---

## Phase 4: Missing Paper Discovery

### Step 4.1: Reference-Based Discovery
```
Launch parallel agents (one per section group):
1. For each paper in the survey, search for papers it cites and papers that cite it
2. Check existing survey papers in the field for papers they cover but we don't
3. Search "[topic] [year]" for recent top-venue publications
4. For each candidate, check: Is it within scope? Within time window? Already included?
5. Report: title, authors, venue, year, arXiv, which section, importance (critical/important/nice-to-have)
```

### Step 4.2: Gap-Based Discovery
```
For each gap identified in Phase 3 cross-cutting observations:
1. Search whether any paper addresses this gap
2. If found: add to survey
3. If not found: confirm as genuine open problem
```

### Step 4.3: Integration
```
For each missing paper to add:
1. Add row to survey.md table
2. Create detailed file with all standard sections
3. Update section _overview.md counts and observations
4. Run objectivity review on new entries only
```

---

## Phase 5: Enrichment (field-dependent, ask user)

Some enrichment passes are field-specific. Ask the user which apply:

### 5.1: Inference / Deployment (for ML/robotics)
- Add `## Inference / Deployment` to each paper:
  - Inference latency (ms, Hz)
  - Deployment hardware (GPU model, edge device)
  - Real-time capability assessment
  - Training hardware is optional (note but don't emphasize)

### 5.2: Dataset / Data Collection (for data-driven methods)
- Add `## Dataset / Data Collection` to each paper:
  - Dataset name (custom vs established benchmark)
  - Collection method (teleop, simulation, human video, MoCap, procedural)
  - Scale (episodes, hours, trajectories)
  - Equipment used (if teleop: what device)
  - Format and public availability

### 5.3: Theoretical Analysis (for theory-heavy fields)
- Add `## Theoretical Guarantees` to each paper:
  - What is proven (convergence, stability, safety, optimality)
  - Under what assumptions
  - Tightness of bounds
  - Gap between theory and practice

### 5.4: Computational Complexity (for algorithms)
- Add `## Complexity` to each paper:
  - Time/space complexity
  - Scaling behavior with problem dimension
  - Practical runtime on benchmark problems

---

## Phase 6: Final Structure & Cleanup

### 6.1: File Organization
```
project_root/
  1_survey.md                  # This prompt (Stage 1)
  2_review.md                  # Stage 2 prompt: scoring + interactive HTML
  survey.md                    # Main tables + cross-cutting analysis
  detailed_survey/
    section1/
      _overview.md             # Section summary + observations
      paper_name.md            # One per paper
      ...
    section2/
      ...
  reference/                   # Downloaded PDFs: "{Year}, {Venue}, {Title}.pdf"
  review/                      # Stage 2 output (papers.md ledger, reviews/, papers.html)
  scripts/paper/               # download.py · rename_by_title.py · decrypt.py · build_html.py
```

### 6.2: Final Checks

**Content quality:**
- [ ] All papers have quantitative results (or explicit "not reported" flag)
- [ ] All papers address every consistency dimension
- [ ] No unqualified "First" / "state-of-the-art" / "groundbreaking" claims
- [ ] All (sim)/(real)/(theoretical) labels on results
- [ ] Cross-cutting observation counts match individual entries
- [ ] No hardware/platform contradictions within entries
- [ ] Missing paper search completed for all sections
- [ ] Overview files updated after all additions

**Table formatting:**
- [ ] Column order consistent within each section table
- [ ] Venue column contains venue name only (no year)
- [ ] Year column contains integer only
- [ ] Paper column uses DOI hyperlink (preferred) or arXiv link (fallback)

**Link integrity (re-verify after EVERY edit pass):**
- [ ] All DOI links resolve correctly (HTTP 200 or 302 redirect)
- [ ] All arXiv IDs resolve to the correct paper (not just HTTP 200 — verify title match)
- [ ] All GitHub/project page links accessible
- [ ] 0 broken links total

**Language:**
- [ ] Output language matches user preference from Q7
- [ ] Paper titles and technical terms remain in English regardless of output language

---

## Parallelism Strategy

The following operations are independent and should run in parallel:

**Within each phase:**
- Phase 1: One agent per section for paper discovery + detailed file creation
- Phase 2: Split link verification by type (GitHub / arXiv / other)
- Phase 3: One review agent per 2-3 sections
- Phase 4: One missing-paper agent per section group
- Phase 5: Split enrichment by type (inference vs dataset) AND by section

**Across phases:**
- Phases 1-2 must complete before Phase 3
- Phase 3 must complete before Phase 4 (gaps inform missing paper search)
- Phase 5 can run in parallel with Phase 4

**Agent sizing:**
- Keep each agent to ~20-40 papers max for quality
- Large sections (40+ papers) should be split across 2 agents
- Review agents should read ALL files in their scope (no skipping)

---

## Anti-Patterns to Avoid

1. **Don't accept paper claims at face value.** Every "First X" and "SOTA" claim must be verified or attributed.
2. **Don't let WebFetch failures become permanent placeholders.** If arXiv is blocked, retry. If still blocked, use training knowledge but flag as unverified.
3. **Don't apply limitations unevenly.** If Paper A is criticized for lacking sim2real, Paper B with the same limitation must also be criticized.
4. **Don't inflate contributions by restating the method.** Contributions = what is new. Method = how it works. These are different.
5. **Don't use vague results.** "Improved performance" is not a result. "15% higher success rate on 100 objects (sim)" is a result.
6. **Don't confuse uniqueness-within-the-survey with paper contribution.** "Only paper in this section that does X" is a survey observation, not a paper contribution.
7. **Don't propagate stale arXiv IDs.** Verify every arXiv ID resolves to the correct paper.
7b. **Don't trust DOI resolution as DOI correctness.** A DOI that returns HTTP 200 can point to the wrong paper (common with Elsevier off-by-one article numbers). Always verify via CrossRef API title match: `https://api.crossref.org/works/{DOI}` → check `title` field. This is the single most common source of silent reference errors.
8. **Don't mix up hardware.** If the deployment section contradicts the header, resolve it from the source paper.
9. **Don't skip the final verification pass.** Fixes can introduce new inconsistencies.
10. **Don't treat all papers equally in depth.** Foundational papers deserve more context; incremental papers should be honestly positioned as incremental.
