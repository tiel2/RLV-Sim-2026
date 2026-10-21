#!/usr/bin/env python3
# ruff: noqa: E501
"""Build a self-contained, filterable/sortable HTML view of a paper scores ledger.

Reads the "Scores ledger" section of docs/reference/review/papers.md
(papers grouped A, B, C, …) plus per-group critical reviews in
docs/reference/review/detailed_reviews/<group>.md, and emits a single
docs/reference/review/papers.html you can open in any browser. Read/unread checkmarks persist in
the browser's localStorage and can be shared across machines via read_status.json.

Edit the PROJECT CONFIG block below to match your survey topic and scoring rubric.
2_review.md walks an agent through producing papers.md / detailed_reviews/ and this config.

Usage:  python build_html.py
"""

import hashlib
import html
import json
import os
import pathlib
import re
from urllib.parse import quote

# This script sits in scripts/paper/. The paper corpus lives under docs/reference/.
REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
REFERENCE_DIR = REPO_ROOT / "docs" / "reference"
DATA = REFERENCE_DIR / "review"
REVIEW_DIR = DATA / "detailed_reviews"
SRC = DATA / "papers.md"
OUT = DATA / "papers.html"
READ_STATUS = DATA / "read_status.json"  # committed read-state, baked in as the default

# ====================== PROJECT CONFIG (edit me) ========================
PROJECT_TITLE = "Literature Review"  # page <h1>
LANG = "en"  # HTML UI language: "en" or "ko" (see STRINGS)

# C1..Cn rubric: (id, full label, weight). Weights should sum to 100%.
# C4 = uncertainty & robustness treatment: how rigorously/appropriately a paper
# models and handles uncertainty/disturbance/risk for its own problem, judged
# method-agnostically (robust, adaptive, stochastic, chance all equally eligible).
CRITERIA = [
    ("C1", "Theoretical rigor & correctness", "17%"),
    ("C2", "Novelty & originality", "15%"),
    ("C3", "Methodological soundness", "16%"),
    ("C4", "Uncertainty & robustness treatment", "9%"),
    ("C5", "Empirical validation & baselines", "15%"),
    ("C6", "Reproducibility", "6%"),
    ("C7", "Practical impact & deployment", "10%"),
    ("C8", "Significance & influence", "12%"),
]
# Short labels for the per-paper score table header (1:1 with CRITERIA above).
SCORE_SHORT = ["Rigor", "Novelty", "Method", "Uncert.", "Empirical", "Repro", "Deploy", "Signif."]
# Pillar summary line under the rubric (free text).
PILLAR_NOTE = "Rigor 63% · Novelty 15% · Impact 22%."

# Score -> band: (name, min score inclusive, reviewer verdict at a top venue).
BANDS = [
    ("flagship", 85, "accept (often as-is)"),
    ("strong", 75, "accept / minor revision"),
    ("good", 65, "minor–major revision"),
    ("fair", 55, "major revision"),
    ("weak", 45, "reject & resubmit"),
    ("poor", 0, "reject"),
]

# Optional one-line gloss per group {"A": "...", ...}. Leave {} to use the group
# titles parsed straight from the ledger headers in papers.md.
GROUP_GLOSS = {}

# Per-paper review sections: section number -> heading (parsed from detailed_reviews/<group>.md).
SEC_LABEL = {
    "1": "Core idea",
    "2": "Method",
    "3": "Critical review · gaps",
    "4": "Experiments",
    "5": "Contributions",
    "6": "Limitations · future work",
}
# Bold marker preceding the one-line verdict in each review, e.g. **Verdict:** ...
VERDICT_MARK = "Verdict"
VERDICT_MARKS = (VERDICT_MARK, "평가")

# Venue-tier axis.
TIER_RANK = {"T1": 7, "T2": 6, "T3": 5, "T4": 4, "T5": 3, "Conf": 2, "arXiv": 1}
TIER_NOTE = (
    "Tier = venue prestige (SJR/field, a separate axis): "
    "T1 ≳2.0 · T2 ~1.5–2.0 · T3 ~1.0–1.5 · T4 ~0.8–1.0 · T5 ~0.6–0.8 · "
    "Conf=conference · arXiv=preprint."
)
# ==================== END PROJECT CONFIG ================================

# --- derived rubric structures ---------------------------------------------
NCRIT = len(CRITERIA)
SCORE_COLS = [
    (SCORE_SHORT[i] if i < len(SCORE_SHORT) else cid, f"{cid} {label} ({wt})")
    for i, (cid, label, wt) in enumerate(CRITERIA)
]
BAND_NAMES = [b[0] for b in BANDS]
BAND_RANK = {b[0]: len(BANDS) - i for i, b in enumerate(BANDS)}

# --- UI strings (LANG selects one set; agents may add more languages) ------
STRINGS = {
    "en": {
        "page_title": "Scores & Reading Tracker",
        "export": "⬇ Export",
        "export_t": "Save current read-state to read_status.json (commit it to share across machines)",
        "import": "⬆ Import",
        "import_t": "Load a read_status.json made on another machine",
        "baked": "↻ Committed",
        "baked_t": "Apply the committed read-state baked into this build",
        "font_t": "Font size",
        "theme_t": "Theme",
        "papers": "papers",
        "read": "read",
        "read_flabel": "Read",
        "unread": "unread",
        "read_chip": "read",
        "search_ph": "Search author · title · venue · note…",
        "reset": "Reset filters",
        "shown": "shown",
        "empty": "No papers match the filters.",
        "rubric_summary": "Scoring rubric · band definitions",
        "rubric_h_crit": "Criteria (weighted)",
        "rubric_h_band": "Bands",
        "imported": "Imported · read ",
        "parse_fail": "JSON parse failed: ",
        "confirm_baked": "Overwrite with the committed read-state baked into this build?",
        "footer": (
            "Source: docs/reference/review/papers.md + detailed_reviews/*.md · generated by build_html.py "
            "(re-run after editing to refresh). Read-state: Export → read_status.json "
            "→ commit to share; on another machine Import or rebuild."
        ),
    },
    "ko": {
        "page_title": "점수 · 읽기 트래커",
        "export": "⬇ 내보내기",
        "export_t": "현재 읽음 상태를 read_status.json으로 저장 (git에 커밋하면 다른 기기와 공유)",
        "import": "⬆ 가져오기",
        "import_t": "read_status.json 불러오기 (다른 기기에서 만든 파일)",
        "baked": "↻ 빌드값",
        "baked_t": "이 빌드에 커밋된 읽음 상태를 적용",
        "font_t": "글자 크기",
        "theme_t": "테마",
        "papers": "편",
        "read": "읽음",
        "read_flabel": "읽음",
        "unread": "안읽음",
        "read_chip": "읽음",
        "search_ph": "저자·제목·venue·노트 검색…",
        "reset": "필터 초기화",
        "shown": "편 표시",
        "empty": "조건에 맞는 논문이 없습니다.",
        "rubric_summary": "채점 기준 · 밴드 정의",
        "rubric_h_crit": "기준 (가중)",
        "rubric_h_band": "밴드",
        "imported": "가져왔습니다 · 읽음 ",
        "parse_fail": "JSON 파싱 실패: ",
        "confirm_baked": "이 빌드에 커밋된 읽음 상태로 덮어씁니다. 계속할까요?",
        "footer": (
            "출처: docs/reference/review/papers.md + detailed_reviews/*.md · build_html.py 생성 "
            "(수정 후 다시 빌드하면 갱신). 읽음 상태: 내보내기 → read_status.json "
            "공유 → 다른 기기에서 가져오기/재빌드."
        ),
    },
}
S = STRINGS.get(LANG, STRINGS["en"])

# Group badge palette (cycled for however many groups the ledger has).
PALETTE = [
    "#024ad8",
    "#0891b2",
    "#7c3aed",
    "#db2777",
    "#ea580c",
    "#16a34a",
    "#64748b",
    "#b45309",
    "#0d9488",
    "#9333ea",
]

# --- review parsing (detailed_reviews/<group>.md -> per-paper 6-section analysis) -----
_TOKEN_RE = re.compile(r"[a-zà-ÿğıöşüçə']+", re.I)
_MARK_HEAD = re.compile(r"(?m)^###\s*([1-9])\.\s*(.+?)\s*$")  # '### 3. critical review'
_MARK_BOLD = re.compile(r"\*\*\s*([1-9])\.\s*([^*\n]+?)\s*\*\*")  # '**3. critical review.**'
_MARK_VERDICT = re.compile(
    r"\*\*\s*(?:" + "|".join(re.escape(mark) for mark in VERDICT_MARKS) + r")\s*[:：]\s*\*\*"
)


def _toks(s):
    return {t.lower() for t in _TOKEN_RE.findall(s) if len(t) >= 3}


def _review_title(head):
    """Extract the paper title from a review header '(YEAR, VENUE, TITLE: AUTHORS)'.

    Year and venue are the first two comma-separated fields; authors are the final
    ': '-separated field (the title may itself contain ': ', so split on the last)."""
    h = head.strip()
    if h.startswith("("):
        h = h[1:]
    j = h.rfind(")")
    if j != -1:
        h = h[:j]
    parts = h.split(",", 2)  # YEAR, VENUE, "TITLE: AUTHORS"
    if len(parts) < 3:
        return ""
    rest = parts[2].strip()
    return rest.rsplit(": ", 1)[0].strip() if ": " in rest else rest


def _parse_sections(body):
    """Split one review body into {section-number: text} + the one-line verdict."""
    marks = [(m.start(), m.end(), m.group(1)) for m in _MARK_HEAD.finditer(body)]
    marks += [(m.start(), m.end(), m.group(1)) for m in _MARK_BOLD.finditer(body)]
    vm = _MARK_VERDICT.search(body)
    if vm:
        marks.append((vm.start(), vm.end(), "v"))
    marks.sort()
    secs, verdict = {}, ""
    for i, (_s, e, key) in enumerate(marks):
        nxt = marks[i + 1][0] if i + 1 < len(marks) else len(body)
        chunk = body[e:nxt].strip()
        chunk = re.sub(r"\n-{3,}\s*$", "", chunk).strip()  # drop trailing '---'
        if key == "v":
            verdict = chunk.splitlines()[0].strip() if chunk else ""
        elif key not in secs:
            secs[key] = chunk
    return secs, verdict


def _parse_reviews(letter):
    """Parse detailed_reviews/<letter>.md into a list of review blocks."""
    path = REVIEW_DIR / f"{letter}.md"
    if not path.exists():
        return []
    blocks, cur = [], None
    for ln in path.read_text(encoding="utf-8").split("\n"):
        if ln.startswith("## ") and not ln.startswith("###"):
            if cur:
                blocks.append(cur)
            cur = {"head": ln[3:].strip(), "lines": []}
        elif cur is not None:
            cur["lines"].append(ln)
    if cur:
        blocks.append(cur)
    out = []
    for b in blocks:
        body = "\n".join(b["lines"])
        if not _MARK_VERDICT.search(body):
            continue
        ym = re.search(r"\b(?:19|20)\d{2}\b", b["head"])
        secs, verdict = _parse_sections(body)
        out.append(
            {
                "year": ym.group(0) if ym else "",
                "tok": _toks(b["head"]),
                "title": _review_title(b["head"]),
                "secs": secs,
                "verdict": verdict,
            }
        )
    return out


def attach_reviews(groups, papers):
    """Match each ledger paper to its review block (per group, by year + token overlap)."""
    matched = 0
    for g, _, _ in groups:
        pool = _parse_reviews(g)
        for p in [p for p in papers if p["group"] == g]:
            ptok = _toks(p["authors"] + " " + p["title"] + " " + p["venue"])
            cands = [r for r in pool if r["year"] == p["year"]] or pool
            best, best_overlap = None, -1
            for r in cands:
                ov = len(ptok & r["tok"])
                if ov > best_overlap:
                    best, best_overlap = r, ov
            if best is not None and best_overlap > 0:
                pool.remove(best)
                p["secs"], p["verdict"] = best["secs"], best["verdict"]
                if best.get("title"):  # prefer the review's full canonical title
                    p["title"] = best["title"]
                matched += 1
            else:
                p["secs"], p["verdict"] = {}, ""
    return matched


def md_inline(s):
    """Escape + render inline **bold** for review prose."""
    s = html.escape(s, quote=False)
    return re.sub(r"\*\*(.+?)\*\*", r"<strong>\1</strong>", s)


def md_block(text):
    """Render a review section: '- ' bullets -> <ul>, blank lines split <p>."""
    if not text:
        return ""
    out, ul = [], []

    def flush():
        if ul:
            out.append("<ul>" + "".join(f"<li>{md_inline(x)}</li>" for x in ul) + "</ul>")
            ul.clear()

    for raw in text.split("\n"):
        line = raw.rstrip()
        if not line.strip():
            flush()
            continue
        m = re.match(r"^\s*-\s+(.*)$", line)
        if m:
            ul.append(m.group(1))
        else:
            flush()
            out.append(f"<p>{md_inline(line)}</p>")
    flush()
    return "".join(out)


def parse_ledger(text):
    """Return (groups, papers) parsed from the scores-ledger section of papers.md.

    The section is the first '## ... ledger ...' H2 (else from the first group
    header) up to the next H2 — so a trailing '## Distribution' summary is excluded."""
    lines = text.splitlines()
    grp_re = re.compile(r"^###\s+([A-Z])\.\s+(.*?)\s+\((\d+)\)\s*$")
    bold_re = re.compile(r"\*\*(.+?)\*\*\s*$")

    start = None
    for i, ln in enumerate(lines):
        if ln.startswith("## ") and "ledger" in ln.lower():
            start = i
            break
    if start is None:
        for i, ln in enumerate(lines):
            if grp_re.match(ln.strip()):
                start = i
                break
    if start is None:
        raise SystemExit(
            "No '## ... ledger' section or '### A. … (n)' group header found in papers.md"
        )
    end = len(lines)
    for j in range(start + 1, len(lines)):
        if lines[j].startswith("## "):
            end = j
            break
    section = "\n".join(lines[start:end])

    groups = []  # (letter, title, count)
    papers = []
    cur_group = None
    orig_idx = 0

    for line in section.splitlines():
        gm = grp_re.match(line.strip())
        if gm:
            cur_group = gm.group(1)
            groups.append((gm.group(1), gm.group(2), int(gm.group(3))))
            continue
        if not line.lstrip().startswith("|") or cur_group is None:
            continue
        cells = [c.strip() for c in line.strip().strip("|").split("|")]
        if not cells or cells[0] in ("Paper", ""):
            continue
        if set(cells[0]) <= set("-: "):  # separator row
            continue
        if len(cells) < 6:
            continue
        paper, scores_raw, score_raw, band, tier = cells[:5]
        note = "|".join(cells[5:]).strip()  # rejoin in case a note ever holds a pipe

        bm = bold_re.search(paper)
        venue_year = bm.group(1).strip() if bm else ""
        paper_main = (paper[: bm.start()] if bm else paper).rstrip().rstrip(",").rstrip()

        ym = re.search(r"(\d{4})\s*$", venue_year)
        year = ym.group(1) if ym else ""
        venue = venue_year[: ym.start()].strip() if ym else venue_year

        if "," in paper_main:
            authors, title = paper_main.split(",", 1)
            authors, title = authors.strip(), title.strip()
        else:
            authors, title = "", paper_main

        c_scores = [s.strip() for s in scores_raw.split("·")]
        score = int(re.sub(r"[^\d]", "", score_raw) or 0)
        pid = hashlib.md5((paper_main + "|" + venue_year).encode("utf-8")).hexdigest()[:10]

        papers.append(
            {
                "id": pid,
                "group": cur_group,
                "authors": authors,
                "title": title,
                "pdf_title": title,
                "venue": venue,
                "year": year,
                "scores": c_scores,
                "score": score,
                "band": band,
                "tier": tier,
                "note": note,
                "threat": "⚠️" in note,
                "orig": orig_idx,
            }
        )
        orig_idx += 1

    return groups, papers


def score_band_class(s):
    for name, mn, _ in BANDS:
        if s >= mn:
            return name
    return BANDS[-1][0]


def esc(s):
    return html.escape(s, quote=True)


def _sanitize_filename_part(name):
    return re.sub(r'[<>:"/\\|?*]', "", name)


def _pdf_url(path):
    rel = pathlib.Path(os.path.relpath(path, OUT.parent)).as_posix()
    return quote(rel, safe="/")


def _title_tokens(text):
    return {t for t in re.findall(r"[a-z0-9]+", text.lower()) if len(t) > 3}


def find_pdf(p):
    """Return the matching PDF path for a ledger paper, if present."""
    fname = (
        _sanitize_filename_part(f"{p['year']}, {p['venue']}, {p.get('pdf_title') or p['title']}")
        + ".pdf"
    )
    direct = REFERENCE_DIR / fname
    if direct.exists():
        return direct

    prefix = f"{p['year']}, {p['venue']}, "
    candidates = sorted(REFERENCE_DIR.glob(f"{p['year']}, {p['venue']}, *.pdf"))
    if not candidates:
        candidates = sorted(REFERENCE_DIR.glob(f"{p['year']}, *.pdf"))
    target = _title_tokens(p.get("pdf_title") or p["title"])
    threshold = max(2, min(4, len(target)))

    def best_match(pool):
        best, best_overlap = None, 0
        for cand in pool:
            cand_title = cand.stem[len(prefix) :] if cand.name.startswith(prefix) else cand.stem
            overlap = len(target & _title_tokens(cand_title))
            if overlap > best_overlap:
                best, best_overlap = cand, overlap
        return best, best_overlap

    best, best_overlap = best_match(candidates)
    if best_overlap >= threshold:
        return best
    best, best_overlap = best_match(sorted(REFERENCE_DIR.glob("*.pdf")))
    return best if best_overlap >= threshold else None


def render_pdf_link(p):
    path = find_pdf(p)
    if path is None:
        return ""
    return (
        f'<a class="pdf-link" href="{esc(_pdf_url(path))}" target="_blank" '
        'rel="noopener" title="open PDF in new tab">PDF</a>'
    )


def render_detail(p):
    head = "".join(f'<th title="{esc(full)}">{esc(short)}</th>' for short, full in SCORE_COLS)
    cells = "".join(
        f'<td class="v-{"na" if sc == "–" else esc(sc)}">{esc(sc)}</td>'
        for sc in (p["scores"][:NCRIT] + ["–"] * NCRIT)[:NCRIT]
    )
    table = (
        f'<table class="ctable"><thead><tr>{head}</tr></thead>'
        f"<tbody><tr>{cells}</tr></tbody></table>"
    )

    verdict = p.get("verdict", "")
    verdict_html = f'<div class="verdict">{md_inline(verdict)}</div>' if verdict else ""

    secs = p.get("secs", {})
    body = ""
    for k in sorted(SEC_LABEL):
        if secs.get(k):
            body += (
                f'<div class="sec"><h5>{esc(SEC_LABEL[k])}</h5>'
                f'<div class="sec-body">{md_block(secs[k])}</div></div>'
            )
    if not body and p["note"]:
        body = f'<div class="sec"><div class="sec-body"><p>{esc(p["note"])}</p></div></div>'

    return f'<div class="detail">{table}{verdict_html}{body}</div>'


def render_rows(papers, group_rank):
    out = []
    for p in papers:
        searchable = " ".join(
            [
                p["authors"],
                p["title"],
                p["venue"],
                p["year"],
                p["band"],
                p["tier"],
                p["note"],
                p.get("verdict", ""),
                "group " + p["group"],
            ]
        ).lower()
        year_bucket = "older" if (p["year"] and int(p["year"]) < 2023) else p["year"]
        threat = (
            '<span class="threat" title="direct competitor / must-cite">⚠️</span>'
            if p["threat"]
            else ""
        )
        authors_html = f'<span class="authors">{esc(p["authors"])}</span> ' if p["authors"] else ""
        pdf_link = render_pdf_link(p)

        out.append(f'''<tr class="paper-row" data-id="{p["id"]}" data-origidx="{p["orig"]}"
   data-group="{p["group"]}" data-grouprank="{group_rank.get(p["group"], 0)}"
   data-sortname="{esc((p["authors"] + " " + p["title"]).lower())}"
   data-venue="{esc(p["venue"].lower())}"
   data-year="{p["year"] or 0}" data-yearbucket="{year_bucket}"
   data-score="{p["score"]}"
   data-band="{esc(p["band"])}" data-bandrank="{BAND_RANK.get(p["band"], 0)}"
   data-tier="{esc(p["tier"])}" data-tierrank="{TIER_RANK.get(p["tier"], 0)}"
   data-search="{esc(searchable)}">
  <td class="c-read"><input type="checkbox" class="read-check" aria-label="read"></td>
  <td class="c-group"><span class="gbadge g-{p["group"]}">{p["group"]}</span></td>
  <td class="c-paper"><span class="paper-title">{authors_html}{esc(p["title"])}</span>{pdf_link}{threat}<span class="expand-hint">▸</span></td>
  <td class="c-venue">{esc(p["venue"])}</td>
  <td class="c-year">{esc(p["year"])}</td>
  <td class="c-score"><span class="score-pill band-{score_band_class(p["score"])}">{p["score"]}</span></td>
  <td class="c-band"><span class="band-badge band-{esc(p["band"])}">{esc(p["band"])}</span></td>
  <td class="c-tier"><span class="tbadge">{esc(p["tier"])}</span></td>
</tr>
<tr class="detail-row" hidden><td colspan="8">{render_detail(p)}</td></tr>''')
    return "\n".join(out)


def render_rubric():
    rows = "".join(
        f"<tr><td><b>{k}</b></td><td>{esc(label)}</td><td>{w}</td></tr>" for k, label, w in CRITERIA
    )
    band_rows = "".join(
        f'<tr><td><span class="band-badge band-{b}">{b}</span></td>'
        f"<td>{mn}+</td><td>{esc(v)}</td></tr>"
        for b, mn, v in BANDS
    )
    return f"""<details class="rubric">
  <summary>{esc(S["rubric_summary"])}</summary>
  <div class="rubric-body">
    <div>
      <h4>{esc(S["rubric_h_crit"])}</h4>
      <table class="mini"><tbody>{rows}</tbody></table>
      <p class="muted">{esc(PILLAR_NOTE)} <b>–</b> = N/A (renormalized). Score = paper quality only (/100, relative across the corpus).</p>
    </div>
    <div>
      <h4>{esc(S["rubric_h_band"])}</h4>
      <table class="mini"><tbody>{band_rows}</tbody></table>
      <p class="muted">{esc(TIER_NOTE)}</p>
    </div>
  </div>
</details>"""


def group_css(groups):
    out = []
    for i, (g, _, _) in enumerate(groups):
        out.append(f".g-{g}{{background:{PALETTE[i % len(PALETTE)]}}}")
    return "".join(out)


def load_baked_read():
    """Read committed read_status.json (the cross-machine source of truth)."""
    if not READ_STATUS.exists():
        return []
    try:
        data = json.loads(READ_STATUS.read_text(encoding="utf-8"))
    except Exception as exc:
        print(f"  (warning: could not parse read_status.json: {exc})")
        return []
    items = data.get("read", []) if isinstance(data, dict) else data
    ids = []
    for it in items:
        ids.append(it["id"] if isinstance(it, dict) and "id" in it else it)
    return [i for i in ids if isinstance(i, str)]


def build():
    text = SRC.read_text(encoding="utf-8")
    groups, papers = parse_ledger(text)
    group_rank = {g: i + 1 for i, (g, _, _) in enumerate(groups)}
    matched = attach_reviews(groups, papers)

    band_counts = {b: 0 for b in BAND_NAMES}
    for p in papers:
        band_counts[p["band"]] = band_counts.get(p["band"], 0) + 1
    total = len(papers)
    baked = load_baked_read()

    group_chips = "".join(
        f'<button class="chip" data-fkind="group" data-fval="{g}" title="{esc(GROUP_GLOSS.get(g, t))}">{g}</button>'
        for g, t, _ in groups
    )
    band_chips = "".join(
        f'<button class="chip" data-fkind="band" data-fval="{b}">{b}<span class="ct">{band_counts.get(b, 0)}</span></button>'
        for b in BAND_NAMES[:-1]  # omit the lowest band from the quick filter row
    )
    stat_chips = "".join(
        f'<span class="stat band-{b}">{b} {band_counts.get(b, 0)}</span>' for b in BAND_NAMES[:-1]
    )
    legend_items = "".join(
        f'<div class="leg"><span class="gbadge g-{g}">{g}</span>'
        f'<span class="leg-t">{esc(GROUP_GLOSS.get(g, t))}</span>'
        f'<span class="leg-n">{c}</span></div>'
        for g, t, c in groups
    )

    html_out = (
        TEMPLATE.replace("{{LANG}}", LANG)
        .replace("{{PROJECT_TITLE}}", esc(PROJECT_TITLE))
        .replace("{{PAGE_TITLE}}", esc(S["page_title"]))
        .replace("{{GROUP_CSS}}", group_css(groups))
        .replace("{{TOTAL}}", str(total))
        .replace("{{GROUP_CHIPS}}", group_chips)
        .replace("{{BAND_CHIPS}}", band_chips)
        .replace("{{STAT_CHIPS}}", stat_chips)
        .replace("{{LEGEND}}", legend_items)
        .replace("{{BAKED_READ}}", json.dumps(baked))
        .replace("{{ROWS}}", render_rows(papers, group_rank))
        .replace("{{RUBRIC}}", render_rubric())
        # UI strings
        .replace("{{S_EXPORT}}", esc(S["export"]))
        .replace("{{S_EXPORT_T}}", esc(S["export_t"]))
        .replace("{{S_IMPORT}}", esc(S["import"]))
        .replace("{{S_IMPORT_T}}", esc(S["import_t"]))
        .replace("{{S_BAKED}}", esc(S["baked"]))
        .replace("{{S_BAKED_T}}", esc(S["baked_t"]))
        .replace("{{S_FONT_T}}", esc(S["font_t"]))
        .replace("{{S_THEME_T}}", esc(S["theme_t"]))
        .replace("{{S_PAPERS}}", esc(S["papers"]))
        .replace("{{S_READ}}", esc(S["read"]))
        .replace("{{S_READ_FLABEL}}", esc(S["read_flabel"]))
        .replace("{{S_UNREAD}}", esc(S["unread"]))
        .replace("{{S_READ_CHIP}}", esc(S["read_chip"]))
        .replace("{{S_SEARCH_PH}}", esc(S["search_ph"]))
        .replace("{{S_RESET}}", esc(S["reset"]))
        .replace("{{S_SHOWN}}", esc(S["shown"]))
        .replace("{{S_EMPTY}}", esc(S["empty"]))
        .replace("{{S_IMPORTED}}", json.dumps(S["imported"]))
        .replace("{{S_PARSE_FAIL}}", json.dumps(S["parse_fail"]))
        .replace("{{S_CONFIRM_BAKED}}", json.dumps(S["confirm_baked"]))
        .replace("{{S_FOOTER}}", S["footer"])
    )

    OUT.write_text(html_out, encoding="utf-8")
    print(
        f"Wrote {OUT}  ({total} papers, {len(groups)} groups, "
        f"{matched} reviews matched, {len(baked)} baked read-marks)"
    )


# --- single-file HTML template (CSS + JS inlined) ---------------------------
TEMPLATE = r"""<!DOCTYPE html>
<html lang="{{LANG}}">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>{{PROJECT_TITLE}} — {{PAGE_TITLE}}</title>
<link rel="preconnect" href="https://cdn.jsdelivr.net">
<link rel="stylesheet" href="https://cdn.jsdelivr.net/gh/orioncactus/pretendard@v1.3.9/dist/web/static/pretendard.min.css">
<style>
:root{
  --canvas:#ffffff; --panel:#fbfbfc; --text:#1a1a1a; --muted:#6b7280;
  --accent:#024ad8; --accent-bright:#296ef9;
  --hairline:#e8e8e8; --steel:#c2c2c2; --chip-bg:#f3f4f6; --chip-bg-h:#e9eaee;
  --row-h:#f6f8fe; --read-bg:#f7f8fa;
}
[data-theme=dark]{
  --canvas:#0e0e14; --panel:#15151d; --text:#e4e4ec; --muted:#9aa0ae;
  --accent:#4d8df7; --accent-bright:#6ba3ff;
  --hairline:#252535; --steel:#353548; --chip-bg:#1b1b25; --chip-bg-h:#23232f;
  --row-h:#161a26; --read-bg:#12121a;
}
*{box-sizing:border-box}
html{scroll-behavior:smooth}
body{
  margin:0; background:var(--canvas); color:var(--text);
  font-family:"Pretendard Variable",Pretendard,-apple-system,BlinkMacSystemFont,"Apple SD Gothic Neo","Segoe UI",Roboto,sans-serif;
  font-size:15px; line-height:1.4; -webkit-font-smoothing:antialiased;
}
a{color:var(--accent);text-decoration:none}
.wrap{max-width:1180px;margin:0 auto;padding:0 22px}

/* utility strip */
.util{display:flex;justify-content:flex-start;align-items:center;gap:8px;padding:10px 0;border-bottom:1px solid var(--hairline)}
.iconbtn{border:1px solid var(--hairline);background:var(--chip-bg);color:var(--text);border-radius:6px;
  padding:5px 11px;font-size:13px;cursor:pointer;transition:all .15s}
.iconbtn:hover{border-color:var(--steel);background:var(--chip-bg-h)}

/* hero */
.hero{padding:34px 0 18px}
.hero h1{font-size:46px;font-weight:600;margin:0 0 6px;letter-spacing:-.5px;line-height:1.05}
.hero .sub{color:var(--muted);font-size:15px;margin:0 0 18px;max-width:780px}
.stats{display:flex;flex-wrap:wrap;gap:8px;align-items:center}
.stat{font-size:12px;padding:3px 10px;border-radius:20px;font-weight:600;letter-spacing:.2px}
.stat.big{background:var(--accent);color:#fff;font-size:13px}
.stat.big b{font-weight:700}

/* filter band */
.filterbar{position:sticky;top:0;z-index:20;background:var(--canvas);
  border-bottom:1px solid var(--hairline);padding:12px 0;margin-bottom:0}
.filterbar.shadow{box-shadow:0 4px 14px rgba(0,0,0,.06)}
.frow{display:flex;flex-wrap:wrap;gap:7px 16px;align-items:center;margin-bottom:9px}
.frow:last-child{margin-bottom:0}
.flabel{font-size:11px;text-transform:uppercase;letter-spacing:.6px;color:var(--muted);
  font-weight:700;min-width:42px}
.chip{font-size:12px;text-transform:uppercase;letter-spacing:.4px;font-weight:600;
  border:1px solid var(--hairline);background:var(--chip-bg);color:var(--text);
  border-radius:5px;padding:5px 11px;cursor:pointer;transition:all .15s;display:inline-flex;align-items:center;gap:5px}
.chip:hover{border-color:var(--steel);background:var(--chip-bg-h)}
.chip.active{background:var(--accent);border-color:var(--accent);color:#fff}
.chip .ct{font-size:10px;opacity:.7;font-weight:700}
.chip.active .ct{opacity:.85}
.search{flex:1;min-width:200px;display:flex;align-items:center;gap:8px;
  border:1px solid var(--hairline);background:var(--chip-bg);border-radius:7px;padding:7px 12px}
.search:focus-within{border-color:var(--accent)}
.search input{border:0;background:transparent;color:var(--text);font-size:14px;width:100%;outline:none;
  font-family:inherit}
.search svg{flex:none;opacity:.5}
.reset{font-size:12px;color:var(--muted);background:none;border:0;cursor:pointer;text-decoration:underline;padding:4px}
.reset:hover{color:var(--accent)}
.resultline{font-size:12px;color:var(--muted);padding:9px 0 2px}
.resultline b{color:var(--text)}

/* table */
.tablewrap{overflow-x:auto}
table.survey{width:100%;border-collapse:collapse;font-size:14px}
.survey thead th{
  position:sticky;top:0;text-align:left;font-size:11px;text-transform:uppercase;letter-spacing:.5px;
  color:var(--muted);font-weight:700;padding:11px 10px;border-bottom:1.5px solid var(--steel);
  background:var(--canvas);white-space:nowrap;user-select:none;z-index:5}
.survey th.sortable{cursor:pointer}
.survey th.sortable:hover{color:var(--accent)}
.survey th .arrow{opacity:.35;font-size:10px;margin-left:3px}
.survey th.sort-asc .arrow,.survey th.sort-desc .arrow{opacity:1;color:var(--accent)}
.survey tbody td{padding:9px 10px;border-bottom:1px solid var(--hairline);vertical-align:top}
.paper-row{cursor:pointer;transition:background .12s}
.paper-row:hover{background:var(--row-h)}
.paper-row.is-read{background:var(--read-bg);color:var(--muted)}
.paper-row.is-read .paper-title{color:var(--muted)}
.paper-row.open .expand-hint{transform:rotate(90deg)}
.c-read{width:34px;text-align:center}
.read-check{width:16px;height:16px;cursor:pointer;accent-color:var(--accent)}
.c-group{width:38px}
.c-year{width:54px;color:var(--muted);font-variant-numeric:tabular-nums}
.c-venue{color:var(--muted);white-space:nowrap}
.c-score{width:62px}
.c-tier{width:54px}
.paper-title{font-weight:500}
.authors{color:var(--muted);font-weight:400}
.expand-hint{display:inline-block;margin-left:7px;color:var(--steel);font-size:11px;transition:transform .15s}
.threat{margin-left:6px;font-size:12px}
.pdf-link{display:inline-block;margin-left:8px;font-size:10px;font-weight:700;letter-spacing:.4px;
  text-transform:uppercase;color:var(--accent);border:1px solid var(--hairline);border-radius:4px;
  padding:0 5px;vertical-align:1px}
.pdf-link:hover{border-color:var(--accent);background:var(--chip-bg)}

/* badges */
.gbadge{display:inline-flex;align-items:center;justify-content:center;width:22px;height:22px;border-radius:5px;
  font-size:12px;font-weight:700;color:#fff}
{{GROUP_CSS}}
.score-pill{display:inline-block;min-width:34px;text-align:center;font-weight:700;font-size:13px;
  padding:3px 8px;border-radius:7px;font-variant-numeric:tabular-nums}
.band-badge{display:inline-block;font-size:11px;font-weight:700;padding:2px 9px;border-radius:8px;text-transform:capitalize}
.tbadge{font-size:12px;color:var(--muted);font-weight:600}

.band-flagship{color:#15803d;background:#dcfce7}
.band-strong{color:#1d4ed8;background:#dbeafe}
.band-good{color:#0e7490;background:#cffafe}
.band-fair{color:#b45309;background:#fef3c7}
.band-weak{color:#b91c1c;background:#fee2e2}
.band-poor{color:#7f1d1d;background:#fecaca}
[data-theme=dark] .band-flagship{color:#5be584;background:#13321f}
[data-theme=dark] .band-strong{color:#7aacff;background:#16223f}
[data-theme=dark] .band-good{color:#4fd6e6;background:#0d2b30}
[data-theme=dark] .band-fair{color:#f3b85a;background:#2f2410}
[data-theme=dark] .band-weak{color:#f87171;background:#33161a}
[data-theme=dark] .band-poor{color:#fca5a5;background:#3a1414}

/* legend (group descriptions) */
.legend{display:grid;grid-template-columns:repeat(2,1fr);gap:6px 26px;padding:16px 0 4px}
@media(max-width:760px){.legend{grid-template-columns:1fr}}
.leg{display:flex;align-items:center;gap:9px;font-size:13px;color:var(--text)}
.leg .leg-t{flex:1}
.leg .leg-n{color:var(--muted);font-size:11.5px;font-variant-numeric:tabular-nums}

/* detail */
.detail-row td{background:var(--panel);border-bottom:1px solid var(--hairline)}
.detail{padding:10px 4px 16px 44px;max-width:980px}
.ctable{border-collapse:collapse;margin:2px 0 12px;font-size:12.5px}
.ctable th{font-weight:600;color:var(--muted);padding:4px 10px;border:1px solid var(--hairline);
  text-align:center;white-space:nowrap;background:var(--canvas)}
.ctable td{font-weight:700;text-align:center;padding:5px 10px;border:1px solid var(--hairline);
  font-variant-numeric:tabular-nums;font-size:14px}
.v-5{color:#15803d}.v-4{color:#1d4ed8}.v-3{color:#b45309}.v-2{color:#b91c1c}.v-1{color:#7f1d1d}
.v-na{color:var(--steel)}
[data-theme=dark] .v-5{color:#5be584}[data-theme=dark] .v-4{color:#7aacff}
[data-theme=dark] .v-3{color:#f3b85a}[data-theme=dark] .v-2{color:#f87171}[data-theme=dark] .v-1{color:#fca5a5}
.verdict{font-size:13.5px;line-height:1.5;color:var(--text);font-style:italic;
  border-left:3px solid var(--accent);padding:2px 0 2px 12px;margin:0 0 14px}
.verdict strong{font-style:normal}
.sec{margin:0 0 13px}
.sec h5{margin:0 0 5px;font-size:12px;font-weight:700;text-transform:uppercase;letter-spacing:.5px;color:var(--accent)}
.sec-body{font-size:13.5px;line-height:1.62;color:var(--text)}
.sec-body p{margin:0 0 6px}
.sec-body ul{margin:0 0 6px;padding-left:18px}
.sec-body li{margin:0 0 3px}
.sec-body strong{font-weight:700}

/* rubric + footer */
.rubric{margin:26px 0;border:1px solid var(--hairline);border-radius:10px;background:var(--panel)}
.rubric summary{cursor:pointer;padding:13px 16px;font-weight:600;font-size:14px;list-style:none}
.rubric summary::-webkit-details-marker{display:none}
.rubric summary::before{content:"▸ ";color:var(--accent)}
.rubric[open] summary::before{content:"▾ "}
.rubric-body{display:grid;grid-template-columns:1fr 1fr;gap:24px;padding:4px 18px 18px}
@media(max-width:760px){.rubric-body{grid-template-columns:1fr}}
.rubric h4{margin:8px 0;font-size:13px}
table.mini{width:100%;border-collapse:collapse;font-size:12.5px}
table.mini td{padding:4px 8px;border-bottom:1px solid var(--hairline)}
.muted{color:var(--muted);font-size:12px;line-height:1.5;margin:8px 0 0}
footer{border-top:1px solid var(--hairline);margin-top:30px;padding:20px 0 40px;color:var(--muted);font-size:12.5px}
.empty{text-align:center;padding:46px;color:var(--muted)}
</style>
</head>
<body>
<div class="wrap">

  <div class="util">
    <button class="iconbtn" id="export-btn" title="{{S_EXPORT_T}}">{{S_EXPORT}}</button>
    <label class="iconbtn" title="{{S_IMPORT_T}}">{{S_IMPORT}}<input type="file" id="import-input" accept=".json,application/json" hidden></label>
    <button class="iconbtn" id="apply-baked" title="{{S_BAKED_T}}" hidden>{{S_BAKED}}</button>
    <span style="flex:1"></span>
    <button class="iconbtn" id="font-toggle" title="{{S_FONT_T}}">A+</button>
    <button class="iconbtn" id="theme-toggle" title="{{S_THEME_T}}">🌙 / ☀︎</button>
  </div>

  <header class="hero">
    <h1>{{PROJECT_TITLE}}</h1>
    <div class="stats">
      <span class="stat big"><b>{{TOTAL}}</b> {{S_PAPERS}}</span>
      <span class="stat big">{{S_READ}} <b id="read-count">0</b></span>
      {{STAT_CHIPS}}
    </div>
  </header>

  <section class="legend">{{LEGEND}}</section>

  <div class="filterbar" id="filterbar">
    <div class="frow">
      <span class="flabel">Group</span>{{GROUP_CHIPS}}
    </div>
    <div class="frow">
      <span class="flabel">Band</span>{{BAND_CHIPS}}
    </div>
    <div class="frow">
      <span class="flabel">{{S_READ_FLABEL}}</span>
      <button class="chip" data-fkind="read" data-fval="unread">{{S_UNREAD}}</button>
      <button class="chip" data-fkind="read" data-fval="read">{{S_READ_CHIP}}</button>
    </div>
    <div class="frow">
      <div class="search">
        <svg width="15" height="15" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><circle cx="11" cy="11" r="7"/><path d="M21 21l-4.3-4.3"/></svg>
        <input type="text" id="search" placeholder="{{S_SEARCH_PH}}">
      </div>
      <button class="reset" id="reset">{{S_RESET}}</button>
    </div>
  </div>

  <div class="resultline"><b id="result-count">{{TOTAL}}</b> / {{TOTAL}} {{S_SHOWN}}</div>

  <div class="tablewrap">
  <table class="survey">
    <thead><tr>
      <th class="c-read" title="{{S_READ_FLABEL}}">✓</th>
      <th class="sortable c-group" data-key="grouprank" data-type="num">Grp<span class="arrow">↕</span></th>
      <th class="sortable" data-key="sortname" data-type="str">Paper<span class="arrow">↕</span></th>
      <th class="sortable c-venue" data-key="venue" data-type="str">Venue<span class="arrow">↕</span></th>
      <th class="sortable c-year" data-key="year" data-type="num">Year<span class="arrow">↕</span></th>
      <th class="sortable c-score" data-key="score" data-type="num">/100<span class="arrow">↕</span></th>
      <th class="sortable c-band" data-key="bandrank" data-type="num">Band<span class="arrow">↕</span></th>
      <th class="sortable c-tier" data-key="tierrank" data-type="num">Tier<span class="arrow">↕</span></th>
    </tr></thead>
    <tbody id="tbody">
{{ROWS}}
    </tbody>
  </table>
  </div>
  <div class="empty" id="empty" hidden>{{S_EMPTY}}</div>

  {{RUBRIC}}

  <footer>{{S_FOOTER}}</footer>
</div>

<script>
(function(){
  "use strict";
  var KEY_THEME="lit_theme", KEY_FONT="lit_font", KEY_READ="lit_read_v1";

  // ---- theme ----
  var root=document.documentElement;
  function setTheme(t){root.setAttribute("data-theme",t);localStorage.setItem(KEY_THEME,t);}
  setTheme(localStorage.getItem(KEY_THEME) || (matchMedia("(prefers-color-scheme: dark)").matches?"dark":"light"));
  document.getElementById("theme-toggle").onclick=function(){
    setTheme(root.getAttribute("data-theme")==="dark"?"light":"dark");
  };
  // ---- font size ----
  function setFont(f){ if(f==="large"){document.body.style.fontSize="17px";} else {document.body.style.fontSize="";} localStorage.setItem(KEY_FONT,f);}
  setFont(localStorage.getItem(KEY_FONT)||"");
  document.getElementById("font-toggle").onclick=function(){
    setFont(localStorage.getItem(KEY_FONT)==="large"?"":"large");
  };

  // ---- read state (persisted; baked default shared via git) ----
  var BAKED_READ = {{BAKED_READ}};   // ids committed in read_status.json
  var readState={};
  function saveRead(){localStorage.setItem(KEY_READ,JSON.stringify(readState));}
  if(localStorage.getItem(KEY_READ)===null){
    BAKED_READ.forEach(function(id){readState[id]=1;});
    if(BAKED_READ.length) saveRead();
  }else{
    try{readState=JSON.parse(localStorage.getItem(KEY_READ)||"{}");}catch(e){readState={};}
  }
  var rows=[].slice.call(document.querySelectorAll("tr.paper-row"));

  rows.forEach(function(r){
    var cb=r.querySelector(".read-check"), id=r.dataset.id;
    cb.checked=!!readState[id];
    r.classList.toggle("is-read",cb.checked);
    cb.addEventListener("click",function(e){e.stopPropagation();});
    cb.addEventListener("change",function(){
      if(cb.checked) readState[id]=1; else delete readState[id];
      saveRead(); r.classList.toggle("is-read",cb.checked); updateStats();
    });
    r.addEventListener("click",function(e){
      if(e.target.classList.contains("read-check")) return;
      var d=r.nextElementSibling;
      if(d && d.classList.contains("detail-row")){d.hidden=!d.hidden;r.classList.toggle("open",!d.hidden);}
    });
  });

  // ---- filters ----
  var filters={group:new Set(),band:new Set(),read:null,q:""};
  var chips=[].slice.call(document.querySelectorAll(".chip"));
  chips.forEach(function(ch){
    ch.addEventListener("click",function(){
      var k=ch.dataset.fkind, v=ch.dataset.fval;
      if(k==="read"){
        if(filters.read===v){filters.read=null;ch.classList.remove("active");}
        else{
          filters.read=v;
          chips.forEach(function(c){if(c.dataset.fkind==="read")c.classList.toggle("active",c.dataset.fval===v);});
        }
      }else{
        if(filters[k].has(v)){filters[k].delete(v);ch.classList.remove("active");}
        else{filters[k].add(v);ch.classList.add("active");}
      }
      applyFilters();
    });
  });
  var searchEl=document.getElementById("search");
  searchEl.addEventListener("input",function(){filters.q=searchEl.value.trim().toLowerCase();applyFilters();});
  document.getElementById("reset").onclick=function(){
    filters={group:new Set(),band:new Set(),read:null,q:""};
    chips.forEach(function(c){c.classList.remove("active");});
    searchEl.value=""; applyFilters();
  };

  function matchRow(r){
    if(filters.group.size && !filters.group.has(r.dataset.group)) return false;
    if(filters.band.size && !filters.band.has(r.dataset.band)) return false;
    if(filters.read){
      var isRead=!!readState[r.dataset.id];
      if(filters.read==="read" && !isRead) return false;
      if(filters.read==="unread" && isRead) return false;
    }
    if(filters.q && r.dataset.search.indexOf(filters.q)===-1) return false;
    return true;
  }
  function applyFilters(){
    var shown=0;
    rows.forEach(function(r){
      var ok=matchRow(r);
      r.hidden=!ok;
      var d=r.nextElementSibling;
      if(d && d.classList.contains("detail-row")){ if(!ok){d.hidden=true;r.classList.remove("open");} }
      if(ok) shown++;
    });
    document.getElementById("result-count").textContent=shown;
    document.getElementById("empty").hidden=shown!==0;
    updateStats();
  }
  function updateStats(){
    var n=0; rows.forEach(function(r){if(readState[r.dataset.id])n++;});
    document.getElementById("read-count").textContent=n;
  }

  // ---- cross-machine sync (read_status.json) ----
  function refreshAll(){
    rows.forEach(function(r){
      var cb=r.querySelector(".read-check");
      cb.checked=!!readState[r.dataset.id];
      r.classList.toggle("is-read",cb.checked);
    });
    applyFilters();
  }
  document.getElementById("export-btn").onclick=function(){
    var arr=[];
    rows.forEach(function(r){
      if(readState[r.dataset.id]) arr.push({
        id:r.dataset.id,
        paper:(r.querySelector(".paper-title").textContent||"").trim(),
        venue:(r.querySelector(".c-venue").textContent||"").trim(),
        year:r.dataset.year
      });
    });
    arr.sort(function(a,b){return a.paper.localeCompare(b.paper);});
    var blob=new Blob([JSON.stringify({version:1,read:arr},null,2)],{type:"application/json"});
    var a=document.createElement("a");
    a.href=URL.createObjectURL(blob); a.download="read_status.json"; a.click();
    setTimeout(function(){URL.revokeObjectURL(a.href);},1000);
  };
  document.getElementById("import-input").onchange=function(e){
    var f=e.target.files[0]; if(!f) return;
    var rd=new FileReader();
    rd.onload=function(){
      try{
        var d=JSON.parse(rd.result);
        var items=(d&&d.read)?d.read:(Array.isArray(d)?d:[]);
        readState={};
        items.forEach(function(it){var id=(typeof it==="string")?it:it.id; if(id)readState[id]=1;});
        saveRead(); refreshAll();
        alert({{S_IMPORTED}}+Object.keys(readState).length);
      }catch(err){alert({{S_PARSE_FAIL}}+err);}
    };
    rd.readAsText(f); e.target.value="";
  };
  var applyBakedBtn=document.getElementById("apply-baked");
  if(BAKED_READ.length){
    applyBakedBtn.hidden=false;
    applyBakedBtn.onclick=function(){
      if(!confirm({{S_CONFIRM_BAKED}})) return;
      readState={}; BAKED_READ.forEach(function(id){readState[id]=1;});
      saveRead(); refreshAll();
    };
  }

  // ---- sorting (3-state: asc -> desc -> original) ----
  var ths=[].slice.call(document.querySelectorAll("th.sortable"));
  var sort={key:null,dir:0};
  var tbody=document.getElementById("tbody");
  ths.forEach(function(th){
    th.addEventListener("click",function(){
      var key=th.dataset.key;
      if(sort.key!==key){sort.key=key;sort.dir=1;}
      else{sort.dir = sort.dir===1?-1:(sort.dir===-1?0:1);}
      ths.forEach(function(t){t.classList.remove("sort-asc","sort-desc");});
      if(sort.dir===1) th.classList.add("sort-asc");
      else if(sort.dir===-1) th.classList.add("sort-desc");
      sortRows(th.dataset.type);
    });
  });
  function sortRows(type){
    var pairs=rows.map(function(r){return {r:r,d:r.nextElementSibling};});
    if(sort.dir===0){
      pairs.sort(function(a,b){return (+a.r.dataset.origidx)-(+b.r.dataset.origidx);});
    }else{
      var k=sort.key, dir=sort.dir;
      pairs.sort(function(a,b){
        var va=a.r.dataset[k], vb=b.r.dataset[k], c;
        if(type==="num"){c=parseFloat(va)-parseFloat(vb);}
        else{c=String(va).localeCompare(String(vb));}
        if(c===0) c=(+a.r.dataset.origidx)-(+b.r.dataset.origidx);
        return c*dir;
      });
    }
    pairs.forEach(function(p){
      tbody.appendChild(p.r);
      if(p.d && p.d.classList.contains("detail-row")) tbody.appendChild(p.d);
    });
  }

  // ---- sticky filterbar shadow ----
  var fb=document.getElementById("filterbar");
  window.addEventListener("scroll",function(){
    fb.classList.toggle("shadow", fb.getBoundingClientRect().top<=0 && window.scrollY>40);
  });

  updateStats();
})();
</script>
</body>
</html>
"""

if __name__ == "__main__":
    build()
