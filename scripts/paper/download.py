"""Download paper PDFs into docs/reference/ as "{Year}, {Venue}, {Title}.pdf".

Driven entirely by the papers you pass in — there is NO built-in paper list, so
the script is reusable across surveys. Stage 1 (1_survey.md) calls this once the
survey table is built.

Sources (the SOURCE field of each spec):
  arxiv     arXiv id (e.g. 2401.01234)             -> open access, always works
  url       direct PDF url (publisher content url,  -> works on a university IP for
            author site, MLR proceedings, ...)         most non-Elsevier publishers
  ieee_doi  IEEE DOI (10.1109/...)                  -> via Playwright + Chrome
  elsevier  ScienceDirect / Elsevier url or DOI     -> NOT scriptable; listed for
                                                        manual browser download

Institutional access: the agent's WebFetch routes through Anthropic's servers and
does NOT use your IP, so it cannot reach paywalled PDFs. This script runs on your
local network, so on a university IP direct-PDF urls for paywalled non-Elsevier
journals (Springer, Wiley, IEEE, ...) succeed. Elsevier (ScienceDirect) blocks
scripted access even on a university IP and must be downloaded manually.

Usage:
  # one or more papers on the command line (repeatable):
  #   --paper YEAR VENUE TITLE SOURCE URL_OR_ID
  python download.py \
      --paper 2021 "IEEE RA-L" "Data-Driven MPC for Quadrotors" arxiv 2102.13093 \
      --paper 2013 "Auton. Robots" "OctoMap" url http://example.org/octomap.pdf

  # from a JSON file: list of [year, venue, title, source, url_or_id]
  python download.py --from-json papers_to_get.json

Flags:
  --force        re-download even if the target already exists (overwrite)
  --no-validate  skip the page 1-2 title-match content check

CONTENT VALIDATION: after each download the first two pages are scanned and must
contain >= 50% of the title's words, else the file is kept as "<name>.pdf.suspect"
and the existing target is left untouched. Catches wrong-id / mismatched downloads.
"""

import argparse
import json
import os
import re
import sys
import time
import urllib.request
from pathlib import Path

REFERENCE_DIR = Path(__file__).resolve().parents[2] / "docs" / "reference"

UA = (
    "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 "
    "(KHTML, like Gecko) Chrome/137.0.0.0 Safari/537.36"
)


def sanitize(name: str) -> str:
    return re.sub(r'[<>:"/\\|?*]', "", name)


def is_elsevier(url_or_id: str) -> bool:
    s = (url_or_id or "").lower()
    return (
        "sciencedirect.com" in s or "elsevier" in s or s.startswith("10.1016/") or "/10.1016/" in s
    )


def _is_pdf(data: bytes) -> bool:
    return len(data) > 10000 and data[:4] == b"%PDF"


def download_url(url: str, save_path: Path) -> bool:
    """Direct PDF fetch (arXiv, publisher content urls, author sites, MLR proc.)."""
    req = urllib.request.Request(url, headers={"User-Agent": UA})
    try:
        with urllib.request.urlopen(req, timeout=45) as resp:
            data = resp.read()
        if _is_pdf(data):
            save_path.write_bytes(data)
            return True
        print(f"    Not a PDF ({len(data)} bytes returned)")
    except Exception as e:
        print(f"    Error: {e}")
    return False


def download_arxiv(arxiv_id: str, save_path: Path) -> bool:
    return download_url(f"https://arxiv.org/pdf/{arxiv_id}", save_path)


def download_ieee(doi: str, save_path: Path, holder: dict) -> bool:
    """IEEE Xplore via Playwright (lazy import so the script runs without it)."""
    try:
        from playwright.sync_api import sync_playwright
    except Exception:
        print("    Skipping IEEE: playwright not installed (python -m pip install playwright)")
        return False
    try:
        if holder.get("browser") is None:
            holder["pw"] = sync_playwright().start()
            # e.g. http://localhost:9222 drives an existing logged-in Chrome.
            cdp = os.environ.get("IEEE_CDP")
            if cdp:
                holder["browser"] = holder["pw"].chromium.connect_over_cdp(cdp)
                # Reuse the signed-in context: cookies, IP, and fingerprint.
                holder["ctx"] = holder["browser"].contexts[0]
                holder["cdp"] = True
            else:
                holder["browser"] = holder["pw"].chromium.launch(
                    headless=True,
                    channel="chrome",
                    args=["--disable-blink-features=AutomationControlled"],
                )
                holder["ctx"] = holder["browser"].new_context(user_agent=UA)
        ctx = holder["ctx"]
        page = ctx.new_page()
        page.goto(f"https://doi.org/{doi}", wait_until="domcontentloaded", timeout=30000)
        time.sleep(3)
        # arnumber lives in the resolved URL; xplGlobal JS is often bot-gated.
        landed = page.url
        page.close()
        m = re.search(r"/document/(\d+)", landed)
        if not m:
            print(f"    no arnumber in resolved URL: {landed}")
            return False
        arnumber = m.group(1)
        pdf_url = f"https://ieeexplore.ieee.org/stampPDF/getPDF.jsp?tp=&arnumber={arnumber}&ref="
        resp = ctx.request.get(pdf_url)
        body = resp.body()
        if resp.status == 200 and _is_pdf(body):
            save_path.write_bytes(body)
            return True
    except Exception as e:
        print(f"    Error: {e}")
    return False


def validate_content(path: Path, title: str):
    """Check page 1-2 text contains >=50% of the title's words. (year/venue ignored.)"""
    try:
        import fitz
    except Exception:
        return True, "(fitz unavailable; validation skipped)"
    try:
        d = fitz.open(path)
        txt = "".join((d[i].get_text() or "") for i in range(min(2, d.page_count)))
        d.close()
        low = txt.lower()
        tokens = {t for t in re.findall(r"[a-z0-9]+", title.lower()) if len(t) > 3}
        if not tokens:
            return True, "(no usable title tokens)"
        hits = sum(1 for t in tokens if t in low)
        frac = hits / len(tokens)
        snippet = " ".join(txt.split())[:140]
        return frac >= 0.5, f"title match {hits}/{len(tokens)} ({frac:.0%}) | P1: {snippet}"
    except Exception as e:
        return True, f"(validation error: {e})"


def download_one(spec, holder, force=False, validate=True):
    year, venue, title, source, url_or_id = spec
    fname = sanitize(f"{year}, {venue}, {title}") + ".pdf"
    save_path = REFERENCE_DIR / fname

    if source == "elsevier" or (source == "url" and is_elsevier(url_or_id)):
        print(f"[MANUAL] {fname}  (Elsevier — download in a browser)\n    {url_or_id}")
        return "manual", {"file": fname, "url": url_or_id}

    if save_path.exists() and save_path.stat().st_size > 10000 and not force:
        print(f"[SKIP] {fname} (exists; use --force to overwrite)")
        return "skip", fname

    print(f"[DL] {fname}")
    tmp = save_path.with_name(save_path.name + ".tmp")

    if source == "arxiv":
        ok = download_arxiv(url_or_id, tmp)
    elif source in ("url", "springer"):
        ok = download_url(url_or_id, tmp)
    elif source == "ieee_doi":
        if "XXXX" in url_or_id:
            print("    Skipping: placeholder DOI")
            return "fail", fname
        ok = download_ieee(url_or_id, tmp, holder)
    else:
        print(f"    Unknown source: {source}")
        return "fail", fname

    if not ok:
        tmp.unlink(missing_ok=True)
        print("    FAILED")
        return "fail", fname

    matches, info = (True, "(validation off)") if not validate else validate_content(tmp, title)
    print(f"    {info}")
    if matches:
        tmp.replace(save_path)
        print(f"    OK ({save_path.stat().st_size // 1024} KB)")
        return "ok", fname

    suspect = save_path.with_name(save_path.name + ".suspect")
    tmp.replace(suspect)
    print(f"    SUSPECT content -> kept as {suspect.name}; existing target left untouched")
    return "suspect", fname


def parse_args():
    ap = argparse.ArgumentParser(description="Download paper PDFs into docs/reference/")
    ap.add_argument(
        "--paper",
        nargs=5,
        action="append",
        metavar=("YEAR", "VENUE", "TITLE", "SOURCE", "URL_OR_ID"),
        help="One paper; SOURCE in {arxiv,url,ieee_doi,elsevier}. Repeatable.",
    )
    ap.add_argument(
        "--from-json",
        help="JSON file: list of [year, venue, title, source, url_or_id]",
    )
    ap.add_argument("--force", action="store_true", help="overwrite existing files")
    ap.add_argument("--no-validate", action="store_true", help="skip title-match content check")
    return ap.parse_args()


def main():
    # Avoid cp949 crashes on non-ASCII prints (em-dash, titles).
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")  # type: ignore[attr-defined]
    REFERENCE_DIR.mkdir(parents=True, exist_ok=True)
    args = parse_args()

    specs = []
    if args.paper:
        specs.extend(args.paper)
    if args.from_json:
        specs.extend(json.loads(Path(args.from_json).read_text(encoding="utf-8")))
    if not specs:
        raise SystemExit(
            "No papers given. Pass --paper YEAR VENUE TITLE SOURCE URL_OR_ID (repeatable) "
            "or --from-json FILE. See the module docstring for examples."
        )

    results = {"ok": [], "fail": [], "suspect": [], "skip": [], "manual": []}
    holder = {}

    print("=" * 60)
    print(f"Downloading {len(specs)} paper(s) into {REFERENCE_DIR}")
    print("=" * 60)
    for spec in specs:
        status, payload = download_one(
            tuple(spec), holder, force=args.force, validate=not args.no_validate
        )
        results[status].append(payload)

    # Never close a CDP-attached Chrome; it belongs to the user.
    if holder.get("browser") and not holder.get("cdp"):
        try:
            holder["browser"].close()
        except Exception:
            pass
    if holder.get("pw"):
        try:
            holder["pw"].stop()
        except Exception:
            pass

    print("\n" + "=" * 60)
    print(
        f"OK: {len(results['ok'])} | suspect: {len(results['suspect'])} | "
        f"fail: {len(results['fail'])} | skip: {len(results['skip'])} | "
        f"manual (Elsevier): {len(results['manual'])}"
    )
    for k in ("suspect", "fail"):
        for f in results[k]:
            print(f"  [{k}] {f}")
    if results["manual"]:
        print(
            "\nElsevier — download these manually in a browser (use your university IP),\n"
            "save into docs/reference/ with the shown filename, then run rename_by_title.py:"
        )
        for m in results["manual"]:
            print(f"  {m['file']}\n    {m['url']}")

    (REFERENCE_DIR / "_download_log.json").write_text(
        json.dumps(results, indent=2, ensure_ascii=False), encoding="utf-8"
    )


if __name__ == "__main__":
    main()
