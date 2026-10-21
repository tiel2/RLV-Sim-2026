"""Rename PDFs in docs/reference/ to "{Year}, {Venue}, {Title}.pdf" using the title read
from page 1.

Year and venue are kept from the existing "{Year}, {Venue}, ..." filename prefix
(so a manually-downloaded file should at least start that way), and the title part
is (re)derived from the PDF text. Use this to normalize names of papers you
downloaded by hand (e.g. Elsevier).

Usage:
  python rename_by_title.py [reference_dir] [--dry-run]
    --dry-run   print proposed "old -> new" renames (and the extracted title)
                without touching anything — run this first to sanity-check.

Defaults reference_dir to docs/reference. Requires PyMuPDF (python -m pip install pymupdf).
"""

import re
import sys
from pathlib import Path

import fitz

# Generic header noise to skip when guessing the title from page 1.
GENERIC_SKIP = [
    "arxiv:",
    "contents lists",
    "sciencedirect",
    "www.elsevier",
    "journal homepage",
    "proceedings of",
    "published as",
    "accepted",
    "preprint",
    "copyright",
    "ieee trans",
    "letters",
    "under review",
    "available online",
    "received:",
    "revised:",
    "research article",
    "original article",
    "doi:",
    "http",
    "manuscript",
    "the author(s)",
    "vol.",
    "no.",
]


def extract_title(path: Path) -> str:
    doc = fitz.open(path)
    text = doc[0].get_text()
    doc.close()
    lines = [l.strip() for l in text.split("\n") if l.strip()]
    title_lines = []
    for l in lines[:20]:
        low = l.lower()
        if any(w in low for w in GENERIC_SKIP):
            continue
        if len(l) < 5 or l.startswith("arXiv:"):
            continue
        if re.match(r"^(AAS\s+\d|\d{4}\s+IEEE)", l):
            continue
        title_lines.append(l)
        if len(" ".join(title_lines)) > 100:
            break
    title = " ".join(title_lines[:3]).strip()
    for c in ["*", "∗", "✩", "†", "‡", "⋆", "☆"]:
        title = title.rstrip(c).strip()
    # Heuristic: truncate at a likely author first name after a long-enough title.
    parts = title.split()
    for i in range(min(8, len(parts)), len(parts)):
        if re.match(r"^[A-Z][a-z]+$", parts[i]) and i > 6:
            title = " ".join(parts[:i]).strip()
            break
    return title


def sanitize(s: str) -> str:
    s = re.sub(r'[<>:"/\\|?*]', "", s)
    s = s.replace("–", "-").replace("—", "-")
    return s.strip().rstrip(".")


def main():
    argv = sys.argv[1:]
    dry = "--dry-run" in argv
    argv = [a for a in argv if a != "--dry-run"]
    script_dir = Path(__file__).resolve().parent
    ref = Path(argv[0]).resolve() if argv else script_dir.parents[1] / "docs" / "reference"

    n = 0
    for path in sorted(ref.glob("*.pdf")):
        fname = path.name
        try:
            title = extract_title(path)
        except Exception as e:
            print(f"ERROR extracting: {fname}: {e}")
            continue

        parts = fname[:-4].split(", ", 2)
        if len(parts) < 3:
            print(f"SKIP (need 'Year, Venue, Title' prefix): {fname}  | title='{title}'")
            continue
        year, venue = parts[0], parts[1]

        new_title = sanitize(title)
        if len(new_title) < 10:
            print(f"SKIP (title too short): {fname} -> '{new_title}'")
            continue

        new_fname = f"{year}, {venue}, {new_title}.pdf"
        if new_fname == fname:
            continue
        new_path = ref / new_fname
        if new_path.exists() and new_path != path:
            print(f"SKIP (collision): {new_fname}")
            continue

        print(f"  {fname[:55]:55s} -> {new_fname[:55]}")
        if not dry:
            path.rename(new_path)
        n += 1

    print(f"\n{'Would rename' if dry else 'Renamed'} {n} files")


if __name__ == "__main__":
    main()
