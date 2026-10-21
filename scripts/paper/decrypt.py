"""Decrypt encrypted PDFs in docs/reference/ into docs/reference/_decrypted/.

Some publisher / manually-downloaded PDFs are encrypted (usually an empty user
password), which blocks text extraction by the other scripts. This writes a
decrypted copy of each encrypted file into a _decrypted/ subfolder and leaves the
originals untouched.

Usage:  python decrypt.py [reference_dir]      (defaults to docs/reference)
Backends tried in order: pikepdf, pypdf, PyPDF2 (install any one with uv pip).
"""

import sys
from pathlib import Path

# Reference dir resolves relative to this script, so it runs from any cwd.
# Optional override: `python decrypt.py <reference_dir>`.
script_dir = Path(__file__).resolve().parent
ref = (
    Path(sys.argv[1]).resolve()
    if len(sys.argv) > 1
    else script_dir.parents[1] / "docs" / "reference"
)
out = ref / "_decrypted"
out.mkdir(parents=True, exist_ok=True)
pdfs = sorted(ref.glob("*.pdf"))

# pick a decryption backend
backend = None
for cand in ("pikepdf", "pypdf", "PyPDF2"):
    try:
        __import__(cand)
        backend = cand
        break
    except Exception:
        pass


def text_snip(path: Path) -> str:
    try:
        import fitz

        d = fitz.open(path)
        t = (d[0].get_text() or "").strip().replace("\n", " ")
        d.close()
        return t[:160]
    except Exception as e:
        return f"<no-text: {str(e)[:50]}>"


print("REF DIR:", ref)
print("BACKEND:", backend)
print("N PDFs:", len(pdfs))
print("=" * 100)

enc_list, fail_list = [], []
for f in pdfs:
    name = f.name
    enc = False
    status = "OK"
    dest = out / name
    try:
        if backend == "pikepdf":
            import pikepdf

            with pikepdf.open(f) as pdf:  # empty user password
                enc = pdf.is_encrypted
                if enc:
                    pdf.save(dest)
        else:
            mod = __import__(backend)
            reader = mod.PdfReader(str(f))
            enc = bool(reader.is_encrypted)
            if enc:
                try:
                    reader.decrypt("")
                except Exception:
                    pass
                writer = mod.PdfWriter()
                for p in reader.pages:
                    writer.add_page(p)
                with open(dest, "wb") as fh:
                    writer.write(fh)
    except Exception as e:
        status = "FAIL: " + str(e)[:90]
        fail_list.append(name)
    if status == "OK" and enc:
        status = "DECRYPTED"
        enc_list.append(name)
    read_path = dest if (enc and dest.exists()) else f
    print(f"[{status}{' ENC' if enc else ''}] {name}")
    print("    P1:", text_snip(read_path))
print("=" * 100)
print("ENCRYPTED+DECRYPTED:", len(enc_list))
print("FAILED:", fail_list)
