#!/usr/bin/env python3
"""Check maintained Markdown links and explicit relative C includes after moves.

No network access. Vendor docs and generated build output are outside this check.
URL fragments are left to GitHub; local file/directory destinations must exist.
"""

from pathlib import Path
import re
import sys
from urllib.parse import unquote, urlsplit

ROOT = Path(__file__).resolve().parents[1]
SCOPES = ("application", "module", "bsp", "docs", "tests", "tools", "build_scripts")


def check():
    errors = []
    documents = list(ROOT.glob("*.md"))
    sources = []
    for name in SCOPES:
        documents.extend((ROOT / name).rglob("*.md"))
        sources.extend((ROOT / name).rglob("*.[ch]"))
    destinations = 0
    for path in sorted(documents):
        text = path.read_text(encoding="utf-8-sig")
        # Examples inside fenced blocks do not create Markdown links.
        text = re.sub(r"^```.*?^```[^\n]*", "", text, flags=re.M | re.S)
        for match in re.finditer(r"\]\(([^\s)]+)(?:\s+\"[^\"]*\")?\)", text):
            link = match[1].strip("<>")
            url = urlsplit(link)
            if url.scheme or url.netloc or not url.path:
                continue
            target = (path.parent / unquote(url.path)).resolve()
            destinations += 1
            if not target.exists():
                errors.append(f"{path.relative_to(ROOT)}: missing link destination {link}")
    for path in sorted(sources):
        for name in re.findall(r'^\s*#\s*include\s*"([^\"]+/[^\"]+)"',
                               path.read_text(encoding="utf-8-sig", errors="replace"), flags=re.M):
            if not (path.parent / name).is_file():
                errors.append(f"{path.relative_to(ROOT)}: missing relative include {name}")
    if errors:
        print("\n".join(errors), file=sys.stderr)
        return 1
    print(f"Layout passed: {len(documents)} Markdown files, {destinations} local links, "
          f"{len(sources)} C sources/headers checked")
    return 0


if __name__ == "__main__":
    raise SystemExit(check())
