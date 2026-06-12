from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[2]

CYRILLIC_RE = re.compile(r"[\u0400-\u04FF]")

SKIP_DIRS = {
    ".git",
    ".venv",
    "__pycache__",
    ".pytest_cache",
    ".mypy_cache",
    ".ruff_cache",
    "build",
    "dist",
    "dist_installer",
    "datasets",
    "results",
}

TEXT_SUFFIXES = {
    ".py",
    ".md",
    ".txt",
    ".bat",
    ".ps1",
    ".json",
    ".yaml",
    ".yml",
    ".toml",
    ".cfg",
    ".ini",
}


def should_skip(path: Path) -> bool:
    return any(part in SKIP_DIRS for part in path.parts)


def is_probably_comment(line: str, suffix: str) -> bool:
    stripped = line.strip()

    if suffix == ".py":
        return (
            stripped.startswith("#")
            or '"""' in stripped
            or "'''" in stripped
            or stripped.startswith('r"""')
            or stripped.startswith("r'''")
        )

    if suffix in {".bat", ".ps1"}:
        return stripped.lower().startswith(("rem ", "::", "#"))

    if suffix in {".md", ".txt"}:
        return True

    if suffix in {".json", ".yaml", ".yml", ".toml", ".cfg", ".ini"}:
        return stripped.startswith(("#", "//"))

    return False


def main():
    rows = []

    for path in sorted(ROOT.rglob("*")):
        if should_skip(path):
            continue

        if not path.is_file():
            continue

        if path.suffix.lower() not in TEXT_SUFFIXES:
            continue

        try:
            text = path.read_text(encoding="utf-8")
        except UnicodeDecodeError:
            try:
                text = path.read_text(encoding="utf-8-sig")
            except Exception:
                continue
        except Exception:
            continue

        for line_no, line in enumerate(text.splitlines(), start=1):
            if not CYRILLIC_RE.search(line):
                continue

            kind = "comment/doc/text" if is_probably_comment(line, path.suffix.lower()) else "code/string"

            rel = path.relative_to(ROOT).as_posix()
            rows.append((rel, line_no, kind, line.rstrip()))

    out = ROOT / "cyrillic_text_report.txt"

    with out.open("w", encoding="utf-8") as f:
        for rel, line_no, kind, line in rows:
            f.write(f"{rel}:{line_no}: [{kind}] {line}\n")

    print(f"Found {len(rows)} Cyrillic lines.")
    print(f"Report written to: {out}")


if __name__ == "__main__":
    main()
