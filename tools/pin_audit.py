#!/usr/bin/env python3
"""Reject mutable references in CI workflows, Dockerfiles, and .repos manifests."""
from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SHA40 = re.compile(r"^[0-9a-fA-F]{40}$")
SHA64 = re.compile(r"^sha256:[0-9a-fA-F]{64}$")
USES = re.compile(r"^\s*-?\s*uses:\s*([^\s#]+)")
FROM = re.compile(r"^\s*FROM\s+([^\s]+)", re.IGNORECASE)
REPOS_VERSION = re.compile(r"^\s*version:\s*['\"]?([^'\"\s#]+)")

EXCLUDED = {".git", ".venv", "build", "install", "log", "__pycache__"}


def files(pattern: str):
    return (p for p in ROOT.rglob(pattern) if not EXCLUDED.intersection(p.parts))


def audit_actions(errors: list[str]) -> None:
    for path in (*files("*.yml"), *files("*.yaml")):
        for line_no, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
            match = USES.search(line)
            if not match:
                continue
            ref = match.group(1).split("@", 1)[-1]
            if not SHA40.fullmatch(ref):
                errors.append(f"{path.relative_to(ROOT)}:{line_no}: action not pinned to 40-hex SHA: {match.group(1)}")


def audit_dockerfiles(errors: list[str]) -> None:
    for path in files("*.Dockerfile"):
        for line_no, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
            match = FROM.search(line)
            if not match or match.group(1).startswith("${"):
                continue
            image = match.group(1)
            if "@" not in image or not SHA64.fullmatch(image.rsplit("@", 1)[1]):
                errors.append(f"{path.relative_to(ROOT)}:{line_no}: base image not pinned by digest: {image}")


def audit_repos(errors: list[str]) -> None:
    for path in files("*.repos"):
        for line_no, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
            match = REPOS_VERSION.match(line)
            if match and not SHA40.fullmatch(match.group(1)):
                errors.append(f"{path.relative_to(ROOT)}:{line_no}: .repos revision not a commit hash: {match.group(1)}")


def main() -> int:
    errors: list[str] = []
    audit_actions(errors)
    audit_dockerfiles(errors)
    audit_repos(errors)
    if errors:
        print("pin audit failed:\n" + "\n".join(f"- {e}" for e in errors), file=sys.stderr)
        return 1
    print("pin audit passed: actions, container bases, and .repos revisions are immutable")
    return 0


if __name__ == "__main__":
    sys.exit(main())