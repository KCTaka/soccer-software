#!/usr/bin/env bash
# ADR-009-07: regenerate every derived artifact and fail on any difference.
set -euo pipefail
cd "$(git rev-parse --show-toplevel)"
python3 -m model.generators.cli check
echo "verify_generated: OK"