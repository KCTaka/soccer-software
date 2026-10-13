#!/usr/bin/env bash
# Vendor the generated wire-protocol artifacts from a PINNED soccer-firmware tag.
#
# soccer-software does not hand-write the wire layout: it pins a firmware release
# tag and copies that tag's generated C++ header, golden fixtures and the schema
# text into this directory. FIRMWARE_PIN records the tag + schema hash; the build
# test (test_wire_pin) asserts the vendored header's SCHEMA_HASH matches the pin.
#
# Usage:
#   vendor.sh [path-to-soccer-firmware]     (default: $SOCCER_FIRMWARE or /firmware)
#
# Refuses anything but a clean checkout sitting exactly on the pinned tag, so a
# vendored artifact is always reproducible from a named release.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
FW="${1:-${SOCCER_FIRMWARE:-/firmware}}"

# shellcheck disable=SC1090
source "$HERE/FIRMWARE_PIN"

if ! git -C "$FW" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    echo "vendor: '$FW' is not a git checkout of soccer-firmware" >&2
    echo "        pass the path, or set \$SOCCER_FIRMWARE" >&2
    exit 1
fi

# The checkout must be EXACTLY the pinned tag and clean — no 'whatever is mounted'.
got_tag="$(git -C "$FW" describe --exact-match --tags HEAD 2>/dev/null || true)"
if [[ "$got_tag" != "$FIRMWARE_TAG" ]]; then
    echo "vendor: '$FW' HEAD is '${got_tag:-<not a tagged commit>}', need '$FIRMWARE_TAG'" >&2
    echo "        git -C '$FW' checkout $FIRMWARE_TAG" >&2
    exit 1
fi
if [[ -n "$(git -C "$FW" status --porcelain)" ]]; then
    echo "vendor: '$FW' has local modifications — refusing (vendored bytes must be" >&2
    echo "        reproducible from $FIRMWARE_TAG). Stash/clean, then re-run." >&2
    exit 1
fi

copy() { install -m 0644 "$FW/$1" "$HERE/$2"; echo "  vendored $2"; }
echo "vendoring from $FW @ $FIRMWARE_TAG:"
copy generated/cpp/robot_wire_gen.hpp        robot_wire_gen.hpp
copy generated/cpp/fixtures/robot_cmd.bin    fixtures/robot_cmd.bin
copy generated/cpp/fixtures/robot_tele.bin   fixtures/robot_tele.bin
copy firmware/common/protocol.yaml           protocol.yaml

# Fail loudly if the tag's header no longer matches the pinned hash.
hdr_hash="$(grep -oiE 'PROTO_SCHEMA_HASH = 0x[0-9A-Fa-f]+' "$HERE/robot_wire_gen.hpp" \
            | grep -oiE '0x[0-9A-Fa-f]+' | head -1)"
if [[ "${hdr_hash,,}" != "${SCHEMA_HASH,,}" ]]; then
    echo "vendor: $FIRMWARE_TAG header hash $hdr_hash != pinned $SCHEMA_HASH" >&2
    echo "        the tag and FIRMWARE_PIN disagree — update the pin deliberately." >&2
    exit 1
fi
echo "ok: vendored $FIRMWARE_TAG, schema $SCHEMA_HASH"
