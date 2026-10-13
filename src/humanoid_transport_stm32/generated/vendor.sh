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

mkdir -p "$HERE/fixtures" "$HERE/firmware_sm"
copy() { install -m 0644 "$FW/$1" "$HERE/$2"; echo "  vendored $2"; }
echo "vendoring from $FW @ $FIRMWARE_TAG:"
copy generated/cpp/robot_wire_gen.hpp        robot_wire_gen.hpp
copy generated/cpp/fixtures/robot_cmd.bin    fixtures/robot_cmd.bin
copy generated/cpp/fixtures/robot_tele.bin   fixtures/robot_tele.bin
copy firmware/common/protocol.yaml           protocol.yaml
# The firmware's PURE decision logic, wrapped verbatim by the fake master (isolated TU, because
# protocol.h's macros would clobber the C++ wrapper). The emulator thus runs the real state
# machine, not a hand-written mirror that could drift.
copy firmware/common/include/drive_sm.h      firmware_sm/drive_sm.h
copy firmware/common/include/host_watchdog.h firmware_sm/host_watchdog.h
copy firmware/common/include/protocol.h      firmware_sm/protocol.h
copy firmware/common/include/protocol_gen.h  firmware_sm/protocol_gen.h
# The firmware timing constants the fake master must match (enable/clear/stop-ramp durations, the
# host-death cycle count). The fake master sources them from here, through the shim, so a value
# like HOST_LOST_CYCLES can never silently drift from the firmware. Config, not wire: these are NOT
# part of SCHEMA_HASH, but ride the same pinned tag, and fw_boundary_test pins the enum values.
copy firmware/common/include/motor_config.h  firmware_sm/motor_config.h

# Fail loudly if the tag's headers no longer match the pinned hash (both the C++ mirror and the
# C header the firmware SM compiles against).
check_hash() {
    local f="$1"
    local h
    h="$(grep -oiE 'PROTO_SCHEMA_HASH[ =]+0x[0-9A-Fa-f]+' "$HERE/$f" \
         | grep -oiE '0x[0-9A-Fa-f]+' | head -1)"
    if [[ "${h,,}" != "${SCHEMA_HASH,,}" ]]; then
        echo "vendor: $FIRMWARE_TAG $f hash $h != pinned $SCHEMA_HASH" >&2
        echo "        the tag and FIRMWARE_PIN disagree — update the pin deliberately." >&2
        exit 1
    fi
}
check_hash robot_wire_gen.hpp
check_hash firmware_sm/protocol_gen.h
echo "ok: vendored $FIRMWARE_TAG, schema $SCHEMA_HASH"
