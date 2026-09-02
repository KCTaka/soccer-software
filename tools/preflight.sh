#!/usr/bin/env bash
# Pre-flight health check for the soccer-bot Jetson.
#
# Every check here exists because the corresponding failure has actually
# happened on this robot and cost hours to diagnose. See docs/TROUBLESHOOTING.md
# for the incident each one guards against (§ references below).
#
# Run before any bring-up, and after every reboot:
#     ./tools/preflight.sh              # full check
#     ./tools/preflight.sh --quick      # skip the container smoke tests
#
# Exit codes:  0 = all good (warnings allowed)   1 = at least one FAIL
#
# Deliberately does NOT require sudo. Anything needing root is reported as a
# manual follow-up rather than silently skipped.

set -uo pipefail

QUICK=0
[[ "${1:-}" == "--quick" ]] && QUICK=1

FAILS=0
WARNS=0

if [[ -t 1 ]]; then
    R=$'\e[31m'; G=$'\e[32m'; Y=$'\e[33m'; B=$'\e[1m'; N=$'\e[0m'
else
    R=""; G=""; Y=""; B=""; N=""
fi

section() { printf '\n%s── %s %s\n' "$B" "$1" "$N"; }
pass()    { printf '  %s[ PASS ]%s %s\n' "$G" "$N" "$1"; }
fail()    { printf '  %s[ FAIL ]%s %s\n' "$R" "$N" "$1"; FAILS=$((FAILS + 1)); }
warn()    { printf '  %s[ WARN ]%s %s\n' "$Y" "$N" "$1"; WARNS=$((WARNS + 1)); }
note()    { printf '           %s\n' "$1"; }

have() { command -v "$1" >/dev/null 2>&1; }

CDI_SPEC=/etc/cdi/nvidia.yaml
CDI_ENV=/etc/nvidia-container-toolkit/nvidia-cdi-refresh.env

printf '%ssoccer-bot pre-flight%s  —  %s  —  %s\n' "$B" "$N" "$(hostname)" "$(date -Is)"

# ─────────────────────────────────────────────────────────────────────────────
# 1. GPU driver liveness  (TROUBLESHOOTING §1.5 — ACR firmware boot failure)
# ─────────────────────────────────────────────────────────────────────────────
# The iGPU can fail to initialise at boot while everything else looks healthy:
# the modules load, /dev/nvgpu exists, and Docker starts. Only the GPU-backed
# device nodes are missing. Check the driver, not the module list.
section "1. GPU driver"

if [[ -d /dev/nvgpu/igpu0 ]]; then
    nodes=$(ls /dev/nvgpu/igpu0/ 2>/dev/null | tr '\n' ' ')
    if [[ "$(echo "$nodes" | tr -d ' ')" == "power" ]]; then
        fail "iGPU did not initialise — /dev/nvgpu/igpu0 has only 'power'"
        note "The driver failed nvgpu_finalize_poweron. Check:"
        note "  journalctl -b -k | grep -E 'ACR bootstrap|finalize_poweron|error -40'"
        note "Recovery is a reboot. See docs/TROUBLESHOOTING.md §1.5."
    else
        pass "GPU device nodes present ($nodes)"
    fi
else
    fail "/dev/nvgpu/igpu0 missing entirely — nvgpu driver not loaded"
fi

if have nvidia-smi; then
    if smi=$(nvidia-smi -L 2>&1) && [[ "$smi" == GPU* ]]; then
        pass "nvidia-smi: ${smi%%$'\n'*}"
    else
        fail "nvidia-smi cannot see the GPU"
        note "${smi%%$'\n'*}"
    fi
else
    warn "nvidia-smi not on PATH"
fi

# The firmware loader failure is silent unless you go looking for it.
if have journalctl; then
    acr=$(journalctl -b -k --no-pager 2>/dev/null | grep -c "ACR bootstrap failed")
    if [[ "$acr" -gt 0 ]]; then
        fail "$acr ACR bootstrap failures in this boot's kernel log"
        note "GPU firmware (ga10b/acr-gsp.*) failed to load. Reboot to recover."
    else
        pass "no GPU firmware errors this boot"
    fi
fi

# ─────────────────────────────────────────────────────────────────────────────
# 2. CDI spec  (TROUBLESHOOTING §1 — tmpfs + boot-race)
# ─────────────────────────────────────────────────────────────────────────────
# daemon.json sets default-runtime=nvidia, so a bad CDI spec breaks EVERY
# container, not just GPU ones.
section "2. NVIDIA CDI"

if [[ -s "$CDI_SPEC" ]]; then
    pass "spec present on persistent storage ($CDI_SPEC, $(stat -c %s "$CDI_SPEC") bytes)"
else
    fail "$CDI_SPEC missing or empty — every 'docker run' will fail"
    note "Fix: sudo systemctl restart nvidia-cdi-refresh.service"
fi

if [[ -e /var/run/cdi/nvidia.yaml ]]; then
    warn "a second spec exists at /var/run/cdi/nvidia.yaml (tmpfs)"
    note "Two specs of kind nvidia.com/gpu conflict. Remove the tmpfs copy."
else
    pass "no conflicting tmpfs spec"
fi

if grep -q "^NVIDIA_CTK_CDI_OUTPUT_FILE_PATH=$CDI_SPEC" "$CDI_ENV" 2>/dev/null; then
    pass "refresh service is configured to write to persistent storage"
else
    fail "NVIDIA_CTK_CDI_OUTPUT_FILE_PATH is not pinned to $CDI_SPEC"
    note "Without it the spec lands in tmpfs and vanishes on reboot."
    note "Fix: run deploy/ansible/provision.yml"
fi

# The check that would have caught this boot's failure immediately: a spec can
# be perfectly valid and still unusable if the devices it names do not exist.
if [[ -s "$CDI_SPEC" ]] && have python3; then
    missing=$(python3 - "$CDI_SPEC" <<'PY'
import sys, os, re
paths = set()
with open(sys.argv[1]) as fh:
    for line in fh:
        m = re.match(r"\s*-?\s*path:\s*(/dev/\S+)", line)
        if m:
            paths.add(m.group(1))
print("\n".join(sorted(p for p in paths if not os.path.exists(p))))
PY
)
    if [[ -z "$missing" ]]; then
        pass "every device node named in the spec exists on the host"
    else
        fail "spec references device nodes that do not exist:"
        while read -r p; do [[ -n "$p" ]] && note "  $p"; done <<<"$missing"
        note "Containers will fail with: failed to stat CDI host device"
        note "This means the GPU is down (see check 1), not that CDI is misconfigured."
    fi
fi

# ─────────────────────────────────────────────────────────────────────────────
# 3. Container runtime
# ─────────────────────────────────────────────────────────────────────────────
section "3. Docker + GPU injection"

if have docker && docker info >/dev/null 2>&1; then
    pass "docker daemon reachable"
    if docker info --format '{{.DefaultRuntime}}' 2>/dev/null | grep -q nvidia; then
        pass "default runtime is nvidia"
    else
        warn "default runtime is not nvidia — GPU needs an explicit --runtime"
    fi

    if [[ "$QUICK" -eq 0 ]]; then
        # Every container goes through CDI injection because the default runtime
        # is nvidia, so any local image proves the whole path works.
        probe=$(docker images --format '{{.Repository}}:{{.Tag}}' 2>/dev/null \
                | grep -E '^(soccer-app|soccer-zed|ubuntu):' | head -1)
        if [[ -n "$probe" ]]; then
            if err=$(docker run --rm --entrypoint true "$probe" 2>&1); then
                pass "CDI injection works (smoke-tested with $probe)"
            else
                fail "container start fails through the nvidia runtime"
                note "${err##*: }"
            fi
        else
            warn "no local image available for a CDI smoke test"
        fi
    fi
else
    fail "docker unavailable"
fi

# ─────────────────────────────────────────────────────────────────────────────
# 4. Image self-tests  (TROUBLESHOOTING §3 — runtime image had no ROS)
# ─────────────────────────────────────────────────────────────────────────────
# A multi-stage image can copy its build artifacts correctly and still be unable
# to execute its own CMD. Test the CMD's dependencies, not the file listing.
section "4. Image runtime dependencies"

if [[ "$QUICK" -eq 1 ]]; then
    note "skipped (--quick)"
elif have docker && docker info >/dev/null 2>&1; then
    for img in soccer-app:jazzy soccer-zed:jazzy; do
        if ! docker image inspect "$img" >/dev/null 2>&1; then
            warn "$img not built locally — skipping"
            continue
        fi
        out=$(docker run --rm --entrypoint bash "$img" -c '
            source /opt/ros/jazzy/setup.bash 2>/dev/null || { echo "NO_ROS"; exit 1; }
            command -v ros2 >/dev/null || { echo "NO_ROS2_CLI"; exit 1; }
            for w in /ws /ros2_ws; do
                [ -f "$w/install/setup.bash" ] && source "$w/install/setup.bash"
            done
            broken=0
            for so in $(find /ws/install /ros2_ws/install -name "*.so" 2>/dev/null); do
                if ldd "$so" 2>/dev/null | grep -q "not found"; then
                    echo "BROKEN_LINK $so"; broken=1
                fi
            done
            [ "$broken" -eq 0 ] && echo OK
        ' 2>&1)
        case "$out" in
            *NO_ROS2_CLI*|*NO_ROS*)
                fail "$img cannot run its own CMD — no ROS runtime"
                note "The runtime stage inherited a base without ROS installed."
                note "Fix: apply deploy/docker/soccer-app-deps.apt in that stage." ;;
            *BROKEN_LINK*)
                fail "$img has libraries with unresolved dependencies:"
                grep BROKEN_LINK <<<"$out" | while read -r _ p; do note "  $p"; done ;;
            *OK*)
                pass "$img has a working ROS runtime and no broken links" ;;
            *)
                warn "$img self-test inconclusive: ${out##*$'\n'}" ;;
        esac
    done
else
    warn "docker unavailable — skipping image self-tests"
fi

# ─────────────────────────────────────────────────────────────────────────────
# 5. Host tuning
# ─────────────────────────────────────────────────────────────────────────────
section "5. Host tuning"

rmem=$(sysctl -n net.core.rmem_max 2>/dev/null || echo 0)
if [[ "$rmem" -ge 16777216 ]]; then
    pass "net.core.rmem_max = $rmem"
else
    warn "net.core.rmem_max = $rmem (expected >= 16777216)"
    note "Fix: run deploy/ansible/provision.yml, then sysctl --system"
fi

if have nvpmodel; then
    # `nvpmodel -q` prints "NV Power Mode: <name>" then the mode number on the
    # next line; take the name from the labelled line, not the number.
    mode=$(nvpmodel -q 2>/dev/null | sed -n 's/^NV Power Mode:[[:space:]]*//p' | head -1)
    if [[ "$mode" == *MAXN* ]]; then
        pass "power mode: $mode"
    else
        warn "power mode: ${mode:-unknown} (expected MAXN_SUPER)"
        note "Fix: sudo nvpmodel -m 2"
    fi
fi

swap=$(free -m | awk '/^Swap:/ {print $2}')
if [[ "${swap:-0}" -ge 4096 ]]; then
    pass "swap: ${swap} MB"
else
    warn "swap: ${swap:-0} MB — on-device colcon builds may OOM"
fi

# ─────────────────────────────────────────────────────────────────────────────
# 6. ZED camera
# ─────────────────────────────────────────────────────────────────────────────
# The ZED Mini presents TWO USB functions: 2b03:f681 (HID, full-speed) and
# 2b03:f682 (UVC video, SuperSpeed). The HID one enumerates even on a USB 2.0
# link, so "lsusb sees a ZED" is not enough — check for the video function and
# the /dev/video nodes the SDK actually opens.
section "6. ZED camera"

if have lsusb; then
    if lsusb -d 2b03:f682 >/dev/null 2>&1; then
        pass "ZED video interface present (2b03:f682)"
    elif lsusb -d 2b03: >/dev/null 2>&1; then
        fail "only the ZED HID interface enumerated — the video function is missing"
        note "The UVC function needs a SuperSpeed link. Check the cable and port,"
        note "then power-cycle the board. See docs/TROUBLESHOOTING.md §1.5."
    else
        fail "no Stereolabs camera on USB (vendor 2b03)"
    fi

    # Both the ZED and the GPU failing to come up together usually means the
    # board browned out during boot rather than two independent faults.
    if lsusb -t 2>/dev/null | grep -qE '(5000M|10000M)$'; then
        speed_ok=1
    else
        speed_ok=0
    fi
    zed_bus=$(lsusb -t 2>/dev/null | grep -B99 -m1 'Driver=uvcvideo' | tail -1)
    if [[ -n "$zed_bus" ]]; then
        pass "camera bound to uvcvideo"
    elif [[ "$speed_ok" -eq 0 ]]; then
        warn "no SuperSpeed USB link is up at all — suspect power or cabling"
    fi
else
    warn "lsusb unavailable"
fi

if compgen -G "/dev/video*" >/dev/null; then
    pass "video nodes: $(ls -d /dev/video* | tr '\n' ' ')"
else
    fail "no /dev/video* nodes — the ZED SDK cannot open the camera"
fi

# ─────────────────────────────────────────────────────────────────────────────
# Summary
# ─────────────────────────────────────────────────────────────────────────────
printf '\n%s────────────────────────────────────────%s\n' "$B" "$N"
if [[ "$FAILS" -eq 0 ]]; then
    printf '%sPRE-FLIGHT OK%s  (%d warning(s))\n' "$G" "$N" "$WARNS"
    exit 0
fi
printf '%sPRE-FLIGHT FAILED%s  (%d failure(s), %d warning(s))\n' "$R" "$N" "$FAILS" "$WARNS"
printf 'Do not start the stack until the failures above are resolved.\n'
exit 1
