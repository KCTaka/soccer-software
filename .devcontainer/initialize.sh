#!/usr/bin/env bash
# Runs on the dev container's *host* (WSL, the Jetson over Remote-SSH, ...) before every container
# create/start. It is the only place that knows what the host looks like; devcontainer.json stays
# identical everywhere and reads what this script writes to .devcontainer/.generated/:
#
#   host.env    passed to `docker run --env-file`: DISPLAY, when the host has one
#   Xauthority  X11 cookies for that display, rewritten to match any hostname
#
# The container picks these up when it is created. After the host's display changes (new ssh -X
# session, different monitor), run "Dev Containers: Rebuild Container".
set -euo pipefail

here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
out="$here/.generated"
mkdir -p "$out"

# 1. Which X server? An exported DISPLAY wins (WSLg, ssh -X). VS Code's Remote-SSH server does not
#    inherit the DISPLAY of an interactive ssh session, so otherwise take the first local socket.
display=${DISPLAY:-}
if [[ -z $display ]]; then
  for sock in /tmp/.X11-unix/X*; do
    if [[ -S $sock ]]; then
      display=":${sock##*/X}"
      break
    fi
  done
fi

# 2. X11 cookies. Always create the file: it is bind-mounted via the workspace, and an empty cookie
#    file is what Xlib needs for servers that do not authenticate (WSLg).
auth="$out/Xauthority"
src=${XAUTHORITY:-$HOME/.Xauthority}
: >"$auth"
chmod 600 "$auth"
cookies=0
if [[ -n $display && -r $src ]]; then
  if command -v xauth >/dev/null 2>&1; then
    # Family 0xffff matches any hostname, so the cookie works whatever the container is called.
    xauth -f "$src" nlist 2>/dev/null | sed -e 's/^..../ffff/' |
      xauth -f "$auth" nmerge - 2>/dev/null || true
    cookies=$(xauth -f "$auth" list 2>/dev/null | wc -l)
  else
    cp "$src" "$auth"
    cookies=-1
  fi
fi

# 3. Environment for the container. Omit DISPLAY entirely when there is none, so the container is
#    not left pointing at a server that does not exist.
if [[ -n $display ]]; then
  echo "DISPLAY=$display" >"$out/host.env"
else
  : >"$out/host.env"
fi

echo "[devcontainer] DISPLAY=${display:-<none>} X11 cookies=$cookies ($(uname -m))" >&2
if [[ -z $display ]]; then
  echo "[devcontainer] no X display on this host: launch with rviz:=false foxglove:=true" >&2
fi
