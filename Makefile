WS := ros2_ws

.PHONY: build build-pkg check preflight clean sim robot help

## Run the static repository invariant checks (no ROS, GPU or Docker needed)
check:
	@python3 tools/check_repo_invariants.py

## Run the on-device pre-flight health check
preflight:
	@./tools/preflight.sh

## Build the entire workspace
build:
	@command -v colcon >/dev/null 2>&1 || { \
	  echo "colcon is not on PATH."; \
	  echo "This host has no ROS installation - build inside a container:"; \
	  echo "    ./tools/dev_shell.sh      # dev container with colcon + ROS"; \
	  echo "    make robot                # or let compose build the images"; \
	  exit 1; }
	cd $(WS) && colcon build --symlink-install

## Build a single package.  Usage: make build-pkg pkg=soccer_bringup
build-pkg:
	@test -n "$(pkg)" || (echo "Usage: make build-pkg pkg=<package_name>" && exit 1)
	@command -v colcon >/dev/null 2>&1 || { \
	  echo "colcon is not on PATH - use ./tools/dev_shell.sh"; exit 1; }
	cd $(WS) && colcon build --symlink-install --packages-select $(pkg)

## Remove build/install/log artifacts
clean:
	rm -rf $(WS)/build $(WS)/install $(WS)/log

## Launch a 2-robot simulation locally
sim:
	cd deploy/compose && docker compose -f sim.compose.yaml up --build

## Bring up the real robot stack (pre-flight check runs first)
robot: preflight
	cd deploy/compose && docker compose -f robot.compose.yaml up --build

## Show this help
help:
	@grep -E '^##' Makefile | sed 's/^## //'
