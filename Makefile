WS := ros2_ws

.PHONY: build build-pkg clean sim robot help

## Build the entire workspace
build:
	cd $(WS) && colcon build --symlink-install

## Build a single package.  Usage: make build-pkg pkg=soccer_bringup
build-pkg:
	@test -n "$(pkg)" || (echo "Usage: make build-pkg pkg=<package_name>" && exit 1)
	cd $(WS) && colcon build --symlink-install --packages-select $(pkg)

## Remove build/install/log artifacts
clean:
	rm -rf $(WS)/build $(WS)/install $(WS)/log

## Launch a 2-robot simulation locally
sim:
	cd deploy/compose && docker compose -f sim.compose.yaml up --build

## Bring up the real robot stack
robot:
	cd deploy/compose && docker compose -f robot.compose.yaml up --build

## Show this help
help:
	@grep -E '^##' Makefile | sed 's/^## //'
