.PHONY: all clean up down exec logs build local_perms compile_commands colcon shell

LOCAL_UID := $(shell id -u)
LOCAL_GID := $(shell id -g)
COMPOSE := LOCAL_UID=$(LOCAL_UID) LOCAL_GID=$(LOCAL_GID) docker compose

all:
	if [ -f "/.dockerenv" ] || [ -f "/opt/ros/humble/setup.bash" ]; then \
	  colcon build --symlink-install --event-handlers console_cohesion+ --cmake-args -G Ninja; \
	else \
	  $(COMPOSE) run --rm ros bash -lc \
  		"cd /workspace && \
   		source /opt/ros/humble/setup.bash && \
   		if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi && \
   		colcon build --symlink-install --event-handlers console_cohesion+ --cmake-args -G Ninja"; \
	fi

clean:
	rm -rf build/ install/ log/

up:
	$(COMPOSE) up --build -d

down:
	$(COMPOSE) down

exec:
	$(COMPOSE) exec ros bash

shell: exec

logs:
	$(COMPOSE) logs -f ros

build:
	$(COMPOSE) build

local_perms:
	sudo chown -R $(id -u):$(id -g) .

# Must be ran in container / Pi
compile_commands:
	colcon build --symlink-install --event-handlers console_direct+ --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -G Ninja

colcon:
	colcon build --symlink-install
