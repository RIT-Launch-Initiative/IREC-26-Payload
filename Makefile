.PHONY: up down exec logs build

all:
	if [ -f "/.dockerenv" ] || [ -f "/opt/ros/humble/setup.bash" ]; then \
	  colcon build --symlink-install --event-handlers console_cohesion+ --cmake-args -G Ninja; \
	else \
	  docker compose run --rm ros bash -lc \
  		"cd /workspace && \
   		source /opt/ros/humble/setup.bash && \
   		colcon build --symlink-install --event-handlers console_cohesion+" --cmake-args -G Ninja; \
	fi

clean:
	rm -rf build/ install/ log/

up:
	LOCAL_UID=$$(id -u) LOCAL_GID=$$(id -g) docker compose up --build -d

down:
	docker compose down

exec:
	docker compose exec ros bash

logs:
	docker compose logs -f ros

build:
	docker compose build

local_perms:
	sudo chown -R $(id -u):$(id -g) .

# Must be ran in container / Pi
compile_commands:
	colcon build --symlink-install --event-handlers console_direct+ --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -G Ninja

colcon:
	colcon build --symlink-install
