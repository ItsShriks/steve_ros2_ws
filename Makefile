# ==============================================================================
# Steve Robot - Multi-Machine ROS2 Makefile
# ==============================================================================
# Main robot: Native ROS2 (add to ~/.bashrc: export ROS_DOMAIN_ID=42)
# Secondary machines: Docker (use targets below)
# ==============================================================================

# Configuration
IMAGE_NAME ?= steve-ros2-humble
CONTAINER_NAME ?= steve_robot
ROS_DOMAIN_ID ?= 42

# Optional: Robot namespace for multi-robot scenarios
ROBOT_NAMESPACE ?= steve

# ------------------------------------------------------------------------------
# Auto-Detected Paths (System-Agnostic)
# ------------------------------------------------------------------------------

# Automatically detect workspace directory (where this Makefile lives)
WORKSPACE_DIR := $(shell pwd)

# Container workspace path (mirrors host path for ROS overlay compatibility)
WORKSPACE_CONTAINER := $(WORKSPACE_DIR)

# Dockerfile location (relative to Makefile)
DOCKERFILE := $(WORKSPACE_DIR)/.devcontainer/Dockerfile

# Auto-detect user info for consistent UID/GID mapping
CURRENT_USER := $(shell id -un)
CURRENT_UID := $(shell id -u)
CURRENT_GID := $(shell id -g)

# Docker commands
DOCKER_BUILD_CMD := docker build \
	-t $(IMAGE_NAME):latest \
	--build-arg USERNAME=$(CURRENT_USER) \
	--build-arg USER_UID=$(CURRENT_UID) \
	--build-arg USER_GID=$(CURRENT_GID) \
	-f $(DOCKERFILE) \
	$(WORKSPACE_DIR)

# ------------------------------------------------------------------------------
# Docker Run Command (GPU + Multi-Machine Support)
# ------------------------------------------------------------------------------

DOCKER_RUN_CMD := docker run -it --rm \
	--name $(CONTAINER_NAME) \
	--gpus all \
	--runtime nvidia \
	--net host \
	--ipc host \
	--privileged \
	-e DISPLAY=$(DISPLAY) \
	-e QT_X11_NO_MITSHM=1 \
	-e ROS_DOMAIN_ID=$(ROS_DOMAIN_ID) \
	-e NVIDIA_VISIBLE_DEVICES=all \
	-e NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute \
	-e XAUTHORITY=/tmp/.docker.xauth \
	-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
	-v $(HOME)/.Xauthority:/tmp/.docker.xauth:rw \
	-v $(WORKSPACE_DIR):$(WORKSPACE_DIR) \
	-w $(WORKSPACE_DIR) \
	$(IMAGE_NAME):latest

# ==============================================================================
# BUILD TARGETS
# ==============================================================================

.PHONY: build build-no-cache clean

## build: Build the Docker image
build:
	@echo "================================================================"
	@echo "Building Docker image: $(IMAGE_NAME):latest"
	@echo "Workspace: $(WORKSPACE_DIR)"
	@echo "User: $(CURRENT_USER) (UID:$(CURRENT_UID), GID:$(CURRENT_GID))"
	@echo "================================================================"
	$(DOCKER_BUILD_CMD)
	@echo "Build complete!"

## build-no-cache: Build Docker image without using cache
build-no-cache:
	@echo "Building image (no cache): $(IMAGE_NAME):latest"
	$(DOCKER_BUILD_CMD) --no-cache
	@echo "Build complete!"

## clean: Remove Docker container and image
clean:
	@echo "Removing container and image"
	-docker rm -f $(CONTAINER_NAME) 2>/dev/null || true
	-docker rmi -f $(IMAGE_NAME):latest 2>/dev/null || true
	@echo "Cleanup complete!"

# ==============================================================================
# CONTAINER
# ==============================================================================

.PHONY: run shell stop restart status

## run: Start a new interactive container (use 'shell' for persistent access)
run:
	@echo "================================================================"
	@echo "Starting container: $(CONTAINER_NAME)"
	@echo "ROS_DOMAIN_ID: $(ROS_DOMAIN_ID)"
	@echo "Workspace: $(WORKSPACE_DIR)"
	@echo "================================================================"
	@xhost +local:docker 2>/dev/null || echo "Warning: xhost not available"
	$(DOCKER_RUN_CMD) /bin/bash

## shell: Attach to running container OR start new one if not running
shell:
	@if docker inspect -f '{{.State.Running}}' $(CONTAINER_NAME) >/dev/null 2>&1; then \
		echo "Attaching to running container: $(CONTAINER_NAME)"; \
		docker exec -it $(CONTAINER_NAME) /bin/bash; \
	else \
		echo "Container not running. Starting new container..."; \
		xhost +local:docker 2>/dev/null || true; \
		$(DOCKER_RUN_CMD) /bin/bash; \
	fi

## stop: Stop the running container
stop:
	@echo "Stopping container: $(CONTAINER_NAME)"
	-docker stop $(CONTAINER_NAME) 2>/dev/null || echo "Container not running"
	@echo "Container stopped"

## restart: Restart the container
restart: stop
	@echo "Restarting container..."
	@sleep 1
	$(MAKE) shell

## status: Check if container is running
status:
	@if docker inspect -f '{{.State.Running}}' $(CONTAINER_NAME) 2>/dev/null | grep -q true; then \
		echo "Container $(CONTAINER_NAME) is RUNNING"; \
		echo "ROS_DOMAIN_ID: $(ROS_DOMAIN_ID)"; \
		docker exec $(CONTAINER_NAME) bash -c 'echo "Container ROS_DOMAIN_ID: $$ROS_DOMAIN_ID"' 2>/dev/null || true; \
	else \
		echo "Container $(CONTAINER_NAME) is NOT RUNNING"; \
	fi

# ==============================================================================
# UTILITIES
# ==============================================================================

.PHONY: check-domain network-info logs ps info help

check-domain:
	@echo "ROS_DOMAIN_ID: $(ROS_DOMAIN_ID)"
	@echo "Set on all machines: export ROS_DOMAIN_ID=$(ROS_DOMAIN_ID)"

network-info:
	@echo "ROS_DOMAIN_ID: $(ROS_DOMAIN_ID)"
	@echo "IP: $(shell hostname -I)"

logs:
	@docker logs -f $(CONTAINER_NAME)

ps:
	@docker ps

info:
	@echo "Workspace: $(WORKSPACE_DIR)"
	@echo "Image: $(IMAGE_NAME):latest"
	@echo "Container: $(CONTAINER_NAME)"
	@echo "ROS_DOMAIN_ID: $(ROS_DOMAIN_ID)"
	@echo "User: $(CURRENT_USER) ($(CURRENT_UID):$(CURRENT_GID))"

help:
	@echo "Build:      make build, make clean"
	@echo "Container:  make shell, make stop, make status"
	@echo "Workspace:  make build-ws, make build-pkg PKG=name"
	@echo "Network:    make check-domain, make network-info"
	@echo "Utils:      make info, make logs, make ps"
	@echo ""
	@echo "Override: make shell ROS_DOMAIN_ID=99"

.DEFAULT_GOAL := help
