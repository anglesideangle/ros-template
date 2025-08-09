.PHONY: all sync-submodules dev build test final refresh clean
.DEFAULT_GOAL := all

# TODO Configure these
ROS_DISTRO ?= kilted
REPO_NAME ?= ros-template

CONTAINERFILE := Containerfile
CONTAINER_ENGINE := docker

# Tag names
# These are dependent on the distro, meaning changing distro will cause a rebuild
BASE_TAG := $(ROS_DISTRO)-base
DEV_TAG := $(ROS_DISTRO)-dev
FINAL_TAG := $(ROS_DISTRO)-final

# Image names
BASE_IMAGE := $(REPO_NAME):$(BASE_TAG)
DEV_IMAGE := $(REPO_NAME):$(DEV_TAG)
FINAL_IMAGE := $(REPO_NAME):$(FINAL_TAG)

# Marker files
# These are used by `build`, `test`, `dev`, and `final` commands
# to ensure rebuilds only happen when necessary
MARKER_DIR := .make-markers
BASE_BUILT := $(MARKER_DIR)/base.built
DEV_BUILT := $(MARKER_DIR)/dev.built
FINAL_BUILT := $(MARKER_DIR)/final.built

all: final

sync-submodules:
	@echo "==> Syncing submodules to .gitmodules..."
	@git submodule sync
	@git submodule update --init --recursive

dev: $(DEV_BUILT) sync-submodules
	@echo "==> Entering workspace..."
	@$(CONTAINER_ENGINE) run \
	  --rm \
	  --tty \
	  --interactive \
	  --volume=./:/workspace/:rw \
	  --workdir=/workspace \
	  --name=$(REPO_NAME)-dev \
	  --userns=host \
	  --network=host \
	  --pid=host \
	  --ipc=host \
	  -e QT_X11_NO_MITSHM=1 \
	  -e NVIDIA_DRIVER_CAPABILITIES=all \
	  -e DISPLAY=$$DISPLAY \
	  --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
	  $(DEV_IMAGE) \
	  bash

build: $(BASE_BUILT) sync-submodules
	@echo "==> Building colcon workspace..."
	@$(CONTAINER_ENGINE) run \
	  --rm \
	  --volume=./:/workspace/:rw \
	  --workdir=/workspace \
	  --name=$(REPO_NAME)-build \
	  $(BASE_IMAGE) \
	  colcon build --symlink-install

test: $(BASE_BUILT) sync-submodules
	@echo "==> Testing colcon workspace..."
	@$(CONTAINER_ENGINE) run \
	  --rm \
	  --volume=./:/workspace/:rw \
	  --workdir=/workspace \
	  --name=$(REPO_NAME)-test \
	  $(BASE_IMAGE) \
	  colcon test --event-handlers console_cohesion+

final: $(FINAL_BUILT) sync-submodules

$(BASE_BUILT): $(CONTAINERFILE)
	@echo "==> Building base image..."
	@$(CONTAINER_ENGINE) build \
		-t $(BASE_IMAGE) \
		-f $(CONTAINERFILE) \
		--target=base \
		--build-arg ROS_DISTRO=$(ROS_DISTRO) .
	@mkdir -p $(MARKER_DIR)
	@touch $@

$(DEV_BUILT): $(BASE_BUILT) $(CONTAINERFILE)
	@echo "==> Building dev image..."
	@$(CONTAINER_ENGINE) build \
		-t $(DEV_IMAGE) \
		-f $(CONTAINERFILE) \
		--target=dev \
		--build-arg ROS_DISTRO=$(ROS_DISTRO) .
	@mkdir -p $(MARKER_DIR)
	@touch $@

$(FINAL_BUILT): $(BASE_BUILT) $(CONTAINERFILE)
	@echo "==> Building final image..."
	@$(CONTAINER_ENGINE) build \
  	  -t $(BASE_IMAGE) \
  	  -f $(CONTAINERFILE) \
  	  --target=final \
  	  --build-arg ROS_DISTRO=$(ROS_DISTRO) .
	@mkdir -p $(MARKER_DIR)
	@touch $@

refresh:
	@rm -rf $(MARKER_DIR)
	@echo "==> Removed build markers, which will trigger rebuilds on next runs."

clean:
	@rm -rf $(MARKER_DIR)
	-@$(CONTAINER_ENGINE) rmi $(BASE_IMAGE) $(DEV_IMAGE) $(FINAL_IMAGE) 2>/dev/null
	@echo "==> Removed build markers and deleted images..."

