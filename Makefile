.PHONY: all update-submodules dev build final refresh clean
.DEFAULT_GOAL := all

# Configure these
ROS_DISTRO := rolling
REPO_NAME := ros-template
CONTAINERFILE := container/Containerfile

# Image names
BASE_IMAGE := $(REPO_NAME):latest-base
DEV_IMAGE := $(REPO_NAME):latest-dev
BUILD_IMAGE := $(REPO_NAME):latest-build
FINAL_IMAGE := $(REPO_NAME):latest-final

# Marker files
MARKER_DIR := .make-markers
BASE_BUILT := $(MARKER_DIR)/base.built
DEV_BUILT := $(MARKER_DIR)/dev.built
FINAL_BUILT := $(MARKER_DIR)/final.built

# TARGETS

all: final

update-submodules:
	@git submodule update --init --recursive

dev: $(DEV_BUILT)
	@echo "==> Entering workspace..."
	@docker run \
	  --rm \
	  --tty \
	  --replace \
	  --interactive \
	  --volume=./:/workspace/:rw \
	  --workdir=/workspace \
	  --name=$(REPO_NAME)-dev \
	  --userns=host \
	  --network=host \
	  -e QT_X11_NO_MITSHM=1 \
	  -e NVIDIA_DRIVER_CAPABILITIES=all \
	  -e DISPLAY=$$DISPLAY \
	  --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
	  $(DEV_IMAGE) \
	  bash

build: $(BASE_BUILT)
	@echo "==> Building colcon workspace..."
	@docker run \
	  --rm \
	  --volume=./:/workspace/:rw \
	  --workdir=/workspace \
	  --name=$(REPO_NAME)-build \
	  $(BASE_IMAGE) \
	  colcon build --symlink-install

final: $(FINAL_BUILT)

$(BASE_BUILT): $(CONTAINERFILE)
	@echo "==> Building base image..."
	@docker build \
		-t $(BASE_IMAGE) \
		-f $(CONTAINERFILE) \
		--target=base \
		--build-arg ROS_DISTRO=$(ROS_DISTRO) .
	@mkdir -p $(MARKER_DIR)
	@touch $@

$(DEV_BUILT): $(BASE_BUILT) $(CONTAINERFILE)
	@echo "==> Building dev image..."
	@docker build \
		-t $(DEV_IMAGE) \
		-f $(CONTAINERFILE) \
		--target=dev \
		--build-arg ROS_DISTRO=$(ROS_DISTRO) .
	@mkdir -p $(MARKER_DIR)
	@touch $@

$(FINAL_BUILT): $(BASE_BUILT) $(CONTAINERFILE)
	@echo "==> Building final image..."
	@docker build \
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
	-@docker rmi $(BASE_IMAGE) $(DEV_IMAGE) $(FINAL_IMAGE) 2>/dev/null
	@echo "==> Removed build markers and deleted images..."

