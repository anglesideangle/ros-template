.PHONY: all sync-submodules dev build test final refresh clean pull push
.DEFAULT_GOAL := all

include .env
export

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
	-t $(FINAL_IMAGE) \
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

pull:
	@echo "==> Pulling images from registry and then removing the tag..."
	@$(CONTAINER_ENGINE) login $(REG) || true
	-@$(CONTAINER_ENGINE) pull $(REG)/$(BASE_IMAGE) && $(CONTAINER_ENGINE) tag $(REG)/$(BASE_IMAGE) $(BASE_IMAGE) && $(CONTAINER_ENGINE) rmi $(REG)/$(BASE_IMAGE) 2>/dev/null || true
	-@$(CONTAINER_ENGINE) pull $(REG)/$(DEV_IMAGE) && $(CONTAINER_ENGINE) tag $(REG)/$(DEV_IMAGE) $(DEV_IMAGE) && $(CONTAINER_ENGINE) rmi $(REG)/$(DEV_IMAGE) 2>/dev/null || true
	-@$(CONTAINER_ENGINE) pull $(REG)/$(FINAL_IMAGE) && $(CONTAINER_ENGINE) tag $(REG)/$(FINAL_IMAGE) $(FINAL_IMAGE) && $(CONTAINER_ENGINE) rmi $(REG)/$(FINAL_IMAGE) 2>/dev/null || true

push:
	@echo "==> Pushing images that have been built..."
	@$(CONTAINER_ENGINE) login $(REG) || true
	@if [ -f "$(MARKER_DIR)/base.built" ]; then \
		$(CONTAINER_ENGINE) tag $(BASE_IMAGE) $(REG)/$(BASE_IMAGE); \
		$(CONTAINER_ENGINE) push $(REG)/$(BASE_IMAGE); \
	fi
	@if [ -f "$(MARKER_DIR)/dev.built" ]; then \
		$(CONTAINER_ENGINE) tag $(DEV_IMAGE) $(REG)/$(DEV_IMAGE); \
		$(CONTAINER_ENGINE) push $(REG)/$(DEV_IMAGE); \
	fi
	@if [ -f "$(MARKER_DIR)/final.built" ]; then \
		$(CONTAINER_ENGINE) tag $(FINAL_IMAGE) $(REG)/$(FINAL_IMAGE); \
		$(CONTAINER_ENGINE) push $(REG)/$(FINAL_IMAGE); \
	fi