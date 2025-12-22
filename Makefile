#/*
# * Copyright 2025 frodobots.ai. All rights reserved.
# */

IMAGE_NAME := robocap-calib
ROOT_DIR := $(shell pwd)
CODE_UTILS_SRC_DIR := $(ROOT_DIR)/third_party/code_utils
KALIBR_SRC_DIR := $(ROOT_DIR)/third_party/kalibr
CODE_UTILS_PATCH_FILE := $(ROOT_DIR)/patches/code_utils_20251217.patch
KALIBR_PATCH_FILE := $(ROOT_DIR)/patches/kalibr_20251222.patch

.PHONY: all patches docker_image docker_run clean help

all : patches docker_image
	@echo "All done!"

help:
	@echo "You do not need help!"

patches:
	cd $(CODE_UTILS_SRC_DIR) && git checkout . && git apply $(CODE_UTILS_PATCH_FILE)
	cd $(KALIBR_SRC_DIR) && git checkout . && git apply $(KALIBR_PATCH_FILE)
	@echo "Patch done!"

docker_image:
	docker build -t $(IMAGE_NAME) -f $(ROOT_DIR)/Dockerfile_ros1_20_04 .
	echo "Docker image build done!"

docker_run:
	@echo "Nothing to run!"

clean:
	@echo "Nothing needs cleaning!"

