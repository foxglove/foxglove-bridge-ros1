FOXGLOVE_BRIDGE_REMOTE_ACCESS := ON

# The slim runtime image (the Dockerfile's final stage) for deployment.
.PHONY: docker-build
docker-build:
	docker build \
		--build-arg FOXGLOVE_BRIDGE_REMOTE_ACCESS=$(FOXGLOVE_BRIDGE_REMOTE_ACCESS) \
		-t foxglove-bridge-ros1 .

# The full build environment (sources, build trees, compilers); the test
# targets run inside this stage.
.PHONY: docker-build-test-image
docker-build-test-image:
	docker build \
		--target bridge \
		--build-arg FOXGLOVE_BRIDGE_REMOTE_ACCESS=$(FOXGLOVE_BRIDGE_REMOTE_ACCESS) \
		-t foxglove-bridge-ros1-build .

.PHONY: docker-test
docker-test: docker-build-test-image
	docker run --rm foxglove-bridge-ros1-build bash -c "\
		cd /bridge_ws \
		&& catkin_make_isolated --install --install-space /opt/foxglove \
			--catkin-make-args run_tests \
		&& catkin_test_results build_isolated"

# Build and test against a locally-built SDK instead of the pinned release
# zip: point FOXGLOVE_CPP_SDK_DIR at a `cpp/dist` tree produced by
# `make build-cpp-dist` in a foxglove-sdk checkout. Reuses the prebuilt image
# (the slow Noetic-from-source stage) and mounts the SDK dist and the current
# working tree into it, rebuilding just the bridge inside the container. The
# workspace is wiped first so the image's CMake cache (which points at the
# downloaded release zip) can't leak into the local-SDK build.
.PHONY: docker-test-local-sdk
docker-test-local-sdk: docker-build-test-image
ifndef FOXGLOVE_CPP_SDK_DIR
	$(error FOXGLOVE_CPP_SDK_DIR must point at a cpp/dist tree built with `make build-cpp-dist` in a foxglove-sdk checkout)
endif
	docker run --rm \
		-v $(abspath $(FOXGLOVE_CPP_SDK_DIR)):/sdk/cpp/dist:ro \
		-v $(CURDIR):/bridge_ws/src/foxglove_bridge:ro \
		foxglove-bridge-ros1-build bash -c "\
		cd /bridge_ws \
		&& rm -rf build_isolated devel_isolated \
		&& catkin_make_isolated --install --install-space /opt/foxglove \
			--cmake-args \
			--no-warn-unused-cli \
			-DCMAKE_BUILD_TYPE=RelWithDebInfo \
			-DFOXGLOVE_BRIDGE_REMOTE_ACCESS=$(FOXGLOVE_BRIDGE_REMOTE_ACCESS) \
			-DFETCHCONTENT_SOURCE_DIR_FOXGLOVE_SDK=/sdk/cpp/dist \
		&& catkin_make_isolated --install --install-space /opt/foxglove \
			--catkin-make-args run_tests \
		&& catkin_test_results build_isolated"

.PHONY: lint
lint:
	find src include tests examples -name '*.cpp' -o -name '*.hpp' \
		| xargs clang-format --dry-run -Werror

.PHONY: fix
fix:
	find src include tests examples -name '*.cpp' -o -name '*.hpp' \
		| xargs clang-format -i
