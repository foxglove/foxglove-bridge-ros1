FOXGLOVE_BRIDGE_REMOTE_ACCESS := ON

.PHONY: docker-build
docker-build:
	docker build \
		--build-arg FOXGLOVE_BRIDGE_REMOTE_ACCESS=$(FOXGLOVE_BRIDGE_REMOTE_ACCESS) \
		-t foxglove-bridge-ros1 .

.PHONY: docker-test
docker-test: docker-build
	docker run --rm foxglove-bridge-ros1 bash -c "\
		cd /bridge_ws \
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
