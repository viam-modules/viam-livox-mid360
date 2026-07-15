SDK_DIR = third_party/Livox-SDK2
SDK_BUILD_DIR = $(SDK_DIR)/build
SDK_STATIC_LIB = $(SDK_BUILD_DIR)/sdk_core/liblivox_lidar_sdk_static.a
MODULE_BIN = livox-mid-360
TEST_BIN = cmd/test/smoketest

.PHONY: all sdk build module test clean

all: sdk build

sdk: $(SDK_STATIC_LIB)

$(SDK_STATIC_LIB):
	# Apply macOS compatibility patches (build-time only, not committed)
	./patches/apply_macos.sh $(SDK_DIR)
	mkdir -p $(SDK_BUILD_DIR)
	cd $(SDK_BUILD_DIR) && cmake .. -DCMAKE_POLICY_VERSION_MINIMUM=3.5
	cd $(SDK_BUILD_DIR) && make -j$$(sysctl -n hw.ncpu 2>/dev/null || nproc)
	# Restore patched files
	cd $(SDK_DIR) && mv sdk_core/CMakeLists.txt.bak sdk_core/CMakeLists.txt
	cd $(SDK_DIR) && mv sdk_core/device_manager.cpp.bak2 sdk_core/device_manager.cpp

build: sdk
	go build -o $(MODULE_BIN) .

module: build
	rm -f module.tar.gz
	tar -czf module.tar.gz $(MODULE_BIN) meta.json

test: sdk
	go build -o $(TEST_BIN) ./cmd/test/

clean:
	rm -rf $(SDK_BUILD_DIR) $(MODULE_BIN) $(TEST_BIN) module.tar.gz
