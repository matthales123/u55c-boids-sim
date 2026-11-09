SHELL := /usr/bin/env bash

ROOT_DIR := $(abspath $(dir $(lastword $(MAKEFILE_LIST)))/../..)
TARGET ?= hw
PLATFORM ?= xilinx_u55c_gen3x16_xdma_3_202210_1
PART ?= xcu55c-fsvh2892-2L-e

KERNEL_NAME ?= boids_step
KERNEL_TOP ?= boids_step

BUILD_DIR ?= $(ROOT_DIR)/build/hardware/$(TARGET)
XO := $(BUILD_DIR)/$(KERNEL_NAME)_rtl.xo
XCLBIN := $(BUILD_DIR)/$(KERNEL_NAME)_rtl.xclbin
IP_DIR := $(BUILD_DIR)/$(KERNEL_NAME)_rtl_ip
KERNEL_XML := $(ROOT_DIR)/hardware/rtl_kernel/kernel.xml
PACKAGE_TCL := $(ROOT_DIR)/hardware/rtl_kernel/scripts/package_xo.tcl
CONNECTIVITY_CFG := $(BUILD_DIR)/$(KERNEL_NAME)_rtl_connectivity.cfg
LINK_HOOK_SCRIPT := $(ROOT_DIR)/hardware/hls/scripts/u55c_clk_metadata_adapter_workaround.tcl

ENABLE_EXPLICIT_HBM_MAP ?= 1
HBM_BANK_IN0 ?= HBM[0]
HBM_BANK_IN1 ?= HBM[1]
HBM_BANK_OUT ?= HBM[2]

LINK_HOOK_ARGS :=
LINK_CFG_ARGS :=

ifneq ($(wildcard $(LINK_HOOK_SCRIPT)),)
LINK_HOOK_ARGS += --linkhook.custom "postSysLink,$(LINK_HOOK_SCRIPT)"
endif

ifneq ($(ENABLE_EXPLICIT_HBM_MAP),0)
LINK_CFG_ARGS += --config $(CONNECTIVITY_CFG)
XCLBIN_DEPS := $(XO) $(CONNECTIVITY_CFG)
else
XCLBIN_DEPS := $(XO)
endif

ARTIFACT_DIR ?= $(HOME)/.local/share/fpga-dashboard-web/artifacts/xclbin
ARTIFACT_NAME ?= u55c_boids_sim_$(KERNEL_NAME)_rtl_$(TARGET).xclbin
ARTIFACT_PATH := $(ARTIFACT_DIR)/$(ARTIFACT_NAME)

.PHONY: all xo xclbin publish clean

all: xclbin

xo: $(XO)

xclbin: $(XCLBIN) publish

$(XO): $(PACKAGE_TCL) $(KERNEL_XML)
	@mkdir -p "$(BUILD_DIR)"
	@rm -f "$(XO)"
	vivado -mode batch -source "$(PACKAGE_TCL)" \
	  -tclargs "$(KERNEL_NAME)" "$(KERNEL_TOP)" "$(XO)" "$(KERNEL_XML)" "$(IP_DIR)" "$(PART)"

$(CONNECTIVITY_CFG):
	@mkdir -p "$(BUILD_DIR)"
	@{ \
	  echo "[connectivity]"; \
	  echo "sp=$(KERNEL_NAME)_1.state_mem:$(HBM_BANK_IN0)"; \
	  echo "sp=$(KERNEL_NAME)_1.vel_mem:$(HBM_BANK_IN1)"; \
	  echo "sp=$(KERNEL_NAME)_1.next_state_mem:$(HBM_BANK_OUT)"; \
	} > "$@"
	@echo "Generated connectivity map: $@"

$(XCLBIN): $(XCLBIN_DEPS)
	v++ -l -t $(TARGET) --platform $(PLATFORM) $(LINK_HOOK_ARGS) $(LINK_CFG_ARGS) "$(XO)" -o "$(XCLBIN)"

publish: $(XCLBIN)
	@mkdir -p "$(ARTIFACT_DIR)"
	cp -f "$(XCLBIN)" "$(ARTIFACT_PATH)"
	@echo "Published dashboard artifact: $(ARTIFACT_PATH)"

clean:
	rm -rf "$(IP_DIR)" "$(XO)" "$(XCLBIN)" "$(CONNECTIVITY_CFG)"
