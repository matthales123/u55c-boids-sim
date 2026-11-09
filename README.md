# U55C Boids Simulation

Cornell ECE5760 boids port with HLS and RTL kernel paths, XRT host programs,
CPU reference/trace comparisons and a local web interface. Original source
attribution is retained.

## Build and run

```bash
./scripts/check_setup.sh
./scripts/build_host.sh
make -C hardware/hls xclbin TARGET=hw KERNEL_NAME=boids_step PLATFORM=xilinx_u55c_gen3x16_xdma_3_202210_1
./scripts/build_rtl_xclbin.sh
./scripts/run_webapp_boids.sh
```

Open `http://127.0.0.1:9893`. `make correctness` provides the existing correctness
workflow; hardware execution requires the matching image and runtime configuration.
Raw development results are omitted. This source export does not assert complete
UVM/formal verification or new board validation.

Original course project: https://github.com/matthales123/ece5760-boids

## Requirements

For FPGA builds, install AMD Vitis/Vivado 2025.1, XRT, and the compatible
`xilinx_u55c_gen3x16_xdma_3_202210_1` platform. Source your installation's
`settings64.sh` and XRT setup script before building. Python 3, a C++ compiler,
CMake and Make are required for the host/tooling. Run commands from this repository.
Keep the checkout path free of spaces for Vitis HLS.

The source includes host programs, HDL/HLS, tests and local web tooling. Generated
builds, debugging sessions, reports and results are excluded from this public copy.
Web applications are local development interfaces with hardware controls; public
Internet demo access is a separate future deployment.
