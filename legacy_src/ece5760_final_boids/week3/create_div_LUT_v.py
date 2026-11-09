import math

def create_div_lookup_table(min_speed, max_speed, num_values):
    num_vx_values = num_vy_values = int(math.sqrt(num_values))
    vx_step = (max_speed - min_speed) / (num_vx_values - 1)
    vy_step = (max_speed - min_speed) / (num_vy_values - 1)
    vx_values = [min_speed + vx_step * i for i in range(num_vx_values)]
    vy_values = [min_speed + vy_step * i for i in range(num_vy_values)]
    div_lookup_table = {}
    for vx in vx_values:
        for vy in vy_values:
            div_value = vx / vy
            div_lookup_table[(vx, vy)] = div_value
    return div_lookup_table, vx_step, vy_step

def print_div_lookup_table(div_lookup_table, vx_step, vy_step):
    with open("div_lookup_table.v", 'w') as f:
        f.write("module div(\n")
        f.write("    input [15:0] vx,\n")
        f.write("    input [15:0] vy,\n")
        f.write("    output reg signed [15:0] output\n")
        f.write(");\n")
        
        f.write("reg [31:0] lut [0:{}];\n".format(len(div_lookup_table) - 1))
        f.write("integer i;\n")
        f.write("initial begin\n")
        
        for i, ((vx, vy), div_value) in enumerate(div_lookup_table.items()):
            f.write(f"    lut[{i}] = 32'h{int(div_value * 2**16):08x}; // (vx = {vx:.4f}, vy = {vy:.4f}, div = {div_value:.4f})\n")

        f.write("end\n")
        
        f.write("// return the closest approximation to the values in the LUT")

        f.write("reg [15:0] lower_index;\n")
        f.write("reg [15:0] upper_index;\n")
        f.write("reg [31:0] lower_value;\n")
        f.write("reg [31:0] upper_value;\n")
        f.write("wire [15:0] delta_v;\n")
        f.write("wire [31:0] delta_y;\n")
        f.write("wire [31:0] scaled_diff;\n")

        f.write("always @(*) begin\n")
        f.write("    lower_index = (vx + vy) / {};\n".format(vx_step + vy_step))
        f.write("    upper_index = lower_index + 1;\n")
        f.write("    lower_value = lut[lower_index];\n")
        f.write("    upper_value = lut[upper_index];\n")
        f.write("end\n")

        f.write("assign delta_v = (vx + vy) - (lower_index * {});\n".format(vx_step + vy_step))
        f.write("assign delta_y = upper_value - lower_value;\n")
        f.write("assign scaled_diff = (delta_v * delta_y) / {};\n".format(vx_step + vy_step))

        f.write("always @(*) begin\n")
        f.write("    output = (lower_value >> 16) + (scaled_diff >> 16);\n")
        f.write("end\n")

        f.write("endmodule\n")
        
        
def generate_div_testbench(min_speed, max_speed, num_vx_values, num_vy_values):
    with open("div_LUT_testbench.v", 'w') as f:
        f.write("`timescale 1ns/1ps\n")
        f.write("\n")
        f.write("module div_tb;\n")
        f.write("\n")
        f.write("    // Internal signals\n")
        f.write("    reg clk;\n")
        f.write("    reg rst;\n")
        f.write("    reg [15:0] vx;\n")
        f.write("    reg [15:0] vy;\n")
        f.write("    wire [15:0] output;\n")
        f.write("\n")
        f.write("    // Instantiate the DUT\n")
        f.write("    div dut (\n")
        f.write("        .vx(vx),\n")
        f.write("        .vy(vy),\n")
        f.write("        .output(output)\n")
        f.write("    );\n")
        f.write("\n")
        f.write("    // Clock generator\n")
        f.write("    always #1 clk = ~clk;\n")
        f.write("\n")
        f.write("    // Reset generator\n")
        f.write("    initial begin\n")
        f.write("        rst = 1;\n")
        f.write("        #3 rst = 0;\n")
        f.write("    end\n")
        f.write("\n")
        f.write("    // Test cases\n")
        f.write("    initial begin\n")
        
        for i in range(num_vx_values):
            for j in range(num_vy_values):
                f.write(f"        // Test case: vx = {min_speed + (max_speed - min_speed) * i / (num_vx_values - 1)}, vy = {min_speed + (max_speed - min_speed) * j / (num_vy_values - 1)}\n")
                f.write(f"        vx = 16'h{int((min_speed + (max_speed - min_speed) * i / (num_vx_values - 1)) * 2**15):04x};\n")
                f.write(f"        vy = 16'h{int((min_speed + (max_speed - min_speed) * j / (num_vy_values - 1)) * 2**15):04x};\n")
                f.write("        #10 $display(\"output = %d\", output);\n")
                f.write("\n")

        f.write("    end\n")
        f.write("\n")
        f.write("endmodule\n")




        
def main():
    min_speed = 3
    max_speed = 6
    num_values = 2000
    num_vx_values = 3
    num_vy_values = 3

    div_lookup_table, vx_step, vy_step = create_div_lookup_table(min_speed, max_speed, num_values)
    print_div_lookup_table(div_lookup_table, vx_step, vy_step)
    generate_div_testbench(min_speed, max_speed, num_vx_values, num_vy_values)

if __name__ == "__main__":
    main()
