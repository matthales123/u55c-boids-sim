import math

def create_sqrt_lookup_table(min_speed, max_speed, num_values):
    # Find the smallest power of 2 greater than or equal to num_values
    num_values_power_of_2 = 2 ** math.ceil(math.log2(num_values))
    
    num_vx_values = num_vy_values = int(math.sqrt(num_values_power_of_2))
    vx_step = (max_speed - min_speed) / (num_vx_values - 1)
    vy_step = (max_speed - min_speed) / (num_vy_values - 1)
    vx_values = [min_speed + vx_step * i for i in range(num_vx_values)]
    vy_values = [min_speed + vy_step * i for i in range(num_vy_values)]
    sqrt_lookup_table = {}
    for vx in vx_values:
        for vy in vy_values:
            temp_add = vx + vy
            sqrt_value = math.sqrt(temp_add)
            sqrt_lookup_table[(vx, vy)] = sqrt_value
    return sqrt_lookup_table, vx_step, vy_step


import math

def print_sqrt_lookup_table(sqrt_lookup_table, vx_step, vy_step):
    with open("sqrt_lookup_table.v", 'w') as f:

        f.write("`include \"signed_mult.v\"\n")
        f.write("module sqrt(\n")
        f.write("    input [26:0] vx,\n")
        f.write("    input [26:0] vy,\n")
        f.write("    output reg signed [26:0] output_val\n")
        f.write(");\n")

        f.write("reg [26:0] lut [0:{}];\n".format(len(sqrt_lookup_table) - 1))
        f.write("integer i;\n")
        f.write("initial begin\n")

        for i, ((vx, vy), sqrt_value) in enumerate(sqrt_lookup_table.items()):
            f.write(f"    lut[{i}] = 27'b{int(sqrt_value * 2**15):027b}; // (vx = {vx:.4f}, vy = {vy:.4f}, sqrt = {sqrt_value:.4f})\n")

        f.write("end\n")

        f.write("// return the closest approximation to the values in the LUT\n")

        f.write("reg [26:0] temp_idx;\n")
        f.write("reg [26:0] lower_index;\n")
        f.write("reg [26:0] upper_index;\n")
        f.write("reg [26:0] lower_value;\n")
        f.write("reg [26:0] upper_value;\n")
        f.write("wire [26:0] delta_v;\n")
        f.write("wire [26:0] delta_y;\n")
        f.write("wire [26:0] scaled_diff_temp;\n")
        f.write("wire [26:0] scaled_diff;\n")

        C = vx_step + vy_step
        P = 2 ** math.ceil(math.log2(C))
        log2_P = int(math.log2(P))

        f.write("always @(*) begin\n")
        f.write("    temp_idx = (vx + vy);\n")
        f.write("    lower_index = (temp_idx + (temp_idx >> 1) + (temp_idx >> 3)) >> 1;\n")
        f.write("    upper_index = lower_index + 1;\n")
        f.write("    lower_value = lut[lower_index];\n")
        f.write("    upper_value = lut[upper_index];\n")
        f.write("end\n")

        f.write("assign delta_v = (vx + vy) - ((lower_index << 1) + lower_index - (lower_index >> 2));\n")
        f.write("assign delta_y = upper_value - lower_value;\n")
        f.write("signed_mult mult1(\n")
        f.write("    .out(scaled_diff_temp),\n")
        f.write("    .a(delta_v),\n")
        f.write("    .b(delta_y)\n")
        f.write(");\n")
        f.write("assign scaled_diff = (scaled_diff_temp + (scaled_diff_temp >> 1) + (scaled_diff_temp >> 3)) >> 1;\n")
        f.write("always @(*) begin\n")
        f.write("    output_val = (lower_value >> 27) + (scaled_diff >> 27);\n")
        f.write("end\n")

        f.write("endmodule\n")


        
        
def generate_sqrt_testbench(min_value, max_value, num_vx_values, num_vy_values):
    with open("sqrt_LUT_testbench.v", 'w') as f:
        f.write("`timescale 1ns/1ps\n")
        f.write("\n")
        f.write("module sqrt_tb;\n")
        f.write("\n")
        f.write("    // Internal signals\n")
        f.write("    reg clk;\n")
        f.write("    reg rst;\n")
        f.write("    reg [26:0] vx;\n")
        f.write("    reg [26:0] vy;\n")
        f.write("    wire [26:0] output_val;\n")
        f.write("\n")
        f.write("    // Instantiate the DUT\n")
        f.write("    sqrt dut (\n")
        f.write("        .vx(vx),\n")
        f.write("        .vy(vy),\n")
        f.write("        .output_val(output_val)\n")
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
        f.write("        clk = 0;\n")

        for i in range(num_vx_values):
            for j in range(num_vy_values):
                f.write(f"        // Test case: vx = {min_value + (max_value - min_value) * i / (num_vx_values - 1)}, vy = {min_value + (max_value - min_value) * j / (num_vy_values - 1)}\n")
                f.write(f"        vx = 27'b{int((min_value + (max_value - min_value) * i / (num_vx_values - 1)) * 2**15):027b};\n")
                f.write(f"        vy = 27'b{int((min_value + (max_value - min_value) * j / (num_vy_values - 1)) * 2**15):027b};\n")
                f.write("        #10 $display(\"output_val = %d\", output_val);\n")
                f.write("\n")

        f.write("    end\n")
        f.write("\n")
        f.write("endmodule\n")






def main():
    min_speed = 9
    max_speed = 36
    num_values = 1024

    sqrt_lookup_table, vx_step, vy_step = create_sqrt_lookup_table(min_speed, max_speed, num_values)
    print_sqrt_lookup_table(sqrt_lookup_table, vx_step, vy_step)
    
    num_vx_values = 3
    num_vy_values = 3
    generate_sqrt_testbench(min_speed, max_speed, num_vx_values, num_vy_values)

if __name__ == "__main__":
    main()


if __name__ == "__main__":
    main()