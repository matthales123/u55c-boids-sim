import math

def create_lookup_table(max_boids):
    lookup_table = {}
    for i in range(1, max_boids + 1):
        reciprocal = 1 / i
        lookup_table[i] = reciprocal
    return lookup_table

def format_verilog_hex(value, num_frac_bits, num_hex_digits):
    hex_value = hex(int(value * (2 ** num_frac_bits)))[2:].zfill(num_hex_digits)
    return f"{num_hex_digits * 4}'h{hex_value}"

def main():
    max_boids = int(input("Enter the maximum number of boids: "))
    
    if max_boids < 1:
        max_boids = 1
    elif max_boids > 2000:
        max_boids = 2000
    
    lookup_table = create_lookup_table(max_boids)

    filename = "neighboring_Boids_table.v"
    with open(filename, 'w') as f:
        f.write("module neighboring_boids_LUT (neighboring_boids, neighboring_boids_val);\n")
        f.write("input [10:0] neighboring_boids;\n")
        f.write("output [26:0] neighboring_boids_val;\n")
        f.write("reg signed [26:0] neighboring_boids_val;\n")
        f.write("always@(*)\n")
        f.write("begin\n")
        f.write("    case(neighboring_boids)\n")

        num_frac_bits = math.ceil(math.log2(2000))
        for key, value in lookup_table.items():
            verilog_key = format_verilog_hex(key, num_frac_bits, 11 // 4)
            verilog_value = format_verilog_hex(value, num_frac_bits, 27 // 4)
            f.write(f"        {verilog_key}: neighboring_boids_val = {verilog_value} ; // (1/{key}) or {value:.6f}\n")

        f.write("    endcase\n")
        f.write("end\n")
        f.write("endmodule\n")

    print(f"Lookup table saved to {filename}")

if __name__ == "__main__":
    main()
