import math

def create_lookup_table(max_boids):
    lookup_table = {}
    for i in range(1, max_boids + 1):
        reciprocal = 1 / i
        lookup_table[i] = reciprocal
    return lookup_table

def format_verilog_binary(value, num_frac_bits, num_binary_digits):
    binary_value = bin(int(value * (2 ** num_frac_bits)))[2:].zfill(num_binary_digits)
    return f"{num_binary_digits}'b{binary_value}"

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
        f.write("input [26:0] neighboring_boids;\n")
        f.write("output [26:0] neighboring_boids_val;\n")
        f.write("always@(neighboring_boids)\n")
        f.write("begin\n")
        f.write("    case(neighboring_boids)\n")

        num_frac_bits = 15
        for key, value in lookup_table.items():
            verilog_key = format_verilog_binary(key, num_frac_bits, 27)
            verilog_value = format_verilog_binary(value, num_frac_bits, 27)
            f.write(f"        {verilog_key}: neighboring_boids_val = {verilog_value} ; // (1/{key}) or {value:.8f}\n")

        f.write("    endcase\n")
        f.write("end\n")
        f.write("endmodule\n")

    print(f"Lookup table saved to {filename}")

if __name__ == "__main__":
    main()
