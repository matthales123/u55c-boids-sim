`include "signed_mult.v"
`include "non_restoring_sqrt.v"
module ambv (
    input [26:0] in_a,
    input [26:0] in_b,
    output reg [26:0] result
);

reg [26:0] max_val;
reg [26:0] min_val;
reg [26:0] diff;
reg [26:0] sum;
reg [26:0] product;
reg [26:0] temp_result;

// Set alpha to 1
localparam [26:0] alpha = 27'b0000000000001000000000000000;

always @(*) begin
    // Find max and min of inputs
    if (in_a > in_b) begin
        max_val = in_a;
        min_val = in_b;
    end else begin
        max_val = in_b;
        min_val = in_a;
    end

    // Calculate difference (max - min)
    diff = max_val - min_val;

    // Calculate sum (alpha * max + beta * min) using shifting
    // alpha = 1, so alpha * max = max
    // beta = 1/4, so beta * min = min >> 2
    sum = max_val + (min_val >> 2);

    // Calculate product (sum * diff)
    product = sum * diff;

    // Calculate square root of the product
    // Note: The CORDIC algorithm can be used here for efficient computation
    // You can replace this with the appropriate code for a square root computation
    // For example, you can use a pre-built square root module or IP core
    // Instantiate the non_restoring_sqrt module

end

non_restoring_sqrt sqrt_inst (
    .square_distance(product),
    .sqrt_distance(temp_result)
);

assign result = temp_result;

endmodule


