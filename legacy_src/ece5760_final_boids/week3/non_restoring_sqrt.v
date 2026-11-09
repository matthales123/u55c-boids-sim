`include "signed_mult.v"
module non_restoring_sqrt (
    input [26:0] square_distance,
    output reg [26:0] sqrt_distance
);

reg [26:0] low;
reg [26:0] high;
reg [26:0] mid;
reg [53:0] mid_squared;
integer i;

always @(*) begin
    low = 27'b000000000000_000000000000000;
    high = square_distance;
    sqrt_distance = 27'b000000000000_000000000000000;

    for (i = 0; i < 27; i = i + 1) begin
        mid = (low + high) >> 1;
        // mid_squared = mid * mid;
        signed_mult mult1(
            .out(mid_squared),
            .a(mid),
            .b(mid)
        );  

        if (mid_squared <= square_distance) begin
            low = mid;
            sqrt_distance = mid;
        end else begin
            high = mid;
        end
    end
end

endmodule
