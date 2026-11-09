`timescale 1ns/1ps

module non_restoring_sqrt_tb;

    // Internal signals
    reg clk;
    reg rst;
    reg [26:0] square_distance;
    wire [26:0] sqrt_distance;

    // Instantiate the DUT
    non_restoring_sqrt dut (
        .square_distance(square_distance),
        .sqrt_distance(sqrt_distance)
    );

    // Clock generator
    always #1 clk = ~clk;

    // Reset generator
    initial begin
        rst = 1;
        #3 rst = 0;
    end

    // Test cases
    initial begin

        clk = 0;

        // Test case: square_distance = 3.0^2 = 9.0
        square_distance = 27'b000000001001_000000000000000;
        #10 $display("sqrt_distance = %h", sqrt_distance);

        // Test case: square_distance = 4.5^2 = 20.25
        square_distance = 27'b000000010100_001000000000000;
        #10 $display("sqrt_distance = %h", sqrt_distance);

        // Test case: square_distance = 6.0^2 = 36.0
        square_distance = 27'b000000100100_000000000000000;
        #10 $display("sqrt_distance = %h", sqrt_distance);

    end

endmodule
