`timescale 1ns/1ps

module ambv_tb;

    // Internal signals
    reg clk;
    reg rst;
    reg [26:0] in_a;
    reg [26:0] in_b;
    wire [26:0] result;

    // Instantiate the DUT
    alpha_max_beta_min_shift dut (
        .in_a(in_a),
        .in_b(in_b),
        .result(result)
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
        // Test case: in_a = 3.0, in_b = 3.0
        in_a = 27'b0000000000001_100000000000000;
        in_b = 27'b0000000000001_100000000000000;
        #10 $display("result = %h", result);

        // Test case: in_a = 3.0, in_b = 4.5
        in_a = 27'b0000000000001_100000000000000;
        in_b = 27'b0000000000001_110000000000000;
        #10 $display("result = %h", result);

        // Test case: in_a = 3.0, in_b = 6.0
        in_a = 27'b0000000000001_100000000000000;
        in_b = 27'b0000000000001_110000000000000;
        #10 $display("result = %h", result);

        // Test case: in_a = 4.5, in_b = 3.0
        in_a = 27'b0000000000001_110000000000000;
        in_b = 27'b0000000000001_100000000000000;
        #10 $display("result = %h", result);

        // Test case: in_a = 4.5, in_b = 4.5
        in_a = 27'b0000000000001_110000000000000;
        in_b = 27'b0000000000001_110000000000000;
        #10 $display("result = %h", result);

        // Test case: in_a = 4.5, in_b = 6.0
        in_a = 27'b0000000000001_110000000000000;
        in_b = 27'b0000000000001_111000000000000;
        #10 $display("result = %h", result);

    end
endmodule
