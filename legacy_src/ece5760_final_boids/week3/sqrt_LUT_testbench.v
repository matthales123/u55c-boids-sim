`timescale 1ns/1ps

module sqrt_tb;

    // Internal signals
    reg clk;
    reg rst;
    reg [15:0] vx;
    reg [15:0] vy;
    wire [15:0] output_val;

    // Instantiate the DUT
    sqrt dut (
        .vx(vx),
        .vy(vy),
        .output_val(output_val)
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
        // Test case: vx = 9.0, vy = 9.0
        vx = 16'h48000;
        vy = 16'h48000;
        #10 $display("output_val = %d", output_val);

        // Test case: vx = 9.0, vy = 22.5
        vx = 16'h48000;
        vy = 16'hb4000;
        #10 $display("output_val = %d", output_val);

        // Test case: vx = 9.0, vy = 36.0
        vx = 16'h48000;
        vy = 16'h120000;
        #10 $display("output_val = %d", output_val);

        // Test case: vx = 22.5, vy = 9.0
        vx = 16'hb4000;
        vy = 16'h48000;
        #10 $display("output_val = %d", output_val);

        // Test case: vx = 22.5, vy = 22.5
        vx = 16'hb4000;
        vy = 16'hb4000;
        #10 $display("output_val = %d", output_val);

        // Test case: vx = 22.5, vy = 36.0
        vx = 16'hb4000;
        vy = 16'h120000;
        #10 $display("output_val = %d", output_val);

        // Test case: vx = 36.0, vy = 9.0
        vx = 16'h120000;
        vy = 16'h48000;
        #10 $display("output_val = %d", output_val);

        // Test case: vx = 36.0, vy = 22.5
        vx = 16'h120000;
        vy = 16'hb4000;
        #10 $display("output_val = %d", output_val);

        // Test case: vx = 36.0, vy = 36.0
        vx = 16'h120000;
        vy = 16'h120000;
        #10 $display("output_val = %d", output_val);

    end

endmodule
