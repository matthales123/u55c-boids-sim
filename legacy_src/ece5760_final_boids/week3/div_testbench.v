`timescale 1ns/1ps

module div_tb;

    // Internal signals
    reg clk;
    reg rst;
    reg [15:0] vx;
    reg [15:0] vy;
    wire [15:0] output;

    // Instantiate the DUT
    div dut (
        .clk(clk),
        .rst(rst),
        .vx(vx),
        .vy(vy),
        .output(output)
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
        // Test case: vx = 3.0, vy = 3.0
        vx = 16'h18000;
        vy = 16'h18000;
        #10 $display("output = %d", output);

        // Test case: vx = 3.0, vy = 4.5
        vx = 16'h18000;
        vy = 16'h24000;
        #10 $display("output = %d", output);

        // Test case: vx = 3.0, vy = 6.0
        vx = 16'h18000;
        vy = 16'h30000;
        #10 $display("output = %d", output);

        // Test case: vx = 4.5, vy = 3.0
        vx = 16'h24000;
        vy = 16'h18000;
        #10 $display("output = %d", output);

        // Test case: vx = 4.5, vy = 4.5
        vx = 16'h24000;
        vy = 16'h24000;
        #10 $display("output = %d", output);

        // Test case: vx = 4.5, vy = 6.0
        vx = 16'h24000;
        vy = 16'h30000;
        #10 $display("output = %d", output);

        // Test case: vx = 6.0, vy = 3.0
        vx = 16'h30000;
        vy = 16'h18000;
        #10 $display("output = %d", output);

        // Test case: vx = 6.0, vy = 4.5
        vx = 16'h30000;
        vy = 16'h24000;
        #10 $display("output = %d", output);

        // Test case: vx = 6.0, vy = 6.0
        vx = 16'h30000;
        vy = 16'h30000;
        #10 $display("output = %d", output);

    end

endmodule
