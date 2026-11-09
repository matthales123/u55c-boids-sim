`timescale 1ns/1ns

module boids_step_soa_dual_tb #(
    parameter NUM_OF_BOIDS = 8
)();

    logic clk;
    logic reset;
    logic start;
    logic [12:0] active_boids;
    logic done;

    logic [NUM_OF_BOIDS-1:0] [26:0] in_pos_x;
    logic [NUM_OF_BOIDS-1:0] [26:0] in_pos_y;
    logic [NUM_OF_BOIDS-1:0] [26:0] in_vel_x;
    logic [NUM_OF_BOIDS-1:0] [26:0] in_vel_y;

    logic [NUM_OF_BOIDS-1:0] [26:0] out_pos_x;
    logic [NUM_OF_BOIDS-1:0] [26:0] out_pos_y;
    logic [NUM_OF_BOIDS-1:0] [26:0] out_vel_x;
    logic [NUM_OF_BOIDS-1:0] [26:0] out_vel_y;

    integer i;
    integer timeout_ctr;

    boids_step_soa_dual #(
        .NUM_OF_BOIDS(NUM_OF_BOIDS),
        .CELL_SHIFT(5)
    ) dut (
        .clk(clk),
        .reset(reset),
        .start(start),
        .active_boids(active_boids),
        .in_pos_x(in_pos_x),
        .in_pos_y(in_pos_y),
        .in_vel_x(in_vel_x),
        .in_vel_y(in_vel_y),
        .done(done),
        .out_pos_x(out_pos_x),
        .out_pos_y(out_pos_y),
        .out_vel_x(out_vel_x),
        .out_vel_y(out_vel_y)
    );

    always #5 clk = ~clk;

    initial begin
        clk = 1'b0;
        reset = 1'b1;
        start = 1'b0;
        active_boids = 13'd5;

        for (i = 0; i < NUM_OF_BOIDS; i = i + 1) begin
            in_pos_x[i] = 27'sd0;
            in_pos_y[i] = 27'sd0;
            in_vel_x[i] = 27'sd0;
            in_vel_y[i] = 27'sd0;
        end

        in_pos_x[0] = {12'sd200, 15'sd0};
        in_pos_y[0] = {12'sd200, 15'sd0};
        in_vel_x[0] = {12'sd4,   15'sd0};
        in_vel_y[0] = {12'sd4,   15'sd0};

        in_pos_x[1] = {12'sd208, 15'sd0};
        in_pos_y[1] = {12'sd205, 15'sd0};
        in_vel_x[1] = {12'sd4,   15'sd0};
        in_vel_y[1] = {12'sd4,   15'sd0};

        in_pos_x[2] = {12'sd210, 15'sd0};
        in_pos_y[2] = {12'sd212, 15'sd0};
        in_vel_x[2] = {12'sd4,   15'sd0};
        in_vel_y[2] = {12'sd4,   15'sd0};

        in_pos_x[3] = {12'sd300, 15'sd0};
        in_pos_y[3] = {12'sd100, 15'sd0};
        in_vel_x[3] = {12'sd3,   15'sd0};
        in_vel_y[3] = {12'sd2,   15'sd0};

        in_pos_x[4] = {12'sd150, 15'sd0};
        in_pos_y[4] = {12'sd260, 15'sd0};
        in_vel_x[4] = {12'sd2,   15'sd0};
        in_vel_y[4] = {12'sd3,   15'sd0};

        #30;
        reset = 1'b0;

        #20;
        start = 1'b1;
        #10;
        start = 1'b0;

        timeout_ctr = 0;
        while (!done && timeout_ctr < 2000) begin
            @(posedge clk);
            timeout_ctr = timeout_ctr + 1;
        end

        if (!done) begin
            $fatal(1, "Timed out waiting for done");
        end

        if (^out_pos_x[0] === 1'bx || ^out_pos_y[0] === 1'bx) begin
            $fatal(1, "Lane output contained X for active boid");
        end

        if (out_pos_x[7] !== in_pos_x[7] || out_pos_y[7] !== in_pos_y[7]) begin
            $fatal(1, "Inactive boids should pass through unchanged");
        end

        $display("boids_step_soa_dual_tb PASS");
        $finish;
    end

endmodule
