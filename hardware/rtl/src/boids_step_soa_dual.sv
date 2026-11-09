`timescale 1ns/1ns

module boids_step_soa_dual #(
    parameter NUM_OF_BOIDS = 64,
    parameter CELL_SHIFT   = 5
)(
    input  logic clk,
    input  logic reset,
    input  logic start,
    input  logic [12:0] active_boids,

    // SoA state/velocity banks (HBM-friendly split layout).
    input  logic [NUM_OF_BOIDS-1:0] [26:0] in_pos_x,
    input  logic [NUM_OF_BOIDS-1:0] [26:0] in_pos_y,
    input  logic [NUM_OF_BOIDS-1:0] [26:0] in_vel_x,
    input  logic [NUM_OF_BOIDS-1:0] [26:0] in_vel_y,

    output logic done,
    output logic [NUM_OF_BOIDS-1:0] [26:0] out_pos_x,
    output logic [NUM_OF_BOIDS-1:0] [26:0] out_pos_y,
    output logic [NUM_OF_BOIDS-1:0] [26:0] out_vel_x,
    output logic [NUM_OF_BOIDS-1:0] [26:0] out_vel_y
);

    localparam logic [12:0] NUM_OF_BOIDS_13 = NUM_OF_BOIDS[12:0];

    logic [12:0] active_boids_use;
    logic [(NUM_OF_BOIDS<<2)-1:0] [26:0] boids_attr_array;

    logic [12:0] idx_base;
    logic [12:0] lane0_idx;
    logic [12:0] lane1_idx;
    logic lane1_valid;

    logic core_reset;
    logic core_enable;
    logic lane0_finish;
    logic lane1_finish;
    logic signed [26:0] lane0_pos_x;
    logic signed [26:0] lane0_pos_y;
    logic signed [26:0] lane0_vel_x;
    logic signed [26:0] lane0_vel_y;
    logic signed [26:0] lane1_pos_x;
    logic signed [26:0] lane1_pos_y;
    logic signed [26:0] lane1_vel_x;
    logic signed [26:0] lane1_vel_y;

    logic [2:0] state;
    localparam ST_IDLE    = 3'd0;
    localparam ST_RELEASE = 3'd1;
    localparam ST_WAIT    = 3'd2;
    localparam ST_CAPTURE = 3'd3;
    localparam ST_ADVANCE = 3'd4;
    localparam ST_DONE    = 3'd5;

    integer i_pack;
    integer i_reset;
    integer i_copy;

    always_comb begin
        if (active_boids > NUM_OF_BOIDS_13) begin
            active_boids_use = NUM_OF_BOIDS_13;
        end
        else begin
            active_boids_use = active_boids;
        end
    end

    always_comb begin
        for (i_pack = 0; i_pack < NUM_OF_BOIDS; i_pack = i_pack + 1) begin
            boids_attr_array[(i_pack << 2) + 0] = in_pos_x[i_pack];
            boids_attr_array[(i_pack << 2) + 1] = in_pos_y[i_pack];
            boids_attr_array[(i_pack << 2) + 2] = in_vel_x[i_pack];
            boids_attr_array[(i_pack << 2) + 3] = in_vel_y[i_pack];
        end
    end

    assign lane0_idx = idx_base;
    assign lane1_valid = ((idx_base + 13'd1) < active_boids_use);
    assign lane1_idx = lane1_valid ? (idx_base + 13'd1) : idx_base;

    boids_update_spatial_dual #(
        .NUM_OF_BOIDS(NUM_OF_BOIDS),
        .CELL_SHIFT(CELL_SHIFT)
    ) dual_core (
        .clk(clk),
        .reset(core_reset),
        .enable(core_enable),
        .current_boids_num_lane0(lane0_idx),
        .current_boids_num_lane1(lane1_idx),
        .boids_attr_array(boids_attr_array),
        .finish_lane0(lane0_finish),
        .finish_lane1(lane1_finish),
        .pos_x_lane0(lane0_pos_x),
        .pos_y_lane0(lane0_pos_y),
        .vel_x_lane0(lane0_vel_x),
        .vel_y_lane0(lane0_vel_y),
        .pos_x_lane1(lane1_pos_x),
        .pos_y_lane1(lane1_pos_y),
        .vel_x_lane1(lane1_vel_x),
        .vel_y_lane1(lane1_vel_y)
    );

    always_ff @(posedge clk) begin
        if (reset) begin
            state <= ST_IDLE;
            idx_base <= 13'd0;
            core_reset <= 1'b0;
            core_enable <= 1'b0;
            done <= 1'b0;
            for (i_reset = 0; i_reset < NUM_OF_BOIDS; i_reset = i_reset + 1) begin
                out_pos_x[i_reset] <= '0;
                out_pos_y[i_reset] <= '0;
                out_vel_x[i_reset] <= '0;
                out_vel_y[i_reset] <= '0;
            end
        end
        else begin
            case (state)
                ST_IDLE: begin
                    done <= 1'b0;
                    core_enable <= 1'b0;
                    core_reset <= 1'b0;
                    idx_base <= 13'd0;

                    if (start && (active_boids_use != 13'd0)) begin
                        for (i_copy = 0; i_copy < NUM_OF_BOIDS; i_copy = i_copy + 1) begin
                            out_pos_x[i_copy] <= in_pos_x[i_copy];
                            out_pos_y[i_copy] <= in_pos_y[i_copy];
                            out_vel_x[i_copy] <= in_vel_x[i_copy];
                            out_vel_y[i_copy] <= in_vel_y[i_copy];
                        end
                        core_reset <= 1'b1;
                        state <= ST_RELEASE;
                    end
                end

                ST_RELEASE: begin
                    core_reset <= 1'b0;
                    core_enable <= 1'b0;
                    state <= ST_WAIT;
                end

                ST_WAIT: begin
                    core_enable <= 1'b0;
                    if (lane0_finish && (!lane1_valid || lane1_finish)) begin
                        state <= ST_CAPTURE;
                    end
                end

                ST_CAPTURE: begin
                    out_pos_x[lane0_idx] <= lane0_pos_x;
                    out_pos_y[lane0_idx] <= lane0_pos_y;
                    out_vel_x[lane0_idx] <= lane0_vel_x;
                    out_vel_y[lane0_idx] <= lane0_vel_y;

                    if (lane1_valid) begin
                        out_pos_x[lane1_idx] <= lane1_pos_x;
                        out_pos_y[lane1_idx] <= lane1_pos_y;
                        out_vel_x[lane1_idx] <= lane1_vel_x;
                        out_vel_y[lane1_idx] <= lane1_vel_y;
                    end

                    if ((idx_base + 13'd2) >= active_boids_use) begin
                        state <= ST_DONE;
                    end
                    else begin
                        idx_base <= idx_base + 13'd2;
                        core_enable <= 1'b1;
                        state <= ST_ADVANCE;
                    end
                end

                ST_ADVANCE: begin
                    core_enable <= 1'b0;
                    state <= ST_WAIT;
                end

                ST_DONE: begin
                    done <= 1'b1;
                    core_enable <= 1'b0;
                    core_reset <= 1'b0;
                    if (!start) begin
                        state <= ST_IDLE;
                    end
                end

                default: begin
                    state <= ST_IDLE;
                end
            endcase
        end
    end

endmodule
