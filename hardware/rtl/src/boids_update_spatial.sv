`timescale 1ns/1ns

module boids_update_spatial #(
    parameter NUM_OF_BOIDS = 10,
    parameter CELL_SHIFT   = 5
)(
    input  logic clk,
    input  logic reset,
    input  logic [12:0] current_boids_num,
    input  logic [(NUM_OF_BOIDS<<2)-1:0] [26:0] boids_attr_array,
    input  logic enable,
    output logic finish,
    output logic signed [26:0] pos_x,
    output logic signed [26:0] pos_y,
    output logic signed [26:0] vel_x,
    output logic signed [26:0] vel_y
);

    localparam COUNTER_W = (NUM_OF_BOIDS <= 1) ? 1 : $clog2(NUM_OF_BOIDS);

    logic [COUNTER_W-1:0] counter;
    logic [3:0] curr_state;
    logic [3:0] next_state;

    localparam INIT            = 4'd0;
    localparam BOIDS_NUM_CHECK = 4'd1;
    localparam CALC            = 4'd2;
    localparam SPD_UPDATE      = 4'd3;
    localparam BOUND_CHK       = 4'd4;
    localparam POS_UPDATE      = 4'd5;
    localparam FINISH          = 4'd6;

    // 12 integer + 15 fraction fixed-point constants.
    localparam signed [26:0] turnfactor      = {12'sd0,   15'sb001100110011001}; // 0.2
    localparam signed [26:0] visualRange     = {12'sd75,  15'sd0};                // 75
    localparam signed [26:0] protectedRange  = {12'sd8,   15'sd0};                // 8
    localparam signed [26:0] centeringfactor = {12'sd0,   15'sb000000000010000};  // 0.0005
    localparam signed [26:0] avoidfactor     = {12'sd0,   15'sb000011001100110};  // 0.05
    localparam signed [26:0] matchingfactor  = {12'sd0,   15'sb000011001100110};  // 0.05
    localparam signed [26:0] maxspeed        = {12'sd6,   15'sd0};                // 6
    localparam signed [26:0] minspeed        = {12'sd3,   15'sd0};                // 3

    localparam signed [26:0] up_bound    = {12'sd0,   15'sd0}; // 0
    localparam signed [26:0] down_bound  = {12'sd480, 15'sd0}; // 480
    localparam signed [26:0] left_bound  = {12'sd0,   15'sd0}; // 0
    localparam signed [26:0] right_bound = {12'sd640, 15'sd0}; // 640

    logic signed [26:0] xpos_avg;
    logic signed [26:0] ypos_avg;
    logic signed [26:0] xvel_avg;
    logic signed [26:0] yvel_avg;
    logic signed [26:0] close_dx;
    logic signed [26:0] close_dy;
    logic [26:0] neighboring_boids;

    logic signed [26:0] x;
    logic signed [26:0] y;
    logic signed [26:0] vx;
    logic signed [26:0] vy;
    logic signed [26:0] x_o;
    logic signed [26:0] y_o;
    logic signed [26:0] vx_o;
    logic signed [26:0] vy_o;

    logic signed [26:0] bound_chk_vx;
    logic signed [26:0] bound_chk_vy;
    logic signed [26:0] update_vx;
    logic signed [26:0] update_vy;

    logic signed [26:0] dx;
    logic signed [26:0] dy;
    logic signed [26:0] dx_out;
    logic signed [26:0] dy_out;
    logic signed [26:0] protectedRange_out;
    logic signed [26:0] visualRange_out;
    logic signed [26:0] squared_distance;
    logic signed [26:0] neg_visualRange;
    logic signed [26:0] speed_approx;

    logic signed [26:0] x_vx;
    logic signed [26:0] y_vy;

    logic signed [11:0] self_x_int;
    logic signed [11:0] self_y_int;
    logic signed [11:0] other_x_int;
    logic signed [11:0] other_y_int;
    logic signed [11:0] self_cell_x;
    logic signed [11:0] self_cell_y;
    logic signed [11:0] other_cell_x;
    logic signed [11:0] other_cell_y;
    logic signed [11:0] cell_dx;
    logic signed [11:0] cell_dy;
    logic cell_near;

    assign x_vx = x + vx;
    assign y_vy = y + vy;

    assign dx = x - x_o;
    assign dy = y - y_o;
    assign squared_distance = dx_out + dy_out;
    assign neg_visualRange = -visualRange;

    assign self_x_int  = $signed(x[26:15]);
    assign self_y_int  = $signed(y[26:15]);
    assign other_x_int = $signed(x_o[26:15]);
    assign other_y_int = $signed(y_o[26:15]);

    assign self_cell_x  = self_x_int >>> CELL_SHIFT;
    assign self_cell_y  = self_y_int >>> CELL_SHIFT;
    assign other_cell_x = other_x_int >>> CELL_SHIFT;
    assign other_cell_y = other_y_int >>> CELL_SHIFT;

    assign cell_dx = self_cell_x - other_cell_x;
    assign cell_dy = self_cell_y - other_cell_y;
    assign cell_near = (cell_dx <= 12'sd1) && (cell_dx >= -12'sd1) &&
                       (cell_dy <= 12'sd1) && (cell_dy >= -12'sd1);

    signed_mult square_x(
        .out(dx_out),
        .a(dx),
        .b(dx)
    );

    signed_mult square_y(
        .out(dy_out),
        .a(dy),
        .b(dy)
    );

    signed_mult protectedRange_1(
        .out(protectedRange_out),
        .a(protectedRange),
        .b(protectedRange)
    );

    signed_mult visualRange_1(
        .out(visualRange_out),
        .a(visualRange),
        .b(visualRange)
    );

    logic signed [26:0] close_dx_avoidfactor;
    logic signed [26:0] close_dy_avoidfactor;

    signed_mult close_dx_avoidfactor_1(
        .out(close_dx_avoidfactor),
        .a(close_dx),
        .b(avoidfactor)
    );

    signed_mult close_dy_avoidfactor_1(
        .out(close_dy_avoidfactor),
        .a(close_dy),
        .b(avoidfactor)
    );

    speed_update spd_update(
        .xpos_avg(xpos_avg),
        .ypos_avg(ypos_avg),
        .xvel_avg(xvel_avg),
        .yvel_avg(yvel_avg),
        .x(x),
        .y(y),
        .ori_vx(vx),
        .ori_vy(vy),
        .neighboring_boids(neighboring_boids),
        .centeringfactor(centeringfactor),
        .matchingfactor(matchingfactor),
        .update_vx(update_vx),
        .update_vy(update_vy)
    );

    bound_check bnd_chk(
        .x(x),
        .y(y),
        .turnfactor(turnfactor),
        .vx(vx),
        .vy(vy),
        .vx_chk(bound_chk_vx),
        .vy_chk(bound_chk_vy)
    );

    speed_norm_approx spd_norm(
        .vx(vx),
        .vy(vy),
        .speed_approx(speed_approx)
    );

    always_ff @(posedge clk) begin
        if (reset) begin
            curr_state <= INIT;
        end
        else begin
            curr_state <= next_state;
        end
    end

    always_ff @(posedge clk) begin
        case (curr_state)
            INIT: begin
                counter <= '0;
                close_dx <= 27'sd0;
                close_dy <= 27'sd0;
                xpos_avg <= 27'sd0;
                ypos_avg <= 27'sd0;
                xvel_avg <= 27'sd0;
                yvel_avg <= 27'sd0;
                neighboring_boids <= 27'd0;

                x <= boids_attr_array[current_boids_num<<2];
                y <= boids_attr_array[(current_boids_num<<2) + 1];
                vx <= boids_attr_array[(current_boids_num<<2) + 2];
                vy <= boids_attr_array[(current_boids_num<<2) + 3];
            end

            BOIDS_NUM_CHECK: begin
                if (counter != current_boids_num) begin
                    x_o <= boids_attr_array[(counter << 2)];
                    y_o <= boids_attr_array[(counter << 2) + 1];
                    vx_o <= boids_attr_array[(counter << 2) + 2];
                    vy_o <= boids_attr_array[(counter << 2) + 3];
                end
                else begin
                    counter <= counter + 1'b1;
                end
            end

            CALC: begin
                if (cell_near &&
                    (dx < visualRange) && (dy < visualRange) &&
                    (dx > neg_visualRange) && (dy > neg_visualRange)) begin

                    if (squared_distance < protectedRange_out) begin
                        close_dx <= close_dx + dx;
                        close_dy <= close_dy + dy;
                    end
                    else if (squared_distance < visualRange_out) begin
                        xpos_avg <= xpos_avg + x_o;
                        ypos_avg <= ypos_avg + y_o;
                        xvel_avg <= xvel_avg + vx_o;
                        yvel_avg <= yvel_avg + vy_o;
                        neighboring_boids <= neighboring_boids + 1'b1;
                    end
                end

                if (counter != (NUM_OF_BOIDS - 1)) begin
                    counter <= counter + 1'b1;
                end
            end

            SPD_UPDATE: begin
                if (neighboring_boids > 0) begin
                    vx <= update_vx + close_dx_avoidfactor;
                    vy <= update_vy + close_dy_avoidfactor;
                end
                else begin
                    vx <= vx + close_dx_avoidfactor;
                    vy <= vy + close_dy_avoidfactor;
                end
            end

            BOUND_CHK: begin
                if (speed_approx < minspeed) begin
                    vx <= vx[26] ? -minspeed : minspeed;
                    vy <= vy[26] ? -minspeed : minspeed;
                end
                else if (speed_approx > maxspeed) begin
                    vx <= vx[26] ? -maxspeed : maxspeed;
                    vy <= vy[26] ? -maxspeed : maxspeed;
                end
                else begin
                    vx <= bound_chk_vx;
                    vy <= bound_chk_vy;
                end
            end

            POS_UPDATE: begin
                if (x_vx > right_bound) begin
                    x <= x + vx - {12'sd15, 15'sd0};
                    vx <= -vx;
                end
                else if (x_vx < left_bound) begin
                    x <= x + vx + {12'sd15, 15'sd0};
                    vx <= -vx;
                end
                else begin
                    x <= x + vx;
                end

                if (y_vy > down_bound) begin
                    y <= y + vy - {12'sd15, 15'sd0};
                    vy <= -vy;
                end
                else if (y_vy < up_bound) begin
                    y <= y + vy + {12'sd15, 15'sd0};
                    vy <= -vy;
                end
                else begin
                    y <= y + vy;
                end
            end

            FINISH: begin
                pos_x <= x;
                pos_y <= y;
                vel_x <= vx;
                vel_y <= vy;
                counter <= '0;
            end

            default: begin
            end
        endcase
    end

    always_comb begin
        next_state = curr_state;
        case (curr_state)
            INIT: begin
                next_state = BOIDS_NUM_CHECK;
            end

            BOIDS_NUM_CHECK: begin
                if (counter == (NUM_OF_BOIDS - 1)) begin
                    next_state = SPD_UPDATE;
                end
                else if (counter != current_boids_num) begin
                    next_state = CALC;
                end
                else begin
                    next_state = BOIDS_NUM_CHECK;
                end
            end

            CALC: begin
                if (counter == (NUM_OF_BOIDS - 1)) begin
                    next_state = SPD_UPDATE;
                end
                else begin
                    next_state = BOIDS_NUM_CHECK;
                end
            end

            SPD_UPDATE: begin
                next_state = BOUND_CHK;
            end

            BOUND_CHK: begin
                next_state = POS_UPDATE;
            end

            POS_UPDATE: begin
                next_state = FINISH;
            end

            FINISH: begin
                if (enable) begin
                    next_state = INIT;
                end
                else begin
                    next_state = FINISH;
                end
            end

            default: begin
                next_state = curr_state;
            end
        endcase
    end

    always_comb begin
        finish = (curr_state == FINISH);
    end

endmodule
