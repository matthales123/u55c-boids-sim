`timescale 1ns/1ns

module speed_update(
    input signed [26:0] xpos_avg,
    input signed [26:0] ypos_avg,
    input signed [26:0] xvel_avg,
    input signed [26:0] yvel_avg,
    input signed [26:0] x,
    input signed [26:0] y,
    input signed [26:0] ori_vx,
    input signed [26:0] ori_vy,
    input logic [26:0] neighboring_boids,
    input signed [26:0] centeringfactor,
    input signed [26:0] matchingfactor,
    output signed [26:0] update_vx,
    output signed [26:0] update_vy
);
    logic signed [26:0] xpos_avg_;
    logic signed [26:0] ypos_avg_;
    logic signed [26:0] xvel_avg_;
    logic signed [26:0] yvel_avg_;
    logic signed [26:0] neighboring_boids_signed;
    logic signed [26:0] mult1_out,mult2_out,mult3_out,mult4_out;

    assign neighboring_boids_signed = (neighboring_boids == 0) ? 27'sd1 : $signed(neighboring_boids);
    assign xpos_avg_ = (neighboring_boids == 0) ? 27'sd0 : (xpos_avg / neighboring_boids_signed);
    assign ypos_avg_ = (neighboring_boids == 0) ? 27'sd0 : (ypos_avg / neighboring_boids_signed);
    assign xvel_avg_ = (neighboring_boids == 0) ? 27'sd0 : (xvel_avg / neighboring_boids_signed);
    assign yvel_avg_ = (neighboring_boids == 0) ? 27'sd0 : (yvel_avg / neighboring_boids_signed);
    
    signed_mult mult1(
        .a(xpos_avg_ - x),
        .b(centeringfactor),
        .out(mult1_out)
    );
    signed_mult mult2(
        .a(xvel_avg_ - ori_vx),
        .b(matchingfactor),
        .out(mult2_out)
    );
    signed_mult mult3(
        .a(ypos_avg_ - y),
        .b(centeringfactor),
        .out(mult3_out)
    );
    signed_mult mult4(
        .a(yvel_avg_ - ori_vy),
        .b(matchingfactor),
        .out(mult4_out)
    );
    
    // Add the centering/matching contributions to velocity
    assign update_vx = ori_vx + mult1_out + mult2_out;                      
    assign update_vy = ori_vy + mult3_out + mult4_out;
endmodule
