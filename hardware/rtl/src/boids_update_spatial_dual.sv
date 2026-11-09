`timescale 1ns/1ns

module boids_update_spatial_dual #(
    parameter NUM_OF_BOIDS = 10,
    parameter CELL_SHIFT   = 5
)(
    input  logic clk,
    input  logic reset,
    input  logic enable,
    input  logic [12:0] current_boids_num_lane0,
    input  logic [12:0] current_boids_num_lane1,
    input  logic [(NUM_OF_BOIDS<<2)-1:0] [26:0] boids_attr_array,
    output logic finish_lane0,
    output logic finish_lane1,
    output logic signed [26:0] pos_x_lane0,
    output logic signed [26:0] pos_y_lane0,
    output logic signed [26:0] vel_x_lane0,
    output logic signed [26:0] vel_y_lane0,
    output logic signed [26:0] pos_x_lane1,
    output logic signed [26:0] pos_y_lane1,
    output logic signed [26:0] vel_x_lane1,
    output logic signed [26:0] vel_y_lane1
);

    boids_update_spatial #(
        .NUM_OF_BOIDS(NUM_OF_BOIDS),
        .CELL_SHIFT(CELL_SHIFT)
    ) lane0 (
        .clk(clk),
        .reset(reset),
        .current_boids_num(current_boids_num_lane0),
        .boids_attr_array(boids_attr_array),
        .enable(enable),
        .finish(finish_lane0),
        .pos_x(pos_x_lane0),
        .pos_y(pos_y_lane0),
        .vel_x(vel_x_lane0),
        .vel_y(vel_y_lane0)
    );

    boids_update_spatial #(
        .NUM_OF_BOIDS(NUM_OF_BOIDS),
        .CELL_SHIFT(CELL_SHIFT)
    ) lane1 (
        .clk(clk),
        .reset(reset),
        .current_boids_num(current_boids_num_lane1),
        .boids_attr_array(boids_attr_array),
        .enable(enable),
        .finish(finish_lane1),
        .pos_x(pos_x_lane1),
        .pos_y(pos_y_lane1),
        .vel_x(vel_x_lane1),
        .vel_y(vel_y_lane1)
    );

endmodule
