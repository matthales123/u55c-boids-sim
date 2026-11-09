module boids_update#(parameter NUM_OF_BOIDS = 10)(
    input clk,
    input reset,
    input current_boids_num,
    input [NUM_OF_BOIDS<<2-1:0] [26:0] boids_attr_array,
    output [26:0]pos_x, //12 bit integer, 15 fraction
    output [26:0]pos_y,
    output [26:0]vel_x,
    output [26:0]vel_y
);

//addtional local vars
logic [7:0] counter;
logic [3:0] cur_state;
logic [3:0] next_state;

//localparams
localparam IDLE = 4'd0;
localparam BOIDS_NUM_CHECK = 4'd1;
localparam SPD_UPDATE = 4'd2;
localparam BOUND_CHK = 4'd3;
localparam POS_UPDATE = 4'd4;
localparam FINISH = 4'd5;

//? Define the turn factor for adjusting the boid's direction when near an edge
localparam turnfactor = {12'sd0, 15'sb001100110011001};//(0.2);
//? Define the visual range for boids to perceive other boids within this distance
localparam visualRange = {12'sd40, 15'sd0};//(40);
//? Define the protected range for boids to maintain a safe distance from other boids
localparam protectedRange = {12'sd8, 15'sd0};//(8);
//? Define the centering factor for how strongly boids try to match the position of other boids
localparam centeringfactor = {12'sd0, 15'sb000000000010000};//(0.0005);
//? Define the avoidance factor for how strongly boids try to avoid other boids within the protected range
localparam avoidfactor = {12'sd0, 12'sb000011001100110};//(0.05);
//? Define the matching factor for how strongly boids try to match the velocity of other boids
localparam matchingfactor = {12'sd0, 12'sb000011001100110};//(0.05);
//? Define the maximum speed a boid can move
localparam maxspeed = {12'sd6, 15'sd0};//(6);
//? Define the minimum speed a boid can move
localparam minspeed = {12'sd3, 15'sd0};//(3);
//? Define the maximum bias value to change boid's direction
localparam maxbias = {12'sd0, 15'sb000000101000111}//(0.01);
//? Define the increment value for bias adjustment
localparam bias_increment = {12'sd0, 15'sb000000000000001}//(0.00004);
//? Define the initial bias value for adjusting the boid's direction
localparam biasval = {12'sd0, 15'sb000000000100000};//(0.001);

localparam up_bound = {12'sd0, 15'sd0};  //0
localparam down_bound = {12'sd480, 15'sd0}; //480
localparam left_bound = {12'sd0, 15'sd0};  //0
localparam right_bound = {12'sd640, 15'sd0}; //640

logic[26:0] xpos_avg = 0;
logic[26:0] ypos_avg = 0;
logic[26:0] xvel_avg = 0; 
logic[26:0] yvel_avg = 0;  
logic[26:0] close_dx = 0; 
logic[26:0] close_dy = 0;
logic[26:0] neighboring_boids = 0;

logic[26:0] xpos_avg_updated;
logic[26:0] ypos_avg_updated;

logic[NUM_OF_BOIDS-1:0] [26:0] local_boids_attr_array,
logic signed [26:0] x;
logic signed [26:0] y;
logic signed [26:0] vx;
logic signed [26:0] vy;
logic signed [26:0] x_o ;
logic signed [26:0] y_o ;
logic signed [26:0] vx_o;
logic signed [26:0] vy_o;
logic signed [26:0] bound_chk_vx;
logic signed [26:0] bound_chk_vy;

// Compute differences in x and y coordinates
logic signed [26:0] dx;
logic signed [26:0] dy;
logic signed [26:0] dx_out;
logic signed [26:0] dy_out;
logic signed [26:0] protectedRange_out;
logic signed [26:0] visualRange_out;
logic signed [26:0] squared_distance;

assign dx = x - x_o;
assign dy = y - y_o;
assign squared_distance = dx_out + dy_out;

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
signed_mult protectedRange(
    .out(protectedRange_out),
    .a(protectedRange),
    .b(protectedRange)
);
signed_mult visualRange(
    .out(visualRange_out),
    .a(visualRange),
    .b(visualRange)
);

logic signed [26:0] close_dx_avoidfactor;
logic signed [26:0] close_dy_avoidfactor;
signed_mult close_dx_avoidfactor(
    .out(close_dx_avoidfactor),
    .a(close_dx),
    .b(avoidfactor)
);
signed_mult close_dy_avoidfactor(
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
    .neighboring_boids_val(neighboring_boids_val),
    .centeringfactor(centeringfactor),
    .matchingfactor(matchingfactor),
    .update_vx(update_vx),
    .update_vy(update_vy)
);

lookuptable_div LUT_div(
    .neighboring_boids(neighboring_boids),
    .neighboring_boids_val(neighboring_boids_val)
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

always@(posedge clk) begin
    if (reset) begin
        cur_state <= reset;
    end 
    else begin
        cur_state <= next_state;
    end
end

always@(posedge clk)begin
    case(curr_state)
        INIT:begin
            counter <= 8'd0;
            x <= boids_attr_array[current_boids_num<<2];
            y <= boids_attr_array[current_boids_num<<2]+1;
            vx <= boids_attr_array[current_boids_num<<2]+2;
            vy <=boids_attr_array[current_boids_num<<2]+3;
            next_state <= BOIDS_NUM_CHECK;
        end
        BOIDS_NUM_CHECK:begin
            if(counter != current_boids_num)begin
                x_o  <= boids_attr_array[counter << 2];
                y_o  <= boids_attr_array[counter << 2]+1;
                vx_o <= boids_attr_array[counter << 2]+2;
                vy_o <= boids_attr_array[counter << 2]+3;
                next_state <= CALC;
            end
            else begin
                counter <= counter + 1;
                next_state <= BOIDS_NUM_CHECK;
            end
        end
        CALC:begin
            if(dx < visualRange && dy < visualRange && dx > -visualRange && dy > -visualRange)begin

                // Is squared distance less than the protected range?
                if (squared_distance < protectedRange_out) { 

                    // If so, calculate difference in x/y-coordinates to nearfield boid
                    close_dx <= close_dx+dx;
                    close_dy <= close_dy+dy;
                }

                // If not in protected range, is the boid in the visual range?
                else if (squared_distance < visualRange_out) {

                    // Add other boid's x/y-coord and x/y vel to accumulator variables
                    xpos_avg <= xpos_avg+x_o;
                    ypos_avg <= ypos_avg+y_o;
                    xvel_avg <= xvel_avg+vx_o;
                    yvel_avg <= yvel_avg+vy_o;

                    // Increment number of boids within visual range
                    neighboring_boids <= neighboring_boids+1;
                }
            end
            if(counter == NUM_OF_BOIDS << 2 - 1)next_state <= SPD_UPDATE;
            else next_state <= BOIDS_NUM_CHECK;
        end
        SPD_UPDATE:begin
            // If there were any boids in the visual range . . .            
            if (neighboring_boids > 0) {
                vx <= update_vx + close_dx_avoidfactor;
                vy <= update_vy + close_dy_avoidfactor;
            } 
            else begin
                vx <= vx + close_dx_avoidfactor;
                vy <= vy + close_dy_avoidfactor;
            end
            next_state <= BOUND_CHK;
        end
        BOUND_CHK:begin
            //TODO: boundary_check module
            //bound_chk_vx
            //bound_chk_vy
            vx <= bound_chk_vx;
            vy <= bound_chk_vy;
            next_state <= POS_UPDATE;
        end
        POS_UPDATE:begin
            //TODO: sqrt module
            if(x + vx > right_bound)begin
                x <= x + vx - {12'sd15, 15'sd0};
                vx <= -vx;
            end
            else if (x + vx < left_bound)begin
                x <= x + vx + {12'sd15, 15'sd0};
                vx <= -vx;
            end
            else begin
                x <= x + vx;
                vx <= vx;
            end
            if(y + vy > down_bound)begin
                y <= y + vy - {12'sd15, 15'sd0};
                vy <= -vy;
            end
            else if(y + vy < up_bound)begin
                y <= y + vy + {12'sd15, 15'sd0};
                vy <= -vy;
            end
            else begin
                y <= y + vy;
                vy <= vy;
            end
            next_state <= FINISH;
        end
        FINISH:begin
            if(enable) next_state <= INIT;
            else next_state <= FINISH;
        end
    endcase
end

endmodule

