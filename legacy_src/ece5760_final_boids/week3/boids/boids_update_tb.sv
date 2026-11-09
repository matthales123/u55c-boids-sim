`timescale 1ns/1ns

module boids_update_tb#(parameter NUM_OF_BOIDS = 3)();
	
	reg clk, sys_reset, reset;
	reg enable;
	logic [(NUM_OF_BOIDS<<2) - 1 : 0][26:0]boids_array;
    logic [(NUM_OF_BOIDS<<2) - 1 : 0][26:0]boids_output_array;
    logic [26:0] counter;
    logic [3:0] curr_state;
    logic [3:0] next_state;
	logic finish;
    logic all_finish_flag;
    logic signed [26:0] pos_x,pos_y, vel_x, vel_y;
    
    localparam INIT = 4'd0;
    localparam CALC = 4'd1;
    localparam DONE = 4'd2;
    localparam ALL_FINISH = 4'd3;
    
	boids_update #(NUM_OF_BOIDS) DUT (
	    .clk(clk),
        .reset(reset),
        .current_boids_num(counter),
        .boids_attr_array(boids_array),
        .enable(enable),
        .finish(finish),
        .pos_x(pos_x), //12 bit integer, 15 fraction
        .pos_y(pos_y),
        .vel_x(vel_x),
        .vel_y(vel_y)
	);
	
	
	//Toggle the clocks
	always #10 clk  = ~clk;
	
    always@(posedge clk) begin
        if (sys_reset) begin
            curr_state <= INIT;
        end 
        else begin
            curr_state <= next_state;
        end
    end
    
    always@(posedge clk)begin
        case(curr_state)
            INIT:begin
                counter <= 0;
            end
            CALC:begin
            end
            DONE:begin
                counter <= counter + 1;
            end
            ALL_FINISH:begin
                counter <= 0;
            end
        endcase
    end
    
    always@(*)begin
        case(curr_state)
            INIT:begin
                next_state = CALC;
                all_finish_flag = 0;
                enable = 0;
                reset = 1;
            end
            CALC:begin
                enable = 0;
                reset = 0;
                all_finish_flag = 0;
                if(finish == 1 && counter == NUM_OF_BOIDS - 1)begin
                    next_state = ALL_FINISH;
                end
                else if (finish == 1)begin
                    next_state = DONE;
                end
                else begin
                    next_state = CALC;
                end
            end
            DONE:begin
                all_finish_flag = 0;
                enable = 1;
                reset = 0;
                next_state = CALC;
            end
            ALL_FINISH:begin
                enable = 0;
                reset = 0;
                all_finish_flag = 1;
                next_state = INIT;
            end
            default:begin
                all_finish_flag = 0;
                enable = 0;
                reset = 0;
            end
        endcase
    end
	
	//Toggle iteration_enable signal
	always begin
        enable = 1'b0;
		#20000
		enable = 1'b1;
		#20
		enable = 1'b0;
	end

	//Intialize and drive signals
	initial begin
        clk = 1'b0;
		sys_reset  = 1'b0;
		#10 
		sys_reset  = 1'b1;
		#30
		sys_reset  = 1'b0;
	end
    
    initial begin
        boids_array[0] = {12'sd200,15'sd0};
        boids_array[1] = {12'sd200,15'sd0};
        boids_array[2] = {12'sd4,15'sd0};
        boids_array[3] = {12'sd4,15'sd0};
        boids_array[4] = {12'sd208,15'sd0};
        boids_array[5] = {12'sd205,15'sd0};
        boids_array[6] = {12'sd4,15'sd0};
        boids_array[7] = {12'sd4,15'sd0};
        boids_array[8] = {12'sd210,15'sd0};
        boids_array[9] = {12'sd212,15'sd0};
        boids_array[10] = {12'sd4,15'sd0};
        boids_array[11] = {12'sd4,15'sd0};
    end
    
    always@(posedge clk)begin
        if(all_finish_flag == 1)begin
            boids_array[0] <= boids_output_array[0];
            boids_array[1] <= boids_output_array[1];
            boids_array[2] <= boids_output_array[2];
            boids_array[3] <= boids_output_array[3];
            boids_array[4] <= boids_output_array[4];
            boids_array[5] <= boids_output_array[5];
            boids_array[6] <= boids_output_array[6];
            boids_array[7] <= boids_output_array[7];
            boids_array[8] <= boids_output_array[8];
            boids_array[9] <= boids_output_array[9];
            boids_array[10] <= boids_output_array[10];
            boids_array[11] <= boids_output_array[11];
        end
        if(finish == 1)begin
            boids_output_array[(counter << 2)+0] <= pos_x;
            boids_output_array[(counter << 2)+1] <= pos_y;
            boids_output_array[(counter << 2)+2] <= vel_x;
            boids_output_array[(counter << 2)+3] <= vel_y;
        end
    end
	
	
endmodule