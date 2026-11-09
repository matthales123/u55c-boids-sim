`timescale 1ns/1ns

module speed_norm_approx(
    input  logic signed [26:0] vx,
    input  logic signed [26:0] vy,
    output logic signed [26:0] speed_approx
);

    logic signed [26:0] abs_vx;
    logic signed [26:0] abs_vy;
    logic signed [26:0] max_val;
    logic signed [26:0] min_val;

    always_comb begin
        abs_vx = vx[26] ? -vx : vx;
        abs_vy = vy[26] ? -vy : vy;

        if (abs_vx >= abs_vy) begin
            max_val = abs_vx;
            min_val = abs_vy;
        end
        else begin
            max_val = abs_vy;
            min_val = abs_vx;
        end

        // Alpha-max plus beta-min norm (alpha=1, beta=1/2).
        speed_approx = max_val + (min_val >>> 1);
    end

endmodule
