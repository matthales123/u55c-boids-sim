sqr_rt_fix

module fixed_point_sqrt (
    input logic [26:0] value,
    output logic [26:0] sqrt_value
);

    logic [26:0] value_abs;
    logic value_sign;
    logic [53:0] temp;
    logic [26:0] q_temp;
    int i;

    // Convert signed fixed-point value to unsigned
    assign value_abs = value[26] ? -value : value;
    assign value_sign = value[26];

    // Check for negative input, return 0 if negative
    always_comb begin
        if (value_sign) begin
            sqrt_value = 27'b0;
        end else begin
            temp = {value_abs[26:0], 15'b0, 1'b0}; // Extend value_abs to 54 bits
            q_temp = 0;

            for (i = 53; i >= 0; i = i - 2) begin
                if (temp >= (q_temp << 1) + 2'b01) begin
                    temp = temp - ((q_temp << 1) + 2'b01);
                    q_temp = (q_temp << 1) + 1;
                end else begin
                    q_temp = q_temp << 1;
                end
            end

            sqrt_value = q_temp[26:0];
        end
    end

endmodule
