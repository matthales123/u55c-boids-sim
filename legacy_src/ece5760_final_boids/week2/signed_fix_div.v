signed_fix_div

module signed_fix_div (
    input wire [26:0] dividend,
    input wire [26:0] divisor,
    output wire [26:0] quotient
);

    wire [26:0] dividend_abs;
    wire [26:0] divisor_abs;
    wire [26:0] quotient_unsigned;
    wire dividend_sign;
    wire divisor_sign;
    wire quotient_sign;

    // Convert signed fixed-point values to unsigned
    assign dividend_abs = dividend[26] ? ~dividend + 27'b1 : dividend;
    assign divisor_abs = divisor[26] ? ~divisor + 27'b1 : divisor;
    assign dividend_sign = dividend[26];
    assign divisor_sign = divisor[26];

    // Unsigned fixed-point divider
    reg [42:0] temp;
    reg [26:0] q_temp;
    integer i;

    always @(*) begin
        temp = {dividend_abs[26:0], 16'b0};
        q_temp = 0;

        for (i = 26; i >= 0; i = i - 1) begin
            temp = temp << 1;
            temp[42:17] = temp[42:17] - divisor_abs;

            if (temp[42]) begin
                temp[42:17] = temp[42:17] + divisor_abs;
                q_temp[i] = 0;
            end
            else begin
                q_temp[i] = 1;
            end
        end

        quotient_unsigned = {q_temp[11:0], q_temp[26:12], 16'b0};
    end

    // Determine the sign of the quotient
    assign quotient_sign = dividend_sign ^ divisor_sign;

    // Convert the unsigned fixed-point quotient back to signed
    assign quotient = quotient_sign ? ~quotient_unsigned + 27'b1 : quotient_unsigned;

endmodule
