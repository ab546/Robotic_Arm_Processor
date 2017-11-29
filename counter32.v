module counter32(clock, reset, out);

    input clock, reset;
    output [4:0] out;
    reg [4:0] next;

    ecedffe dff0(.d(next[0]), .clk(clock), .q(out[0]), .clrn(~reset), .ena(1'b1));
    ecedffe dff1(.d(next[1]), .clk(clock), .q(out[1]), .clrn(~reset), .ena(1'b1));
    ecedffe dff2(.d(next[2]), .clk(clock), .q(out[2]), .clrn(~reset), .ena(1'b1));
    ecedffe dff3(.d(next[3]), .clk(clock), .q(out[3]), .clrn(~reset), .ena(1'b1));
    ecedffe dff4(.d(next[4]), .clk(clock), .q(out[4]), .clrn(~reset), .ena(1'b1));

    always@(*) begin
        casex({reset, out})
            5'b00000: next = 5'b00001;
            5'b00001: next = 5'b00010;
            5'b00010: next = 5'b00011;
            5'b00011: next = 5'b00100;
            5'b00100: next = 5'b00101;
            5'b00101: next = 5'b00110;
            5'b00110: next = 5'b00111;
            5'b00111: next = 5'b01000;
            5'b01000: next = 5'b01001;
            5'b01001: next = 5'b01010;
            5'b01010: next = 5'b01011;
            5'b01011: next = 5'b01100;
            5'b01100: next = 5'b01101;
            5'b01101: next = 5'b01110;
            5'b01110: next = 5'b01111;
            5'b01111: next = 5'b10000;
            5'b10000: next = 5'b10001;
            5'b10001: next = 5'b10010;
            5'b10010: next = 5'b10011;
            5'b10011: next = 5'b10100;
            5'b10100: next = 5'b10101;
            5'b10101: next = 5'b10110;
            5'b10110: next = 5'b10111;
            5'b10111: next = 5'b11000;
            5'b11000: next = 5'b11001;
            5'b11001: next = 5'b11010;
            5'b11010: next = 5'b11011;
            5'b11011: next = 5'b11100;
            5'b11100: next = 5'b11101;
            5'b11101: next = 5'b11110;
            5'b11110: next = 5'b11111;
            5'b11111: next = 5'b11111;
            default:  next = 5'b00000;
        endcase
    end
endmodule //counter32