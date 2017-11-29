module counter34(clock, reset, out);

	input clock, reset;
    output [5:0] out;
    reg [5:0] next;

    ecedffe dff0(.d(next[0]), .clk(clock), .q(out[0]), .clrn(~reset), .ena(1'b1));
    ecedffe dff1(.d(next[1]), .clk(clock), .q(out[1]), .clrn(~reset), .ena(1'b1));
    ecedffe dff2(.d(next[2]), .clk(clock), .q(out[2]), .clrn(~reset), .ena(1'b1));
    ecedffe dff3(.d(next[3]), .clk(clock), .q(out[3]), .clrn(~reset), .ena(1'b1));
    ecedffe dff4(.d(next[4]), .clk(clock), .q(out[4]), .clrn(~reset), .ena(1'b1));
    ecedffe dff5(.d(next[5]), .clk(clock), .q(out[5]), .clrn(~reset), .ena(1'b1));

    always@(*) begin
        casex({reset, out})
            6'b000000: next = 6'b000001;
            6'b000001: next = 6'b000010;
            6'b000010: next = 6'b000011;
            6'b000011: next = 6'b000100;
            6'b000100: next = 6'b000101;
            6'b000101: next = 6'b000110;
            6'b000110: next = 6'b000111;
            6'b000111: next = 6'b001000;
            6'b001000: next = 6'b001001;
            6'b001001: next = 6'b001010;
            6'b001010: next = 6'b001011;
            6'b001011: next = 6'b001100;
            6'b001100: next = 6'b001101;
            6'b001101: next = 6'b001110;
            6'b001110: next = 6'b001111;
            6'b001111: next = 6'b010000;
            6'b010000: next = 6'b010001;
            6'b010001: next = 6'b010010;
            6'b010010: next = 6'b010011;
            6'b010011: next = 6'b010100;
            6'b010100: next = 6'b010101;
            6'b010101: next = 6'b010110;
            6'b010110: next = 6'b010111;
            6'b010111: next = 6'b011000;
            6'b011000: next = 6'b011001;
            6'b011001: next = 6'b011010;
            6'b011010: next = 6'b011011;
            6'b011011: next = 6'b011100;
            6'b011100: next = 6'b011101;
            6'b011101: next = 6'b011110;
            6'b011110: next = 6'b011111;
            6'b011111: next = 6'b100000;
            6'b100000: next = 6'b100001;
            6'b100001: next = 6'b100001;
            default:   next = 6'b000000;
        endcase
    end
endmodule //counter34