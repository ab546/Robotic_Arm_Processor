module leftshifter(a, shamt, out);

	input[31:0] a;
	input[4:0] shamt;

	output[31:0] out;

	wire[31:0] out16, out8, out4, out2, out1;
	wire[31:0] in8, in4, in2, in1;

	shift16 sixteen(.a(a), .out(out16));

	assign in8 = shamt[4] ? out16 : a;
	shift8  eight(.a(in8), .out(out8));

	assign in4 = shamt[3] ? out8 : in8;
	shift4  four(.a(in4), .out(out4));

	assign in2 = shamt[2] ? out4 : in4;
	shift2  two(.a(in2), .out(out2));

	assign in1 = shamt[1] ? out2 : in2;
	shift1  one(.a(in1), .out(out1));

	assign out = shamt[0] ? out1 : in1;
endmodule //leftshifter


module shift16(a, out);

	input[31:0] a;
	output[31:0] out;

	assign out[15:0] = 16'b0;
	assign out[31:16] = a[15:0];
endmodule


module shift8(a, out);

	input[31:0] a;
	output[31:0] out;

	assign out[7:0] = 8'b0;
	assign out[31:8] = a[23:0];
endmodule


module shift4(a, out);

	input[31:0] a;
	output[31:0] out;

	assign out[3:0] = 4'b0;
	assign out[31:4] = a[27:0];
endmodule


module shift2(a, out);

	input[31:0] a;
	output[31:0] out;

	assign out[1:0] = 2'b0;
	assign out[31:2] = a[29:0];
endmodule


module shift1(a, out);

	input[31:0] a;
	output[31:0] out;

	assign out[0] = 1'b0;
	assign out[31:1] = a[30:0];
endmodule