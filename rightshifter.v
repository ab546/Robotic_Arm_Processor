module rightshifter(a, shamt, out);

	input[31:0] a;
	input[4:0] shamt;

	output[31:0] out;

	wire[31:0] out16, out8, out4, out2, out1;
	wire[31:0] in8, in4, in2, in1;

	shift16r sixteen(.a(a), .out(out16));

	assign in8 = shamt[4] ? out16 : a;
	shift8r eight(.a(in8), .out(out8));

	assign in4 = shamt[3] ? out8 : in8;
	shift4r four(.a(in4), .out(out4));

	assign in2 = shamt[2] ? out4 : in4;
	shift2r two(.a(in2), .out(out2));

	assign in1 = shamt[1] ? out2 : in2;
	shift1r one(.a(in1), .out(out1));

	assign out = shamt[0] ? out1 : in1;
endmodule //leftshifter


module shift16r(a, out);

	input[31:0] a;
	output[31:0] out;

	assign out[15:0] = a[31:16];
	assign out[31:16] = a[31];
endmodule


module shift8r(a, out);

	input[31:0] a;
	output[31:0] out;

	assign out[23:0] = a[31:8];
	assign out[31:24] = a[31];
endmodule


module shift4r(a, out);

	input[31:0] a;
	output[31:0] out;

	assign out[27:0] = a[31:4];
	assign out[31:28] = a[31];
endmodule


module shift2r(a, out);

	input[31:0] a;
	output[31:0] out;

	assign out[29:0] = a[31:2];
	assign out[31:30] = a[31];
endmodule


module shift1r(a, out);

	input[31:0] a;
	output[31:0] out;

	assign out[30:0] = a[31:1];
	assign out[31] = a[31];
endmodule