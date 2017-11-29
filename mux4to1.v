module mux4to1(in0, in1, in2, in3, sel, out);

	input[31:0] in0, in1, in2, in3;
	input[1:0] sel;

	output[31:0] out;

	assign out = ~sel[1] & ~sel[0] ? in0 : 32'bz;
	assign out = ~sel[1] &  sel[0] ? in1 : 32'bz;
	assign out =  sel[1] & ~sel[0] ? in2 : 32'bz;
	assign out =  sel[1] &  sel[0] ? in3 : 32'bz;

endmodule //mux4to1