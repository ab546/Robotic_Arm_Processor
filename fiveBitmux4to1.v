module fiveBitmux4to1(in0, in1, in2, in3, sel, out);

	input[4:0] in0, in1, in2, in3;
	input[1:0] sel;

	output[4:0] out;

	assign out = ~sel[1] & ~sel[0] ? in0 : 5'bz;
	assign out = ~sel[1] &  sel[0] ? in1 : 5'bz;
	assign out =  sel[1] & ~sel[0] ? in2 : 5'bz;
	assign out =  sel[1] &  sel[0] ? in3 : 5'bz;

endmodule //mux4to1