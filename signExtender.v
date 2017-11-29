/**
 * Sign extend 17 to 32 bits
 */
module signExtender(a, sxa);

	input[16:0] a;
	output[31:0] sxa;

	assign sxa[16:0] = a;
	assign sxa[31:17] = a[16];
endmodule //signExtender