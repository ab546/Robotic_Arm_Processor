module equalityChecker(a, b, out);

	input[4:0] a, b;
	output out;

	wire[4:0] bits;

	xnor(bits[4], a[4], b[4]);
	xnor(bits[3], a[3], b[3]);
	xnor(bits[2], a[2], b[2]);
	xnor(bits[1], a[1], b[1]);
	xnor(bits[0], a[0], b[0]);

	and(out, bits[4], bits[3], bits[2], bits[1], bits[0]);
endmodule //equalityChecker;