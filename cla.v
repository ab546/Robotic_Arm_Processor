module cla(a, b, sub, sum, ovf, lessThan);

	input[31:0] a, b;
	input sub;

	output[31:0] sum;
	output ovf, lessThan;

	wire[1:0] gVals, pVals;
	wire carry, ovfPart;
	wire[31:0] bArg;
	
	assign bArg = sub ? ~b : b; //Negate b if a-b is desired

	//Logic for carry in to second block
	logic1 carry1(.gVals(gVals[0]), .pVals(pVals[0]), .cin(sub), .cout(carry));

	//block0 is bits 0-15, block1 is bits 16-31
	blockL1 block0(.a(a[15:0]), .b(bArg[15:0]), .cin(sub), .g(gVals[0]), 
		.p(pVals[0]), .s(sum[15:0]), .ovf(ovfPart));

	blockL1 block1(.a(a[31:16]), .b(bArg[31:16]), .cin(carry), .g(gVals[1]), 
		.p(pVals[1]), .s(sum[31:16]), .ovf(ovf));

	//Check if sum is negative (useful for subtraction)
	assign lessThan = ovf ? ~sum[31] : sum[31];

endmodule //cla


//One bit CLA
module oneBit(a, b, cin, s, g, p);
	
	input a, b, cin;
	output s, g, p;
	wire p0;

	xor(s, a, b, cin);    //Sum calculation

	and gOut(g, a, b);    //Carry Generate signal (ab)
	or  pIn(p0, a, b);
	and pOut(p, p0, cin); //Carry Propagate signal c(a+b)
endmodule //oneBit


//Logic for carry in to bit 1 (one over from LSB)
module logic1(gVals, pVals, cin, cout);

	input gVals, pVals, cin;

	output cout;

	wire pred1;

	and predAnd1(pred1, pVals, cin);
	or predOr1(cout, pred1, gVals);
endmodule //logic1


//Logic for carry in to bit 2
module logic2(gVals, pVals, cin, cout);

	input[1:0] gVals, pVals;
	input cin;

	output cout;

	wire[1:0] pred2;

	and predAnd2_0(pred2[0], pVals[1], gVals[0]);
	and predAnd2_1(pred2[1], pVals[1], pVals[0], cin);
	or predOr2(cout, gVals[1], pred2[0], pred2[1]);
endmodule //logic2


//Logic for carry in to bit 3
module logic3(gVals, pVals, cin, cout);

	input[2:0] gVals, pVals;
	input cin;

	output cout;

	wire[2:0] pred3;

	and predAnd3_0(pred3[0], pVals[2], gVals[1]);
	and predAnd3_1(pred3[1], pVals[2], pVals[1], gVals[0]);
	and predAnd3_2(pred3[2], pVals[2], pVals[1], pVals[0], cin);
	or predOr3(cout, gVals[2], pred3[0], pred3[1], pred3[2]);
endmodule //logic3


module logic4(gVals, pVals, cin, cout);

	input[3:0] gVals, pVals;
	input cin;

	output cout;

	wire[3:0] pred4;

	and predAnd4_0(pred4[0], pVals[3], gVals[2]);
	and predAnd4_1(pred4[1], pVals[3], pVals[2], gVals[1]);
	and predAnd4_2(pred4[2], pVals[3], pVals[2], pVals[1], gVals[0]);
	and predAnd4_3(pred4[3], pVals[3], pVals[2], pVals[1], pVals[0], cin);
	or predOr4(cout, gVals[3], pred4[0], pred4[1], pred4[2], pred4[3]);
endmodule //logic4


//4-bit CLA block
module blockL0(a, b, cin, g, p, s, ovf);
	
	input [3:0] a, b;
	input cin;

	output [3:0] s;
	output g, p, ovf;

	/* carry holds the carry in values for each bit
	 * gVals holds the carry generate values for each bit (determined by oneBitCLA)
	 * pVals holds the carry propagate values for each bit (determined by oneBitCLA)
	 */
	wire [3:0] carry, gVals, pVals;
	wire ovfPart; //Intermediate value for overflow calculation
	wire [2:0] gCalc; //Intermediate values for block-level generate signal
	
	//Determine carries with logic circuits
	assign carry[0] = cin;
	logic1 carry1(.gVals(gVals[0]),   .pVals(pVals[0]),   .cin(cin), .cout(carry[1]));
	logic2 carry2(.gVals(gVals[1:0]), .pVals(pVals[1:0]), .cin(cin), .cout(carry[2]));
	logic3 carry3(.gVals(gVals[2:0]), .pVals(pVals[2:0]), .cin(cin), .cout(carry[3]));
	logic4 carry4(.gVals(gVals[3:0]), .pVals(pVals[3:0]), .cin(cin), .cout(ovfPart));

	//Create the oneBitCLAs
	genvar k;
	generate
		for(k = 0; k < 4; k = k + 1) begin: oneCLALoop
		oneBit oneCLA(.a(a[k]), .b(b[k]), .cin(carry[k]),
			.s(s[k]), .g(gVals[k]), .p(pVals[k]));
		end
	endgenerate

	//Determine block-level generate (G) and propagate (P), as well as ovf
	and pOut(p, pVals[0], pVals[1], pVals[2], pVals[3]);

	and gOut0(gCalc[0], pVals[3], gVals[2]);
	and gOut1(gCalc[1], pVals[3], pVals[2], gVals[1]);
	and gOut2(gCalc[2], pVals[3], pVals[2], pVals[1], gVals[0]);
	or gOut3(g, gVals[3], gCalc[0], gCalc[1], gCalc[2]);

	xor(ovf, carry[3], ovfPart);
endmodule //blockL0


//16-bit CLA block 
module blockL1(a, b, cin, g, p, s, ovf);

	input [15:0] a, b;
	input cin;

	output [15:0] s;
	output g, p, ovf;

	/* carry holds the carry in values for each 4-bit block
	 * gVals holds the generate values for each 4-bit block
	 * pVals holds the propagate values for each 4-bit block
	 */
	wire[3:0] gVals, pVals, carry, overflows;
	wire[2:0] gCalc; //Intermediate values for block-level generate signal

	//Logic circuits for carry in to each 4-bit block
	assign carry[0] = cin;
	logic1 carry1(.gVals(gVals[0]),   .pVals(pVals[0]),   .cin(cin), .cout(carry[1]));
	logic2 carry2(.gVals(gVals[1:0]), .pVals(pVals[1:0]), .cin(cin), .cout(carry[2]));
	logic3 carry3(.gVals(gVals[2:0]), .pVals(pVals[2:0]), .cin(cin), .cout(carry[3]));

	//Create the 4-bit CLA blocks
	genvar i;
	generate
		for(i = 0; i < 4; i = i + 1) begin: claLoop
			blockL0 block(.a(a[4*(i+1)-1 : 4*i]) , .b(b[4*(i+1)-1 : 4*i]), 
				.cin(carry[i]), .g(gVals[i]), .p(pVals[i]), 
				.s(s[4*(i+1)-1 : 4*i]), .ovf(overflows[i]));
		end
	endgenerate

	//Determine 16-bit block generate and propagate signals, as well as overflow
	and pOut(p, pVals[0], pVals[1], pVals[2], pVals[3]);

	and gOut0(gCalc[0], pVals[3], gVals[2]);
	and gOut1(gCalc[1], pVals[3], pVals[2], gVals[1]);
	and gOut2(gCalc[2], pVals[3], pVals[2], pVals[1], gVals[0]);
	or gOut3(g, gVals[3], gCalc[0], gCalc[1], gCalc[2]);

	assign ovf = overflows[3];
endmodule //blockL1