module alu(data_operandA, data_operandB, ctrl_ALUopcode, ctrl_shiftamt, data_result, isNotEqual, isLessThan, overflow);

    input [31:0] data_operandA, data_operandB;
    input [4:0] ctrl_ALUopcode, ctrl_shiftamt;

    output [31:0] data_result;
    output isNotEqual, isLessThan, overflow;

    wire[31:0] andOut, orOut, adderOut, leftOut, rightOut, encOut;

    decoder outPicker(.regNum(ctrl_ALUopcode), .out(encOut));

    bitwiseAnd andMod(.a(data_operandA), .b(data_operandB), .out(andOut));
    bitwiseOr orMod(.a(data_operandA), .b(data_operandB), .out(orOut));
    cla adder(.a(data_operandA), .b(data_operandB), .sub(encOut[1]), 
    	.sum(adderOut), .ovf(overflow), .lessThan(isLessThan));
    leftshifter left(.a(data_operandA), .shamt(ctrl_shiftamt), .out(leftOut));
    rightshifter right(.a(data_operandA), .shamt(ctrl_shiftamt), .out(rightOut));

    assign data_result = encOut[0] ? adderOut : 32'bz;
    assign data_result = encOut[1] ? adderOut : 32'bz;
    assign data_result = encOut[2] ? andOut   : 32'bz;
    assign data_result = encOut[3] ? orOut    : 32'bz;
    assign data_result = encOut[4] ? leftOut  : 32'bz;
    assign data_result = encOut[5] ? rightOut : 32'bz;

    nequalityChecker neq(.a(data_operandA), .b(data_operandB), .notEqual(isNotEqual));
endmodule


module nequalityChecker(a, b, notEqual);

	input[31:0] a, b;
	output notEqual;

	wire[31:0] isEqual;
	wire[7:0] equality;
	wire[1:0] zeroCheck;
	wire subZero;

	//Check if sum == 0
	genvar k;
	generate
		for(k = 0; k < 32; k = k + 1) begin: equalCheck
			xnor checker(isEqual[k], a[k], b[k]);
		end
	endgenerate
	and equality0(equality[0], isEqual[0], isEqual[1], isEqual[2], isEqual[3]);
	and equality1(equality[1], isEqual[4], isEqual[5], isEqual[6], isEqual[7]);
	and equality2(equality[2], isEqual[8], isEqual[9], isEqual[10], isEqual[11]);
	and equality3(equality[3], isEqual[12], isEqual[13], isEqual[14], isEqual[15]);
	and equality4(equality[4], isEqual[16], isEqual[17], isEqual[18], isEqual[19]);
	and equality5(equality[5], isEqual[20], isEqual[21], isEqual[22], isEqual[23]);
	and equality6(equality[6], isEqual[24], isEqual[25], isEqual[26], isEqual[27]);
	and equality7(equality[7], isEqual[28], isEqual[29], isEqual[30], isEqual[31]);
	and equality8(zeroCheck[0], equality[0], equality[1], equality[2], equality[3]);
	and equality9(zeroCheck[1], equality[4], equality[5], equality[6], equality[7]);
	and equality10(subZero, zeroCheck[0], zeroCheck[1]);
	not equality11(notEqual, subZero);
endmodule


module bitwiseAnd(a, b, out);
	
	input [31:0] a, b;
	output [31:0] out;

	and and0( out[0],  a[0],  b[0]);
	and and1( out[1],  a[1],  b[1]);
	and and2( out[2],  a[2],  b[2]);
	and and3( out[3],  a[3],  b[3]);
	and and4( out[4],  a[4],  b[4]);
	and and5( out[5],  a[5],  b[5]);
	and and6( out[6],  a[6],  b[6]);
	and and7( out[7],  a[7],  b[7]);
	and and8( out[8],  a[8],  b[8]);
	and and9( out[9],  a[9],  b[9]);
	and and10(out[10], a[10], b[10]);
	and and11(out[11], a[11], b[11]);
	and and12(out[12], a[12], b[12]);
	and and13(out[13], a[13], b[13]);
	and and14(out[14], a[14], b[14]);
	and and15(out[15], a[15], b[15]);
	and and16(out[16], a[16], b[16]);
	and and17(out[17], a[17], b[17]);
	and and18(out[18], a[18], b[18]);
	and and19(out[19], a[19], b[19]);
	and and20(out[20], a[20], b[20]);
	and and21(out[21], a[21], b[21]);
	and and22(out[22], a[22], b[22]);
	and and23(out[23], a[23], b[23]);
	and and24(out[24], a[24], b[24]);
	and and25(out[25], a[25], b[25]);
	and and26(out[26], a[26], b[26]);
	and and27(out[27], a[27], b[27]);
	and and28(out[28], a[28], b[28]);
	and and29(out[29], a[29], b[29]);
	and and30(out[30], a[30], b[30]);
	and and31(out[31], a[31], b[31]);
endmodule


module bitwiseOr(a, b, out);
	input [31:0] a, b;
	output [31:0] out;

	or or0( out[0],  a[0],  b[0]);
	or or1( out[1],  a[1],  b[1]);
	or or2( out[2],  a[2],  b[2]);
	or or3( out[3],  a[3],  b[3]);
	or or4( out[4],  a[4],  b[4]);
	or or5( out[5],  a[5],  b[5]);
	or or6( out[6],  a[6],  b[6]);
	or or7( out[7],  a[7],  b[7]);
	or or8( out[8],  a[8],  b[8]);
	or or9( out[9],  a[9],  b[9]);
	or or10(out[10], a[10], b[10]);
	or or11(out[11], a[11], b[11]);
	or or12(out[12], a[12], b[12]);
	or or13(out[13], a[13], b[13]);
	or or14(out[14], a[14], b[14]);
	or or15(out[15], a[15], b[15]);
	or or16(out[16], a[16], b[16]);
	or or17(out[17], a[17], b[17]);
	or or18(out[18], a[18], b[18]);
	or or19(out[19], a[19], b[19]);
	or or20(out[20], a[20], b[20]);
	or or21(out[21], a[21], b[21]);
	or or22(out[22], a[22], b[22]);
	or or23(out[23], a[23], b[23]);
	or or24(out[24], a[24], b[24]);
	or or25(out[25], a[25], b[25]);
	or or26(out[26], a[26], b[26]);
	or or27(out[27], a[27], b[27]);
	or or28(out[28], a[28], b[28]);
	or or29(out[29], a[29], b[29]);
	or or30(out[30], a[30], b[30]);
	or or31(out[31], a[31], b[31]);
endmodule