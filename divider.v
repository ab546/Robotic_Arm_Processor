module divider(data_operandA, data_operandB, clock, res, data_result, data_exception, data_resultRDY);

	input[31:0] data_operandA, data_operandB;
	input clock, res;

	output[31:0] data_result;
	output data_exception, data_resultRDY;

	/**
	* Non-restoring division
	* M always contains the divisor (data_operandB)
	* 
	* Q is initialized to dividend (data_operandA)
	* A is initialized to zero
	* Loop 32 times:
	*     if(A[31] == 0)
	*         A << 1; 
	*         A[0] = Q[31];
	*         Q << 1;
	*         A -= M;
	*     else
	*         A << 1; 
	*         A[0] = Q[31];
	*         Q << 1;
	*         A += M;
	*     end
	*     
	*     if(A[31] == 0)
	*         Q[0] = 1;
	*     else
	*         Q[0] = 0;
	*     end
	*     
	*    if(A[31] == 1)
	*         A += M;
	*    end
	* end
	*/

	wire[31:0] invdata_operandA, invdata_operandB;
	wire[31:0] negdata_operandA, negdata_operandB, negQuotient;
	wire[31:0] divisor, remainder, quotient;
	wire[31:0] dataA, dataQ;
	wire[31:0] midStageA, onExitA, updateQ, updateA;
	wire[5:0] count;
	
	wire initialize, storeAQ;
	wire errorStage1, errorExit;
	wire diffSign, negateQ;
	wire bIsZero, divideByZero;

	//Initialize Q to data_operandA and A to 0
	assign dataQ = initialize ? invdata_operandA : updateQ;
	assign dataA = initialize ? 32'b0         : updateA;

	//Remember M, A, and Q across cycles. Update based on the two stages at bottom
	register m(.data(invdata_operandB), .clock(clock), .enable(initialize), .reset(res), .out(divisor));
	register a(.data(dataA), .clock(clock), .enable(storeAQ), .reset(res), .out(remainder));
	register q(.data(dataQ), .clock(clock), .enable(storeAQ), .reset(res), .out(quotient));

	counter34 counter(.clock(clock), .reset(res), .out(count));

	//Only store multiplier on first cycle
	nor countZero(initialize, count[5], count[4], count[3], count[2], count[1], count[0]);
	
	//Result is ready after 32 cycles or if there is an issue, don't update after that
	and countHigh(data_resultRDY, count[5], ~count[4], ~count[3], ~count[2], ~count[1], count[0]);
	assign storeAQ = ~data_resultRDY;

	//Shift left, add/subtract M
	divisionOp stage1(remainder, divisor, quotient, midStageA, updateQ, errorStage1);

	//On exit, A += M if A < 0
	exitOp onExit(midStageA, divisor, onExitA, errorExit);
	assign updateA = data_resultRDY ? onExitA : midStageA;

	//Negate inputs if they are negative
	negater bNegater(data_operandB, negdata_operandB);
	assign invdata_operandA = data_operandA[31] ? negdata_operandA : data_operandA;

	negater aNegater(data_operandA, negdata_operandA);
	assign invdata_operandB = data_operandB[31] ? negdata_operandB : data_operandB;

	//Negate result if sign bits of inputs differ
	negater resultNegater(quotient, negQuotient);
	xor diffMSB(diffSign, data_operandA[31], data_operandB[31]);
	ecedffe negateResult(.d(diffSign), .clk(clock), .clrn(~res), .ena(initialize), .q(negateQ));
	assign data_result = negateQ ? negQuotient : quotient;

	//Handle divide-by-zero exception on first cycle
	equalZero zeroCheck(data_operandB, bIsZero);
	ecedffe divZero(.d(bIsZero), .clk(clock), .clrn(~res), .ena(initialize), .q(divideByZero));

	//Process overflow
	or overflow(data_exception, errorStage1, errorExit, divideByZero);

endmodule //divider


/*
 * First operation stage: shl, add/subtract M
 */
module divisionOp(a, m, q, newA, newQ, error);

	input[31:0] a, m, q;

	output[31:0] newA, newQ;
	output error;

	wire[31:0] shiftedA, filledA, shiftedQ;
	wire sub;
	wire gnd;

	assign shiftedA = a << 1;
	assign filledA[31:1] = shiftedA[31:1];
	assign filledA[0] = q[31];
	
	assign shiftedQ = q << 1;
	assign newQ[31:1] = shiftedQ[31:1];
	assign newQ[0] = ~newA[31];
	
	assign sub = ~filledA[31];
	cla opAdder(.a(filledA), .b(m), .sub(sub), .sum(newA), .ovf(error), .lessThan(gnd));

endmodule // divisionOp

/**
 * Exit stage: Add back M to A if A is negative
 */
module exitOp(a, m, newA, error);

	input[31:0] a, m;

	output[31:0] newA;
	output error;

	wire[31:0] aNegative;
	wire gnd;
	
	cla restoreAdd(.a(a), .b(m), .sub(1'b0), .sum(aNegative), .ovf(error), .lessThan(gnd));
	assign newA = a[31] ? aNegative : a;

endmodule //exitOp


module negater(a, aBar);

	input[31:0] a;
	output[31:0] aBar;
	wire gnd;

	cla negateAdder(.a(32'd0), .b(a), .sub(1'b1), .sum(aBar), .ovf(gnd), .lessThan(gnd));

endmodule //negater