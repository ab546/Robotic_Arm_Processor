module multdiv(data_operandA, data_operandB, ctrl_MULT, ctrl_DIV, clock, data_result, data_exception, data_resultRDY);
    
    input[31:0] data_operandA, data_operandB;
    input ctrl_MULT, ctrl_DIV, clock;

    output[31:0] data_result;
    output data_exception, data_resultRDY;

    wire[31:0] multiplyResult, divideResult; 
    wire res, checkerA, checkerB, isMult;
    wire multiplyException, divideException;
    wire multiplyRDY, divideRDY;

	/* A is the multiplicand, B is the multiplier, 
     * P is product
	 * Start at 2 LSBs of B + implicit 0
	 *
	 * Loop 16 times:
	 *     Examine grouping of 2 bits in B + 1 bit to the right:
	 *		   000: do nothing
	 *		   001: P +=  A
	 *		   010: P +=  A
	 *		   011: P += (A << 1)
	 *		   100: P -= (A << 1)
	 *		   101: P -=  A
	 *		   110: P -=  A
	 *	       111: do nothing
	 *     A << 2
	 *     Move grouping over by 2 bits
     */
    
    //Reset if ctrl_MULT or ctrl_DIV are high (new arguments received)
    or resetCondition(res, ctrl_MULT, ctrl_DIV); 

    //Remember whether we are multiplying or dividing
    ecedffe whichOp(.d(ctrl_MULT), .clk(clock), .clrn(1'b1), .ena(res), .q(isMult));

    //Booth's algorithm multiplier
	multiplier booth(data_operandA, data_operandB, clock, res, multiplyResult, multiplyException, multiplyRDY);

	//Non-restoring divider
	divider nonRestr(data_operandA, data_operandB, clock, res, divideResult, divideException, divideRDY);

	//Pick between multiplication and division
	assign data_resultRDY = isMult ? multiplyRDY : divideRDY;
	assign data_exception = isMult ? multiplyException : divideException;
	assign data_result = isMult ? multiplyResult : divideResult;
endmodule //multdiv