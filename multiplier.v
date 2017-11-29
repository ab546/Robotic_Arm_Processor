module multiplier(data_operandA, data_operandB, clock, res, data_result, data_exception, data_resultRDY);

	input[31:0] data_operandA, data_operandB;
    input clock, res;

    output[31:0] data_result;
    output data_exception, data_resultRDY;

   /* A is the multiplicand, B is the multiplier, 
    * P is product
	* Start at LSB of B + implicit 0
	*
	* Loop 32 times:
	*     Examine grouping of 2 bits in B:
	*		   00: do nothing
	*		   01: P +=  A
	*		   10: P -=  A
	*		   11: do nothing
	*     A << 1
	*     Move grouping over by 1 bit
    */

    wire[31:0] product;
    wire[31:0] updateMultiplicand, updateProduct, adderOut;
    wire[4:0] count;
    wire countIsZero, countIs31;

    wire[1:0] lsbs;
    wire[3:0] cases;

    wire adderOvf;
    wire aIsZero, bIsZero;
    wire aIsOne, bIsOne;
    wire diffSigns, signOvf;

    wire[31:0] shortcutOne;
    wire useAdder;

    wire maxInputs, aIsMax, bIsMax;
    wire exception0;

    wire gnd;

    counter32 counter(.clock(~clock), .reset(res), .out(count));

    //Initialize register on zero count. Stop updating product on count == 31
    nor zeroCount(countIsZero, count[4], count[3], count[2], count[1], count[0]); 
    and highCount(countIs31, count[4], count[3], count[2], count[1], count[0]);

    register productReg(.data(updateProduct), .clock(clock), .enable(~countIs31), .reset(res), .out(product));

    assign lsbs[0] = countIsZero ? 1'b0 : data_operandB[count-1];
    assign lsbs[1] = data_operandB[count];

    and case0(cases[0], ~lsbs[1], ~lsbs[0]);
    and case1(cases[1], ~lsbs[1],  lsbs[0]);
    and case2(cases[2],  lsbs[1], ~lsbs[0]);
    and case3(cases[3],  lsbs[1],  lsbs[0]);

    cla adder(.a(product), .b(updateMultiplicand), .sub(cases[2]), .sum(adderOut), .ovf(adderOvf), .lessThan(gnd));
    
    assign updateMultiplicand = countIsZero ? data_operandA : data_operandA << count;
    assign updateProduct = cases[0] ? product  : 32'bz;
    assign updateProduct = cases[1] ? adderOut : 32'bz;
    assign updateProduct = cases[2] ? adderOut : 32'bz;
    assign updateProduct = cases[3] ? product  : 32'bz;

    assign data_result = useAdder ? product : shortcutOne;
    assign data_resultRDY = countIs31;

    xor signSwap(diffSigns, data_operandA[31], data_operandB[31]); //Check if sign(a) != sign(b)
    xor signCheck(signOvf, diffSigns, data_result[31]); //If signs are different, result should be negative

    equalZero aZero(data_operandA, aIsZero);
    equalZero bZero(data_operandB, bIsZero);
    and signErrNotZero(exception0, signOvf, ~aIsZero, ~bIsZero, useAdder);
    or finalException(data_exception, exception0, maxInputs);

    equalOne aOne(data_operandA, aIsOne);
    equalOne bOne(data_operandB, bIsOne);
    and adderEnable(useAdder, ~aIsOne, ~bIsOne, ~maxInputs);

    assign shortcutOne = (aIsOne && ~bIsOne) ? data_operandB : 32'bz;
    assign shortcutOne = (bIsOne && ~aIsOne) ? data_operandA : 32'bz;
    assign shortcutOne = maxInputs ? 32'b00000000000000000000000000000001 : 32'bz;

    equalMaxPositive aHigh(data_operandA, aIsMax);
    equalMaxPositive bHigh(data_operandB, bIsMax);
    and bothHigh(maxInputs, aIsMax, bIsMax);


endmodule //multiplier


module equalOne(a, isOne);

	input[31:0] a;
	output isOne;

	wire bits0, bits1, bits2, bits3, bits4, bits5, bits6, bits7;
	wire comb0, comb1;

	nor set0(bits0, ~a[0] , a[1],  a[2],  a[3] );
	nor set1(bits1, a[4]  , a[5],  a[6],  a[7] );
	nor set2(bits2, a[8]  , a[9],  a[10], a[11]);
	nor set3(bits3, a[12] , a[13], a[14], a[15]);
	nor set4(bits4, a[16] , a[17], a[18], a[19]);
	nor set5(bits5, a[20] , a[21], a[22], a[23]);
	nor set6(bits6, a[24] , a[25], a[26], a[27]);
	nor set7(bits7, a[28] , a[29], a[30], a[31]);

	and first4(comb0, bits0, bits1, bits2, bits3);
	and last4(comb1, bits4, bits5, bits6, bits7);
	and check(isOne, comb0, comb1);

endmodule //equalOne


module equalMaxPositive(a, isMax);

    input[31:0] a;
    output isMax;

    wire bits0, bits1, bits2, bits3, bits4, bits5, bits6, bits7;
    wire comb0, comb1;

    and set0(bits0, a[0]  , a[1],  a[2],  a[3] );
    and set1(bits1, a[4]  , a[5],  a[6],  a[7] );
    and set2(bits2, a[8]  , a[9],  a[10], a[11]);
    and set3(bits3, a[12] , a[13], a[14], a[15]);
    and set4(bits4, a[16] , a[17], a[18], a[19]);
    and set5(bits5, a[20] , a[21], a[22], a[23]);
    and set6(bits6, a[24] , a[25], a[26], a[27]);
    and set7(bits7, a[28] , a[29], a[30], ~a[31]); 

    and first4(comb0, bits0, bits1, bits2, bits3);
    and last4(comb1, bits4, bits5, bits6, bits7);
    and check(isMax, comb0, comb1);

endmodule //equalMaxPositive;
