module equalZero(a, isZero);

	input[31:0] a;
	output isZero;

	wire bits0, bits1, bits2, bits3, bits4, bits5, bits6, bits7;
	wire comb0, comb1;

	nor set0(bits0, a[0]  , a[1],  a[2],  a[3] );
	nor set1(bits1, a[4]  , a[5],  a[6],  a[7] );
	nor set2(bits2, a[8]  , a[9],  a[10], a[11]);
	nor set3(bits3, a[12] , a[13], a[14], a[15]);
	nor set4(bits4, a[16] , a[17], a[18], a[19]);
	nor set5(bits5, a[20] , a[21], a[22], a[23]);
	nor set6(bits6, a[24] , a[25], a[26], a[27]);
	nor set7(bits7, a[28] , a[29], a[30], a[31]);

	and first4(comb0, bits0, bits1, bits2, bits3);
	and last4(comb1, bits4, bits5, bits6, bits7);
	and check(isZero, comb0, comb1);

endmodule //equalZero