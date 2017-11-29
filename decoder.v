module decoder(regNum, out);

    input [4:0] regNum;
    output [31:0] out;

    and and0( out[0 ], ~regNum[4], ~regNum[3], ~regNum[2], ~regNum[1], ~regNum[0]);
    and and1( out[1 ], ~regNum[4], ~regNum[3], ~regNum[2], ~regNum[1],  regNum[0]);
    and and2( out[2 ], ~regNum[4], ~regNum[3], ~regNum[2],  regNum[1], ~regNum[0]);
    and and3( out[3 ], ~regNum[4], ~regNum[3], ~regNum[2],  regNum[1],  regNum[0]);
    and and4( out[4 ], ~regNum[4], ~regNum[3],  regNum[2], ~regNum[1], ~regNum[0]);
    and and5( out[5 ], ~regNum[4], ~regNum[3],  regNum[2], ~regNum[1],  regNum[0]);
    and and6( out[6 ], ~regNum[4], ~regNum[3],  regNum[2],  regNum[1], ~regNum[0]);
    and and7( out[7 ], ~regNum[4], ~regNum[3],  regNum[2],  regNum[1],  regNum[0]);
    and and8( out[8 ], ~regNum[4],  regNum[3], ~regNum[2], ~regNum[1], ~regNum[0]);
    and and9( out[9 ], ~regNum[4],  regNum[3], ~regNum[2], ~regNum[1],  regNum[0]);
    and and10(out[10], ~regNum[4],  regNum[3], ~regNum[2],  regNum[1], ~regNum[0]);
    and and11(out[11], ~regNum[4],  regNum[3], ~regNum[2],  regNum[1],  regNum[0]);
    and and12(out[12], ~regNum[4],  regNum[3],  regNum[2], ~regNum[1], ~regNum[0]);
    and and13(out[13], ~regNum[4],  regNum[3],  regNum[2], ~regNum[1],  regNum[0]);
    and and14(out[14], ~regNum[4],  regNum[3],  regNum[2],  regNum[1], ~regNum[0]);
    and and15(out[15], ~regNum[4],  regNum[3],  regNum[2],  regNum[1],  regNum[0]);
    and and16(out[16],  regNum[4], ~regNum[3], ~regNum[2], ~regNum[1], ~regNum[0]);
    and and17(out[17],  regNum[4], ~regNum[3], ~regNum[2], ~regNum[1],  regNum[0]);
    and and18(out[18],  regNum[4], ~regNum[3], ~regNum[2],  regNum[1], ~regNum[0]);
    and and19(out[19],  regNum[4], ~regNum[3], ~regNum[2],  regNum[1],  regNum[0]);
    and and20(out[20],  regNum[4], ~regNum[3],  regNum[2], ~regNum[1], ~regNum[0]);
    and and21(out[21],  regNum[4], ~regNum[3],  regNum[2], ~regNum[1],  regNum[0]);
    and and22(out[22],  regNum[4], ~regNum[3],  regNum[2],  regNum[1], ~regNum[0]);
    and and23(out[23],  regNum[4], ~regNum[3],  regNum[2],  regNum[1],  regNum[0]);
    and and24(out[24],  regNum[4],  regNum[3], ~regNum[2], ~regNum[1], ~regNum[0]);
    and and25(out[25],  regNum[4],  regNum[3], ~regNum[2], ~regNum[1],  regNum[0]);
    and and26(out[26],  regNum[4],  regNum[3], ~regNum[2],  regNum[1], ~regNum[0]);
    and and27(out[27],  regNum[4],  regNum[3], ~regNum[2],  regNum[1],  regNum[0]);
    and and28(out[28],  regNum[4],  regNum[3],  regNum[2], ~regNum[1], ~regNum[0]);
    and and29(out[29],  regNum[4],  regNum[3],  regNum[2], ~regNum[1],  regNum[0]);
    and and30(out[30],  regNum[4],  regNum[3],  regNum[2],  regNum[1], ~regNum[0]);
    and and31(out[31],  regNum[4],  regNum[3],  regNum[2],  regNum[1],  regNum[0]);
endmodule