module regfile (
    clock,
    ctrl_writeEnable,
    ctrl_reset, ctrl_writeReg,
    ctrl_readRegA, ctrl_readRegB, data_writeReg,
    data_readRegA, data_readRegB
);
  
    //Port Declaration
    input clock, ctrl_writeEnable, ctrl_reset;
    input [4:0] ctrl_writeReg, ctrl_readRegA, ctrl_readRegB;
    input [31:0] data_writeReg;

    output [31:0] data_readRegA, data_readRegB;

    wire [31:0] readEncodeA, readEncodeB, writeEncode;
    wire [(32*32 - 1):0] ffOut;
    wire [31:0] writeEncOut;

    decoder decoderA(.regNum(ctrl_readRegA), .out(readEncodeA));
    decoder decoderB(.regNum(ctrl_readRegB), .out(readEncodeB));
    decoder decoderW(.regNum(ctrl_writeReg), .out(writeEncode));

    genvar i, k, n;
    generate 
        
        // Register file  
        
        //Register 0 is always 0;
        assign writeEncOut[0] = 0;
        assign ffOut[31:0] = 32'b0;
        assign data_readRegA = readEncodeA[0] ? ffOut[31:0] : 32'bz;
        assign data_readRegB = readEncodeB[0] ? ffOut[31:0] : 32'bz;

        // Build each register
        for(k = 1; k < 32; k = k + 1) begin: regLoop

            and writeSelect(writeEncOut[k], ctrl_writeEnable, writeEncode[k]);

            register myReg(
                .data(data_writeReg), .clock(clock), 
                .enable(writeEncOut[k]), .reset(ctrl_reset), 
                .out(ffOut[(k+1)*32 - 1 : k*32])
            );

            assign data_readRegA = readEncodeA[k] ? ffOut[(k+1)*32 - 1 : k*32] : 32'bz;
            assign data_readRegB = readEncodeB[k] ? ffOut[(k+1)*32 - 1 : k*32] : 32'bz;
        end
    endgenerate 

endmodule