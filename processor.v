/**
 * The processor takes in two inputs and returns two outputs
 *
 * Inputs
 * clock: this is the clock for your processor at 50 MHz
 * reset: we should be able to assert a reset to start your pc from 0 (sync or
 * async is fine)
 *
 * Outputs
 * dmem_data_in: this should connect to the wire that feeds in data to your dmem
 * dmem_address: this should be the address of data that you write data to
 *
 * Notes
 * You will need to figure out how to instantiate two memory elements, called
 * "syncram," in Quartus: one for imem and one for dmem. Each should take in a
 * 12-bit address and allow for storing a 32-bit value at each address. Each
 * should have a single clock.
 *
 * Each memory element should have a corresponding .mif file that initializes
 * the memory element to certain value on start up. These should be named
 * imem.mif and dmem.mif respectively.
 *
 * imem
 * Inputs:  12-bit address, 1-bit clock enable, and a clock
 * Outputs: 32-bit instruction
 *
 * dmem
 * Inputs:  12-bit address, 1-bit clock, 32-bit data, 1-bit write enable
 * Outputs: 32-bit data at the given address
 *
 */

 /**
  * ISA Updates:
  * Opcode 01001 - move servox (Store in register 26)
  * Opcode 01010 - move servoy (store in register 27)
  * Opcode 01011 - move servoz (store in register 28)
  */

/**TODO:
 * Stalls may clobber instructions
 * Fix multdiv
 */


module processor(clock, res, servoX, servoY, servoZ, IRdata, IRswitch, led, led2,iruse/*
iroutput, irisready,testdata, icount, scount, dcount, instr,PCout*/
 /*,

    dmem_data_in, dmem_address,
	
	gatedClk, fetchInsn, 
	
	//Fetch I/O
	pcF, immF, targetF, opcodeF, rdF, rsF, rtF, shamtF, aluOpF, jumpCtrlF, branchCtrlF,
	regWriteEnF, aluInBF, loadDataF, storeDataF, setxCtrlF,  

	//FD Latch outputs
    pcFD, immFD, targetFD, opcodeFD, rdFD, rsFD, rtFD, shamtFD, aluOpFD, jumpCtrlFD, branchCtrlFD,
	regWriteEnFD, aluInBFD, loadDataFD, storeDataFD, setxCtrlFD,

	//Regfile I/O
    regAValD, regBValD,
    
    //DX Latch outputs
	pcDX, immDX, targetDX, opcodeDX, rdDX, rsDX, rtDX, shamtDX, aluOpDX, jumpCtrlDX, branchCtrlDX, 
	regWriteEnDX, aluInBDX, loadDataDX, storeDataDX, setxCtrlDX, regAValDX, regBValDX,
 
    //Execute I/O
    execResultX, execErrorX, execReadyX, argsLessThanX, argsNotEqualX,

    //Branch logic
    nextPCX, pcOverrideX,

    //XM Latch outputs
    pcXM, immXM, targetXM, opcodeXM, rdXM, rsXM, rtXM, shamtXM, aluOpXM, jumpCtrlXM, branchCtrlXM,
	regWriteEnXM, aluInBXM, loadDataXM, storeDataXM, setxCtrlXM, execErrorXM, execResultXM, regBValXM,

    //Data mem I/O
    dMemOutM,

    //MW Latch outputs
    pcMW, immMW, targetMW, opcodeMW, rdMW, rsMW, rtMW, shamtMW, aluOpMW, jumpCtrlMW, branchCtrlMW,
	regWriteEnMW, aluInBMW, loadDataMW, storeDataMW, setxCtrlMW, execErrorMW, execResultMW, dMemOutMW,

    //Writeback I/O
    regWriteValW, writeRegW, writeSelectW,

    //Bypass logic
    aluASel, aluBSel, regAValByp, regBValByp, dMemInM,

    //Servo outputs
    servoF, servoFD, servoDX, servoXM, servoMW, servoReadyX, coreRegW, servoRegW*/
);


    input clock, res, IRdata, IRswitch;
    output servoX, servoY, servoZ,led,led2;
    output [7:0] iruse;	
    assign iruse=iruse1[7:0];
    /*output [31:0] iroutput,instr,PCout;
	 assign PCout=pcF;
	 assign instr=fetchInsn;
    assign iroutput=IRout;
    output irisready;
    assign irisready=IRready;
    output [31:0] testdata;
	 output [17:0] icount, scount, dcount;*/
    assign led=(pcF>=32'd20);
	 assign led2=(IRswitch);
   	wire[31:0] dmem_data_in;
    wire[11:0] dmem_address;

    wire gatedClk;

    //Fetch outputs
    wire[31:0] fetchInsn;
    wire[31:0] pcF, immF, targetF;
    wire[4:0] opcodeF, rdF, rsF, rtF, shamtF, aluOpF;
    wire[1:0] jumpCtrlF, branchCtrlF, servoF;
	wire regWriteEnF, aluInBF, loadDataF, storeDataF, setxCtrlF, irF;    

    //FD Latch outputs
    wire[31:0] pcFD, immFD, targetFD; 
    wire[4:0] opcodeFD, rdFD, rsFD, rtFD, shamtFD, aluOpFD;
    wire[1:0] jumpCtrlFD, branchCtrlFD, servoFD;
	wire regWriteEnFD, aluInBFD, loadDataFD, storeDataFD, setxCtrlFD, irFD;

    //Regfile I/O
    wire[31:0] regAValD, regBValD;

    //Stall logic
    wire dstall;
    wire loadDX, rsHit, rdHit, storeFD;
    
    //DX Latch outputs
	wire[31:0] pcDX, immDX, targetDX; 
    wire[4:0] opcodeDX, rdDX, rsDX, rtDX, shamtDX, aluOpDX;
    wire[1:0] jumpCtrlDX, branchCtrlDX, servoDX;
	wire regWriteEnDX, aluInBDX, loadDataDX, storeDataDX, setxCtrlDX, irDX;
	wire[31:0] regAValDX, regBValDX;
 
    //Execute I/O
    wire[31:0] execResultX;
    wire execErrorX, execReadyX, argsLessThanX, argsNotEqualX, servoReadyX;
    wire [31:0] IRout;
    wire IRready;

    //X stage bypass logic
    wire[1:0] aluASel, aluBSel;
	wire[31:0] regAValByp, regBValByp;
	wire bypASpecial0, bypASpecial1, bypANormal0, bypANormal1, isSW, isBranch;
	wire bypBNormal0, bypBNormal1;

    //Branch logic
    wire[31:0] nextPCX;
    wire pcOverrideX;

    //XM Latch outputs
    wire[31:0] pcXM, immXM, targetXM, irDataXM; 
    wire[4:0] opcodeXM, rdXM, rsXM, rtXM, shamtXM, aluOpXM;
    wire[1:0] jumpCtrlXM, branchCtrlXM, servoXM;
	wire regWriteEnXM, aluInBXM, loadDataXM, storeDataXM, setxCtrlXM, execErrorXM, irXM;
	wire[31:0] execResultXM, regBValXM;

    //Data mem I/O
    wire[31:0] dMemOutM;
    wire[31:0] dMemInM;

    //MW Latch outputs
    wire[31:0] pcMW, immMW, targetMW, irDataMW; 
    wire[4:0] opcodeMW, rdMW, rsMW, rtMW, shamtMW, aluOpMW;
    wire[1:0] jumpCtrlMW, branchCtrlMW, servoMW;
	wire regWriteEnMW, aluInBMW, loadDataMW, storeDataMW, setxCtrlMW, execErrorMW, irMW;
	wire[31:0] execResultMW, dMemOutMW;

	//WM bypass logic
	wire dMemDataSel;

    //Writeback I/O
    wire[31:0] regWriteValW;
    wire[4:0] coreRegW, servoRegW, writeRegW;
    wire[1:0] writeSelectW;

    wire gnd, reset;

    //Stall if multiplier is running or servo is being moved
    and clockGate(gatedClk, execReadyX, clock, servoReadyX, ~(~IRready&irDX) );
    assign reset = ~res;

    fetch fetchStage(
    	nextPCX, (execReadyX & ~dstall),
    	pcOverrideX,
    	jumpCtrlDX, branchCtrlDX,
    	gatedClk, reset, 
    	pcF, fetchInsn
    );
    register irlatch(iruseful, clock, IRready, 1'b0, iruse1);
    wire [31:0] iruse1;

    decode decodeInsn(fetchInsn, opcodeF, rdF, rsF, rtF, immF, shamtF, aluOpF, targetF[26:0]);
    assign targetF[31:27] = 5'b0; 

    control ctrlSignals(
    	opcodeF, aluOpF,
		regWriteEnF, aluInBF, 
		jumpCtrlF, branchCtrlF, 
		loadDataF, storeDataF,
		setxCtrlF, servoF, irF
    );

	pipelineLatch latchFD(
		(gatedClk & ~dstall), 1'b1, pcOverrideX,
		pcF, opcodeF,
		rdF, rsF, rtF, 
		shamtF, aluOpF, 
		immF, targetF, 32'b0,
		regWriteEnF, aluInBF, 
		jumpCtrlF, branchCtrlF, 
		loadDataF, storeDataF,
		setxCtrlF, servoF, irF,
		1'b0,
		32'b0, 32'b0,
		pcFD, opcodeFD,
		rdFD, rsFD, rtFD, 
		shamtFD, aluOpFD, 
		immFD, targetFD, gnd,
		regWriteEnFD, aluInBFD, 
		jumpCtrlFD, branchCtrlFD, 
		loadDataFD, storeDataFD,
		setxCtrlFD, servoFD, irFD,
		gnd,
		gnd, gnd
	);

	wire isRSZero;

	//STALL LOGIC (probably broken)
		equalityChecker dxLoad(opcodeDX, 5'b01000, loadDX);
		equalityChecker rsConflict(rsFD, rdDX, rsHit);
		equalityChecker rdConflict(rdFD, rdDX, rdHit);
		equalityChecker fdStore(opcodeFD, 5'b00111, storeFD);
		nor rs0(isRSZero, rsFD[0], rsFD[1], rsFD[2], rsFD[3], rsFD[4]);
		assign dstall = (loadDX & (rsHit | (rdHit & storeFD))) & ~isRSZero;

    regFileLogic myRegFile(
    	rdMW, rsFD, rtFD, 
		writeRegW, regWriteValW, 
		pcFD,
		regWriteEnMW, 
		branchCtrlFD, jumpCtrlFD,
		storeDataFD,
		gatedClk, reset, 
		regAValD, regBValD
	);

	pipelineLatch latchDX(
		gatedClk, 1'b1, dstall,
		pcFD, opcodeFD,
		rdFD, rsFD, rtFD, 
		shamtFD, aluOpFD, 
		immFD, targetFD, 32'b0,
		regWriteEnFD, aluInBFD, 
		jumpCtrlFD, branchCtrlFD, 
		loadDataFD, storeDataFD,
		setxCtrlFD, servoFD, irFD,
		1'b0,
		regAValD, regBValD,
		pcDX, opcodeDX,
		rdDX, rsDX, rtDX, 
		shamtDX, aluOpDX, 
		immDX, targetDX, gnd,
		regWriteEnDX, aluInBDX, 
		jumpCtrlDX, branchCtrlDX, 
		loadDataDX, storeDataDX,
		setxCtrlDX, servoDX, irDX,
		gnd,
		regAValDX, regBValDX
	);

	//BYPASSING (WX, MX)
		//ALUinA
		equalityChecker mxBypassAn(rsDX, rdXM, bypANormal0);
		equalityChecker mxBypassAs(rdDX, rdXM, bypASpecial0); //Stores and branches read from $rd and $rs
		equalityChecker wxBypassAn(rsDX, rdMW, bypANormal1);
		equalityChecker wxBypassAs(rdDX, rdMW, bypASpecial1); //Stores and branches read from $rd and $rs
		
		assign isSW = ~opcodeDX[4] & ~opcodeDX[3] & opcodeDX[2] & opcodeDX[1] & opcodeDX[0];
		assign isBranch = branchCtrlDX[1] | branchCtrlDX[0];
		
		assign aluASel[0] = (bypANormal0 & ~(isSW | isBranch)) | (bypASpecial0 & (isSW | isBranch));
		assign aluASel[1] = (bypANormal1 & ~(isSW | isBranch)) | (bypASpecial1 & (isSW | isBranch));
		mux4to1 bypALUa(
			.in0(regAValDX), .in1(execResultXM), .in2(regWriteValW), .in3(execResultXM), 
			.sel(aluASel), .out(regAValByp));

		//ALUinB
		equalityChecker mxBypassB(rtDX, rdXM, bypBNormal0);
		equalityChecker wxBypassBn(rtDX, rdMW, bypBNormal1);

		//if branch or store, second input must be RS
		assign aluBSel[0] = (bypBNormal0 & ~(isSW | isBranch)) | (bypANormal0 & (isSW | isBranch));
		assign aluBSel[1] = (bypBNormal1 & ~(isSW | isBranch)) | (bypANormal1 & (isSW | isBranch));
		mux4to1 bypALUb(
			.in0(regBValDX), .in1(execResultXM), .in2(regWriteValW), .in3(execResultXM), 
			.sel(aluBSel), .out(regBValByp));

    execute executeInsn(
    	regAValByp, regBValByp, 
    	immDX, pcDX, 
    	aluOpDX, shamtDX, opcodeDX, aluInBDX,
    	jumpCtrlDX, 
    	setxCtrlDX, targetDX,
    	clock, gatedClk, //Multiplier continues while everything else stalls
    	execResultX, execReadyX, execErrorX,
    	argsLessThanX, argsNotEqualX
    );

    branchLogic branchHandler(
    	branchCtrlDX, jumpCtrlDX,
		argsNotEqualX, argsLessThanX,
		pcDX, immDX, targetDX, regBValDX,
		nextPCX, pcOverrideX
    );

    servoLogic servoController(clock, (reset | (servoDX == 2'b0)), servoDX, immDX, servoX, servoY, servoZ, servoReadyX);
    IR_Receiver IR(clock&irDX, res, IRdata, IRready, IRout);
    wire [31:0] iruseful;
    assign iruseful[7:0]=IRout[23:16];
    assign iruseful[31:8]=24'b0;

    pipelineLatch latchXM(
    	gatedClk, 1'b1, 1'b0,
		pcDX, opcodeDX,
		rdDX, rsDX, rtDX, 
		shamtDX, aluOpDX, 
		immDX, targetDX, iruseful,
		regWriteEnDX, aluInBDX, 
		jumpCtrlDX, branchCtrlDX, 
		loadDataDX, storeDataDX,
		setxCtrlDX, servoDX, irDX,
		execErrorX, 
		execResultX, regBValDX,
		pcXM, opcodeXM,
		rdXM, rsXM, rtXM, 
		shamtXM, aluOpXM, 
		immXM, targetXM, irDataXM,
		regWriteEnXM, aluInBXM, 
		jumpCtrlXM, branchCtrlXM, 
		loadDataXM, storeDataXM,
		setxCtrlXM, servoXM, irXM,
		execErrorXM,
		execResultXM, regBValXM
    );

    //BYPASSING (WM)
    	//dmem data: something writes to the register about to be written to dmem
		equalityChecker wmBypass0(rdXM, rdMW, dMemDataSel);
		assign dMemInM = dMemDataSel ? regWriteValW : regBValXM;

    dmem mydmem(
    	.aclr       (1'b0),
        .address    (execResultXM[11:0]),    // address of data
        .clock      (gatedClk),              // may need to invert the clock
        .data	    (dMemInM),               // data you want to write
        .wren	    (storeDataXM),           // write enable
        .q          (dMemOutM)               // data from dmem
    );

    pipelineLatch latchMW(
    	gatedClk, 1'b1, 1'b0,
		pcXM, opcodeXM,
		rdXM, rsXM, rtXM, 
		shamtXM, aluOpXM, 
		immXM, targetXM, irDataXM,
		regWriteEnXM, aluInBXM, 
		jumpCtrlXM, branchCtrlXM, 
		loadDataXM, storeDataXM,
		setxCtrlXM, servoXM, irXM,
		execErrorXM, 
		execResultXM, dMemOutM,
		pcMW, opcodeMW,
		rdMW, rsMW, rtMW, 
		shamtMW, aluOpMW, 
		immMW, targetMW, irDataMW,
		regWriteEnMW, aluInBMW, 
		jumpCtrlMW, branchCtrlMW, 
		loadDataMW, storeDataMW,
		setxCtrlMW, servoMW, irMW,
		execErrorMW,
		execResultMW, dMemOutMW
    );

    //WRITEBACK STAGE
    	//Write dmem output for load; write angle for servo; write result of execute otherwise
	    assign regWriteValW = loadDataMW ? dMemOutMW : 32'bz;
	    assign regWriteValW = ~(servoMW == 2'b00) ? immMW : 32'bz;
	    assign regWriteValW = irMW ? irDataMW: 32'bz;
	    assign regWriteValW = ~loadDataMW & ~(servoMW == 2'b00) & ~irMW ? execResultMW : 32'bz;
	    wire [31:0] regvalreal;
	    assign regvalreal= IRready&irDX ? iruseful : regWriteValW;

	    //If error or setx, write to register 30. If jal, write PC + 1 to register 31. If both, do nothing
		and isJAL(writeSelectW[1], ~jumpCtrlMW[1], jumpCtrlMW[0]);
		assign writeSelectW[0] = execErrorMW | setxCtrlMW;				
		fiveBitmux4to1 regW(.in0(rdMW), .in1(5'd30), .in2(5'd31), .in3(5'b0), .sel(writeSelectW), .out(coreRegW));

		//Write to 26 for servox, 27 for servoy, and 28 for servoz. Select between core and servo registers
		fiveBitmux4to1 servoMux(.in0(5'd0), .in1(5'd26), .in2(5'd27), .in3(5'd28), .sel(servoMW), .out(servoRegW));
		assign writeRegW = servoMW == 2'b00 ? 5'bz : servoRegW;
		//ALERT: 29 is threshold, 25 is arm motion. not contiguously defined.
		assign writeRegW= irMW&IRswitch ? 5'd25 : 5'bz;
		assign writeRegW= irMW&~IRswitch ? 5'd25 : 5'bz;
		assign writeRegW= ~irMW & (servoMW==2'd0) ? coreRegW: 5'bz;
		wire  regwreal;
		assign regwreal = IRready&irDX? 5'd29 : writeRegW;


    //Assign outputs for testing
    /*assign dmem_address = execResultXM[11:0];
    assign dmem_data_in = dMemInM;*/
endmodule //processor


/**
 * Fetch module. Increments the PC by 1 every clock unless there is a branch or jump.
 * Contains imem.
 */
module fetch( 
	nextPC, advancePC,
	pcOverride, 
	jumpCtrl, branchCtrl,
    clock, reset, 
    incrPC, insn
);

	input[31:0] nextPC;
	input[1:0] jumpCtrl, branchCtrl;
	input clock, reset, advancePC, pcOverride;

	output[31:0] incrPC, insn;

	wire[31:0] currPC, updatedPC, imemRead;
	wire gnd;

	//Store the PC
	register pc(.data(updatedPC), .clock(clock), .enable(advancePC), .reset(reset), .out(currPC));

	//PC Increment logic (normal, branch, jump)
	cla addOne(.a(currPC), .b(32'd1), .sub(1'b0), .sum(incrPC), .ovf(gnd), .lessThan(gnd)); //PC++

	//By default assume PC increments by 1. If jump or branch, flush insns.
	assign updatedPC = pcOverride & (branchCtrl[1] | branchCtrl[0]) ? nextPC+1 : 32'bz; 
	assign updatedPC = pcOverride & (jumpCtrl[1] | jumpCtrl[0]) ? nextPC-1 : 32'bz;
	assign updatedPC = ~pcOverride ? incrPC : 32'bz;
	
	assign imemRead = pcOverride & (branchCtrl[1] | branchCtrl[0]) ? nextPC-1 : 32'bz; //Pass nextPC to resolve branch in X stage
	assign imemRead = pcOverride & (jumpCtrl[1] | jumpCtrl[0]) ? nextPC-2 : 32'bz;
	assign imemRead = ~pcOverride ? currPC : 32'bz;
	
	imem myimem(
        .address    (imemRead[11:0]),               // address of data
		.clock      (~clock),                        // may need to invert the clock
		.q          (insn)                          // the raw instruction
    );
endmodule //fetch


/**
 * Instruction parse module. Interprets the instruction
 * into: shamt, rd, rs, rt, imm, target, aluOp
 */
module decode(insn, opcode, rd, rs, rt, imm, shamt, aluOp, target);

	input[31:0] insn;

	output[4:0] opcode, rd, rs, rt, shamt, aluOp;
	output[31:0] imm, target;

	assign opcode = insn[31:27];
	assign rd = insn[26:22];
	assign rs = insn[21:17];
	assign rt = insn[16:12];
	assign shamt = insn[11:7];
	//aluOp can be pulled out straight from r-type instructions, but must be 00000 for i-type 
	assign aluOp = (~insn[31] & ~insn[30] & ~insn[29] & ~insn[28] & ~insn[27]) ? insn[6:2] : 5'b0;
	assign target[31:27] = 5'b0;
	assign target[26:0] = insn[26:0];

	signExtender extend(insn[16:0], imm);
endmodule //decode


/**
 * Takes the top 5 bits (opcode) and interprets into control signals
 * for the rest of the hardware
 */
module control(
	opcode, aluOp,
	regWriteEn, aluInB, 
	jumpCtrl, branchCtrl, 
	loadData, storeData,
	setxCtrl, servo, IRCtrl
);

    input[4:0] opcode, aluOp;

    output regWriteEn, aluInB, loadData, storeData, setxCtrl, IRCtrl;
    output[1:0] branchCtrl, jumpCtrl, servo;

    wire arithOp;
    
    assign regWriteEn = 
    	~opcode[4] & (~opcode[1] | opcode[0]) & ~(opcode[2] & opcode[1]) & (opcode[2] | opcode[1] | ~opcode[0]) & ~(jumpCtrl[0] & jumpCtrl[1]) 
    	| setxCtrl | ~(servo == 2'b0)|(opcode==5'd12);

    assign setxCtrl = opcode[4] & ~opcode[3] & opcode[2] & ~opcode[1] & opcode[0];
    
    assign loadData = ~opcode[4] & opcode[3] & ~opcode[2] & ~opcode[1] & ~opcode[0];
    assign storeData = ~opcode[4] & ~opcode[3] & opcode[2] & opcode[1] & opcode[0];

    assign branchCtrl[1] = ~opcode[3] & opcode[2] & opcode[1] & ~opcode[0];
    assign branchCtrl[0] = 
    	(~opcode[4] & ~opcode[3] & ~opcode[2] & opcode[1] & ~opcode[0])
    	|(opcode[4] & ~opcode[3] & opcode[2] & opcode[1] & ~opcode[0]);

    assign jumpCtrl[1] =
    	(~opcode[4] & ~opcode[3] & ~opcode[2] & ~opcode[1] & opcode[0])
    	|(~opcode[4] & ~opcode[3] & opcode[2] & ~opcode[1] & ~opcode[0]);
    assign jumpCtrl[0] = 
    	(~opcode[4] & ~opcode[3] & ~opcode[2] & opcode[1] & opcode[0])
    	|(~opcode[4] & ~opcode[3] & opcode[2] & ~opcode[1] & ~opcode[0]);

    assign aluInB = loadData | storeData | (~opcode[4] & ~opcode[3] & opcode[2] & ~opcode[1] & opcode[0]);
   	
	assign IRCtrl=(opcode==5'd12);

	assign servo = opcode == 5'd9 ? 2'b01 : 2'bz;
	assign servo = opcode == 5'd10 ? 2'b10 : 2'bz;
	assign servo = opcode == 5'd11 ? 2'b11 : 2'bz;
   	assign servo = ~(opcode == 5'd9 | opcode == 5'd10 | opcode == 5'd11) ? 2'b00 : 2'bz;

endmodule //control


/**
 * Pipeline Latch. Stores instruction, PC, and two 32-bit values.
 * F/D: 32-bit values are both zero
 * D/X: valA = regValA, valB = regValB
 * X/M: valA = execResult, valB = regValB
 * M/W: valA = execResult, valB = dMemOut
 */
module pipelineLatch(
	clock, enable, reset,
	pc0, opcode0,
	rd0, rs0, rt0, 
	shamt0, aluOp0, 
	imm0, target0, irData0,
	regWriteEn0, aluInB0, 
	jumpCtrl0, branchCtrl0, 
	loadData0, storeData0,
	setxCtrl0, servo0, ir0,
	execError0,
	valA0, valB0,
	pc1, opcode1,
	rd1, rs1, rt1, 
	shamt1, aluOp1, 
	imm1, target1, irData1,
	regWriteEn1, aluInB1, 
	jumpCtrl1, branchCtrl1, 
	loadData1, storeData1,
	setxCtrl1, servo1, ir1,
	execError1,
	valA1, valB1
);

	input clock, enable, reset, execError0, ir0;
	input[31:0] pc0, imm0, target0, irData0;
	input[4:0] opcode0, rd0, rs0, rt0, shamt0, aluOp0;

	input regWriteEn0, aluInB0, loadData0, storeData0, setxCtrl0;
	input[1:0] jumpCtrl0, branchCtrl0, servo0;

	input[31:0] valA0, valB0;

	output[31:0] pc1, imm1, target1, irData1;
	output[4:0] opcode1, rd1, rs1, rt1, shamt1, aluOp1;

	output regWriteEn1, aluInB1, loadData1, storeData1, setxCtrl1, execError1, ir1;
	output[1:0] jumpCtrl1, branchCtrl1, servo1;

	output[31:0] valA1, valB1;

	wire[31:0] decodeCtrl0, opTarget0;
	wire[31:0] decodeCtrl1, opTarget1;
	reg[1:0] servoReg;
	reg IRreg;
	reg [31:0] irDatareg;

	//Buses - in
   	assign decodeCtrl0[31:27] = rd0;
   	assign decodeCtrl0[26:22] = rs0;
   	assign decodeCtrl0[21:17] = rt0;
   	assign decodeCtrl0[16:12] = shamt0;
   	assign decodeCtrl0[11:9] = aluOp0[2:0];
   	assign decodeCtrl0[8] = regWriteEn0;
   	assign decodeCtrl0[7] = aluInB0;
    assign decodeCtrl0[6:5] = jumpCtrl0;
    assign decodeCtrl0[4:3] = branchCtrl0;
    assign decodeCtrl0[2] = loadData0;
    assign decodeCtrl0[1] = storeData0;
    assign decodeCtrl0[0] = setxCtrl0;

   	assign opTarget0[31:27] = opcode0;
   	assign opTarget0[26:0] = target0[26:0];

   	always @(posedge clock) begin
   		servoReg = servo0;
   		IRreg = ir0;
   		irDatareg=irData0;
   	end

   	//Registers (buses)
   	register decodeCtrlReg(.data(decodeCtrl0), .clock(clock), .enable(enable), .reset(reset), .out(decodeCtrl1));
   	register opTargetReg(  .data(opTarget0),   .clock(clock), .enable(enable), .reset(reset), .out(opTarget1));

   	//Registers (direct assign outputs)
   	register pcReg(  .data(pc0),     .clock(clock), .enable(enable), .reset(reset), .out(pc1));
    register immReg( .data(imm0),    .clock(clock), .enable(enable), .reset(reset), .out(imm1));
    register valAReg(.data(valA0),   .clock(clock), .enable(enable), .reset(reset), .out(valA1));
    register valBReg(.data(valB0),   .clock(clock), .enable(enable), .reset(reset), .out(valB1));
    ecedffe errorReg(.d(execError0), .clk(clock),   .ena(enable),    .clrn(~reset), .q(execError1));

    //Assign outputs
    assign rd1 = decodeCtrl1[31:27];
    assign rs1 = decodeCtrl1[26:22];
    assign rt1 = decodeCtrl1[21:17];
    assign shamt1 = decodeCtrl1[16:12];
    assign aluOp1[2:0] = decodeCtrl1[11:9];
    assign aluOp1[4:3] = 2'b0;
    assign regWriteEn1 = decodeCtrl1[8];
    assign aluInB1 = decodeCtrl1[7];
    assign jumpCtrl1 = decodeCtrl1[6:5];
    assign branchCtrl1 = decodeCtrl1[4:3];
    assign loadData1 = decodeCtrl1[2];
    assign storeData1 = decodeCtrl1[1];
    assign setxCtrl1 = decodeCtrl1[0];

    assign servo1 = servoReg;
    assign ir1 = IRreg;
    assign irData1=irDatareg;

    assign opcode1 = opTarget1[31:27];
    assign target1[26:0] = opTarget1[26:0];
    assign target1[31:27] = 5'b0;
endmodule //pipelineLatch


/**
 * Register file with input-determining logic (based on branch)
 */
module regFileLogic(
	rd, rs, rt, 
	writeReg, regWriteVal, 
	pc,
	regWriteEn, 
	branchCtrl, jumpCtrl,
	storeData,
	clock, reset, 
	regAVal, regBVal
);

	input[4:0] rd, rs, rt, writeReg;
	input[31:0] regWriteVal, pc;
	input[1:0] branchCtrl, jumpCtrl;
	input regWriteEn, clock, reset, storeData; 

	output[31:0] regAVal, regBVal;

	wire[4:0] readRegA, readRegB, writeReg;
	wire[4:0] branchRegSelB;
	wire[1:0] writeSelect;
	wire[31:0] writeToReg;

	//Select registers to read based on branchCtrl
	fiveBitmux4to1 regA(.in0(rs), .in1(rd), .in2(rd), .in3(5'd30), .sel(branchCtrl), .out(readRegA));
	fiveBitmux4to1 regB(.in0(rt), .in1(rs), .in2(rs), .in3(5'b0),  .sel(branchCtrl), .out(branchRegSelB));

	//On jr or sw, register B must read rd
	assign readRegB = storeData ? rd : 5'bz;
	assign readRegB = jumpCtrl[1] & jumpCtrl[0] ? 5'd31 : 5'bz;
	assign readRegB = ~storeData & ~(jumpCtrl[1] & jumpCtrl[0]) ? branchRegSelB : 5'bz;

	regfile registerfile(
		.clock(clock),
   		.ctrl_writeEnable(regWriteEn|IRready),
    	.ctrl_reset(reset), 
    	.ctrl_writeReg(regwreal),
    	.ctrl_readRegA(readRegA), 
    	.ctrl_readRegB(readRegB), 
    	.data_writeReg(regvalreal),
    	.data_readRegA(regAVal), 
    	.data_readRegB(regBVal)
	);
endmodule //regFileLogic


/**
 * Executes the instruction
 */
module execute(
	regAVal, regBVal, imm, pc, 
	aluOp, shamt, opcode, aluInB, 
	jumpCtrl, 
	setxCtrl, target,
	clock, reset, 
	result, ready, error,
	argsLessThan, argsNotEqual
);

	input[31:0] regAVal, regBVal, imm, pc;
	input[31:0] target;
	input[4:0] aluOp, shamt, opcode;
	input[1:0] jumpCtrl;
	input aluInB, setxCtrl, clock, reset;

	output[31:0] result;
	output ready, error, argsLessThan, argsNotEqual;

	wire[31:0] operandA, operandB;
	wire[31:0] mathResult, aluOut, multOut;
	
	wire aluErr, multErr, multReady;
	wire isMult, isDiv, needMult;
	wire count, arithOp, multCtrl, divCtrl;
	
	wire[31:0] errorCode;
	wire[1:0] outputSel;

	//Output error code if error happens
	errorCodeFinder errFinder(aluOp, opcode, errorCode);
	
	//If jal, output PC + 1
	assign operandA = (~jumpCtrl[1] & jumpCtrl[0]) ? pc    : regAVal;
	assign operandB = (~jumpCtrl[1] & jumpCtrl[0]) & ~aluInB ? 32'd1 : 32'bz;

	//If not i-type and no jal, then just pick the register output
	assign operandB = (jumpCtrl[1] | ~jumpCtrl[0]) & ~aluInB ? regBVal : 32'bz;

	//If i-type, operandB must be imm
	assign operandB = (jumpCtrl[1] | ~jumpCtrl[0]) & aluInB ? imm : 32'bz;

	//MultDiv control signals. Need to make sure that multCtrl/divCtrl goes to 0 after one cycle
	//Assumes master gated clock sits on 0 as multDiv operates
	ecedffe multdivcount(.d(~reset & (isMult | isDiv)), .clk(clock), .clrn(1'b1), .ena(1'b1), .q(count));

	//MultDiv operation only on opcode 00000
	nor arithmetic(arithOp, opcode[4], opcode[3], opcode[2], opcode[1], opcode[0]);

	//Mult or Div control signal logic (opcode & aluOp & count is zero)
	assign isMult = ~aluOp[4] & ~aluOp[3] & aluOp[2] & aluOp[1] & ~aluOp[0];
	assign isDiv  = ~aluOp[4] & ~aluOp[3] & aluOp[2] & aluOp[1] &  aluOp[0];
    assign multCtrl = arithOp & isMult & ~count;
    assign divCtrl =  arithOp & isDiv  & ~count;

	alu myALU(
		.data_operandA(operandA), 
		.data_operandB(operandB), 
		.ctrl_ALUopcode(aluOp), 
		.ctrl_shiftamt(shamt), 
		.data_result(aluOut), 
		.isNotEqual(argsNotEqual),
		.isLessThan(argsLessThan),
		.overflow(aluErr)
	);

	multdiv myMultDiv(
		.data_operandA(regAVal), 
		.data_operandB(regBVal), 
		.ctrl_MULT(multCtrl), 
		.ctrl_DIV(divCtrl), 
		.clock(clock), 
		.data_result(multOut), 
		.data_exception(multErr), 
		.data_resultRDY(multReady)
	);
	
	//Result and error condition from math
	or(needMult, isMult, isDiv);
	assign mathResult = needMult ? multOut : aluOut; //Pick which result to use. Note: aluOut covers JAL case
	assign error = needMult ? multErr : aluErr; //Pick which error to use
	
	//Assign output based on setx and error. If both, write error code
	assign outputSel[1] = setxCtrl;
	assign outputSel[0] = error;
	mux4to1 execOut(.in0(mathResult), .in1(errorCode), .in2(target), .in3(errorCode), .sel(outputSel), .out(result));

	assign ready = needMult ? multReady : 1'b1; //ALU and setx are ready within one cycle
endmodule //execute


/**
 * Determines the right error code depending on the aluOp
 */
module errorCodeFinder(aluOp, opcode, code);

	input[4:0] aluOp, opcode;
	output[31:0] code;

	wire aluOp0, op5, aluOp1, aluOp6, aluOp7, noErr;

	//Figure out which ALUop to get the error code for
	nor add(aluOp0, aluOp[4], aluOp[3], aluOp[2], aluOp[1], aluOp[0]);
	and addi(op5, ~opcode[4], ~opcode[3], opcode[2], ~opcode[1], opcode[0]);
	nor sub(aluOp1, aluOp[4], aluOp[3], aluOp[2], aluOp[1], ~aluOp[0]);
	nor mul(aluOp6, aluOp[4], aluOp[3], ~aluOp[2], ~aluOp[1], aluOp[0]);
	and div(aluOp7, ~aluOp[4], ~aluOp[3], aluOp[2], aluOp[1], aluOp[0]);
	
	//Default case (does not throw error)
	nor fallback(noErr, aluOp0, op5, aluOp1, aluOp6, aluOp7);

	assign code = noErr  ? 32'b0 : 32'bz;
	assign code = aluOp0 ? 32'd1 : 32'bz;
	assign code = op5    ? 32'd2 : 32'bz;
	assign code = aluOp1 ? 32'd3 : 32'bz;
	assign code = aluOp6 ? 32'd4 : 32'bz;
	assign code = aluOp7 ? 32'd5 : 32'bz;
endmodule //errorCodeFinder


/**
 * Handles all branch logic
 * Branch syntax: 00-no branch, 01-beq, 10-blt, 11-bex
 * Jump sntax: 00-no jump, 01-j, 10-jal, 11-jr 
 */
module branchLogic(
	branchCtrl, jumpCtrl,
	argsNotEqual, argsLessThan,
	pc, imm, target, regBVal,
	nextPC, pcOverride
);

	input[1:0] branchCtrl, jumpCtrl;
	input argsNotEqual, argsLessThan;
	input[31:0] pc, imm, target, regBVal;

	output[31:0] nextPC;
	output pcOverride;

	wire[1:0] branchTaken;
	wire[31:0] branchPC, summedPC;
	wire gnd;
	  
	//Reassign PC based on branchCtrl and branch condition
	assign branchTaken = ~branchCtrl[1] & ~branchCtrl[0]                 ? 2'b0       : 2'bz; //no branch
	assign branchTaken = ~branchCtrl[1] &  branchCtrl[0] & ~argsNotEqual ? branchCtrl : 2'bz; //beq taken
	assign branchTaken = ~branchCtrl[1] &  branchCtrl[0] &  argsNotEqual ? 2'b0       : 2'bz; //beq not taken
	assign branchTaken =  branchCtrl[1] & ~branchCtrl[0] &  argsLessThan ? branchCtrl : 2'bz; //blt taken
	assign branchTaken =  branchCtrl[1] & ~branchCtrl[0] &  argsLessThan ? 2'b0       : 2'bz; //blt not taken
	assign branchTaken =  branchCtrl[1] &  branchCtrl[0] &  argsNotEqual ? branchCtrl : 2'bz; //bex taken
	assign branchTaken =  branchCtrl[1] &  branchCtrl[0] & ~argsNotEqual ? 2'b0       : 2'bz; //bex not taken

	cla addBranch(.a(pc), .b(imm), .sub(1'b0), .sum(branchPC), .ovf(gnd), .lessThan(gnd)); //PC += 1 + branchImm

	//Select next PC based on branch or jump
	mux4to1 branchMux(.in0(pc), .in1(branchPC), .in2(branchPC), .in3(target), .sel(branchTaken), .out(summedPC));
	mux4to1 jumpMux(.in0(summedPC), .in1(target), .in2(target), .in3(regBVal), .sel(jumpCtrl), .out(nextPC));

	assign pcOverride = (branchTaken[1] | branchTaken[0]) | (jumpCtrl[1] | jumpCtrl[0]);
endmodule //branchLogic


/**
 * Logic for reading servo values
 */ 
module servoLogic(clock, reset, servoCtrl, angle, servoX, servoY, servoZ, servoReady);

	input[1:0] servoCtrl;
    input[31:0] angle;
    input clock, reset;

    output servoX, servoY, servoZ, servoReady;

    reg[7:0] xPos, yPos, zPos;

    always @(posedge clock) begin
    	casex(servoCtrl)
    		2'b01: xPos = angle[7:0];
    		2'b10: yPos = angle[7:0];
    		2'b11: zPos = angle[7:0];
    	endcase
    end

    armController servos(clock, reset, xPos, yPos, zPos, servoX, servoY, servoZ, servoReady, servoCtrl);

endmodule //servoLogic