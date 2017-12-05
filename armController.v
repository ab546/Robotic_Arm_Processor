module armController(clk, reset, x, /*y, z,*/ servoX/*, servoY, servoZ*/);

	input clk, reset;
	input[7:0] x/*, y, z*/;
	output servoX/*, servoY, servoZ*/;

	reg servoClk;
	wire[18:0] widthX/*, widthY, widthZ*/;
	wire[18:0] count;

	//Slow 50 MHz clock down to 50 Hz
	milCounter counter(~clk, ~reset, count);
	always @(posedge clk) begin
		//Flip clock every 25 Hz (500k ticks)
		if(count == 19'd500000) begin
			servoClk <= ~servoClk;
		end
	end

	//First convert angle to pulse width in microseconds, then convert to ticks
	assign widthX = 19'd20000 - ((19'd2000)*x/(19'd180) + 19'd500)*19'd50;
	//assign widthY = 19'd20000 - ((19'd2000)*y/(19'd180) + 19'd500)*19'd50;
	//assign widthZ = 19'd20000 - ((19'd2000)*z/(19'd180) + 19'd500)*19'd50;

	assign servoX = servoClk & (count < widthX);
	//assign servoY = servoClk & (count < widthY);
	//assign servoZ = servoClk & (count < widthZ);

endmodule // armController


/**
 * Counts to 1000000
 */
module milCounter(clk, reset, out);

	input clk, reset;
	output[18:0] out;

	reg[18:0] out;

	always @(posedge clk) begin
		out = (reset | out == 19'd500000) ? 19'b0 : out + 19'd1;
	end

endmodule // milCounter