module armController(clk, reset, x, y, z, servoX, servoY, servoZ, ready, servoCtrl);

	input clk, reset;
	input [1:0] servoCtrl;
	input[7:0] x, y, z;
	output servoX, servoY, servoZ, ready;

	reg servoClk;
	wire[18:0] widthX, widthY, widthZ;
	wire[19:0] count;

	//Slow 50 MHz clock down to 50 Hz
	milCounter counter(~clk, reset, count);
	always @(posedge clk) begin
		//Flip clock every 25 Hz (500k ticks)
		if(count == 20'd999999 | count == 20'd499999) begin
			servoClk <= ~servoClk;
		end
	end

	assign ready = (count == 20'd999999) | reset;
	
	assign widthX = (((19'd2000)*x/(19'd180) + 19'd500)*19'd50); //3-185 degrees
	assign widthY = (((19'd2000)*y/(19'd180) + 19'd500)*19'd50); //8-156 degrees
	assign widthZ = (((19'd2000)*z/(19'd180) + 19'd500)*19'd50); //4-128 degrees

	assign servoX = ~(servoClk & (count < widthX) & (servoCtrl == 2'b01));
	assign servoY = ~(servoClk & (count < widthY) & (servoCtrl == 2'b10));
	assign servoZ = ~(servoClk & (count < widthZ) & (servoCtrl == 2'b11));

endmodule // armController


/**
 * Counts to 1000000
 */
module milCounter(clk, reset, out);

	input clk, reset;
	output[19:0] out;

	reg[19:0] out;

	always @(posedge clk) begin
		out = (reset | out == 20'd999999) ? 20'b0 : out + 20'd1;
	end

endmodule // milCounter