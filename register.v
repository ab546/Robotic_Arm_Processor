module register(data, clock, enable, reset, out);

input[31:0] data;
input clock, reset, enable;

output[31:0] out;

genvar n;
generate
    // Build each flip flop in register
    for(n = 0; n < 32; n = n + 1) begin: ffLoop
      ecedffe my_dff(.d(data[n]), .clk(clock), .clrn(~reset), .ena(enable), .q(out[n]));
    end
endgenerate
endmodule // register