module ecedffe(d, clk, clrn, ena, q);
    input d, clk, ena, clrn;
    wire clr;
    output q;
    reg q;

    assign clr = ~clrn;
    initial
    begin
        q = 1'b0;
    end

    always @(posedge clk or posedge clr) begin
        if (q == 1'bx) begin
            q = 1'b0;
        end else if (clr) begin
            q <= 1'b0;
        end else if (ena) begin
            q <= d;
        end
    end
endmodule