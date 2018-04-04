module score_system(clock, HEX, enable, reset);
    input clock, enable, reset;
    output [6:0] HEX;

    reg [6:0] score = 7'd0;
    assign HEX = score;
    always @(posedge  clock)
    begin
        if (reset == 1'b1)
        begin 
            score <= 7'd0;
        end
        else if(enable )
        begin
            score <= score + 7'd1;
        end
        else 
    end

endmodule


module counter(clock, reset_n, enable, q);
	input 				clock, reset_n, enable;
	output reg 	[2:0] 	q;
	
	always @(posedge clock) begin
		if(reset_n == 1'b0)
			q <= 3'b000;
		else if(enable == 1'b1) 
		begin
		
		  if (q == 3'b101)
			  q <= 3'b000;
		  else
			  q <= q + 1'b1;
		end
   end
endmodule