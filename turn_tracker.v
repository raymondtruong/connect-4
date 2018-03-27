// Synchronous active low reset T flip flop that keeps track of whose turn it is. Output
// toggles between 01 and 10, representing each colour of game piece.
module turn_tracker(enable, reset, q);

	input enable, reset; 
	output reg [1:0] q = 2'b01;

	always @(posedge enable or posedge reset)
	begin
		if (reset)
			q <= 2'b01;
		else if (q == 2'b01) 
			q <= 2'b10;
		else if (q == 2'b10)
			q <= 2'b01;
	end

endmodule 