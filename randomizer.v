module randomizer(go, CLOCK_50, out);
	input go;
	input CLOCK_50;
	output reg [2:0] out;
	
	reg [2:0] q = 3'b000;
	always @(posedge CLOCK_50)
	begin
		if (go)		// enable or disable counting
			begin
				if (q == 3'b111)
					q <= 3'b000;
				else
					q <= q + 1'b1;
			end
			
		out <= q;
	end
endmodule
