module decoder(HEX, SW);
    input [3:0] SW;
    output [6:0] HEX;
	
	 wire w1;
	 wire w2;
	 
    assign HEX[0] = ~((SW[1] & ~SW[3]) | (SW[1] & SW[2]) | (~SW[0] & ~SW[2]) | 
							(SW[3] & ~SW[2] & ~SW[1]) | (~SW[1] & ~SW[0] & SW[3]) |
							(~SW[3] & SW[2] & SW[0]));
 
    assign HEX[1] = ~((~SW[3] & ~SW[2]) | (~SW[0] & ~SW[2]) |
							(~SW[1] & ~SW[0] & ~SW[3]) | (~SW[1] & SW[0] & SW[3]) |
							(SW[1] & SW[0] & ~SW[3]));
							
	 assign HEX[2] = ~((SW[0] & ~SW[3]) | (~SW[1] & ~SW[3]) | (SW[3] & ~SW[2]) | 
							(~SW[1] & SW[0]) | (~SW[3] & SW[2]));
							
	 assign HEX[3] = ~((~SW[1] & SW[3]) | (SW[1] & SW[0] & ~SW[2]) | (SW[1] & ~SW[0] & SW[2]) | 
							(~SW[3] & ~SW[2] & ~SW[0]) | (~SW[1] & SW[0] & SW[2]));
							
	 assign HEX[4] = ~((~SW[0] & ~SW[2]) | (SW[1] & SW[3]) | (SW[3] & SW[2]) | 
							(SW[1] & ~SW[0]));
							
	 assign HEX[5] = ~((~SW[1] & ~SW[0]) | (SW[1] & SW[3]) | (SW[3] & ~SW[2]) | 
							(~SW[3] & SW[2] & ~SW[1]) | (SW[1] & ~SW[0] & SW[2]));
							
	 assign HEX[6] = ~((SW[1] & ~SW[0]) | (SW[0] & SW[3]) | (~SW[3] & SW[2] & ~SW[1]) | 
							(SW[3] & ~SW[2] & ~SW[1]) | (~SW[3] & ~SW[2] & SW[1]));
			
endmodule
