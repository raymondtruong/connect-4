// This module accepts strings of two-bit game pieces in sequence, and will set the two-bit output
// to the same as the input piece when if it detects four of them in a row. Accepts a "go" signal
// and a synchronous active-low reset.

module sequence_recognizer(clock, reset, in, out);
    	
	input clock;
	input reset;
	input [1:0] in;
	output reg [1:0] out = 2'd0;
	
	reg [3:0] current_state, next_state;
	localparam INITIAL = 4'd0,
				  RED_1 = 4'd1,
				  RED_2 = 4'd2,
				  RED_3 = 4'd3,
				  RED_WIN = 4'd4,
				  YELLOW_1 = 4'd5,
				  YELLOW_2 = 4'd6,
				  YELLOW_3 = 4'd7,
				  YELLOW_WIN = 4'd8;
				  
	// State table
	always @(*)
	begin
		case (current_state)
		
			INITIAL: begin
					if (in == 2'b01) next_state = RED_1;
					else if (in == 2'b10) next_state = YELLOW_1;
					else next_state = INITIAL;
				end
			RED_1: begin
					if (in == 2'b01) next_state = RED_2;
					else if (in == 2'b10) next_state = YELLOW_1;
					else next_state = INITIAL;
				end
			RED_2: begin
					if (in == 2'b01) next_state = RED_3;
					else if (in == 2'b10) next_state = YELLOW_1;
					else next_state = INITIAL;
				end
			RED_3: begin
					if (in == 2'b01) next_state = RED_WIN;
					else if (in == 2'b10) next_state = YELLOW_1;
					else next_state = INITIAL;
				end
			RED_WIN: begin
					if (in == 2'b01) next_state = RED_WIN;
					else if (in == 2'b10) next_state = YELLOW_1;
					else next_state = INITIAL;
				end
			YELLOW_1: begin
					if (in == 2'b01) next_state = RED_1;
					else if (in == 2'b10) next_state = YELLOW_2;
					else next_state = INITIAL;
				end
			YELLOW_2: begin
					if (in == 2'b01) next_state = RED_1;
					else if (in == 2'b10) next_state = YELLOW_3;
					else next_state = INITIAL;
				end
			YELLOW_3: begin
					if (in == 2'b01) next_state = RED_1;
					else if (in == 2'b10) next_state = YELLOW_WIN;
					else next_state = INITIAL;
				end
			YELLOW_WIN: begin
					if (in == 2'b01) next_state = RED_1;
					else if (in == 2'b10) next_state = YELLOW_WIN;
					else next_state = INITIAL;
				end
			default: next_state = INITIAL;
			
		endcase
	end
   
	// State register
	always @(posedge clock)
	begin
		if (reset == 1'b0)
			current_state <= INITIAL;
		else
			current_state <= next_state;
	end
	
	// Output logic
	always @(*)
	begin
		if (current_state == RED_WIN) 
			out <= 2'b01;
		else if (current_state == YELLOW_WIN)
			out <= 2'b10;
		else
			out <= 2'b00;
	end
	
endmodule
