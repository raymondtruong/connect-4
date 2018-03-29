module shifter(clock, enable, input_seq, out);
    input clock, enable;
    input [451:0] input_seq;
	 
	 reg [451:0] sequence_reg;
	 initial begin
		{sequence_reg} = input_seq;
	end
	//assign sequence_reg = input_seq;
    output reg [1:0] out;

    always @(posedge clock)
    begin 
		out <= sequence_reg [451:450];
		
		if (enable)
        sequence_reg <= sequence_reg << 2;
		else
			sequence_reg <= input_seq;
	 end
endmodule 



// This module accepts strings of two-bit game pieces in sequence, and will set the two-bit output
// to the same as the input piece when if it detects four of them in a row. Accepts a "go" signal
// and a synchronous active-low reset.

module sequence_recognizer(clock, enable, combos, out, cs);
	input clock;
	input enable;
	output [3:0] cs;
	wire reset = ~enable;
    wire [1:0] in;
	input [451:0] combos;
	output reg [1:0] out = 2'd0;
	
    shifter ss(clock, enable, combos, in);

	assign cs = current_state;
	reg [3:0] current_state, next_state;

	localparam INITIAL = 4'd0,
				  BLUE_1 = 4'd1,
				  BLUE_2 = 4'd2,
				  BLUE_3 = 4'd3,
				  BLUE_WIN = 4'd4,
				  RED_1 = 4'd5,
				  RED_2 = 4'd6,
				  RED_3 = 4'd7,
				  RED_WIN = 4'd8;
				  
	// State table
	always @(*)
	begin
		case (current_state)
		
			INITIAL: begin
					if (in == 2'b01) next_state = BLUE_1;
					else if (in == 2'b10) next_state = RED_1;
					else next_state = INITIAL;
				end
			BLUE_1: begin
					if (in == 2'b01) next_state = BLUE_2;
					else if (in == 2'b10) next_state = RED_1;
					else next_state = INITIAL;
				end
			BLUE_2: begin
					if (in == 2'b01) next_state = BLUE_3;
					else if (in == 2'b10) next_state = RED_1;
					else next_state = INITIAL;
				end
			BLUE_3: begin
					if (in == 2'b01) next_state = BLUE_WIN;
					else if (in == 2'b10) next_state = RED_1;
					else next_state = INITIAL;
				end
			BLUE_WIN: next_state = BLUE_WIN;
			RED_1: begin
					if (in == 2'b01) next_state = BLUE_1;
					else if (in == 2'b10) next_state = RED_2;
					else next_state = INITIAL;
				end
			RED_2: begin
					if (in == 2'b01) next_state = BLUE_1;
					else if (in == 2'b10) next_state = RED_3;
					else next_state = INITIAL;
				end
			RED_3: begin
					if (in == 2'b01) next_state = BLUE_1;
					else if (in == 2'b10) next_state = RED_WIN;
					else next_state = INITIAL;
				end
			RED_WIN: next_state = RED_WIN;
			default: next_state = INITIAL;
			
		endcase
		

	end
   
	// State register
	always @(posedge clock)
	begin
		if (reset)
			current_state <= INITIAL;
		else
			current_state <= next_state;
	end
	
	// Output logic
	always @(*)
	begin
		if (current_state == BLUE_WIN) 
			out <= 2'b01;
		else if (current_state == RED_WIN)
			out <= 2'b10;
		else
			out <= 2'b00;
	end
	
endmodule
