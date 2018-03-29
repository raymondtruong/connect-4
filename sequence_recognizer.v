module sequence_recognizer(clock, enable, combos, winner);
	input clock;
	input enable;
	input [451:0] combos;
	output reg [1:0] winner = 2'd0;
	
	reg [451:0] sequence_reg;
	reg [1:0] q;	
	
	initial begin
		 sequence_reg = combos;
	 end
	
	
	always @(posedge clock)
	begin
		if (enable)
		begin
			q <= sequence_reg[451:450];
			sequence_reg <= sequence_reg << 2;
		end
		else
			sequence_reg <= combos;
	end
	
	 
	
	
	
	 
	 
	 
	 
	
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
					if (q == 2'b01) next_state = RED_1;
					else if (q == 2'b10) next_state = YELLOW_1;
					else next_state = INITIAL;
				end
			RED_1: begin
					if (q == 2'b01) next_state = RED_2;
					else if (q == 2'b10) next_state = YELLOW_1;
					else next_state = INITIAL;
				end
			RED_2: begin
					if (q == 2'b01) next_state = RED_3;
					else if (q == 2'b10) next_state = YELLOW_1;
					else next_state = INITIAL;
				end
			RED_3: begin
					if (q == 2'b01) next_state = RED_WIN;
					else if (q == 2'b10) next_state = YELLOW_1;
					else next_state = INITIAL;
				end
			RED_WIN: next_state = RED_WIN;
			YELLOW_1: begin
					if (q == 2'b01) next_state = RED_1;
					else if (q == 2'b10) next_state = YELLOW_2;
					else next_state = INITIAL;
				end
			YELLOW_2: begin
					if (q == 2'b01) next_state = RED_1;
					else if (q == 2'b10) next_state = YELLOW_3;
					else next_state = INITIAL;
				end
			YELLOW_3: begin
					if (q == 2'b01) next_state = RED_1;
					else if (q == 2'b10) next_state = YELLOW_WIN;
					else next_state = INITIAL;
				end
			YELLOW_WIN: next_state = YELLOW_WIN;			
		endcase
	end
   
	// State register
	always @(posedge clock)
	begin
			current_state <= next_state;
	end
	
	// Output logic
	always @(*)
	begin
		if (current_state == RED_WIN) 
			winner <= 2'b01;
		else if (current_state == YELLOW_WIN)
			winner <= 2'b10;
		else
			winner <= 2'b00;
	end
	
endmodule
