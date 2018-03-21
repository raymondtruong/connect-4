module main(CLOCK_50, SW, KEY, LEDR);
	
	// Declarations
	input CLOCK_50;
	input [9:0] SW;		// replace with keyboard eventually
	input [3:0] KEY;
	input	[9:0] LEDR;		// replace with monitor eventually
	
	wire reset = ~KEY[3];
	wire go = ~KEY[0];
	
	wire [1:0] turn;
	turn_tracker turn_track(go, CLOCK_50, reset, turn);
	
	// Declare memory model
	reg [5:0] address;
	wire [1:0] data;
	reg wren = 1'b0;
	wire [1:0] q;
	ramTP ram(address, CLOCK_50, turn, go, q);
	
	reg [97:0] board = 98'd0;
	reg [397:0] combos;

	// Define rows, columns, diagonals
	reg [13:0] row_0;
	reg [13:0] row_1;
	reg [13:0] row_2;
	reg [13:0] row_3;
	reg [13:0] row_4;
	reg [13:0] row_5;
	reg [13:0] row_6 ;
	reg [13:0] column_0;
	reg [13:0] column_1 ;
	reg [13:0] column_2 ; 
	reg [13:0] column_3 ; 
	reg [13:0] column_4 ;
	reg [13:0] column_5 ; 
	reg [13:0] column_6 ;
	reg [85:0] diagonal_0;
	reg [85:0] diagonal_1;
	
	// Determine which column to insert into
	reg [2:0] column_0_count = 3'd0;
	reg [2:0] column_1_count = 3'd0;
	reg [2:0] column_2_count = 3'd0;
	reg [2:0] column_3_count = 3'd0;
	reg [2:0] column_4_count = 3'd0;
	reg [2:0] column_5_count = 3'd0;
	reg [2:0] column_6_count = 3'd0;
	reg [2:0] column_7_count = 3'd0;
	reg [4:0] column_count;
	
	reg [4:0] column;
	
	
	
	
	
	
	
	
	
	
	// Calculate which position to insert into
	always @(*)
	begin
		if (SW[6])
			column <= 5'd0;
		else if (SW[5])	
			column <= 5'd1;
		else if (SW[4])
			column <= 5'd2;
		else if (SW[3])
			column <= 5'd3;
		else if (SW[2])
			column <= 5'd4;
		else if (SW[1])
			column <= 5'd5;
		else if (SW[0])
			column <= 5'd6;

		if (column == 5'd0)
			column_count <= column_0_count;
		else if (column == 5'd1)
			column_count <= column_1_count;
		else if (column == 5'd2)
			column_count <= column_2_count;
		else if (column == 5'd3)
			column_count <= column_3_count;
		else if (column == 5'd4)
			column_count <= column_4_count;
		else if (column == 5'd5)
			column_count <= column_5_count;
		else if (column == 5'd6)
			column_count <= column_6_count;
	end
	
	
	
	
	
	
	
	
	
	// Main
	always @(posedge CLOCK_50)
	begin
		
		/////////////////////////
		// go button pressed down
		/////////////////////////
		if (go) 
		begin
			
			/////////////////////////////////////////////////////////////////
			//                Insertion 
			/////////////////////////////////////////////////////////////////
			
			
			// write piece to the right RAM location
			if (column == 5'd0)
				column_0_count <= column_0_count + 1;
			else if (column == 5'd1)
				column_1_count <= column_1_count + 1;
			else if (column == 5'd2)
				column_2_count <= column_2_count + 1;
			else if (column == 5'd3)
				column_3_count <= column_3_count + 1;
			else if (column == 5'd4)
				column_4_count <= column_4_count + 1;
			else if (column == 5'd5)
				column_5_count <= column_5_count + 1;
			else if (column == 5'd6)
				column_6_count <= column_6_count + 1;
				
			address <= column + (5'd7 * (5'd6 - column_count)); 
		end
		
		
		
		/////////////////////////
		// go button not pressed
		/////////////////////////
		else
		begin
		
			/////////////////////////////////////////////////////////////////
			//                Populates board
			/////////////////////////////////////////////////////////////////
			if (address != 6'd49)
			begin
				board <= board << 2;						// shift over to next piece
				board <= {board[95:0], q};				// read from memory at that location
				address <= address + 1'd1;				
			end
			// ...or reset the board
			else if (reset)						
			begin
				address <= 6'd0;
			end
				
				
			/////////////////////////////////////////////////////////////////
			//                 Extract winning combos from the board
			/////////////////////////////////////////////////////////////////
			row_0 <= board[13:0];
			row_1 <= board[27:14];
			row_2 <= board[41:28];
			row_3 <= board[55:42];
			row_4 <= board[69:56];
			row_5 <= board[83:70];
			row_6 <= board[97:84]; 
			column_0 <= {board[0], board[1], 
										board[14], board[15],
										board[28], board[29],
										board[42], board[43], 
										board[56], board[57],
										board[70], board[71],
										board[84], board[85]};
			column_1 <= {board[2], board[3], 
										board[16], board[17],
										board[30], board[31],
										board[44], board[45], 
										board[58], board[59],
										board[72], board[73],
										board[86], board[87]};
			column_2 <= {board[4], board[5], 
										board[18], board[19],
										board[32], board[33],
										board[46], board[47], 
										board[60], board[61],
										board[74], board[75],
										board[88], board[89]};
			column_3 <= {board[6], board[7], 
										board[20], board[21],
										board[34], board[35],
										board[48], board[49], 
										board[62], board[63],
										board[76], board[77],
										board[90], board[91]};
			column_4 <= {board[8], board[9], 
										board[22], board[23],
										board[36], board[37],
										board[50], board[51], 
										board[64], board[65],
										board[78], board[79],
										board[92], board[93]};
			column_5 <= {board[10], board[11], 
										board[24], board[25],
										board[38], board[39],
										board[52], board[53], 
										board[66], board[67],
										board[80], board[81],
										board[94], board[95]};
			column_6 <= {board[12], board[13], 
										board[26], board[27],
										board[40], board[41],
										board[54], board[55], 
										board[68], board[69],
										board[82], board[83],
										board[96], board[97]};
			diagonal_0 <= {board[6], board[7], board[22], board[23], board[38], board[39], board[54], board[55], 2'b00,
									 board[4], board[5], board[20], board[21], board[36], board[37], board[52], board[53], board[68], board[69], 2'b00,
									 board[2], board[3], board[18], board[19], board[34], board[35], board[50], board[51], board[66], board[67], board[82], board[83], 2'b00,
									 board[0], board[1], board[16], board[17], board[32], board[33], board[48], board[49], board[64], board[65], board[80], board[81], board[96], board[97], 2'b00,
									 board[14], board[15], board[30], board[31], board[46], board[47], board[62], board[63], board[78], board[79], board[94], board[95], 2'b00,
									 board[28], board[29], board[44], board[45], board[60], board[61], board[76], board[77], board[92], board[93], 2'b00,
									 board[42], board[43], board[58], board[59], board[74], board[75], board[90], board[91]};
			diagonal_1 <= {board[42], board[43], board[30], board[31], board[18], board[19], board[6], board[7], 2'b00,
									 board[56], board[57], board[44], board[45], board[32], board[33], board[20], board[21], board[8], board[9], 2'b00,
									 board[70], board[71], board[58], board[59], board[46], board[47], board[34], board[35], board[22], board[23], board[10], board[11], 2'b00,
									 board[84], board[85], board[72], board[73], board[60], board[61], board[48], board[49], board[36], board[37], board[24], board[25], board[12], board[13], 2'b00,
									 board[86], board[87], board[74], board[75], board[62], board[63], board[50], board[51], board[38], board[39], board[26], board[27], 2'b00,
									 board[88], board[89], board[76], board[77], board[64], board[65], board[52], board[53], board[40], board[41], 2'b00,
									 board[90], board[91], board[78], board[79], board[66], board[67], board[54], board[55]};
			
			combos <= {row_0, 2'b00, row_1, 2'b00, row_2, 2'b00, row_3, 2'b00, row_4, 2'b00, row_5, 2'b00, row_6, 2'b00,
									column_0, 2'b00, column_1, 2'b00, column_2, 2'b00, column_3, 2'b00, column_4, 2'b00, column_5, 2'b00, column_6,	2'b00, 
									diagonal_0, 2'b00, diagonal_1};
			
			
			
			
			
			
			combos <= combos << 2;			
	 
	 
	 
	 
	 
	 
	 
	 
		end
		
	end	
				
				
	

				
				
			/////////////////////////////////////////////////////////////////
			//                Detect win
			/////////////////////////////////////////////////////////////////
	sequence_recognizer rec(CLOCK_50, reset, combos[397:396], LEDR[0]);
	
endmodule
