module main(CLOCK_50, SW, KEY, LEDR);
	
	/////////////////////////////////////////////////////////////////////////////
	//                               Declarations                              //
	/////////////////////////////////////////////////////////////////////////////

	input CLOCK_50;
	input [9:0] SW;			// replace with keyboard eventually
	input [3:0] KEY;
	output [9:0] LEDR;		// replace with monitor eventually
	
	wire go = ~KEY[0];
	wire reset = ~KEY[3];
	
	// Keep track of whose turn it is (and also which piece to insert)
	wire [1:0] turn;
	turn_tracker tt(go, reset, turn);
	
	// One two-bit register for every cell on the board
	reg [5:0] cell_number;
	reg [1:0] c_0, c_1, c_2, c_3, c_4, c_5, c_6, c_7, c_8, c_9, 
				 c_10, c_11, c_12, c_13, c_14, c_15, c_16, c_17, c_18, c_19,
				 c_20, c_21, c_22, c_23, c_24, c_25, c_26, c_27, c_28, c_29,
				 c_30, c_31, c_32, c_33, c_34, c_35, c_36, c_37, c_38, c_39,
				 c_40, c_41, c_42, c_43, c_44, c_45, c_46, c_47, c_48;
	reg [97:0] board; 
	
	reg [397:0] combos;
	reg [13:0] column_0, column_1, column_2, column_3, column_4, column_5, column_6,
				  row_0, row_1, row_2, row_3, row_4, row_5, row_6, diagonal_0, diagonal_1;
				  
	reg [4:0] column;
	reg [2:0] column_0_count = 3'd0;
	reg [2:0] column_1_count = 3'd0;
	reg [2:0] column_2_count = 3'd0;
	reg [2:0] column_3_count = 3'd0;
	reg [2:0] column_4_count = 3'd0;
	reg [2:0] column_5_count = 3'd0;
	reg [2:0] column_6_count = 3'd0;
	reg [2:0] column_7_count = 3'd0;
	reg [4:0] column_count;
	
	
	
	
	/////////////////////////////////////////////////////////////////////////////
	//                               Insertion                                 //
	/////////////////////////////////////////////////////////////////////////////
	
	always @(*)
	begin
		// Determine which column to insert into
		if (SW[6])
		begin
			column <= 5'd0;
			column_count <= column_0_count;
		end
		else if (SW[5])	
		begin
			column <= 5'd1;
			column_count <= column_1_count;
		end
		else if (SW[4])
		begin
			column <= 5'd2;
			column_count <= column_2_count;
		end
		else if (SW[3])
		begin
			column <= 5'd3;
			column_count <= column_3_count;
		end
		else if (SW[2])
		begin	
			column <= 5'd4;
			column_count <= column_4_count;
		end
		else if (SW[1])
		begin
			column <= 5'd5;
			column_count <= column_5_count;
		end
		else if (SW[0])
		begin
			column <= 5'd6;
			column_count <= column_6_count;
		end
		else
		begin
			column <= 5'd6;
			column_count <= column_6_count;
		end
		
		// Determine what cell to go into
		cell_number <= column + (5'd7 * (5'd6 - column_count)); 

		case (cell_number)
			6'd0: c_0 <= turn;
			6'd1: c_1 <= turn;
			6'd2: c_2 <= turn;
			6'd3: c_3 <= turn;
			6'd4: c_4 <= turn;
			6'd5: c_5 <= turn;
			6'd6: c_6 <= turn;
			6'd7: c_7 <= turn;
			6'd8: c_8 <= turn;
			6'd9: c_9 <= turn;
			6'd10: c_10 <= turn;
			6'd11: c_11 <= turn;
			6'd12: c_12 <= turn;
			6'd13: c_13 <= turn;
			6'd14: c_14 <= turn;
			6'd15: c_15 <= turn;
			6'd16: c_16 <= turn;
			6'd17: c_17 <= turn;
			6'd18: c_18 <= turn;
			6'd19: c_19 <= turn;
			6'd20: c_20 <= turn;
			6'd21: c_21 <= turn;
			6'd22: c_22 <= turn;
			6'd23: c_23 <= turn;
			6'd24: c_24 <= turn;
			6'd25: c_25 <= turn;
			6'd26: c_26 <= turn;
			6'd27: c_27 <= turn;
			6'd28: c_28 <= turn;
			6'd29: c_29 <= turn;
			6'd30: c_30 <= turn;
			6'd31: c_31 <= turn;
			6'd32: c_32 <= turn;
			6'd33: c_33 <= turn;
			6'd34: c_34 <= turn;
			6'd35: c_35 <= turn;
			6'd36: c_36 <= turn;
			6'd37: c_37 <= turn;
			6'd38: c_38 <= turn;
			6'd39: c_39 <= turn;
			6'd40: c_40 <= turn;
			6'd41: c_41 <= turn;
			6'd42: c_42 <= turn;
			6'd43: c_43 <= turn;
			6'd44: c_44 <= turn;
			6'd45: c_45 <= turn;
			6'd46: c_46 <= turn;
			6'd47: c_47 <= turn;
			6'd48: c_48 <= turn;
		endcase
		
		// Assemble board
		board <= {c_0, c_1, c_2, c_3, c_4, c_5, c_6, c_7, c_8, c_9, 
				 c_10, c_11, c_12, c_13, c_14, c_15, c_16, c_17, c_18, c_19,
				 c_20, c_21, c_22, c_23, c_24, c_25, c_26, c_27, c_28, c_29,
				 c_30, c_31, c_32, c_33, c_34, c_35, c_36, c_37, c_38, c_39,
				 c_40, c_41, c_42, c_43, c_44, c_45, c_46, c_47, c_48};	
		
		
		// Assemble winning combinations
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
	end
	

	// Insert into column when the button is actually pressed
	always @(posedge go)
	begin
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
	end
	
	
	
	/////////////////////////////////////////////////////////////////////////////
	//                            Determine Winner                             //
	/////////////////////////////////////////////////////////////////////////////
	
	wire [1:0] winner;
	sequence_recognizer(CLOCK_50, go, combos, winner);
	
	
	
endmodule
