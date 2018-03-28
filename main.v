module main(CLOCK_50, PS2_DAT, PS2_CLK, KEY, LEDR);
	
	/////////////////////////////////////////////////////////////////////////////
	//                               Declarations                              //
	/////////////////////////////////////////////////////////////////////////////

	input CLOCK_50;
	input PS2_DAT, PS2_CLK;
	input [3:0] KEY;
	output [9:0] LEDR;	
	
	wire reset = ~KEY[3];
	
	
	
	
	
	/////////////////////////////////////////////////////////////////////////////
	//                                  I/O                                    //
	/////////////////////////////////////////////////////////////////////////////
	
	
	// Credit: https://class.ee.washington.edu/271/hauck2/de1/keyboard/PS2Keyboard.pdf
	//         https://class.ee.washington.edu/271/hauck2/de1/keyboard/KeyboardFiles.zip
	wire valid, makeBreak;
	wire [7:0] outCode;
	keyboard_press_driver keyboard(CLOCK_50, valid, makeBreak, outCode, PS2_DAT, PS2_CLK, reset);
	
	// Keyboard scan codes (set 2)
	localparam KEY_Z = 8'h1A,
				  KEY_X = 8'h22,
				  KEY_C = 8'h21,
				  KEY_V = 8'h2A,
				  KEY_B = 8'h32,
				  KEY_N = 8'h31,
				  KEY_M = 8'h3A;
				  
	wire go = valid && makeBreak;

	
	// Keep track of whose turn it is (and also which piece to insert)
	wire [1:0] turn;
	turn_tracker tt(go, reset, turn);
	
	assign LEDR[9] = turn == 2'b01 ? 1'b1 : 1'b0;
	assign LEDR[0] = turn == 2'b10 ? 1'b1 : 1'b0;

	
	
	
	
	/////////////////////////////////////////////////////////////////////////////
	//                             Memory Model                                //
	/////////////////////////////////////////////////////////////////////////////
	
	// One two-bit register for every cell on the board
	reg [5:0] cell_number;
	reg [1:0] c_0, c_1, c_2, c_3, c_4, c_5, c_6, c_7, c_8, c_9, 
				 c_10, c_11, c_12, c_13, c_14, c_15, c_16, c_17, c_18, c_19,
				 c_20, c_21, c_22, c_23, c_24, c_25, c_26, c_27, c_28, c_29,
				 c_30, c_31, c_32, c_33, c_34, c_35, c_36, c_37, c_38, c_39,
				 c_40, c_41, c_42, c_43, c_44, c_45, c_46, c_47, c_48;
	// Initialize each cell to 00
	// Credit: https://stackoverflow.com/questions/45512230/verilog-assigning-multiple-regs-or-wires-to-the-same-value
	initial begin
		{c_0, c_1, c_2, c_3, c_4, c_5, c_6, c_7, c_8, c_9, 
		 c_10, c_11, c_12, c_13, c_14, c_15, c_16, c_17, c_18, c_19,
		 c_20, c_21, c_22, c_23, c_24, c_25, c_26, c_27, c_28, c_29,
		 c_30, c_31, c_32, c_33, c_34, c_35, c_36, c_37, c_38, c_39,
		 c_40, c_41, c_42, c_43, c_44, c_45, c_46, c_47, c_48} = 2'b00;
	end
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
	//                               Game state                                //
	/////////////////////////////////////////////////////////////////////////////
	
	always @(*)
	begin
		
		// Determine which column to insert into
		case (outCode)
			KEY_Z: begin
					 column <= 5'd0;
					 column_count <= column_0_count;
					 end
			KEY_X: begin
					 column <= 5'd1;
					 column_count <= column_1_count;
					 end
			KEY_C: begin
					 column <= 5'd2;
					 column_count <= column_2_count;
					 end
			KEY_V: begin
					 column <= 5'd3;
					 column_count <= column_3_count;
					 end
			KEY_B: begin
			       column <= 5'd4;
					 column_count <= column_4_count;
					 end
			KEY_N: begin
			       column <= 5'd5;
					 column_count <= column_5_count;
					 end
			KEY_M: begin
			       column <= 5'd6;
					 column_count <= column_6_count;
					 end
		endcase
		
		// Determine what cell to insert into
		cell_number <= column + (5'd7 * (5'd6 - column_count)); 
		
		
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
	

	
	
	
	
	/////////////////////////////////////////////////////////////////////////////
	//                              Insertion                                  //
	/////////////////////////////////////////////////////////////////////////////

	always @(posedge go)
	begin
	
		// Write to the cell if nothing's there
		case (cell_number)
			6'd0: begin
					if (c_0 == 2'b00 & go) c_0 <= turn;
					end
			6'd1: begin
					if (c_1 == 2'b00 & go) c_1 <= turn;
					end
			6'd2: begin
					if (c_2 == 2'b00 & go) c_2 <= turn;
					end
			6'd3: begin
					if (c_3 == 2'b00 & go) c_3 <= turn;
					end
			6'd4: begin
					if (c_4 == 2'b00 & go) c_4 <= turn;
					end
			6'd5: begin
					if (c_5 == 2'b00 & go) c_5 <= turn;
					end
			6'd6: begin
					if (c_6 == 2'b00 & go) c_6 <= turn;
					end
			6'd7: begin
					if (c_7 == 2'b00 & go) c_7 <= turn;
					end
			6'd8: begin
					if (c_8 == 2'b00 & go) c_8 <= turn;
					end
			6'd9: begin
					if (c_9 == 2'b00 & go) c_9 <= turn;
					end
			6'd10: begin
					if (c_10 == 2'b00 & go) c_10 <= turn;
					end
			6'd11: begin
					if (c_11 == 2'b00 & go) c_11 <= turn;
					end
			6'd12: begin
					if (c_12 == 2'b00 & go) c_12 <= turn;
					end
			6'd13: begin
					if (c_13 == 2'b00 & go) c_13 <= turn;
					end
			6'd14: begin
					if (c_14 == 2'b00 & go) c_14 <= turn;
					end
			6'd15: begin
					if (c_15 == 2'b00 & go) c_15 <= turn;
					end
			6'd16: begin
					if (c_16 == 2'b00 & go) c_16 <= turn;
					end
			6'd17: begin
					if (c_17 == 2'b00 & go) c_17 <= turn;
					end
			6'd18: begin
					if (c_18 == 2'b00 & go) c_18 <= turn;
					end
			6'd19: begin
					if (c_19 == 2'b00 & go) c_19 <= turn;
					end
			6'd20: begin
					if (c_20 == 2'b00 & go) c_20 <= turn;
					end
			6'd21: begin
					if (c_21 == 2'b00 & go) c_21 <= turn;
					end
			6'd22: begin
					if (c_22 == 2'b00 & go) c_22 <= turn;
					end
			6'd23: begin
					if (c_23 == 2'b00 & go) c_23 <= turn;
					end
			6'd24: begin
					if (c_24 == 2'b00 & go) c_24 <= turn;
					end
			6'd25: begin
					if (c_25 == 2'b00 & go) c_25 <= turn;
					end
			6'd26: begin
					if (c_26 == 2'b00 & go) c_26 <= turn;
					end
			6'd27: begin
					if (c_27 == 2'b00 & go) c_27 <= turn;
					end
			6'd28: begin
					if (c_28 == 2'b00 & go) c_28 <= turn;
					end
			6'd29: begin
					if (c_29 == 2'b00 & go) c_29 <= turn;
					end
			6'd30: begin
					if (c_30 == 2'b00 & go) c_30 <= turn;
					end
			6'd31: begin
					if (c_31 == 2'b00 & go) c_31 <= turn;
					end
			6'd32: begin
					if (c_32 == 2'b00 & go) c_32 <= turn;
					end
			6'd33: begin
					if (c_33 == 2'b00 & go) c_33 <= turn;
					end
			6'd34: begin
					if (c_34 == 2'b00 & go) c_34 <= turn;
					end
			6'd35: begin
					if (c_35 == 2'b00 & go) c_35 <= turn;
					end
			6'd36: begin
					if (c_36 == 2'b00 & go) c_36 <= turn;
					end
			6'd37: begin
					if (c_37 == 2'b00 & go) c_37 <= turn;
					end
			6'd38: begin
					if (c_38 == 2'b00 & go) c_38 <= turn;
					end
			6'd39: begin
					if (c_39 == 2'b00 & go) c_39 <= turn;
					end
			6'd40: begin
					if (c_40 == 2'b00 & go) c_40 <= turn;
					end
			6'd41: begin
					if (c_41 == 2'b00 & go) c_41 <= turn;
					end
			6'd42: begin
					if (c_42 == 2'b00 & go) c_42 <= turn;
					end
			6'd43: begin
					if (c_43 == 2'b00 & go) c_43 <= turn;
					end
			6'd44: begin
					if (c_44 == 2'b00 & go) c_44 <= turn;
					end
			6'd45: begin
					if (c_45 == 2'b00 & go) c_45 <= turn;
					end
			6'd46: begin
					if (c_46 == 2'b00 & go) c_46 <= turn;
					end
			6'd47: begin
					if (c_47 == 2'b00 & go) c_47 <= turn;
					end
			6'd48: begin
					if (c_48 == 2'b00 & go) c_48 <= turn;
					end
		endcase
		
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
	sequence_recognizer rec(CLOCK_50, go, combos, winner);
	
	
	
endmodule
