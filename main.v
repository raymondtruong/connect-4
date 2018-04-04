module main(CLOCK_50, PS2_DAT, PS2_CLK, SW, KEY, LEDR, HEX0, HEX1, HEX2, HEX3, HEX4,
		VGA_CLK,   						//	VGA Clock 
 		VGA_HS,							//	VGA H_SYNC 
 		VGA_VS,							//	VGA V_SYNC 
 		VGA_BLANK_N,					//	VGA BLANK 
 		VGA_SYNC_N,						//	VGA SYNC 
 		VGA_R,   						//	VGA Red[9:0] 
 		VGA_G,	 						//	VGA Green[9:0] 
 		VGA_B   							//	VGA Blue[9:0] 
);
	
	/////////////////////////////////////////////////////////////////////////////
	//                               Declarations                              //
	/////////////////////////////////////////////////////////////////////////////

	input CLOCK_50;
	input PS2_DAT, PS2_CLK;
	input [3:0] KEY;
	input [9:0] SW;
	output [6:0] HEX0;
	output [6:0] HEX1;
	output [6:0] HEX2;
	output [6:0] HEX3;
	output [6:0] HEX4;
	output [9:0] LEDR;	
	wire reset = ~KEY[3];


		// Credit: https://class.ee.washington.edu/271/hauck2/de1/keyboard/
	wire valid, makeBreak;
	wire [7:0] outCode;

			localparam KEY_Z = 8'h1A,
				  KEY_X = 8'h22,
				  KEY_C = 8'h21,
				  KEY_V = 8'h2A,
				  KEY_B = 8'h32,
				  KEY_N = 8'h31,
				  KEY_M = 8'h3A,
				  KEY_SPACE = 8'h29;
	
	wire insert = valid && makeBreak && outCode == KEY_SPACE;
	

	// Keep track of whose turn it is (and also which piece to insert)
	wire [1:0] turn;
	turn_tracker tt(insert, reset, turn);
	wire [2:0] player_color; 

	assign player_color = turn == 2'b01 ? 3'b100 : 3'b001;


	// AI
	wire ai = SW[9];
	wire [5:0] randomized;
	randomizer ra(select, CLOCK_50, randomized);
	
	// VGA 
 	reg [7:0] cell_x; 
 	reg [6:0] cell_y; 
	wire resetn; 
 	assign resetn = KEY[2]; 

 	 
 	// Do not change the following outputs 
 	output			VGA_CLK;   				//	VGA Clock 
 	output			VGA_HS;					//	VGA H_SYNC 
 	output			VGA_VS;					//	VGA V_SYNC 
 	output			VGA_BLANK_N;				//	VGA BLANK 
 	output			VGA_SYNC_N;				//	VGA SYNC 
 	output	[9:0]	VGA_R;   				//	VGA Red[9:0] 
 	output	[9:0]	VGA_G;	 				//	VGA Green[9:0] 
 	output	[9:0]	VGA_B;   				//	VGA Blue[9:0] 
 	 
 	// Create the colour, x, y and writeEn wires that are inputs to the controller. 
 	//wire [2:0] colour = SW[9:7]; 
 	wire [7:0] x; 
 	wire [6:0] y; 
 	wire writeEn; 

	// Create an Instance of a VGA controller - there can be only one!
	// Define the number of colours as well as the initial background
	// image file (.MIF) for the controller.
	vga_adapter VGA(
			.resetn(resetn),
			.clock(CLOCK_50),
			.colour(player_color),
			.x(cell_x),
			.y(cell_y),
			.plot(writeEn),
			/* Signals for the DAC to drive the monitor. */
			.VGA_R(VGA_R),
			.VGA_G(VGA_G),
			.VGA_B(VGA_B),
			.VGA_HS(VGA_HS),
			.VGA_VS(VGA_VS),
			.VGA_BLANK(VGA_BLANK_N),
			.VGA_SYNC(VGA_SYNC_N),
			.VGA_CLK(VGA_CLK));
		defparam VGA.RESOLUTION = "160x120";		
		defparam VGA.MONOCHROME = "FALSE";
		defparam VGA.BITS_PER_COLOUR_CHANNEL = 1;
		defparam VGA.BACKGROUND_IMAGE = "black.mif";
			
	// Put your code here. Your code should produce signals x,y,colour and writeEn/plot
	// for the VGA controller, in addition to any other functionality your design may require.
    
// Instansiate datapath
datapath d0(
	.clk(CLOCK_50), 
	.resetn(KEY[2]),
	.writeEnable(writeEn),
	.color(player_color),
	.inX(cell_x),
	.inY(cell_y),
	.color_out(colour),
	.x(x),
	.y(y)
);

// Instansiate FSM control
control c0(
	.clk(CLOCK_50),
	.resetn(KEY[2]),
	.go(insert),
	.writeEnable(writeEn)	
);

	
	
	/////////////////////////////////////////////////////////////////////////////
	//                                  I/O                                    //
	/////////////////////////////////////////////////////////////////////////////
	
	




	keyboard_press_driver keyboard(CLOCK_50, valid, makeBreak, outCode, PS2_DAT, PS2_CLK, reset);
	

				  
	wire select = valid && makeBreak && (outCode == KEY_Z ||
										 outCode == KEY_X ||
										 outCode == KEY_C ||
										 outCode == KEY_V ||
										 outCode == KEY_B ||
										 outCode == KEY_N ||
										 outCode == KEY_M);
								 	
	

	
	
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
	
	reg [451:0] combos;	
	reg [13:0] column_0, column_1, column_2, column_3, column_4, column_5, column_6,
				  row_0, row_1, row_2, row_3, row_4, row_5, row_6, diagonal_0, diagonal_1;
				  
	reg [4:0] column;
	
	reg [2:0] column_0_count, column_1_count, column_2_count, column_3_count, column_4_count, column_5_count, column_6_count;
	initial begin
		{column_0_count, column_1_count, column_2_count, column_3_count, column_4_count, column_5_count, column_6_count} = 3'd0;
	end
		
	
		

	
	
	/////////////////////////////////////////////////////////////////////////////
	//                              Insertion                                  //
	/////////////////////////////////////////////////////////////////////////////

	always @(posedge select)
	begin
		
		// Determine which column to insert into
		if (~ai)
		begin
			case (outCode)
				KEY_Z: begin
						 column <= 5'd0;
						 end
				KEY_X: begin
						 column <= 5'd1;
						 end
				KEY_C: begin
						 column <= 5'd2;
						 end
				KEY_V: begin
						 column <= 5'd3;
						 end
				KEY_B: begin
						 column <= 5'd4;
						 end
				KEY_N: begin
						 column <= 5'd5;
						 end
				KEY_M: begin
						 column <= 5'd6; 
						 end
			endcase
		end
		else
			begin
				column <= randomized; 
			end
		end
	end

	always @(negedge select)
	begin
	
		// Determine what column to insert into
		case (column)
			5'd0: cell_number <= 0 + (5'd7 * (5'd6 - column_0_count)); 
			5'd1: cell_number <= 1 + (5'd7 * (5'd6 - column_1_count));
			5'd2: cell_number <= 2 + (5'd7 * (5'd6 - column_2_count));
			5'd3: cell_number <= 3 + (5'd7 * (5'd6 - column_3_count));
			5'd4: cell_number <= 4 + (5'd7 * (5'd6 - column_4_count)); 
			5'd5: cell_number <= 5 + (5'd7 * (5'd6 - column_5_count));
			5'd6: cell_number <= 6 + (5'd7 * (5'd6 - column_6_count));
		endcase
	end

		
	always @(posedge insert)
	begin
									
							
		// Write to the cell if nothing's there
		case (cell_number)
			6'd0: begin
					if (c_0 == 2'b00) 
						begin
							c_0 <= turn;
							cell_x <= 8'd8;
							cell_y <= 7'd5;
						end
					end
			6'd1: begin
					if (c_1 == 2'b00)
						begin
							c_1 <= turn;
							cell_x <= 8'd30;
							cell_y <= 7'd5;
						end
					end
			6'd2: begin
					if (c_2 == 2'b00)
						begin
							c_2 <= turn;
							cell_x <= 8'd52;
							cell_y <= 7'd5;
						end
					end
			6'd3: begin
					if (c_3 == 2'b00)
						begin
							c_3 <= turn;
							cell_x <= 8'd74;
							cell_y <= 7'd5;
						end
					end
			6'd4: begin
					if (c_4 == 2'b00)
						begin
							c_4 <= turn;
							cell_x <= 8'd96;
							cell_y <= 7'd5;
						end
					end
			6'd5: begin
					if (c_5 == 2'b00)
						begin
							c_5 <= turn;
							cell_x <= 8'd118;
							cell_y <= 7'd5;
						end
					end
			6'd6: begin
					if (c_6 == 2'b00)
						begin
							c_6 <= turn;
							cell_x <= 8'd140;
							cell_y <= 7'd5;
						end
					end
			6'd7: begin
					if (c_7 == 2'b00)
						begin
							c_7 <= turn;
							cell_x <= 8'd8;
							cell_y <= 7'd21;
						end
					end
			6'd8: begin
					if (c_8 == 2'b00)
						begin
							c_8 <= turn;
							cell_x <= 8'd30;
							cell_y <= 7'd21;
						end
					end
			6'd9: begin
					if (c_9 == 2'b00)
						begin
							c_9 <= turn;
							cell_x <= 8'd52;
							cell_y <= 7'd21;
						end
					end
			6'd10: begin
					if (c_10 == 2'b00)
						begin
							c_10 <= turn;
							cell_x <= 8'd74;
							cell_y <= 7'd21;
						end
					end
			6'd11: begin
					if (c_11 == 2'b00)
						begin
							c_11 <= turn;
							cell_x <= 8'd96;
							cell_y <= 7'd21;
						end
					end
			6'd12: begin
					if (c_12 == 2'b00)
						begin
							c_12 <= turn;
							cell_x <= 8'd118;
							cell_y <= 7'd21;
						end
					end
			6'd13: begin
					if (c_13 == 2'b00)
						begin
							c_13 <= turn;
							cell_x <= 8'd140;
							cell_y <= 7'd21;
						end
					end
			6'd14: begin
					if (c_14 == 2'b00)
						begin
							c_14 <= turn;
							cell_x <= 8'd8;
							cell_y <= 7'd37;
						end
					end
			6'd15: begin
					if (c_15 == 2'b00)
						begin
							c_15 <= turn;
							cell_x <= 8'd30;
							cell_y <= 7'd37;
						end
					end
			6'd16: begin
					if (c_16 == 2'b00)
						begin
							c_16 <= turn;
							cell_x <= 8'd52;
							cell_y <= 7'd37;
						end
					end
			6'd17: begin
					if (c_17 == 2'b00)
						begin
							c_17 <= turn;
							cell_x <= 8'd74;
							cell_y <= 7'd37;
						end
					end
			6'd18: begin
					if (c_18 == 2'b00)
						begin
							c_18 <= turn;
							cell_x <= 8'd96;
							cell_y <= 7'd37;
						end
					end
			6'd19: begin
					if (c_19 == 2'b00)
						begin
							c_19 <= turn;
							cell_x <= 8'd118;
							cell_y <= 7'd37;
						end
					end
			6'd20: begin
					if (c_20 == 2'b00)
						begin
							c_20 <= turn;
							cell_x <= 8'd140;
							cell_y <= 7'd37;
						end
					end
			6'd21: begin
					if (c_21 == 2'b00)
						begin
							c_21 <= turn;
							cell_x <= 8'd8;
							cell_y <= 7'd53;
						end
					end
			6'd22: begin
					if (c_22 == 2'b00)
						begin
							c_22 <= turn;
							cell_x <= 8'd30;
							cell_y <= 7'd53;
						end
					end
			6'd23: begin
					if (c_23 == 2'b00)
						begin
							c_23 <= turn;
							cell_x <= 8'd52;
							cell_y <= 7'd53;
						end
					end
			6'd24: begin
					if (c_24 == 2'b00)
						begin
							c_24 <= turn;
							cell_x <= 8'd74;
							cell_y <= 7'd53;
						end
					end
			6'd25: begin
					if (c_25 == 2'b00)
						begin
							c_25 <= turn;
							cell_x <= 8'd96;
							cell_y <= 7'd53;
						end
					end
			6'd26: begin
					if (c_26 == 2'b00)
						begin
							c_26 <= turn;
							cell_x <= 8'd118;
							cell_y <= 7'd53;
						end
					end
			6'd27: begin
					if (c_27 == 2'b00)
						begin
							c_27 <= turn;
							cell_x <= 8'd140;
							cell_y <= 7'd53;
						end
					end
			6'd28: begin
					if (c_28 == 2'b00)
						begin
							c_28 <= turn;
							cell_x <= 8'd8;
							cell_y <= 7'd69;
						end
					end
			6'd29: begin
					if (c_29 == 2'b00)
						begin
							c_29 <= turn;
							cell_x <= 8'd30;
							cell_y <= 7'd69;
						end
					end
			6'd30: begin
					if (c_30 == 2'b00)
						begin
							c_30 <= turn;
							cell_x <= 8'd52;
							cell_y <= 7'd69;
						end
					end
			6'd31: begin
					if (c_31 == 2'b00)
						begin
							c_31 <= turn;
							cell_x <= 8'd74;
							cell_y <= 7'd69;
						end
					end
			6'd32: begin
					if (c_32 == 2'b00)
						begin
							c_32 <= turn;
							cell_x <= 8'd96;
							cell_y <= 7'd69;
						end
					end
			6'd33: begin
					if (c_33 == 2'b00)
						begin
							c_33 <= turn;
							cell_x <= 8'd118;
							cell_y <= 7'd69;
						end
					end
			6'd34: begin
					if (c_34 == 2'b00)
						begin
							c_34 <= turn;
							cell_x <= 8'd140;
							cell_y <= 7'd69;
						end
					end
			6'd35: begin
					if (c_35 == 2'b00)
						begin
							c_35 <= turn;
							cell_x <= 8'd8;
							cell_y <= 7'd85;
						end
					end
			6'd36: begin
					if (c_36 == 2'b00)
						begin
							c_36 <= turn;
							cell_x <= 8'd30;
							cell_y <= 7'd85;
						end
					end
			6'd37: begin
					if (c_37 == 2'b00)
						begin
							c_37 <= turn;
							cell_x <= 8'd52;
							cell_y <= 7'd85;
						end
					end
			6'd38: begin
					if (c_38 == 2'b00)
						begin
							c_38 <= turn;
							cell_x <= 8'd74;
							cell_y <= 7'd85;
						end
					end
			6'd39: begin
					if (c_39 == 2'b00)
						begin
							c_39 <= turn;
							cell_x <= 8'd96;
							cell_y <= 7'd85;
						end
					end
			6'd40: begin
					if (c_40 == 2'b00)
						begin
							c_40 <= turn;
							cell_x <= 8'd118;
							cell_y <= 7'd85;
						end
					end
			6'd41: begin
					if (c_41 == 2'b00)
						begin
							c_41 <= turn;
							cell_x <= 8'd140;
							cell_y <= 7'd85;
						end
					end
			6'd42: begin
					if (c_42 == 2'b00)
						begin
							c_42 <= turn;
							cell_x <= 8'd8;
							cell_y <= 7'd101;
						end
					end
			6'd43: begin
					if (c_43 == 2'b00)
						begin
							c_43 <= turn;
							cell_x <= 8'd30;
							cell_y <= 7'd101;
						end
					end
			6'd44: begin
					if (c_44 == 2'b00)
						begin
							c_44 <= turn;
							cell_x <= 8'd52;
							cell_y <= 7'd101;
						end
					end
			6'd45: begin
					if (c_45 == 2'b00)
						begin
							c_45 <= turn;
							cell_x <= 8'd74;
							cell_y <= 7'd101;
						end
					end
			6'd46: begin
					if (c_46 == 2'b00)
						begin
							c_46 <= turn;
							cell_x <= 8'd96;
							cell_y <= 7'd101;
						end
					end
			6'd47: begin
					if (c_47 == 2'b00)
						begin
							c_47 <= turn;
							cell_x <= 8'd118;
							cell_y <= 7'd101;
						end
					end
			6'd48: begin
					if (c_48 == 2'b00)
						begin
							c_48 <= turn;
							cell_x <= 8'd140;
							cell_y <= 7'd101;
						end
					end
		endcase
		
		case (column)
			5'd0: column_0_count <= column_0_count + 1;
			5'd1: column_1_count <= column_1_count + 1;
			5'd2: column_2_count <= column_2_count + 1;
			5'd3: column_3_count <= column_3_count + 1;
			5'd4: column_4_count <= column_4_count + 1;
			5'd5: column_5_count <= column_5_count + 1;
			5'd6: column_6_count <= column_6_count + 1;
		endcase
		
	end
	
	
	always @(negedge insert)
	begin
														
		// Assemble board
		board <= {c_0, c_1, c_2, c_3, c_4, c_5, c_6, c_7, c_8, c_9, 
				 c_10, c_11, c_12, c_13, c_14, c_15, c_16, c_17, c_18, c_19,
				 c_20, c_21, c_22, c_23, c_24, c_25, c_26, c_27, c_28, c_29,
				 c_30, c_31, c_32, c_33, c_34, c_35, c_36, c_37, c_38, c_39,
				 c_40, c_41, c_42, c_43, c_44, c_45, c_46, c_47, c_48};	

		row_0 <= {c_42, c_43, c_44, c_45, c_46, c_47, c_48};
		row_1 <= {c_35, c_36, c_37, c_38, c_39, c_40, c_41};
		row_2 <= {c_28, c_29, c_30, c_31, c_32, c_33, c_34};
		row_3 <= {c_21, c_22, c_23, c_24, c_25, c_26, c_27};
		row_4 <= {c_14, c_15, c_16, c_17, c_18, c_19, c_20};
		row_5 <= {c_7, c_8, c_9, c_10, c_11, c_12, c_13};
		row_6 <= {c_0, c_1, c_2, c_3, c_4, c_5, c_6};
		column_0 <= {c_42, c_35, c_28, c_21, c_14, c_7, c_0};
		column_1 <= {c_43, c_36, c_29, c_22, c_15, c_8, c_1};
		column_2 <= {c_44, c_37, c_30, c_23, c_16, c_9, c_2};
		column_3 <= {c_45, c_38, c_31, c_24, c_17, c_10, c_3};
		column_4 <= {c_46, c_39, c_32, c_25, c_18, c_11, c_4};
		column_5 <= {c_47, c_40, c_33, c_26, c_19, c_12, c_5};
		column_6 <= {c_48, c_41, c_34, c_27, c_20, c_13, c_6};
		diagonal_0 <= {c_3, c_11, c_19, c_27, 4'd0,
							c_2, c_10, c_18, c_26, c_34, 4'd0,
							c_1, c_9, c_17, c_25, c_33, c_41, 4'd0,
							c_0, c_8, c_16, c_24, c_32, c_40, c_48, 4'd0,
							c_7, c_15, c_23, c_31, c_39, c_47, 4'd0,
							c_14, c_22, c_30, c_38, c_46, 4'd0,
							c_21, c_29, c_37, c_45};			
		diagonal_1 <= {c_3, c_9, c_15, c_21, 4'd0,
							c_4, c_10, c_16, c_22, c_28, 4'd0,
							c_5, c_11, c_17, c_23, c_29, c_35, 4'd0,
							c_6, c_12, c_18, c_24, c_30, c_36, c_42, 4'd0,
							c_13, c_19, c_25, c_31, c_37, c_43, 4'd0,
							c_20, c_26, c_32, c_38, c_44, 4'd0,
							c_27, c_33, c_39, c_45};	
		combos <= {row_0, 4'd0, row_1, 4'd0, row_2, 4'd0, row_3, 4'd0, row_4, 4'd0, row_5, 4'd0, row_6, 4'd0,
					  column_0, 4'd0, column_1, 4'd0, column_2, 4'd0, column_3, 4'd0, column_4, 4'd0, column_5, 4'd0, column_6, 4'd0, 
				     diagonal_0, 4'd0, diagonal_1};						

	end
	
	
	
	/////////////////////////////////////////////////////////////////////////////
	//                            Determine Winner                             //
	/////////////////////////////////////////////////////////////////////////////
	
	wire [1:0] winner;
	wire [3:0] cs;
	//sequence_recognizer rec(CLOCK_50, ~insert, combos, winner, cs );

	//TRY THE BELOW
	sequence_recognizer rec(CLOCK_50, writeEN, combos, winner, cs );

	wire player1_score_enable = winner == 2'b01 ? 1'b1 : 1'b0;
	wire player2_score_enable = winner == 2'b10 ? 1'b1 : 1'b0;
	score_system player1_ss(HEX3, player1_score_enable);
	score_system player2_ss(HEX4, player2_score_enable);

	//NEW STUFF ABOVE
	
	
	
	// HEXES & LEDR
	assign LEDR[0] = insert;
	assign LEDR[1] = ~insert;
	assign LEDR[2] = writeEn;
	
	decoder d00(HEX0, {1'b0, 1'b0, winner});
	//USED TO SEE CURRENT STATE
	decoder d01(HEX1, cs);
	
	
endmodule



module datapath(
	input clk, resetn,
	input writeEnable,
	input [2:0] color,
	input [7:0] inX,
	input [6:0] inY,
	output [2:0] color_out,
	output [7:0] x,
	output [6:0] y
);

wire [2:0] c1, c2, c3;
wire enable_Y;
reg [7:0] tempx, tempy;

counter m(clk, resetn,writeEnable, c1);
ratecounter m1(clk, resetn, writeEnable, c2);
assign enable_Y = (c2 == 3'b00) ? 1:0;
counter m2(clk, resetn, enable_Y, c3);


assign x = tempx;
assign y = tempy;
assign color_out = color;
always @(posedge clk)
begin 
	tempx <= 7'd0;
	tempy <= 6'd0;
	if(writeEnable == 1'b1)
		begin 
			tempx <= inX + c1;
			tempy <= inY + c3;
			
		end
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


module ratecounter(clock, reset_n, enable, q);
	input 				clock, reset_n, enable;
	output reg 	[2:0] 	q;
	
	always @(posedge clock) begin
		if(reset_n == 1'b0)
			q <= 3'b101;
		else if(enable == 1'b1)
		begin
		
		  if (q == 3'b000)
			  q <= 3'b101;
		  else
			  q <= q - 1'b1;
		end
   end
endmodule
		
		

module control(
	input clk,resetn, go,
	output reg writeEnable
);

reg [2:0] current_state, next_state; 
reg [3:0] count;

assign offset = count;

localparam  S_RESET 		= 2'd0,
				S_START 		= 2'd1,
				S_DRAW		= 2'd2;
				

				
				
always @(*)
begin: state_table 
		case (current_state)
			S_RESET: next_state = S_START;
			S_START: next_state = go ? S_DRAW : S_START;
			S_DRAW: next_state = go ? S_DRAW : S_RESET;
					default: next_state = S_RESET;

  endcase
end // state_table
				


always @(*)
begin: enable_signals
  // By default make all our signals 0
  writeEnable = 1'b0;
  

  case (current_state)
		S_DRAW: begin
			writeEnable = 1'b1; 
		end 
  endcase
end // enable_signals


// current_state registers
always@(posedge clk)
begin: state_FFs
  if(!resetn)
		current_state <= S_RESET;
  else
		current_state <= next_state;
end // state_FFS		
				
endmodule 

