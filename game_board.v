
module game_board(Clk, enable, reset, out);
	input Clk;
	input enable;
	input reset;
	
	wire [1:0] q;
	
	reg [5:0] address = 6'd0;
	reg [97:0] board = 98'd1;
	output [397:0] out;
	
	ramTP ram(
	.address(address),
	.clock(Clk),
	.data(2'b00),		// doesn't matter what data is, because...
	.wren(1'b0),		// we're always reading
	.q(q)
	);

	
	// Populate board
	always @(posedge Clk)
	begin 
		if (address == 6'd49 || ~reset)
			address <= 6'd0;
		else if (enable)
		begin
			board <= board << 2;						// shift over to next piece
			board <= {board[95:0], q};				// read from memory at that location
			address <= address + 1'd1;				
		end
	end 

	
	// Calculate all possible rows, columns, and diagonals
	wire [13:0] row_0 = board[13:0];
	wire [13:0] row_1 = board[27:14];
	wire [13:0] row_2 = board[41:28];
	wire [13:0] row_3 = board[55:42];
	wire [13:0] row_4 = board[69:56];
	wire [13:0] row_5 = board[83:70];
	wire [13:0] row_6 = board[97:84]; 
	wire [13:0] column_0 = {board[0], board[1], 
									board[14], board[15],
									board[28], board[29],
									board[42], board[43], 
									board[56], board[57],
									board[70], board[71],
									board[84], board[85]};
	wire [13:0] column_1 = {board[2], board[3], 
									board[16], board[17],
									board[30], board[31],
									board[44], board[45], 
									board[58], board[59],
									board[72], board[73],
									board[86], board[87]};
	wire [13:0] column_2 = {board[4], board[5], 
									board[18], board[19],
									board[32], board[33],
									board[46], board[47], 
									board[60], board[61],
									board[74], board[75],
									board[88], board[89]};
	wire [13:0] column_3 = {board[6], board[7], 
									board[20], board[21],
									board[34], board[35],
									board[48], board[49], 
									board[62], board[63],
									board[76], board[77],
									board[90], board[91]};
	wire [13:0] column_4 = {board[8], board[9], 
									board[22], board[23],
									board[36], board[37],
									board[50], board[51], 
									board[64], board[65],
									board[78], board[79],
									board[92], board[93]};
	wire [13:0] column_5 = {board[10], board[11], 
									board[24], board[25],
									board[38], board[39],
									board[52], board[53], 
									board[66], board[67],
									board[80], board[81],
									board[94], board[95]};
	wire [13:0] column_6 = {board[12], board[13], 
									board[26], board[27],
									board[40], board[41],
									board[54], board[55], 
									board[68], board[69],
									board[82], board[83],
									board[96], board[97]};
	wire [85:0] diagonal_0 = {board[6], board[7], board[22], board[23], board[38], board[39], board[54], board[55], 2'b00,
								 board[4], board[5], board[20], board[21], board[36], board[37], board[52], board[53], board[68], board[69], 2'b00,
								 board[2], board[3], board[18], board[19], board[34], board[35], board[50], board[51], board[66], board[67], board[82], board[83], 2'b00,
								 board[0], board[1], board[16], board[17], board[32], board[33], board[48], board[49], board[64], board[65], board[80], board[81], board[96], board[97], 2'b00,
								 board[14], board[15], board[30], board[31], board[46], board[47], board[62], board[63], board[78], board[79], board[94], board[95], 2'b00,
								 board[28], board[29], board[44], board[45], board[60], board[61], board[76], board[77], board[92], board[93], 2'b00,
								 board[42], board[43], board[58], board[59], board[74], board[75], board[90], board[91]};
								 
	// TODO: complete diagonal_1: top right to bottom left direction
	wire [85:0] diagonal_1 = {board[6], board[7], board[22], board[23], board[38], board[39], board[54], board[55], 2'b00,
								 board[4], board[5], board[20], board[21], board[36], board[37], board[52], board[53], board[68], board[69], 2'b00,
								 board[2], board[3], board[18], board[19], board[34], board[35], board[50], board[51], board[66], board[67], board[82], board[83], 2'b00,
								 board[0], board[1], board[16], board[17], board[32], board[33], board[48], board[49], board[64], board[65], board[80], board[81], board[96], board[97], 2'b00,
								 board[14], board[15], board[30], board[31], board[46], board[47], board[62], board[63], board[78], board[79], board[94], board[95], 2'b00,
								 board[28], board[29], board[44], board[45], board[60], board[61], board[76], board[77], board[92], board[93], 2'b00,
								 board[42], board[43], board[58], board[59], board[74], board[75], board[90], board[91]};
								 
								 
	// Combine all rows, columns, and diagonals with padding into one large string holding all possible winning combinations
	assign out = {row_0, 2'b00, row_1, 2'b00, row_2, 2'b00, row_3, 2'b00, row_4, 2'b00, row_5, 2'b00, row_6, 2'b00,
					  column_0, 2'b00, column_1, 2'b00, column_2, 2'b00, column_3, 2'b00, column_4, 2'b00, column_5, 2'b00, column_6,	2'b00, 
					  diagonal_0, 2'b00, diagonal_1};
									
										

endmodule







