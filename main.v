module main(CLOCK_50, SW, KEY, LEDR);

	input CLOCK_50;
	input [9:0] SW;		// replace with keyboard eventually
	input [3:0] KEY;
	input	[9:0] LEDR;		// replace with monitor eventually
	
	wire reset = KEY[3];
	
	// Declare memory model
	reg [5:0] address;
	wire [1:0] data;
	reg wren = 1'b0;
	wire [1:0] q;
	ramTP ram(address, CLOCK_50, 2'b01, ~KEY[0], q);
	
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
	always @(*)
	begin
		// Determine which column to insert into
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

			
		// Determine which row in the column
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
			
			
		// Find where the new piece should go
		address <= column + (5'd7 * (5'd6 - column_count)); 
	end

	// Insert into column
	always @(negedge KEY[0])
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
	
endmodule
