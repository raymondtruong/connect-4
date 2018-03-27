module keyboard_press_driver(
  input  CLOCK_50, 
  output reg valid, makeBreak,
  output reg [7:0] outCode,
  input    PS2_DAT, // PS2 data line
  input    PS2_CLK, // PS2 clock line
  input reset
);

parameter FIRST = 1'b0, SEENF0 = 1'b1;
reg state;
reg [1:0] count;
wire [7:0] scan_code;
reg [7:0] filter_scan;
wire scan_ready;
reg read;
parameter NULL = 8'h00;

initial 
begin
	state = FIRST;
	filter_scan = NULL;
	read = 1'b0;
	count = 2'b00;
end
	

// inner driver that handles the PS2 keyboard protocol
// outputs a scan_ready signal accompanied with a new scan_code
keyboard_inner_driver kbd(
  .keyboard_clk(PS2_CLK),
  .keyboard_data(PS2_DAT),
  .clock50(CLOCK_50),
  .reset(reset),
  .read(read),
  .scan_ready(scan_ready),
  .scan_code(scan_code)
);

always @(posedge CLOCK_50)
	case(count)
		2'b00:
			if(scan_ready)
				count <= 2'b01;
		2'b01:
			if(scan_ready)
				count <= 2'b10;
		2'b10:
			begin
				read <= 1'b1;
				count <= 2'b11;
				valid <= 0;
				outCode <= scan_code;
				case(state)
					FIRST:
						case(scan_code)
							8'hF0:
								begin
									state <= SEENF0;
								end
							8'hE0:
								begin
									state <= FIRST;
								end
							default:
								begin
									filter_scan <= scan_code;
									if(filter_scan != scan_code)
										begin
											valid <= 1'b1;
											makeBreak <= 1'b1;
										end
								end
						endcase
					SEENF0:
						begin
							state <= FIRST;
							if(filter_scan == scan_code)
								begin
									filter_scan <= NULL;
								end
							valid <= 1'b1;
							makeBreak <= 1'b0;
						end
				endcase
			end
		2'b11:
			begin
				read <= 1'b0;
				count <= 2'b00;
				valid <= 0;
			end
	endcase
endmodule 
