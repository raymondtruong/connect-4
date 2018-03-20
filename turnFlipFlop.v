// Synchronus active low reset T Flip flop where if q = 0 => Player1's turn and q = 1 => Player2's turn 
// May be able to change it so that it outputs the color representaion (ie 01 and 10) instead of a 1 bit value.
module turnFlipFlop (
enable  , // Enable Input
clk   , // Clock Input
reset , // Reset input
q       // Q output
);
input data, clk, reset ; 
output q;
reg q;

always @ ( posedge clk)
if (~reset) begin
  q <= 1'b0;
end else if (data) begin
  q <= !q;
end

endmodule 