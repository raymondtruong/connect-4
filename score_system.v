module score_system(HEX, enable);
    input enable;
    output [6:0] HEX;

    reg [6:0] score = 7'd0;
    assign HEX = score;
    always @(posedge enable)
    begin
        score <= score + 7'd1;
    end

endmodule
