module MUX #(parameter size=32 , parameter N=4)
            (input wire [size-1:0] inp [N-1:0],
             input wire [$clog2(N)-1:0] sel,
             output reg [size-1:0] op);
  
    always @(*) begin
        op=inp[sel];
    end      
endmodule     
