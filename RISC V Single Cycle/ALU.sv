`timescale 1ns / 1ps
module ALU (
    input wire [31:0] a, b,
    input wire [3:0] s,   // Control signal should be 4-bit
    output reg [31:0] e,  // Output must be 32-bit
    output reg cout       // Carry output
);
    always_comb begin
        e = 32'b0;
        cout = 32'b0;  // Default carry 
        case (s)
            4'b0000: {cout, e} = a + b;   
            4'b1000: {cout, e} = a - b;   
            4'b0111: e = a & b;           
            4'b0110: e = a | b;           
            4'b0100: e = a ^ b;                                
            4'b0001: e = a << b[4:0];     
            4'b0101: e = a >> b[4:0];    
            4'b1101: e = $signed(a) >>> b[4:0];
            default: e =32'b0;              
        endcase
    end
endmodule
            
