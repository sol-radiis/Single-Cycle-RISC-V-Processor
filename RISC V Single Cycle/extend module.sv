module extend(
    input [31:0] instruction,
    output reg [31:0] dat1
);
    always_comb begin
        case(instruction[6:0])
            // I-type (e.g., addi, andi, ori, slti, lw)
            7'b0010011, 7'b0000011, 7'b1100111: 
                dat1 = {{20{instruction[31]}}, instruction[31:20]};
            // S-type 
            7'b0100011: 
                dat1 = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]};
            
            // B-type 
            7'b1100011: 
                dat1 = {{19{instruction[31]}}, instruction[31], instruction[7], instruction[30:25], instruction[11:8], 1'b0};

            // U-type 
            7'b0110111, 7'b0010111: 
                dat1 = {instruction[31:12], 12'b0};  // Zero-extend

            // J-type (e.g., jal)
            7'b1101111: 
                dat1 = {{11{instruction[31]}}, instruction[31], instruction[19:12], instruction[20], instruction[30:21], 1'b0};

            default: 
                dat1 = 32'b0;
        endcase    
    end
endmodule
