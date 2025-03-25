module extend(
    input [31:0] instruction,
    output reg [31:0] dat1
);
    always_comb begin
        case(instruction[6:0])
            7'b0010011, 7'b0000011, 7'b1100111: // I-type (addi, lw, jalr)
                dat1 = {{20{instruction[31]}}, instruction[31:20]};
            
            7'b0100011: // S-type (sw)
                dat1 = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]};
            
            7'b1100011: // B-type (beq, bne)
                dat1 = {{19{instruction[31]}}, instruction[31], instruction[7], instruction[30:25], instruction[11:8], 1'b0};

            7'b1101111: // J-type (jal)
                dat1 = {{11{instruction[31]}}, instruction[31], instruction[19:12], instruction[20], instruction[30:21], 1'b0};

            default: 
                dat1 = 32'b0;
        endcase    
    end
endmodule
