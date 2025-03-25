module ControlUnit (
    input  logic [6:0] opcode,
    input  logic [2:0] funct3,
    input  logic [6:0] funct7,
    output logic RegWrite, MemRead, MemWrite, Branch, Jump, Jalr, MemtoReg,
    output logic [3:0] ALUOp,
    output logic ALUSrc
);
    
    always_comb begin
        // Default signals
        RegWrite = 0; MemRead = 0; MemWrite = 0;
        ALUSrc = 0; Branch = 0; Jump = 0; Jalr = 0; MemtoReg = 0;
        ALUOp = 4'b0000;

        case (opcode)
            7'b0110011: begin  // R-type
                RegWrite = 1;
                ALUOp    = {funct7[5], funct3};
            end
            
            7'b0010011: begin // I-Type (ADDI, ORI, ANDI)
                RegWrite = 1;
                ALUSrc = 1;
                ALUOp = {funct7[5], funct3};    
            end
                
            7'b0000011: begin  // Load (LW)
                RegWrite = 1;
                MemRead = 1;
                ALUSrc = 1;
                MemtoReg = 1;  
                ALUOp = 4'b0000;
            end
                
            7'b0100011: begin  // Store (SW)
                MemWrite = 1;
                ALUSrc = 1;
                ALUOp = 4'b0000;
            end
                
            7'b1100011: begin  // Branch Instructions
                Branch = 1;
                ALUOp = 4'b0001; // SUB for comparison
            end
                
            7'b1101111: begin  // Jump (JAL)
                Jump = 1;
                RegWrite = 1;
            end

            7'b1100111: begin  // JALR
                Jalr = 1;
                RegWrite = 1;
                ALUSrc = 1;
                ALUOp = 4'b0000;
            end
        endcase
    end
endmodule
