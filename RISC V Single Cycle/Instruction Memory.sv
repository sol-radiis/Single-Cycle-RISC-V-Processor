module InstructionMem(
    input wire[31:0] instraddr,
    output reg[31:0] instruction
);
    
    //memory
    reg[31:0] instructions[127:0];
    
    initial begin
        $readmemh("instructions.mem", instructions); // Load memory from hex file
    end
    
    
    always_comb begin
        instruction = instructions[instraddr[31:2]]; 
    end
endmodule
