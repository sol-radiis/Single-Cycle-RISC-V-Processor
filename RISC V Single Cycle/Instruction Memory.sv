module InstructionMem(
    input wire[31:0] instraddr,
    output reg[31:0] instruction
);
    
    //memory
    reg[31:0] instructions[127:0];
    
    initial begin
        $readmemh("instructions.mem", instructions); // Load memory from hex file
    end
    
    // Fixed: Changed to combinational logic for instruction fetch
    always_comb begin
        instruction = instructions[instraddr[31:2]]; // Address divided by 4 for word alignment
    end
endmodule
