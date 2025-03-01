`timescale 1ns/1ps

module Processor_tb;
    // Clock and Reset
    logic clk, rst;
    
    // Processor Outputs (For Verification)
    logic [31:0] pc, instruction;
    logic [31:0] reg_data1, reg_data2;
    logic [31:0] ALU_result, mem_data, write_data;
    logic RegWrite, MemRead, MemWrite;

    // Instantiate Processor
    Processor uut (
        .clk(clk),
        .rst(rst),
        .pc(pc),
        .instruction(instruction),
        .reg_data1(reg_data1),
        .reg_data2(reg_data2),
        .ALU_result(ALU_result),
        .mem_data(mem_data),
        .write_data(write_data),
        .RegWrite(RegWrite),
        .MemRead(MemRead),
        .MemWrite(MemWrite)
    );

    // Clock Generation
    always #5 clk = ~clk;

    // Test Procedure
    initial begin
        // Initialize Signals
        clk = 0;
        rst = 1;
        
        // Reset System
        #10 rst = 0;
        mem_data = '{default: 0};
        uut.mem_data[2] = 1;  // x2 = 1
        uut.mem_data[3] = 2;  // x3 = 2
        uut.mem_data[5] = 4;  // x5 = 4
        // Monitor Signals
        $display("==== RISC-V Processor Test ====");
        $monitor("Time=%0t | PC=%h | Instruction=%h | Reg1=%h | Reg2=%h | ALURes=%h | MemData=%h | WriteData=%h | RegWrite=%b | MemRead=%b | MemWrite=%b", 
                 $time, pc, instruction, reg_data1, reg_data2, ALU_result, mem_data, write_data, RegWrite, MemRead, MemWrite);

        // Run for 100 cycles
        #500 $finish;
    end
endmodule