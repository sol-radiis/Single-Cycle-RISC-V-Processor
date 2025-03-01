`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/25/2025 11:59:28 AM
// Design Name: 
// Module Name: Processor
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module Processor(
    input logic clk, rst,
    output logic [31:0] pc,
    output logic [31:0] instruction,
    output logic [31:0] reg_data1, reg_data2,
    output logic [31:0] ALU_result, mem_data, write_data,
    output logic RegWrite, MemRead, MemWrite
);
    // Program Counter (pc)
    // Note: We use only pc, not pc_next since we're ignoring branch/jump
    always_ff @(posedge clk or posedge rst) begin
        if (rst)
            pc <= 32'b0;
        else
            pc <= pc + 32'd4; // Simple PC update, ignoring branches/jumps
    end

    // Instruction Memory
    InstructionMem instr_mem (
        .instraddr(pc),
        .instruction(instruction)
    );

    // Decode Stage
    logic [4:0] rs1, rs2, rd;
    logic [31:0] imm_ext;
    logic ALUSrc, MemtoReg;  // Control signals

    assign rs1 = instruction[19:15];
    assign rs2 = instruction[24:20];
    assign rd  = instruction[11:7];

    extend imm_extender (
        .instruction(instruction),
        .dat1(imm_ext)
    );

    registerFile reg_file (
        .clk(clk),
        .RD1(rs1),
        .RD2(rs2),
        .WD(rd),
        .writeData(write_data),
        .wd(RegWrite),
        .Readdata1(reg_data1),
        .Readdata2(reg_data2)
    );

    // Control Unit - branch and jump signals will be ignored
    logic [3:0] ALU_control;
    logic Branch, Jump; // Declared but unused since we're ignoring branch/jump

    ControlUnit controller(
        .opcode(instruction[6:0]),
        .funct3(instruction[14:12]),
        .funct7(instruction[31:25]),
        .RegWrite(RegWrite),
        .ALUSrc(ALUSrc),
        .MemRead(MemRead),
        .MemWrite(MemWrite),
        .MemtoReg(MemtoReg),
        .ALUOp(ALU_control),
        .Branch(Branch),  // Connected but ignored
        .Jump(Jump)       // Connected but ignored
    );

    // Execute Stage
    logic [31:0] alu_input2;
    assign alu_input2 = (ALUSrc) ? imm_ext : reg_data2;

    ALU alu (
        .a(reg_data1),
        .b(alu_input2),
        .s(ALU_control),
        .e(ALU_result),
        .cout()
    ); 

    // Memory Stage
    DataMemory data_mem (
        .clk(clk),
        .rst(rst),
        .address(ALU_result),
        .write(MemWrite),
        .Writedata(reg_data2),
        .ReadData(mem_data)
    );

    // Write Back Stage
    assign write_data = (MemtoReg) ? mem_data : ALU_result;
    
endmodule

module ControlUnit (
    input  logic [6:0] opcode,      // RISC-V opcode
    input  logic [2:0] funct3,      // Function bits (for differentiation)
    input  logic [6:0] funct7,      // Function bits (for R-type)
    output logic RegWrite, MemRead, MemWrite, Branch, Jump, MemtoReg,
    output logic [3:0] ALUOp,       // ALU Control Signals
    output logic ALUSrc              // ALU Input Source Selection
);
    
    always_comb begin
        // Default values
        RegWrite = 0; MemRead = 0; MemWrite = 0;
        ALUSrc = 0; Branch = 0; Jump = 0; MemtoReg = 0; // Fixed: Added MemtoReg default
        ALUOp = 4'b0000;

        case (opcode)
            7'b0110011: begin  // R-type (ADD, SUB, AND, OR, XOR)
                RegWrite = 1;
                ALUOp    = {funct7[5], funct3};
            end
            
            7'b0010011: begin // I-Type (ADDI, ORI, ANDI)
                RegWrite = 1;
                ALUSrc = 1;
                MemRead = 0;
                MemWrite = 0;
                MemtoReg = 0;
                Branch = 0;
                ALUOp    = {funct7[5], funct3};    
            end
                
            7'b0000011: begin  // Load (LW)
                RegWrite = 1;
                MemRead = 1;
                ALUSrc = 1;
                MemtoReg = 1;  // Fixed: Added MemtoReg for loads
                ALUOp = 4'b0000;
            end
                
            7'b0100011: begin  // Store (SW)
                MemWrite = 1;
                ALUSrc = 1;
                ALUOp = 4'b0000;
            end
                
            7'b1100011: begin  // Branch (BEQ)
                Branch = 1;
                ALUOp = 4'b0001; // SUB for comparison
            end
                
            7'b1101111: begin  // Jump (JAL)
                Jump = 1;
                RegWrite = 1;
            end
        endcase
    end
endmodule

module InstructionMem(
    input wire[31:0] instraddr,
    output reg[31:0] instruction
);
    
    //memory
    reg[31:0] instructions[127:0];
    
    initial begin
        $readmemh("D:/Anshu/College/DIGITAL/instructions.mem", instructions); // Load memory from hex file
    end
    
    // Fixed: Changed to combinational logic for instruction fetch
    always_comb begin
        instruction = instructions[instraddr[31:2]]; // Address divided by 4 for word alignment
    end
endmodule

module registerFile(
    input wire clk,
    input[4:0] RD1, RD2, WD,
    input [31:0] writeData,
    input wd,
    output reg[31:0] Readdata1, Readdata2
);
    //prep memory
    reg[31:0] Registers[31:0];
    
    // Fixed: Changed to non-blocking assignment for sequential logic
    always_ff @(posedge clk) begin
        if (wd && WD != 0) begin 
            Registers[WD] <= writeData;
        end 
    end
     
    always_comb begin      
        Readdata1 = (RD1 == 0) ? 32'b0 : Registers[RD1];
        Readdata2 = (RD2 == 0) ? 32'b0 : Registers[RD2]; 
    end
endmodule

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

module DataMemory(
    input clk,
    input rst,
    input [31:0] address,
    input write,
    input [31:0] Writedata,
    output reg [31:0] ReadData
);
    //prep
    reg[31:0] Data[0:511];
    
    // Fixed: Changed to proper async reset sensitivity list
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            integer i;
            for (i = 0; i < 512; i++) begin
                Data[i] <= 0;
            end    
        end else if (write) begin
            Data[address[31:2]] <= Writedata; // Address divided by 4 for word alignment
        end    
    end  
    
    // Fixed: Changed to blocking assignment for combinational logic
    always_comb begin
        ReadData = Data[address[31:2]]; // Address divided by 4 for word alignment
    end         
endmodule

module ALU (
    input wire [31:0] a, b,
    input wire [3:0] s,   // Control signal should be 4-bit
    output reg [31:0] e,  // Output must be 32-bit
    output reg cout       // Carry output
);
    always_comb begin
        e = 32'b0;
        cout = 0;  // Default carry 
        case (s)
            4'b0000: {cout, e} = a + b;   
            4'b1000: {cout, e} = a - b;   
            4'b0111: e = a & b;           
            4'b0110: e = a | b;           
            4'b0100: e = a ^ b;                                
            4'b0001: e = a << b[4:0];     
            4'b0101: e = a >> b[4:0];    
            4'b1101: e = $signed(a) >>> b[4:0];
            default: e = 32'b0;               
        endcase
    end
endmodule

module MUX #(parameter size=32 , parameter N=4)
            (input wire [size-1:0] inp [N-1:0],
             input wire [$clog2(N)-1:0] sel,
             output reg [size-1:0] op);
  
    always @(*) begin
        op=inp[sel];
    end      
endmodule             
            