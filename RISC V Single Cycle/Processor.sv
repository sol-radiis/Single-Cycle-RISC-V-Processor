
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
