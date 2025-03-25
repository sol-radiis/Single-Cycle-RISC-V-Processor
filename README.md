# Single-Cycle RISC-V Processor

## Overview
This project implements a **single-cycle RISC-V processor** in SystemVerilog. The processor supports basic arithmetic, memory operations, and control flow instructions. It includes instruction fetch, decode, execute, memory access, and write-back stages in a single cycle.

## Features
- **Single-Cycle Design**: All instructions complete in a single clock cycle.
- **RISC-V ISA Support**: Implements a subset of the RISC-V instruction set.
- **Instruction Memory**: Fetches instructions from memory.
- **Register File**: 32 general-purpose registers with read/write operations.
- **ALU**: Supports arithmetic, logical, and shift operations.
- **Control Unit**: Decodes instructions and generates control signals.
- **Memory Unit**: Implements data memory with read/write capability.

## Modules
1. **Processor**: Top-level module integrating all components.
2. **InstructionMem**: Stores and fetches instructions.
3. **ControlUnit**: Generates control signals for instruction execution.
4. **registerFile**: Implements a register file with 32 registers.
5. **ALU**: Performs arithmetic and logical operations.
6. **extend**: Sign-extends immediate values for different instruction formats.
7. **DataMemory**: Implements memory for load/store instructions.
8. **MUX**: Implements a parameterized multiplexer for selection logic.

## Instruction Set Support
The processor supports the following RISC-V instructions:
- **R-Type**: ADD, SUB, AND, OR, XOR, SLL, SRL, SRA
- **I-Type**: ADDI, ORI, ANDI, LW, JALR
- **S-Type**: SW
- **B-Type**: BEQ, BNE, BLT, BGE, BLTU, BGEU
- **J-Type**: JAL

## File Descriptions
- `Processor.sv`: Top-level module connecting all components.
- `InstructionMem.sv`: Implements instruction memory.
- `ControlUnit.sv`: Generates control signals for each instruction.
- `registerFile.sv`: Implements the general-purpose registers.
- `ALU.sv`: Implements arithmetic and logical operations.
- `extend.sv`: Extends immediate values for various instruction formats.
- `DataMemory.sv`: Implements the data memory unit.
- `MUX.sv`: Implements multiplexers for control flow selection.
- `instructions.mem`: A file containing machine code for the processor to execute.

## Simulation & Testing
### Prerequisites
- **Vivado** / **ModelSim** / Any Verilog simulator



## Future Improvements
- Implement **pipelining** for improved performance.
- Add **hazard detection and forwarding** mechanisms.
- Extend instruction support for **floating-point operations**.
- Implement a **cache controller** for memory optimization.



