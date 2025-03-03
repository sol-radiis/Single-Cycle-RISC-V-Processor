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
