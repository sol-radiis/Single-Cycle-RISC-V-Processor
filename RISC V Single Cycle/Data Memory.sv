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
    
    
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            integer i;
            for (i = 0; i < 512; i++) begin
                Data[i] <= 0;
            end    
        end else if (write) begin
            Data[address[31:2]] <= Writedata; 
        end    
    end  
    
   
    always_comb begin
        ReadData = Data[address[31:2]]; 
    end         
endmodule
