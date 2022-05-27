//on reset PC should begin at 0xBFC0 0000

module MIPS_Single_Cycle();

    initial begin
        $display("test hello test");
    end
endmodule


//32 bit flip flop with active high reset 
module Dflipflop(Q, clk, D, reset);

    input clk, reset; 
    input [31:0] D;
    output reg [31:0] Q;

    always @ (posedge clk)
        begin
            if(reset==1) Q<=0;

            Q<=D; //output new pc from alu on posedge clk 
        end
endmodule



//register file. Contains 32 internal registers. each 32 bits wide
//address 1 (A1) is read to RD1 asynchronously. Same is true for A2 and RD2.
//To write, input address is A3. Address to read is WD3. occurs on posedge clk
//write enable must also be active. 
module registerFile(RD1, RD2, A1, A2, A3, WD3, clk, WE3);
    
    output [31:0] RD1, RD2;  //outputs of the register file are each 32 bits since the regsiter is 32 bits wide
    input [4:0] A1,A2,A3;   //2^5=32 since 32 total addresses
    input [31:0] WD3; //the data to be written if WE3 is high 
    input clk, WE3; 

    reg [31:0] internal_mem [31:0]; //32 wires each 32 bits long

    
        always @ (posedge clk) 
            if (WE3==1) internal_mem[A3]<=WD3;
        
        assign RD1[0]=0;    //register 0 is always 0
        assign RD1=internal_mem[A1];
        assign RD2=internal_mem[A2];

endmodule


//in single cycle mips architecture, the program and variables are stored in
//two seperate registers. instruction memory stores the program, Data memory
//(TBD) stores variable information
module instruction_memory(RD, A);
    output [31:0] RD;    //output data is 32 bits wide because thats how long instructions are
    input [5:0] A;      //64 possible addresses

    reg [31:0] ram [63:0]; //64 wires each 32 bits wide
    
    initial 
        begin 
        $readmemh("dissassembled.dat",ram); //read dissassembled MIPS code and load it into ram
    end

    assign RD = ram[A]; //output onto bus. no need for clock because ram is asynchronous

endmodule
