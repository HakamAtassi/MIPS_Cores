`include "MIPS_Single_Cycle.v"




module Controller_tb;


    wire [1:0] ALUOp, ALUSrcB;
    wire MemtoReg,MemWrite,RegDst,RegWrite,Jump, 
    IorD, PCSrc,ALUSrcA,IRWrite,PCWrite,Branch;


    reg [5:0] Opcode;



    initial begin 

        $display("starting controller test");
        $dumpfile("Controller.vcd");
        $dumpvars;
    end


    mainDecoder dut(Opcode,MemtoReg,MemWrite,RegDst,RegWrite,Jump,ALUOp, IorD, PCSrc,ALUSrcB,ALUSrcA,IRWrite,PCWrite,Branch);


    initial begin
       Opcode<=6'b000000; #10; 
    end


    initial begin 
        #100; $stop;
    end


    always begin
        Clk<=0; #10; Clk<=10;
    end


endmodule