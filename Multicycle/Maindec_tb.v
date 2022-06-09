`include "MIPS_Multicycle.v"




module Controller_tb;


    wire [1:0] ALUOp, ALUSrcB;
    wire MemtoReg,MemWrite,RegDst,RegWrite,Jump, 
    IorD, PCSrc,ALUSrcA,IRWrite,PCWrite,Branch;


    reg [5:0] Opcode;
    reg Clk,Reset;


    initial begin 

        $display("starting controller test");
        $dumpfile("Maindec.vcd");
        $dumpvars;
    end


    mainDecoder dut(Opcode,Clk,Reset,MemtoReg,MemWrite,RegDst,RegWrite,Jump,ALUOp, IorD, PCSrc,ALUSrcB,ALUSrcA,IRWrite,PCWrite,Branch);


    initial begin
        //Opcode<=6'b100011;#10; //load 
        //Opcode<=6'b101011;#10;   //store
        Opcode<=6'b000000;#10;     //r type
        //Opcode<=6'b000101;#10;     //BEQ
        //6'001000: nextstate=S9;    //ADDI
    end


    initial begin 
        #150; $stop;
    end


    initial begin 
        Reset<=1; #10; Reset<=0;
    end


    always begin
        Clk<=0; #10; Clk<=1; #10;
    end


endmodule