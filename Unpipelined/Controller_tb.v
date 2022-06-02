`include "MIPS_Single_Cycle.v"




module Controller_tb;


    wire [2:0] ALUControl;
    wire PCSrc, MemtoReg, ALUSrc, RegDst, RegWrite, MemWrite, Jump;

    reg [5:0] Funct;
    reg [5:0] Opcode;
    reg Zero;


    initial begin 

        $display("starting controller test");
        $dumpfile("Controller.vcd");
        $dumpvars;
    end


    controller dut(ALUControl, PCSrc, MemtoReg,ALUSrc,RegDst,RegWrite, MemWrite, Opcode, Funct, Zero, Jump);



    initial begin 
        Zero<=0;
        Opcode<=6'b000100; #10;

    end



endmodule