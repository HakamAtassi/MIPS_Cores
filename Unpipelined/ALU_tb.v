`include "MIPS_Single_Cycle.v"



module alu_test;


    wire Zero,C_out;
    wire [31:0] ALUResult;
    
    reg [2:0] ALUControl; 
    reg [31:0] SrcA, SrcB;

    initial begin 
        $dumpfile("alu_test.vcd");
        $dumpvars;
        $display("starting alu test");
    end

    ALU dut(Zero,ALUResult,C_out,ALUControl,SrcA,SrcB);

    initial begin 
 //       ALUControl <=010; //add
 //        ALUControl <=110; //subtract
  //       ALUControl <=000;
           ALUControl <=001;
      //   ALUControl <=111;
    end


    initial begin 
        SrcB<=32'b101;
        SrcA<=32'b110;
        #20;
    end


endmodule