`timescale 1ns / 1ns
`include "MIPS_Single_Cycle.v"

module mux_2_32b_tb;

    integer error=0;
    //reg clk=1'b0;

    reg [31:0] input0=32'b00000000000000000000000000000000;
    reg [31:0] input1=32'b00000000000000000000000000000000;
    wire [31:0] out;
    reg select;

    mux_2_32b dut(out,select,input0,input1);


    initial begin
        $display("32 bit mux test starting...\n");
        $dumpfile("32b_mux_tb.vcd");
        $dumpvars;
    end


    initial begin 
        for(integer i=0; i<=100;i++) begin
            select=1; #10; select=0; #10;
        end
    end


    initial begin 
        for(integer j=0;j<=100000;j++) begin
            input0=input0+1; #20;
        end
    end
 

    initial begin 
        for(integer j=0;j<=100000;j++) begin
            input1=input1+1; #10;
        end
    end
    


    initial begin //check if output 0
        if(select==1'b0 && out!=input0) begin 
            $display("Error. Expected %X, got %X",input0,out);
            error=error+1;
        end
    end


    initial begin //check if output is 1
        if(select==1'b1 && out!=input0) begin 
            $display("Error. Expected %X, got %X",input0,out);
            error=error+1;
        end
    end


    initial begin
        $display("32 bit mux test complete. %d errors \n========\n",error);
    end

endmodule





module mux_4_32b_tb;

    integer error=0;
    //reg clk=1'b0;

    reg [31:0] input0=32'b00000000000000000000000000000000;
    reg [31:0] input1=32'b00000000000000000000000000000000;
    reg [31:0] input2=32'b00000000000000000000000000000000;
    reg [31:0] input3=32'b00000000000000000000000000000000;

    wire [31:0] out;
    reg [1:0] select;

    mux_4_32b dut(out,select,input0,input1,input2,input3);


    initial begin
        $display("32 bit mux (4 way) test starting...\n");
        $dumpfile("32b_mux_4_tb.vcd");
        $dumpvars;
    end


    initial begin 
        for(integer i=0; i<=10000;i++) begin
            select=2'b00; #10; select=2'b01; #10;  select=2'b10; #10; select=2'b11; #10;
        end
    end

///inputs below///
    initial begin 
        for(integer j=0;j<=100000;j++) begin
            input0=input0+1; #10;
        end
    end
 

    initial begin 
        for(integer j=0;j<=100000;j++) begin
            input1=input1+1; #20;
        end
    end
    
    initial begin 
        for(integer j=0;j<=100000;j++) begin
            input2=input2+1; #30;
        end
    end
 

    initial begin 
        for(integer j=0;j<=100000;j++) begin
            input3=input3+1; #40;
        end
    end
///inputs above///



///validity checking below///
    initial begin //check if output 0
        if(select==2'b00 && out!=input0) begin 
            $display("Error. Expected %X, got %X",input0,out);
            error=error+1;
        end
    end


    initial begin //check if output is 1
        if(select==2'b01 && out!=input1) begin 
            $display("Error. Expected %X, got %X",input1,out);
            error=error+1;
        end
    end

    initial begin //check if output 0
        if(select==2'b10 && out!=input2) begin 
            $display("Error. Expected %X, got %X",input2,out);
            error=error+1;
        end
    end


    initial begin //check if output is 1
        if(select==2'b11 && out!=input3) begin 
            $display("Error. Expected %X, got %X",input3,out);
            error=error+1;
        end
    end

//validity checking above///

    initial begin
        $display("32 bit mux (4 way) test complete. %d errors \n=======\n",error);
    end

endmodule



module SignExtender_tb;

    reg [15:0] extend=16'b0000000000000000;
    wire [31:0] extended;

    integer errors=0;
    reg sign=1'b0;
    reg expected=32'b0;


    SignExtender dut(extend,extended);

    initial begin
        $display("SignExtender test starting...\n");
        $dumpfile("SignExtender_tb.vcd");
        $dumpvars;
    end


    initial begin 
        for(integer i=0;i<=65536;i++) begin
            extend=extend+1; #10;
        end
    end

    initial begin 
        expected <= { {16{extend[15]}}, extend[15:0] };
    end

    initial begin
        if (extend[15]==0 && extended[31:16]!=0) 
            $display("Error. Expected %X, got %X",expected,extended);
            errors=errors+1;
        if (extend[15]==1 && extended[31:16]!=16'b1111111111111111)
            $display("Error. Expected %X, got %X",expected,extended);
            errors=errors+1;
    end

    initial begin

        $display("SignExtender tests complete. %d errors \n=======\n",errors);
    end

endmodule





module AluDecoder_tb;

    reg [1:0] AluOp;
    reg [5:0] funct;
    reg [2:0] AluControl;

    


endmodule


module mainDecoder_tb;

    wire MemWrite, RegWrite, RegDst, ALUSrc, MemtoReg, Branch, Jump;
    wire [1:0] ALUOp;
    reg [5:0] Opcode;
    
    //assign {RegWrite,RegDst,ALUSrc,Branch,MemWrite,MemtoReg,ALUOp,Jump}=controls;


    wire controls = {RegWrite,RegDst,ALUSrc,Branch,MemWrite,MemtoReg,ALUOp,Jump};

    integer errors=0;

    initial begin
        $display("mainDecoder test starting...\n");
        $dumpfile("mainDecoder_tb.vcd");
        $dumpvars;
    end

    mainDecoder dut (MemWrite, RegWrite, RegDst, ALUSrc, MemtoReg, Branch, ALUOp, Jump, Opcode);


    initial begin
        Opcode=6'b000000; #10; //R-type
        Opcode=6'b100011; #10; //lw
        Opcode=6'b101011; #10; //sw
        Opcode=6'b000100; #10; //beq
        Opcode=6'b001000; #10; //addi
        Opcode=6'b000010; #10; //j
    end


    initial begin 
        if(Opcode==6'b000000 && controls!=9'b110000100) 
            errors=errors+1;
            $display("Error. Expected 11000100, got %X",controls);
        if(Opcode==6'b100011 && controls!=9'b101001000)
            errors=errors+1;
            $display("Error. Expected 100011, got %X",controls);
        
        if (Opcode==6'b101011 && controls!=9'b0x101x000)
            errors=errors+1;
            $display("Error. Expected 0x101x000, got %X",controls);

        if (Opcode==6'b000100 && controls!=9'b0x010x010)
            errors=errors+1;
            $display("Error. Expected 0x010x010, got %X",controls);

        if (Opcode==6'b001000 && controls!=9'b101000000)
            errors=errors+1;
            $display("Error. Expected 101000000, got %X",controls);
       
        if (Opcode==6'b000010 && controls!=9'b0xxx0xxx1)
            errors=errors+1;
            $display("Error. Expected 0xxx0xxx1, got %X",controls);
    end




    initial begin
        $display("mainDecoder tests complete. %d errors \n=======\n",errors);
    end



endmodule




module shift_left_2_tb;

    reg [31:0] in;
    wire [31:0] out;

    integer errors=0;
    


    initial begin
        $dumpfile("shift_left_2_tb.vcd");
        $dumpvars;
        $display("Shift left 2 test starting...");
    end

    shift_left_2 dut(out,in);

    initial begin
        for(integer i=0;i<1000;i++) begin
            assign in=i; #10;
        end
    end

    initial begin 
        if(out!=in*4) $display("Error. Expected %X got %X",4*in, out);
    end

    initial begin
        $display("shift_left_2 done");
    end

endmodule



module adder_tb;

    reg [31:0] SrcA, SrcB;
    wire [31:0] out;

    integer errors=0;
    


    initial begin
        $dumpfile("adder.vcd");
        $dumpvars;
        $display("Adder test starting...");

    end

    adder dut(out,,0,SrcA, SrcB);


    initial begin 
        for(integer i=0;i<1000;i++) begin
            assign SrcA=i; #10;
        end
    end
 
    initial begin
        for(integer j=0;j<1000;j++) begin
            assign SrcB=j; #20;
        end
    end


    initial begin 
        if(out!=SrcA+SrcB) $display("Error. Expected %X got %X",SrcA+SrcB, out);
    end

    initial begin
        $display("Adder done");
    end

endmodule





