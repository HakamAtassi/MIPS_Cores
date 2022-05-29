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

