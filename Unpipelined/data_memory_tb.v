`include "MIPS_Single_Cycle.v"




module dmem_tb;


    wire [31:0] RD;
    reg clk;
    reg [31:0] A, WD;
    reg WE;


    initial begin

        $display("starting dmem test");
        $dumpfile("dmem.vcd");
        $dumpvars;
    
    end

    data_memory dut(RD,clk,A,WD,WE);


    always begin
        clk<=1; #10; clk=0; #10;
    end
        

    initial begin

        A<=32'b0000;
        WD<=32'b1101;
        WE<=0;
        #10;
        

    end

    initial begin

        #50;
        WE<=1'b1; #20;
        WE<=1'b0;

    end


    initial begin
        
        #100;

        $stop;
        

    end


endmodule

