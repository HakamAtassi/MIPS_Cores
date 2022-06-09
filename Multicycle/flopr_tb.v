`include "MIPS_Multicycle.v"




module Controller_tb;

    wire [31:0] q;
    reg Clk,Reset,EN;
    reg [31:0] d;


    initial begin 

        $display("starting flopr test");
        $dumpfile("flopr_EN.vcd");
        $dumpvars;
    end


    flopr_EN #32 dut(Clk,Reset,EN,d,q);


    initial begin
        EN<=0;#10;
    end

    initial begin
        d<=15;#10;
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