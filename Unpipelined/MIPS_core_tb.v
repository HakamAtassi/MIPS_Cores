`timescale 1ns / 1ns
`include "MIPS_Single_Cycle.v"

module testbench;
    reg clk=0;
    reg reset=0;
    wire [31:0] writedata, dataadr;
    wire WE;
// instantiate device to be tested
    top dut (clk, reset, writedata, dataadr, WE);
// initialize test




    initial begin
        $dumpfile("top.vcd");
        $dumpvars;
    end



    initial
        begin
            reset <= 1; # 22; reset <=0;
        end
// generate clock to sequence tests
    initial
        for(integer i=0;i<1000;i++) begin
            clk <=1; # 5; clk <=0; # 5;
            
        end


// check results
    always @ (negedge clk)
        begin
            if (WE) begin
                if (dataadr === 84 & writedata === 7) begin
                    $display ("Simulation succeeded");
                    $stop;
                end else if (dataadr !==80) begin
                    $display ("Simulation failed");
                    $stop;
            end
        end
    end
endmodule
