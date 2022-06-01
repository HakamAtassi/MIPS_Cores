`include "MIPS_Single_Cycle.v"




module imem_tb;


    reg [5:0] address;
    wire [31:0] data;


    initial begin
        $dumpfile("imem_tb.vcd");
        $dumpvars;
        $display("starting imem test");
    end

    instruction_memory dut(data,address);

    initial begin
 
        for(integer i=0;i<1000;i++) begin
            address=i; #10;
        end
    end

endmodule
