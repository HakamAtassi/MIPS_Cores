

module shift_left_2_tb;

    reg [31:0] in;
    wire [31:0] out;


    initial begin
        $dumpfile("test.vcd");
        $dumpvars;
    end

    shift_left_2 dut(out,in);

    initial begin
       assign in=4; #10;
    end




endmodule












module shift_left_2(shifted_out,shift_in);

    input [31:0] shift_in;
    output [31:0] shifted_out;


    assign shifted_out=shift_in<<2;
endmodule
