`include "MIPS_Single_Cycle.v"


module datapath_tb;

    wire Zero; //zero flag if alu output is 0
    wire [31:0] PC; //program counter output after selection (final PC for instruction retrieving)
    wire [31:0] ALUOut, WriteData;
    
    reg Reset, Clk, MemtoReg, PCSrc, ALUSrc, RegDst, RegWrite, Jump;
    reg [2:0] ALUControl; 
    reg [31:0] ReadData; //inputs datat to datapath
    reg [31:0] Instr;



    initial begin
        $dumpfile("datapath.vcd");
        $dumpvars;
        $display("starting datapath tests");
    end



    datapath dut(ALUOut,WriteData,PC,Zero,Reset, Clk, ALUControl,PCSrc, MemtoReg, ALUSrc, RegDst, RegWrite, Jump, ReadData,Instr);

        
    

    always begin 
        Clk<=0; #5; Clk<=0;
    end


    always begin 
        RegWrite<=0;
        RegDst <=1;
        ALUSrc <=0;
        MemtoReg<=0;
        PCSrc=0;
        MemtoReg<=0;
        Jump<=0;
        Reset<=0;
    end





endmodule
