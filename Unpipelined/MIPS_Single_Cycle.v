//on reset PC should begin at 0xBFC0 0000

module MIPS_Single_Cycle();

    initial begin

    end
endmodule


//32 bit flip flop with active high reset 
module Dflipflop(Q, clk, D, reset);

    input clk, reset; 
    input [31:0] D;
    output reg [31:0] Q;

    always @ (posedge clk)
        begin
            if(reset==1) Q<=0;

            Q<=D; //output new pc from alu on posedge clk 
        end
endmodule



//register file. Contains 32 internal registers. each 32 bits wide
//address 1 (A1) is read to RD1 asynchronously. Same is true for A2 and RD2.
//To write, input address is A3. Address to read is WD3. occurs on posedge clk
//write enable must also be active. 
module registerFile(RD1, RD2, A1, A2, A3, WD3, clk, WE3);
    
    output [31:0] RD1, RD2;  //outputs of the register file are each 32 bits since the regsiter is 32 bits wide
    input [4:0] A1,A2,A3;   //2^5=32 since 32 total addresses
    input [31:0] WD3; //the data to be written if WE3 is high 
    input clk, WE3; 

    reg [31:0] internal_mem [31:0]; //32 wires each 32 bits long

    
        always @ (posedge clk) 
            if (WE3==1) internal_mem[A3]<=WD3;
        
        assign RD1[0]=0;    //register 0 is always 0
        assign RD1=internal_mem[A1];
        assign RD2=internal_mem[A2];

endmodule


//in single cycle mips architecture, the program and variables are stored in
//two seperate registers. instruction memory stores the program, Data memory
//(TBD) stores variable information
module instruction_memory(RD, A);
    output [31:0] RD;    //output data is 32 bits wide because thats how long instructions are
    input [5:0] A;      //64 possible addresses

    reg [31:0] ram [63:0]; //64 wires each 32 bits wide
    
    initial 
        begin 
        //$readmemh("dissassembled.dat",ram); //read dissassembled MIPS code and load it into ram
    end

    assign RD = ram[A]; //output onto bus. no need for clock because ram is asynchronous

endmodule




//more ram. This is where data is written during program exectuion. 
//writing is done on the clock, reading is done asynchronously
//input address is 32 bits, data width is 32 bits. 
module data_memory(RD,clk,A,WD,WE);
    //not using full memory capabilities
    //
    //
    //                                                                                                                      DOUBLE
    //                                                                                                                      CHECK
    //                                                                                                                      THIS
    //                                                                                                                      MODULE
    output [31:0] RD;
    input clk, WE;
    input [31:0] A,WD;


    reg [31:0] data [63:0];


    assign RD = data[A]; //address is always being read out

    always @ (posedge clk)
        if(WE==1) data[A] <=WD;
endmodule



module SignExtender(extend, extended);

    input [15:0] extend;
    output [31:0] extended;

    assign extended[31:0]= {{16{extend[15]}},extend[15:0]};

endmodule

module adder(Y, C_out, SrcA, SrcB);

    output [31:0] Y;
    output C_out;
    input [31:0] SrcA, SrcB;
    

    assign {C_out,Y}=SrcA+SrcB;


endmodule



//The input to the alu control logic depends on 2 things, the op code and the
//funct space (in R type instructions). For instance, an instruction with
//opcode 000000 is always going to have an AluOp of 10. 10 indicates that the
//ALUcontrol depends on the funct field of the instruction, (Or, Nor, Add all
//work differently). However, in I type instructions, there is no funct field,
//so the alu controll only depends on the opcode, and the logic for that is
//quite straight forward. 

//ALU decoder controls the operation that occurs in the alu
module AluDecoder(AluControl, AluOP, funct);

    output reg [2:0]AluControl;
    input [1:0] AluOP;
    input [5:0] funct;
    


    always @(*)
        if (AluOP==2'b0x) AluControl<=3'b010;
        else if (AluOP==2'bx1) AluControl <=3'b110;
        else if(AluOP==2'b1x) case(funct)
            6'b100000 : AluControl <= 3'b010;   //add
            6'b100010 : AluControl <= 3'b110;   //sub
            6'b100100 : AluControl <= 3'b000;   //and
            6'b100101 : AluControl <= 3'b010;   //or
            6'b101010 : AluControl <= 3'b011;   //slt
        default: AluControl <=3'bxxx;   //illegal opcode
        endcase
endmodule


module Or(Y, SrcA,SrcB);

    output [31:0] Y;
    input [31:0] SrcA, SrcB;


    assign Y=SrcA | SrcB;

endmodule

module And(Y, SrcA,SrcB);

    output [31:0] Y;
    input [31:0] SrcA, SrcB;


    assign Y=SrcA & SrcB;

endmodule
//subtract and add are done the same exact way. the only difference is that 
//subtract is compiled as the addition of a twos complement number to
//a posative number
module ALU(Zero, ALUResult, C_out, ALUControl, SrcA, SrcB);

    output reg Zero;
    output [31:0] ALUResult;
    output C_out;
    input [2:0] ALUControl;
    input [31:0] SrcA, SrcB;
    
    wire [31:0] SrcB_not; not(SrcB_not, SrcB);
    wire [31:0] mux1_out;
    wire [31:0] N0,N1,N2,N3;


    mux_2_32b mux1(mux1_out,ALUControl[2], SrcB,SrcB_not);
 
    and(N0,mux1_out,SrcA);
    or(N1,mux1_out,SrcA);
    
    adder add1(N2,C_out,mux1_out,SrcA);
    assign N3={1'b0,N2[30:0]};

    mux_4_32b mux2(ALUResult,ALUControl[1:0],N0,N1,N2,N3);

    always @(*)
        if (ALUResult==0) Zero=1;
        else Zero=0;

endmodule








module shift_left_2(shifted_out,shift_in);

    input [31:0] shift_in;
    output [31:0] shifted_out;


    assign shifted_out=shift_in<<2;
endmodule


module mux_2_32b(out, sel, D0, D1);

    output [31:0] out;
    input sel;
    input [31:0] D0, D1;
    
    assign out=sel? D1:D0;

endmodule


module mux_4_32b(out, sel, D0, D1, D2, D3);

    output [31:0] out;
    input [1:0]sel;
    input [31:0] D0, D1, D2, D3;
    
    assign out=sel[1]? (sel[0]?D3:D2):(sel[0]? D1: D0);

endmodule




//The main decoder for the system. Seperate from ALU decoder. This can be
//viewed as the microcode of the system for each instruction. 
module mainDecoder(MemWrite, RegWrite, RegDst, ALUSrc, MemtoReg, Branch, ALUOp, Jump, Opcode);
    output MemWrite, RegWrite, RegDst, ALUSrc, MemtoReg, Branch, Jump;
    output [1:0] ALUOp;
    input [5:0] Opcode;

    reg [8:0] controls;

    assign {RegWrite,RegDst,ALUSrc,Branch,MemWrite,MemtoReg,ALUOp,Jump}=controls;
    always @(*)
        if(Opcode==6'b000000) assign controls=9'b110000100;
        else if(Opcode==6'b100011) assign controls=9'b101001000;           
        else if (Opcode==6'b101011) assign controls=9'b0x101x000;
        else if (Opcode==6'b000100) assign controls=9'b0x010x010;
        else if (Opcode==6'b001000) assign controls=9'b101000000;
        else if (Opcode==6'b000010) assign controls=9'b0xxx0xxx1;
        else assign controls=9'bxxxxxxxxx;
endmodule









//make alu
//make datapath




//controller involves alu dec and main dec


//mips core involves data path and control path




//                  control path           data path
//heirachy goes (alu dec + main dec) + (alu(s) + pc + registers) = mips core

