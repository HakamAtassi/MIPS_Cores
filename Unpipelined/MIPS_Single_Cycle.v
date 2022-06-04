//on reset PC should begin at 0xBFC0 0000



//32 bit flip flop with active high reset 
module Dflipflop(Q, clk, D, reset);

    input clk, reset; 
    input [31:0] D;
    output reg [31:0] Q;

    always @ (posedge clk,posedge reset)
        if(reset==1) Q<=0;
        else Q<=D; //output new pc from alu on posedge clk 
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
        

    assign RD1 = (A1 !=0) ? internal_mem[A1] :0;
    assign RD2 = (A2 !=0) ? internal_mem[A2] :0;

endmodule


//in single cycle mips architecture, the program and variables are stored in
//two seperate registers. instruction memory stores the program, Data memory
//(TBD) stores variable information
module instruction_memory(RD, A);
    output [31:0] RD;    //output data is 32 bits wide because thats how long instructions are
    input [5:0] A;      //64 possible addresses

    reg [31:0] RAM [63:0]; //64 wires each 32 bits wide
    
    initial
        begin 
            $readmemh("memfile.dat",RAM); //read dissassembled MIPS code and load it into ram
    end

    assign RD = RAM[A]; //output onto bus. no need for clock because ram is asynchronous

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
    output reg [31:0] RD;
    input clk, WE;
    input [31:0] A,WD;


    reg [31:0] data [63:0];


//    assign RD = data[A[31:2]]; //address is always being read out

  //  always @ (posedge clk)
    //    if(WE==1) data[A[31:2]] <=WD;

    always @(*) begin
       RD <= data[A]; //address is always being read out
    end


    always @ (posedge clk)
        if(WE==1) data[A[31:2]] <=WD;

endmodule



module SignExtender(extend, extended);

    input [15:0] extend;
    output [31:0] extended;

    assign extended[31:0]= {{16{extend[15]}},extend[15:0]};

endmodule

module adder(Y,C_out,C_in,SrcA, SrcB);

    output [31:0] Y;
    output C_out;
    input [31:0] SrcA, SrcB;
    input C_in;
    

    assign {C_out,Y}=SrcA+SrcB+C_in;


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
    

    always @ (*)
        case (AluOP)
            2'b00: AluControl <=3'b010;
            2'b01: AluControl <=3'b110;
            default: case(funct)
                6'b100000 : AluControl <= 3'b010;   //add
                6'b100010 : AluControl <= 3'b110;   //sub
                6'b100100 : AluControl <= 3'b000;   //and
                6'b100101 : AluControl <= 3'b001;   //or
                6'b101010 : AluControl <= 3'b111;   //slt
                default: AluControl <=3'bxxx;
            endcase
        endcase
endmodule
/*

    always @(*)
        if (AluOP==2'b0x) AluControl<=3'b010;
        else if (AluOP==2'bx1) AluControl <=3'b110;
        else if(AluOP==2'b1x) case(funct)
            6'b100000 : AluControl <= 3'b010;   //add
            6'b100010 : AluControl <= 3'b110;   //sub
            6'b100100 : AluControl <= 3'b000;   //and
            6'b100101 : AluControl <= 3'b001;   //or
            6'b101010 : AluControl <= 3'b111;   //slt
        default: AluControl <=3'bxxx;   //illegal opcode
        endcase
endmodule
*/

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
//a posative 
module ALU(Zero, ALUResult, C_out, ALUControl, SrcA, SrcB);

    output Zero;
    output [31:0] ALUResult;
    output C_out;
    input [2:0] ALUControl;
    input [31:0] SrcA, SrcB;
    
    wire [31:0] SrcB_not; assign SrcB_not=~SrcB;
    wire [31:0] mux1_out;
    wire [31:0] N0,N1,N2,N3;


    mux_2_32b mux1(mux1_out,ALUControl[2], SrcB,SrcB_not);
 
    assign N0=mux1_out&SrcA;

    assign N1=mux1_out&SrcA;


    adder add1(N2,C_out,ALUControl[2],mux1_out,SrcA);
    assign N3={1'b0,N2[30:0]};

    mux_4_32b mux2(ALUResult,ALUControl[1:0],N0,N1,N2,N3);


    assign Zero = (ALUResult==0) ?   1 :0;

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
        if(Opcode==6'b000000) controls<=9'b110000010;
        else if(Opcode==6'b100011) controls<=9'b101001000;           
        else if (Opcode==6'b101011) controls<=9'b001010000;
        else if (Opcode==6'b000100) controls<=9'b000100001;
        else if (Opcode==6'b001000) controls<=9'b101000000;
        else if (Opcode==6'b000010) controls<=9'b000000100;
        else controls<=9'bxxxxxxxxx;
endmodule




module controller (ALUControl, PCSrc, MemtoReg,ALUSrc,RegDst,RegWrite, MemWrite, Opcode, Funct, Zero, Jump);

    
    output [2:0] ALUControl;
    output PCSrc, MemtoReg, ALUSrc, RegDst, RegWrite, MemWrite,Jump;

    input [5:0] Funct;
    input [5:0] Opcode;
    input Zero;
    
    wire Branch;
    wire [1:0] ALUOp;

    mainDecoder md1(MemWrite, RegWrite, RegDst, ALUSrc, MemtoReg, Branch, ALUOp, Jump, Opcode);

    AluDecoder aludec1(ALUControl, ALUOp,Funct);

    
    assign PCSrc=Branch & Zero; //source of pc. /\


endmodule



module datapath(ALUOut, WriteData, PC, Zero, Reset, Clk, ALUControl, PCSrc, MemtoReg, ALUSrc, RegDst, RegWrite, Jump, ReadData,Instr);

    output Zero; //zero flag if alu output is 0
    output [31:0] PC; //program counter output after selection (final PC for instruction retrieving)
    output [31:0] ALUOut, WriteData;
    
    input Reset, Clk, MemtoReg, PCSrc, ALUSrc, RegDst, RegWrite, Jump;
    input [2:0] ALUControl; 
    input [31:0] ReadData; //inputs datat to datapath
    input [31:0] Instr;

    wire [4:0] RegisterAddress; //address if writing to registers (A3)
    
    
    wire [31:0] PCNext, PCNextbr, PCPlus4, PCBranch; 
    //PCPLUS4 is the output of the ALU after adding 4
    //PCBranch is the PC value after a branch instruction (which uses relative addressing)
    //PCNextbr is the output of the mux after selecting between branching and just adding 4
    
    //PCNext is the value of the PC that is stored in the PC regitser

    wire [31:0] SignImm, SignImmSh;
    //a branch may be posative or negative. Since the PC is 32 bits and the
    //largest possible relative address is 16 bits, it must be extended to 32
    //bits with the correct sign in mind. 
    //SignImm is the unextended branch value (relative address) and SignImmSh
    //is the sign extended relative address inputted to the PC ALU

    wire [31:0] SrcA, SrcB; 
    //inputs to the main ALU

    wire [31:0] Result;
    //This is not the alu resut, it is the output of the mux that selects
    //between data memory outout and aluoutput. This is imporatnt as it
    //determines what is written to the register file
    

    Dflipflop PCRegister(PC,Clk, PCNext, Reset);
    //PCRegister


    adder pcadd1(PCPlus4,,1'b0,32'b100,PC); //the adder for the Program counter iteration

    shift_left_2 immsh(SignImmSh,SignImm);
    //shiftng for branch operatioms (multiply by 4)

    adder pcadd2 (PCBranch,,1'b0,SignImmSh,PCPlus4);
    //the adder for branch operations. number of bytes+PC
    
    mux_2_32b pcbrmux(PCNextbr,PCSrc, PCPlus4, PCBranch);                                        //this module is faulty
    //this mux determines if the pc should use the next address(PC+4) or the
    //branched PC (PCBranch) address based on contrrol logic from main dec

    mux_2_32b pcmux(PCNext, Jump,PCNextbr, {PCPlus4[31:28],Instr[25:0], 2'b00});



    


    //register file  //ERROR WITH WriteData
    
    //registerFile(RD1, RD2, A1, A2, A3, WD3, clk, WE3);
    registerFile rf(SrcA, WriteData,Instr[25:21],Instr[20:16],RegisterAddress,Result,Clk, RegWrite);        //ERROR HERE ERROR HERE ERROR HERE


    //mux_2_32b(out, sel, D0, D1); 
    mux_2_5b A3Mux(RegisterAddress,RegDst,Instr[20:16],Instr[25:21]);

    mux_2_32b WD3Mux(Result,MemtoReg,ALUOut, ReadData);

    SignExtender se(Instr[15:0],SignImm);
    //ALU
    mux_2_32b SRCBmux (SrcB, ALUSrc, WriteData, SignImm);
    
    wire C_out;

    ALU alu_main (Zero, ALUOut,C_out,ALUControl,SrcA, SrcB);




endmodule

module mux_2_5b(out, sel, D0, D1); //for controlling A3

    output [4:0] out;
    input sel;
    input [4:0] D0, D1;
    
    assign out=sel? D1:D0;

endmodule


module MIPS (ALUOut,WriteData,WE, PC,Instr,ReadData,Reset,Clk);

    output  WE; //write enable
    output [31:0] ALUOut,WriteData;
    output [31:0] PC;
    input [31:0] Instr, ReadData;
    input Reset,Clk;

    wire MemToReg, Branch,ALUSrc, RegDst, RegWrite, Jump,PCSrc,Zero;
    wire [2:0] ALUControl;
    
    //controller (ALUControl, PCSrc, MemtoReg,ALUSrc,RegDst,RegWrite, MemWrite, Opcode, Funct, Zero, Jump);
    controller control(ALUControl,PCSrc,MemToReg,ALUSrc,RegDst,RegWrite,WE,Instr[31:26],Instr[5:0],Zero,Jump);


    //datapath(ALUOut, WriteData, PC, Zero, Reset, Clk, ALUControl, PCSrc, MemtoReg, ALUSrc, RegDst, RegWrite, Jump, ReadData,Instr);
    datapath dp(ALUOut, WriteData, PC, Zero, Reset, Clk, ALUControl, PCSrc, MemToReg, ALUSrc, RegDst, RegWrite, Jump, ReadData,Instr);


endmodule


module top(Clk, Reset, WriteData,DataAdr,WE);

    input Clk, Reset;
    output [31:0] WriteData, DataAdr;
    output WE;


    wire [31:0] PC, Instr, ReadData;

    //MIPS (ALUOut,WriteData,WE, PC,Instr,ReadData,Reset,Clk);
    MIPS mips(DataAdr,WriteData,WE,PC,Instr,ReadData,Reset,Clk);

    //instruction_memory(RD, A);
    instruction_memory imem(Instr,PC[7:2]);



    //data_memory(RD,clk,A,WD,WE);    
    data_memory dmem(ReadData, Clk, DataAdr, WriteData, WE);

endmodule
