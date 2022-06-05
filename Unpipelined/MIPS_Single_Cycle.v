//on reset PC should begin at 0xBFC0 0000



//32 bit flip flop with active high reset 
module Dflipflop(Q, Clk, D, reset);

    input Clk, reset; 
    input [31:0] D;
    output reg [31:0] Q;

    always @ (posedge Clk,posedge reset)
        if(reset==1) Q<=0;
        else Q<=D; //output new PC from alu on posedge Clk 
endmodule


module flopr #(parameter WIDTH = 8) (Clk,Reset, d, q);

    input Clk, Reset;
    input [WIDTH-1:0] d;
    output reg [WIDTH-1:0] q;

    
    always @ (posedge Clk,posedge Reset)
        if(Reset) q<=0;
        else q<=d; //output new PC from alu on posedge Clk 
endmodule


module mux2 #(parameter WIDTH = 8) (d0,d1,s,y);

    input [WIDTH-1:0] d0, d1;
    input s;
    output [WIDTH-1:0] y;

    assign y=s?d1:d0;

endmodule


//register file. Contains 32 internal registers. each 32 bits wide
//address 1 (A1) is read to RD1 asynchronously. Same is true for A2 and RD2.
//To write, input address is A3. Address to read is WD3. occurs on posedge Clk
//write enable must also be active. 
module registerFile(Clk,WE3,A1,A2,A3,WD3,RD1,RD2);
    
    output [31:0] RD1, RD2;  //outputs of the register file are each 32 bits since the regsiter is 32 bits wide
    input [4:0] A1,A2,A3;   //2^5=32 since 32 total addresses
    input [31:0] WD3; //the data to be written if WE3 is high 
    input Clk, WE3; 

    reg [31:0] internal_mem [31:0]; //32 wires each 32 bits long

    
        always @ (posedge Clk) 
            if (WE3) internal_mem[A3]<=WD3;
        

    assign RD1 = (A1 !=0) ? internal_mem[A1] :0;
    assign RD2 = (A2 !=0) ? internal_mem[A2] :0;

endmodule


//in single cycle mips architecture, the program and variables are stored in
//two seperate registers. Instruction memory stores the program, Data memory
//(TBD) stores variable information
module Instruction_memory(A, RD);
    output [31:0] RD;    //output data is 32 bits wide because thats how long Instructions are
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
module data_memory(Clk, WE, A, WD, RD);
    //not using full memory capabilities
    //

    output reg [31:0] RD;
    input Clk, WE;
    input [31:0] A,WD;


    reg [31:0] data [63:0];


//    assign RD = data[A[31:2]]; //address is always being read out

  //  always @ (posedge Clk)
    //    if(WE==1) data[A[31:2]] <=WD;

    always @(*) begin
        RD<=data[A[31:2]];
    end

    //assign RD=data[A[31:2]];

    always @ (posedge Clk)
        if(WE) data[A[31:2]] <=WD;

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
//funct space (in R type Instructions). For instance, an Instruction with
//opcode 000000 is always going to have an AluOp of 10. 10 indicates that the
//ALUControl depends on the funct field of the Instruction, (Or, Nor, Add all
//work differently). However, in I type Instructions, there is no funct field,
//so the alu controll only depends on the opcode, and the logic for that is
//quite straight forward. 

//ALU decoder controls the operation that occurs in the alu
module AluDecoder(Funct,AluOP,ALUControl);

    output reg [2:0]ALUControl;
    input [1:0] AluOP;
    input [5:0] Funct;
    

    always @ (*)
        case (AluOP)
            2'b00: ALUControl <=3'b010;
            2'b01: ALUControl <=3'b110;
            default: case(Funct)
                6'b100000 : ALUControl <= 3'b010;   //add
                6'b100010 : ALUControl <= 3'b110;   //sub
                6'b100100 : ALUControl <= 3'b000;   //and
                6'b100101 : ALUControl <= 3'b001;   //or
                6'b101010 : ALUControl <= 3'b111;   //slt
                default: ALUControl <=3'bxxx;
            endcase
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

    assign N1=mux1_out|SrcA;


    adder add1(N2,C_out,ALUControl[2],mux1_out,SrcA);
    assign N3={N2[31]};

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
//viewed as the microcode of the system for each Instruction. 
module mainDecoder(Opcode,MemtoReg,MemWrite,Branch,ALUSrc,RegDst,RegWrite,Jump,ALUOp);


    output MemWrite, RegWrite, RegDst, ALUSrc, MemtoReg, Branch, Jump;
    output [1:0] ALUOp;
    input [5:0] Opcode;

    reg [8:0] controls;


    assign {RegWrite,RegDst,ALUSrc,Branch,MemWrite,MemtoReg,Jump,ALUOp}=controls;

    always @(*)
        if(Opcode==6'b000000) controls<=9'b110000010;
        else if(Opcode==6'b100011) controls<=9'b101001000;           
        else if (Opcode==6'b101011) controls<=9'b001010000;
        else if (Opcode==6'b000100) controls<=9'b000100001;
        else if (Opcode==6'b001000) controls<=9'b101000000;
        else if (Opcode==6'b000010) controls<=9'b000000100;
        else controls<=9'bxxxxxxxxx;
endmodule




module controller (Opcode,Funct,Zero,MemtoReg,MemWrite,PCSrc,ALUSrc,RegDst,RegWrite,Jump,ALUControl);





    
    output [2:0] ALUControl;
    output PCSrc, MemtoReg, ALUSrc, RegDst, RegWrite, MemWrite,Jump;

    input [5:0] Funct;
    input [5:0] Opcode;
    input Zero;
    
    wire Branch;
    wire [1:0] ALUOp;


    mainDecoder md1(Opcode,MemtoReg,MemWrite,Branch,ALUSrc,RegDst,RegWrite,Jump,ALUOp);

    AluDecoder aludec1(Funct,ALUOp,ALUControl);

    
    assign PCSrc=Branch & Zero; //source of PC. /\


endmodule



module datapath(Clk,Reset,MemtoReg,PCSrc,ALUSrc,RegDst,RegWrite,Jump,ALUControl,Zero,PC,Instr,ALUOut,WriteData,ReadData);

    output Zero; //Zero flag if alu output is 0
    output [31:0] PC; //program counter output after selection (final PC for Instruction retrieving)
    output [31:0] ALUOut, WriteData;
    
    input Reset, Clk, MemtoReg, PCSrc, ALUSrc, RegDst, RegWrite, Jump;
    input [2:0] ALUControl; 
    input [31:0] ReadData; //inputs datat to datapath
    input [31:0] Instr;

    wire [4:0] WriteReg; //address if writing to registers (A3)
    
    
    wire [31:0] PCNext, PCNextbr, PCPlus4, PCBranch; 
    //PCPLUS4 is the output of the ALU after adding 4
    //PCBranch is the PC value after a branch Instruction (which uses relative addressing)
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
    //between data memory outout and ALUOutput. This is imporatnt as it
    //determines what is written to the register file
    

    flopr #(32) PCRegister(Clk,Reset,PCNext,PC);
    //PCRegister


    adder pcadd1(PCPlus4,,1'b0,32'b100,PC); //the adder for the Program counter iteration

    shift_left_2 immsh(SignImmSh,SignImm);
    //shiftng for branch operatioms (multiply by 4)

    adder pcadd2 (PCBranch,,1'b0,SignImmSh,PCPlus4);
    //the adder for branch operations. number of bytes+PC
    


    mux2 #(32) pcbrmux(PCPlus4,PCBranch,PCSrc,PCNextbr);

    //this mux determines if the PC should use the next address(PC+4) or the
    //branched PC (PCBranch) address based on contrrol logic from main dec


    
    mux2 #(32) pcmux (PCNextbr,{PCPlus4[31:28],Instr[25:0],2'b00},Jump,PCNext);



    registerFile rf(Clk,RegWrite, Instr[25:21],Instr[20:16],WriteReg,Result,SrcA,WriteData);  


    //mux_2_32b(out, sel, D0, D1); 
    mux2 #5 wrmux(Instr[20:16], Instr[15:11], RegDst, WriteReg);


    mux2 #32 resmux(ALUOut, ReadData, MemtoReg, Result);

    SignExtender se(Instr[15:0],SignImm);
    //ALU
    mux2 #(32) SrcBmux (WriteData,SignImm,ALUSrc,SrcB);
    
    wire C_out;

    ALU alu_main (Zero, ALUOut,C_out,ALUControl,SrcA, SrcB);


endmodule

module mux_2_5b(out, sel, D0, D1); //for controlling A3

    output [4:0] out;
    input sel;
    input [4:0] D0, D1;
    
    assign out=sel? D1:D0;

endmodule


module MIPS (Clk,reset,PC,Instr,memwrite,ALUOut,writedata,ReadData);


    input Clk, reset;
    input [31:0] Instr;
    input [31:0] ReadData;

    output [31:0] PC,ALUOut,writedata;
    output memwrite;



    wire MemToReg, Branch,ALUSrc, RegDst, RegWrite, Jump,PCSrc,Zero;
    wire [2:0] ALUControl;

    controller control(Instr[31:26],Instr[5:0],Zero,MemtoReg,memwrite,PCSrc,ALUSrc,RegDst,RegWrite,Jump,ALUControl);



    //datapath dp(ALUOut, WriteData, PC, Zero, Reset, Clk, ALUControl, PCSrc, MemToReg, ALUSrc, RegDst, RegWrite, Jump, ReadData,Instr);
    datapath dp(Clk,reset,MemtoReg,PCSrc,ALUSrc,RegDst,RegWrite,Jump,ALUControl,Zero,PC,Instr,ALUOut,writedata,ReadData);


endmodule


module top(Clk, Reset, WriteData,DataAdr,MemWrite);


    input Clk, Reset;
    output [31:0] WriteData, DataAdr;
    output MemWrite;


    wire [31:0] PC, Instr, ReadData;

    //MIPS (ALUOut,WriteData,WE, PC,Instr,ReadData,Reset,Clk);
    MIPS mips(Clk,Reset,PC,Instr,MemWrite,DataAdr,WriteData,ReadData);

    //Instruction_memory(RD, A);
    Instruction_memory imem(PC[7:2],Instr);



    //data_memory(RD,Clk,A,WD,WE);    
    data_memory dmem(Clk,MemWrite,DataAdr,WriteData,ReadData);

endmodule
