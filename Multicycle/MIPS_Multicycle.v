//32 bit flip flop with active high reset 
module flopr #(parameter WIDTH = 8) (Clk,Reset, d, q);

    input Clk, Reset;
    input [WIDTH-1:0] d;
    output reg [WIDTH-1:0] q;

    
    always @ (posedge Clk,posedge Reset)
        if(Reset) q<=0;
        else q<=d; //output new PC from alu on posedge Clk 
endmodule


module flopr_EN #(parameter WIDTH = 8) (Clk,Reset,EN,d, q);   //flip flop with enable

    input Clk, Reset,EN;
    input [WIDTH-1:0] d;
    output reg [WIDTH-1:0] q;

    
    always @ (posedge Clk,posedge Reset)
        if(Reset) q<=0;
        else if(EN==1) q<=d; //output new PC from alu on posedge Clk 
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

    initial
        begin 
            $readmemh("memfile.dat",data); //read dissassembled MIPS code and load it into ram
    end


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



//The input to the alu control logic depends on 2 things, the op code and the
//funct space (in R type Instructions). For instance, an Instruction with
//opcode 000000 is always going to have an AluOp of 10. 10 indicates that the
//ALUControl depends on the funct field of the Instruction, (Or, Nor, Add all
//work differently). However, in I type Instructions, there is no funct field,
//so the alu controll only depends on the opcode, and the logic for that is
//quite straight forward. 

//ALU decoder controls the operation that occurs in the alu

//ALU decoder stays the same relative to single cycle 
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



//main decoder takes in opcode and outputs multiplexer selects and register enables
//IMP: it also outputs ALUOp which goes into the ALU decoder to control the operation of the ALU
//Since this MIPS implementation is multi cycle, a state machine needs to controll the control signals based on the clock number (and instruction) its executing.

module mainDecoder(Opcode,Clk,Reset,MemtoReg,MemWrite,RegDst,RegWrite,Jump,ALUOp, 
                    IorD, PCSrc,ALUSrcB,ALUSrcA,IRWrite,PCWrite,Branch); 


    input [5:0] Opcode;
    input Clk,Reset;
    output reg MemtoReg,MemWrite,RegDst,RegWrite,Jump, 
    IorD,ALUSrcA,IRWrite,PCWrite,Branch;
    output reg [1:0] PCSrc;
    output reg [1:0] ALUOp, ALUSrcB;


    reg [3:0] state,nextstate;



    //PARAMETERS (States)

    parameter S0=4'b0000;
    parameter S1=4'b0001;
    parameter S2=4'b0010;
    parameter S3=4'b0011;
    parameter S4=4'b0100;
    parameter S5=4'b0101;
    parameter S6=4'b0110;
    parameter S7=4'b0111;
    parameter S8=4'b1000;
    parameter S9=4'b1001;
    parameter S10=4'b1010;
    parameter S11=4'b1011;
    parameter Reset_state=4'bxxxx;

    always @ (posedge  Clk, posedge Reset)                           
        if (Reset) state<=Reset_state;
        else state <=nextstate;

    always @(*) //convers state transitions
        case (state)
            S0: nextstate=S1;
            S1: case(Opcode)
                6'b100011: nextstate=S2;    //load 
                6'b101011: nextstate=S2;    //store
                6'b000000: nextstate=S6;     //r type
                6'b000100: nextstate=S8;     //BEQ
                6'b001000: nextstate=S9;     //ADDI
                6'b000010: nextstate=S11;     //ADDI
                endcase

            S2: case(Opcode)
                6'b100011: nextstate=S3;    //load 
                6'b101011: nextstate=S5;    //store
                endcase

            S3: nextstate=S4;
            S4: nextstate=S0;
            S5: nextstate=S0;
            S6: nextstate=S7;
            S7: nextstate=S0;
            S8: nextstate=S0;
            S9: nextstate=S10;
            S10:nextstate=S0;
            S11:nextstate=S0;
            Reset_state:nextstate=S0;




        endcase

    

 always @(*) 
        begin//covers outputs (output logic)
            case (state)   
                S0: begin

                    MemtoReg=1'bx;
                    MemWrite=1'bx;
                    RegDst=1'bx;
                    RegWrite=1'bx;
                    Jump=1'bx;
                    IorD=1'b0; 
                    PCSrc=2'b00;
                    ALUSrcA=1'b0;
                    IRWrite=1'b1;
                    PCWrite=1'b1;
                    Branch=1'bx;
                    ALUOp=2'b00;
                    ALUSrcB=2'b01;  
                end

                S1: begin
                    MemtoReg=1'bx;
                    MemWrite=1'bx;
                    RegDst=1'bx;
                    RegWrite=1'bx;
                    Jump=1'bx;
                    IorD=1'bx; 
                    PCSrc=2'bxx;
                    ALUSrcA=1'b0;
                    IRWrite=1'bx;
                    PCWrite=1'bx;
                    Branch=1'bx;
                    ALUOp=2'b00;
                    ALUSrcB=2'b11;
                end
                
                S2: begin
                    MemtoReg=1'bx;
                    MemWrite=1'bx;
                    RegDst=1'bx;
                    RegWrite=1'bx;
                    Jump=1'bx; 
                    IorD=1'bx; 
                    PCSrc=2'bxx;
                    ALUSrcA=1'b1;
                    IRWrite=1'bx;
                    PCWrite=1'bx;
                    Branch=1'bx;
                    ALUOp=2'b00;
                    ALUSrcB=2'b10;
                end

                S3: begin
                    MemtoReg=1'bx;
                    MemWrite=1'bx;
                    RegDst=1'bx;
                    RegWrite=1'bx;
                    Jump=1'bx;
                    IorD=1'b1; 
                    PCSrc=2'bxx;
                    ALUSrcA=1'bx;
                    IRWrite=1'bx;
                    PCWrite=1'bx;
                    Branch=1'bx;
                    ALUOp=2'bxx;
                    ALUSrcB=2'bxx;
                end


                S4: begin
                    MemtoReg=1'b1;
                    MemWrite=1'bx;
                    RegDst=1'b0;
                    RegWrite=1'b1;
                    Jump=1'bx;
                    IorD=1'bx; 
                    PCSrc=2'bxx;
                    ALUSrcA=1'bx;
                    IRWrite=1'bx;
                    PCWrite=1'bx;
                    Branch=1'bx;
                    ALUOp=2'bxx;
                    ALUSrcB=2'bxx;
                end
                
                S5: begin
                    MemtoReg=1'bx;
                    MemWrite=1'b1;
                    RegDst=1'bx;
                    RegWrite=1'bx;
                    Jump=1'bx;
                    IorD=1'b1; 
                    PCSrc=2'bxx;
                    ALUSrcA=1'bx;
                    IRWrite=1'bx;
                    PCWrite=1'bx;
                    Branch=1'bx;
                    ALUOp=2'bxx;
                    ALUSrcB=2'bxx;
                end

                S6: begin
                    MemtoReg=1'bx;
                    MemWrite=1'bx;
                    RegDst=1'bx;
                    RegWrite=1'bx;
                    Jump=1'bx;
                    IorD=1'bx; 
                    PCSrc=2'bxx;
                    ALUSrcA=1'b1;
                    IRWrite=1'bx;
                    PCWrite=1'bx;
                    Branch=1'bx;
                    ALUOp=2'b10;
                    ALUSrcB=2'b00;
                end

                S7: begin
                    MemtoReg=1'b0;
                    MemWrite=1'bx;
                    RegDst=1'b1;
                    RegWrite=1'b1;
                    Jump=1'bx;
                    IorD=1'bx; 
                    PCSrc=2'bxx;
                    ALUSrcA=1'bx;
                    IRWrite=1'bx;
                    PCWrite=1'bx;
                    Branch=1'bx;
                    ALUOp=2'bxx;
                    ALUSrcB=2'bxx;
                end

                S8: begin
                    MemtoReg=1'bx;
                    MemWrite=1'bx;
                    RegDst=1'bx;
                    RegWrite=1'bx;
                    Jump=1'bx;
                    IorD=1'bx; 
                    PCSrc=2'b01;
                    ALUSrcA=1'b1;
                    IRWrite=1'bx;
                    PCWrite=1'bx;
                    Branch=1'b1;
                    ALUOp=2'b01;
                    ALUSrcB=2'b00;
                end

                S9: begin
                    MemtoReg=1'bx;
                    MemWrite=1'bx;
                    RegDst=1'bx;
                    RegWrite=1'bx;
                    Jump=1'bx;
                    IorD=1'bx; 
                    PCSrc=2'bxx;
                    ALUSrcA=1'b1;
                    IRWrite=1'bx;
                    PCWrite=1'bx;
                    Branch=1'bx;
                    ALUOp=2'b00;
                    ALUSrcB=2'b10;
                end
                S10: begin
                    MemtoReg=1'b0;
                    MemWrite=1'bx;
                    RegDst=1'b0;
                    RegWrite=1'b1;
                    Jump=1'bx;
                    IorD=1'bx; 
                    PCSrc=2'bxx;
                    ALUSrcA=1'bx;
                    IRWrite=1'bx;
                    PCWrite=1'bx;
                    Branch=1'bx;
                    ALUOp=2'bxx;
                    ALUSrcB=2'bxx;
                end

                S11: begin                      //ADD Stuff here
                    MemtoReg=1'bx;
                    MemWrite=1'bx;
                    RegDst=1'bx;
                    RegWrite=1'bx;
                    Jump=1'bx;
                    IorD=1'bx; 
                    PCSrc=2'b10;
                    ALUSrcA=1'bx;
                    IRWrite=1'bx;
                    PCWrite=1'b1;
                    Branch=1'bx;
                    ALUOp=2'bxx;
                    ALUSrcB=2'bxx;
                end

                Reset_state: begin
                    MemtoReg=1'bx;
                    MemWrite=1'bx;
                    RegDst=1'bx;
                    RegWrite=1'bx;
                    Jump=1'bx;
                    IorD=1'bx; 
                    PCSrc=1'bx;
                    ALUSrcA=1'bx;
                    IRWrite=1'bx;
                    PCWrite=1'bx;
                    Branch=1'bx;
                    ALUOp=2'bxx;
                    ALUSrcB=2'bxx;
                end
            endcase
        end
        




endmodule




module controller (Opcode,Funct,Zero,Clk,Reset,MemtoReg,MemWrite,PCSrc,RegDst,RegWrite,ALUControl,Branch, IorD,IRWrite, PCWrite,ALUSrcB,ALUSrcA,PCEn);


    input [5:0] Opcode, Funct;
    input Zero,Clk,Reset;

    output MemtoReg,MemWrite,RegDst,RegWrite,Branch, IorD,IRWrite, PCWrite,ALUSrcA,PCEn;
    output [2:0] ALUControl;
    output [1:0] ALUSrcB,PCSrc;


    wire [1:0] ALUOp;


    mainDecoder md (Opcode,Clk,Reset,MemtoReg,MemWrite,RegDst,RegWrite,Jump,ALUOp, 
                    IorD, PCSrc,ALUSrcB,ALUSrcA,IRWrite,PCWrite,Branch);

    AluDecoder ALUDec (Funct,ALUOp,ALUControl);


    assign PCEn = PCWrite||(Zero&&Branch);


endmodule

                             //control inputs                                                                           
module datapath(Clk,Reset,
MemtoReg,PCSrc,RegDst,RegWrite,ALUControl,Branch, IorD,IRWrite, PCWrite,ALUSrcB,ALUSrcA,PCEn,
WriteData, Adr,
ReadData,
Zero, Op, Funct
);
 
    input Clk,Reset;
    input MemtoReg,RegDst,RegWrite,Branch, IorD,IRWrite, PCWrite,ALUSrcA,PCEn;    //control logic input
   
    input [2:0] ALUControl;
    input [1:0] ALUSrcB,PCSrc;    //control logic input
    input [31:0] ReadData;  //output of memory (RAM)

    output [31:0] WriteData, Adr; //input to RAM 
    output Zero;    //ALU Zero flag
    output [5:0] Op, Funct; //input to Control unit
    

    wire [4:0] A3;
    wire [31:0] WD3, RD1, RD2, Data, A, B;

    wire [31:0] Instr;

    mux2 #5 A3_Mux(Instr[20:16],Instr[15:11],RegDst, A3);
    mux2 #32 WD3_Mux(ALUOut,Data,MemtoReg, WD3);

    registerFile regFile(Clk,RegWrite,Instr[25:21],Instr[20:16],A3,WD3,RD1,RD2);

    flopr #32 RD1_ff(Clk,Reset,RD1,A);
    flopr #32 RD2_ff(Clk,Reset,RD2,B);


    assign WriteData=B;

    wire [31:0] SignImm;
    SignExtender se(Instr[15:0],SignImm);

    
    wire [31:0] PC, SrcA;
    mux2 #32 SrcA_Mux(PC,A,ALUSrcA,SrcA);

    wire [31:0] shifted_SignImm;
    shift_left_2 sl2(shifted_SignImm,SignImm);  //out, in

    wire [31:0] SrcB;
    mux_4_32b SrcB_Mux(SrcB,ALUSrcB,B,3'b100,SignImm,shifted_SignImm);

    wire [31:0] ALUResult;
    wire C_out;
    ALU mainALU (Zero, ALUResult, C_out,ALUControl,SrcA,SrcB);

    wire [31:0] ALUOut;
    flopr #32 ALU_reg(Clk,Reset,ALUResult,ALUOut);

    wire [31:0] PC_prime;
    wire [25:0] j_addr;

    shift_left_2 j_addr_shift(j_addr,Instr[25:0]);

    mux_4_32b ALU_Mux(PC_prime,PCSrc,ALUResult,ALUOut,j_addr,); //pcsrc 2 bits now


   // mux2 #32 ALU_Mux(ALUResult,ALUOut,PCSrc,PC_prime);




    flopr_EN #32 PC_reg(Clk,Reset,PCEn,PC_prime,PC);

    mux2 #32 Adr_mux(PC,ALUOut,IorD,Adr);

    flopr_EN #32 instruction_reg (Clk,Reset,IRWrite,ReadData,Instr);

    flopr #32 data_reg (Clk,Reset,ReadData,Data);
    
    assign Op= Instr[31:26];
    assign Funct=Instr[5:0];

endmodule

module mux_2_5b(out, sel, D0, D1); //for controlling A3

    output [4:0] out;
    input sel;
    input [4:0] D0, D1;
    
    assign out=sel? D1:D0;

endmodule






module MIPS (Clk,Reset,ReadData,PC,MemWrite,WriteData);     //connects between dp and controll

    input Clk, Reset; 
    input [31:0] ReadData;

    output MemWrite;
    output [31:0] PC, WriteData;


    wire MemtoReg,MemWrite,RegDst,RegWrite,Branch, IorD,IRWrite, PCWrite,ALUSrcA,PCEn;    //control signals
    wire [2:0] ALUControl;  
    wire [1:0] ALUSrcB,PCSrc;

    wire [5:0] Opcode, Funct;
    wire Zero;

    datapath dp(Clk,Reset,
    MemtoReg,PCSrc,RegDst,RegWrite,ALUControl,Branch, IorD,IRWrite, PCWrite,ALUSrcB,ALUSrcA,PCEn,
    WriteData, PC,
    ReadData,
    Zero, Opcode, Funct
    );

    controller control(Opcode,Funct,Zero,Clk,Reset,
    MemtoReg,MemWrite,PCSrc,RegDst,RegWrite,ALUControl,Branch, IorD,IRWrite, PCWrite,ALUSrcB,ALUSrcA,PCEn);


endmodule


module top(Clk, Reset, WriteData,DataAdr,MemWrite); //Connects memory to mips core

    input Clk,Reset;
    output [31:0] DataAdr,WriteData;
    output MemWrite;
    
    wire [31:0] PC, ReadData;
    
    MIPS core(Clk,Reset,ReadData,DataAdr,MemWrite,WriteData);
    
    data_memory RAM(Clk, MemWrite, DataAdr, WriteData, ReadData);


endmodule
