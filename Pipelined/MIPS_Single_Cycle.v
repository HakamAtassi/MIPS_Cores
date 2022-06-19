

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



//Notation after wire name denotes the output stage it comes from. ie, BranchD comes from Decode stage...
//required to solve several pipeline hazards; 
//RAW data hazard. If reading from the register file the same address that is about to be written to, forward the result of the alu to the 
//fetched value so it is up to date and does not fetch a "stale" value.





module hazard_unit(BranchD,RsD,RtD,RsE,RtE,WriteRegE,WriteRegM,MemtoRegE,RegWriteE,MemtoRegM,RegWriteM,WriteRegW,RegWriteW,StallF,StallD,ForwardAD,ForwardBD,FlushE,ForwardAE,ForwardBE);

    input BranchD,MemtoRegE,RegWriteE,RegWriteM,RegWriteW, MemtoRegM;
    input [4:0] RsD,RtD,RsE,RtE;
    input [4:0] WriteRegE,WriteRegM,WriteRegW;

    output reg StallF,StallD,ForwardAD,ForwardBD,FlushE;
    output reg [1:0] ForwardAE,ForwardBE;


    

    always @ (*)
        if ((RsE !=0) && (RsE==WriteRegM) && RegWriteM) ForwardAE<=2'b10;    //if the souce of a current operation and address being written to match, dont use data from 
        //source, use data from alu (forwarded)

        else if ((RsE !=0) && (RsE==WriteRegW) && RegWriteW) ForwardAE<=2'b01;
        //if the new data was just written to. (you would otherwise need 2 NOPs. the RAW hazard counts for 2 clock cycles.)
        else ForwardAE=2'b00;
        //else just use the normal data. 

    

    //same as above but for srcB
    always @ (*)
        if ((RtE !=0) && (RtE==WriteRegM) && RegWriteM) ForwardBE<=2'b10;    

        else if ((RtE !=0) && (RtE==WriteRegW) && RegWriteW) ForwardBE<=2'b01;

        else ForwardBE=2'b00;



    //A similar hazard occurs when loading a value from ram to a register. 
    //However, forwarding wont work due to the time it takes to retirieve the data from ram. 
    //to overcome this, we can add stalls (by disabling all the registers, hence doing nothing)



    wire lwstall; //placeholder
    wire branchstall;

    assign branchstall=((BranchD && RegWriteE && (WriteRegE == RsD || WriteRegE==RtD)) || (BranchD && MemtoRegM && (WriteRegM == RsD || WriteRegM==RtD)) );
    assign lwstall= (((RsD==RtE) || (RtD==RtE)) && MemtoRegE); 

    always @ (*) begin

        StallD<=lwstall || branchstall;
        StallF<=lwstall || branchstall;
        FlushE<=lwstall || branchstall;

     /*   
        StallD=lwstall;
        StallF=lwstall;
        FlushE=lwstall;
*/
    end

    //for BEQ, a comparison takes place,
    //if the 2 registers are equal, take a branch
    //what if the registers are currently being written to by an earlier instruction?
    //Yet another RAW hazard that must be taken care of.

    always @ (*) begin
        ForwardAD<=((RsD != 0) && (RsD == WriteRegM) && RegWriteM) ; //same controll hazard as earilier
        ForwardBD<=((RtD != 0) && (RtD == WriteRegM) && RegWriteM); //same controll hazard as earilier
    end




endmodule


module equality_checker(A,B,Bool);  //checks equaliy of 2 32 bit values

    input [31:0] A, B;
    output reg Bool;

    always @ (*) begin
        if (A==B) Bool=1'b1;
        else Bool=1'b0;
    end





endmodule


//module datapath(Clk,Reset,MemtoReg,PCSrc,ALUSrc,RegDst,RegWrite,Jump,ALUControl,Zero,PC,Instr,ALUOut,WriteData,ReadData);


//NONE OF THE REGISTERS HAVE THE CLEARS/ENABLES FOR STALLING AND FLUSHING!!!
module datapath(Clk,Reset,RegWriteD,MemtoRegD,MemWriteD,ALUControlD,ALUSrcD,RegDstD,BranchD,
Instr, ReadData,
StallF, StallD, ForwardAD,ForwardBD,FlushE,ForwardAE,ForwardBE,
ALUOutM,WriteDataM,PCF,
Opcode,Funct,
BranchD,RsD,RtD,RsE,RtE,WriteRegE,MemtoRegE,RegWriteE,WriteRegM,MemtoRegM,RegWriteM,WriteRegW,RegWriteW
);

    input Clk,Reset,RegWriteD,MemtoRegD,MemWriteD,ALUSrcD,RegDstD;
    input [2:0] ALUControlD;


    input [31:0] Instr, ReadData;
    input StallF, StallD, ForwardAD,ForwardBD,FlushE;
    input [1:0] ForwardAE,ForwardBE;

    output [31:0] ALUOutM,WriteDataM,PCF;
    output [5:0] Opcode,Funct;
    output BranchD,MemtoRegE,RegWriteE,MemtoRegM,RegWriteM,RegWriteW;
    output [4:0] RsD,RtD,RsE,RtE,WriteRegE,WriteRegM,WriteRegW;


/*************************************************FETCH LOGIC BELOW******************************************************/




    mux2 #(32) PC_prime_mux (PCPlus4F, PC_MUX_1, PCSrcD, PC_prime);


    flopr PC_prime_ff(Clk, Reset, PC_prime, PCF);

    adder pcadd1(PCPlus4F,,1'b0,32'b100,PCF); //the adder for the Program counter iteration


    not(StallD_not,StallD);
    flopr fetch_stage_RD(Clk,StallD_not, Instr,InstrD);
    flopr fetch_stage_PCPlus4(Clk,StallD_not, ReadData, PCPlus4D);


/*************************************************FETCH LOGIC ABOVE******************************************************/

/**************************************************DECODE LOGIC BELOW****************************************************/

    wire [31:0] ResultW, RD1, RD2;
    registerFile rf(Clk,RegWriteW, InstrD[25:21],InstrD[20:16],WriteRegW,ResultW,RD1,RD2);

    wire [31:0] SignImmD;
    SignExtender se(InstrD[15:0],SignImmD);

    wire [31:0] InstrD_extended_shifted;
    shift_left_2 sl2(InstrD_extended_shifted,InstrD_extended);

    wire [31:0] PCPlus4D,PC_MUX_1;
    adder rel_and_PC(PC_MUX_1,,,InstrD_extended_shifted,PCPlus4D);

    wire [31:0] RD1_mux_out;
    mux2 #(32) RD1_br_mux(RD1,ALUOutM, ForwardAD, RD1_mux_out);

    wire [31:0] RD2_mux_out;
    mux2 #(32) RD2_br_mux(RD2,ALUOutM, ForwardBD, RD2_mux_out);

    wire EqualD;
    equality_checker Branch_predictor(RD1_mux_out,RD2_mux_out,EqualD);

    wire PCSrcD;
    and(PCSrcD,BranchD,EqualD);



    flopr #(1) decode_stage_RegWriteD(Clk,,RegWriteD,RegWriteE);
    flopr #(1) decode_stage_MemtoRegD(Clk,,MemtoRegD,MemtoRegE);
    flopr #(1) decode_stage_MemWritD(Clk,,MemWritD,MemWriteE);
    flopr #(2) decode_stage_ALUControlD(Clk,,ALUControlD,ALUControlE);
    flopr #(1) decode_stage_ALUSrcD(Clk,,ALUSrcD,ALUSrcE);
    flopr #(1) decode_stage_RegDstD(Clk,,RegDstD,RegDstE);


    flopr #(5) decode_stage_RsD(Clk,,RsD,RsE);
    flopr #(5) decode_stage_RtD(Clk,,RtD,RtE);
    flopr #(5) decode_stage_RdD(Clk,,RdD,RdE);
    flopr #(32) decode_stage_SignImmD(Clk,,SignImmD,SignImmE);



    /***********************************************DECODE LOGIC ABOVE***************************************************/



    /***********************************************EXECUTE LOGIC BELOW***************************************************/

        //potentially an error here with the double output muxes. 

    wire [31:0] SrcAE;
    mux_4_32b SrcA_mux(SrcAE, ForwardAE, RD1, ResultW, ALUOutM,);

    wire [31:0] SrcBE,WriteDataE;
    mux_4_32b SrcB_mux(WriteDataE, ForwardBE, RD2, ResultW, ALUOutM,);

    wire [31:0] SignImmE;
    mux2 #(32) SrcBE_mux(WriteDataE,SignImmE, ALUSrcE, SrcBE);

    wire [31:0] ALUResult;
    ALU mainALU(,ALUResult, , ALUControlE, SrcAE, SrcBE);


    mux2 #(5) Rt_Rd_mux (RtE,RdE, RegDstE, WriteRegE);


    flopr #(1) execute_stage_RegWriteE(Clk,,RegWriteE,RegWriteM);
    flopr #(1) execute_stage_RegWriteE(Clk,,MemtoRegE,MemtoRegM);

    wire MemWriteM;
    flopr #(1) execute_stage_RegWriteE(Clk,,MemWriteE,MemWriteM);

    flopr #(1) execute_stage_RegWriteE(Clk,,ALUResult,ALUOutM);
    flopr #(1) execute_stage_RegWriteE(Clk,,WriteDataE,WriteDataM);

    flopr #(5) execute_stage_RegWriteE(Clk,,WriteRegE,WriteRegM);







    /***********************************************EXECUTE LOGIC ABOVE***************************************************/


    /***********************************************MEM LOGIC BELOW***************************************************/

    flopr #(32) Mem_stage_RegWriteE(Clk,,ALUOutM,ALUOutW);
    flopr #(5) Mem_stage_RegWriteE(Clk,,WriteRegM,WriteRegW);

    flopr #(32) Mem_stage_RegWriteE(Clk,,ReadData,ReadDataW);


    


    /***********************************************MEM LOGIC ABOVE***************************************************/


    /***********************************************WRITE LOGIC BELOW***************************************************/

        mux2 #(32) resultmux(ReadDataW,ALUOutW,MemtoRegW,ResultW);


    /***********************************************WRITE LOGIC ABOVE***************************************************/


endmodule


module mux_2_5b(out, sel, D0, D1); //for controlling A3

    output [4:0] out;
    input sel;
    input [4:0] D0, D1;
    
    assign out=sel? D1:D0;

endmodule

/**
module MIPS (Clk,reset,PC,Instr,memwrite,ALUOut,writedata,ReadData);




endmodule


module top(Clk, Reset, WriteData,DataAdr,MemWrite);



endmodule
**/