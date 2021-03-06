$date
	Wed Jun 15 20:27:08 2022
$end
$version
	Icarus Verilog
$end
$timescale
	1s
$end
$scope module ALU $end
$var wire 3 ! ALUControl [2:0] $end
$var wire 32 " N0 [31:0] $end
$var wire 32 # N1 [31:0] $end
$var wire 32 $ SrcA [31:0] $end
$var wire 32 % SrcB [31:0] $end
$var wire 32 & SrcB_not [31:0] $end
$var wire 32 ' mux1_out [31:0] $end
$var wire 1 ( Zero $end
$var wire 32 ) N3 [31:0] $end
$var wire 32 * N2 [31:0] $end
$var wire 1 + C_out $end
$var wire 32 , ALUResult [31:0] $end
$scope module add1 $end
$var wire 1 - C_in $end
$var wire 32 . SrcB [31:0] $end
$var wire 32 / Y [31:0] $end
$var wire 32 0 SrcA [31:0] $end
$var wire 1 + C_out $end
$upscope $end
$scope module mux1 $end
$var wire 32 1 D0 [31:0] $end
$var wire 32 2 D1 [31:0] $end
$var wire 1 3 sel $end
$var wire 32 4 out [31:0] $end
$upscope $end
$scope module mux2 $end
$var wire 32 5 D0 [31:0] $end
$var wire 32 6 D1 [31:0] $end
$var wire 32 7 D2 [31:0] $end
$var wire 32 8 D3 [31:0] $end
$var wire 2 9 sel [1:0] $end
$var wire 32 : out [31:0] $end
$upscope $end
$upscope $end
$scope module And $end
$var wire 32 ; SrcA [31:0] $end
$var wire 32 < SrcB [31:0] $end
$var wire 32 = Y [31:0] $end
$upscope $end
$scope module Dflipflop $end
$var wire 1 > Clk $end
$var wire 32 ? D [31:0] $end
$var wire 1 @ reset $end
$var reg 32 A Q [31:0] $end
$upscope $end
$scope module Instruction_memory $end
$var wire 6 B A [5:0] $end
$var wire 32 C RD [31:0] $end
$upscope $end
$scope module Or $end
$var wire 32 D SrcA [31:0] $end
$var wire 32 E SrcB [31:0] $end
$var wire 32 F Y [31:0] $end
$upscope $end
$scope module SignExtender $end
$var wire 16 G extend [15:0] $end
$var wire 32 H extended [31:0] $end
$upscope $end
$scope module controller $end
$var wire 6 I Funct [5:0] $end
$var wire 6 J Opcode [5:0] $end
$var wire 1 K PCSrc $end
$var wire 1 L Zero $end
$var wire 1 M RegWrite $end
$var wire 1 N RegDst $end
$var wire 1 O MemtoReg $end
$var wire 1 P MemWrite $end
$var wire 1 Q Jump $end
$var wire 1 R Branch $end
$var wire 1 S ALUSrc $end
$var wire 2 T ALUOp [1:0] $end
$var wire 3 U ALUControl [2:0] $end
$scope module aludec1 $end
$var wire 6 V Funct [5:0] $end
$var wire 2 W AluOP [1:0] $end
$var reg 3 X ALUControl [2:0] $end
$upscope $end
$scope module md1 $end
$var wire 6 Y Opcode [5:0] $end
$var wire 1 M RegWrite $end
$var wire 1 N RegDst $end
$var wire 1 O MemtoReg $end
$var wire 1 P MemWrite $end
$var wire 1 Q Jump $end
$var wire 1 R Branch $end
$var wire 1 S ALUSrc $end
$var wire 2 Z ALUOp [1:0] $end
$var reg 9 [ controls [8:0] $end
$upscope $end
$upscope $end
$scope module data_memory $end
$var wire 32 \ A [31:0] $end
$var wire 1 ] Clk $end
$var wire 32 ^ WD [31:0] $end
$var wire 1 _ WE $end
$var reg 32 ` RD [31:0] $end
$upscope $end
$scope module flopr $end
$var wire 1 a Clk $end
$var wire 1 b Reset $end
$var wire 8 c d [7:0] $end
$var reg 8 d q [7:0] $end
$upscope $end
$scope module hazard_unit_tb $end
$var wire 1 e StallF $end
$var wire 1 f StallD $end
$var wire 2 g ForwardBE [1:0] $end
$var wire 1 h ForwardBD $end
$var wire 2 i ForwardAE [1:0] $end
$var wire 1 j ForwardAD $end
$var wire 1 k FlushE $end
$var reg 1 l BranchD $end
$var reg 1 m MemtoRegE $end
$var reg 1 n MemtoRegM $end
$var reg 1 o RegWriteE $end
$var reg 1 p RegWriteM $end
$var reg 1 q RegWriteW $end
$var reg 5 r RsD [4:0] $end
$var reg 5 s RsE [4:0] $end
$var reg 5 t RtD [4:0] $end
$var reg 5 u RtE [4:0] $end
$var reg 5 v WriteRegE [4:0] $end
$var reg 5 w WriteRegM [4:0] $end
$var reg 1 x WriteRegW $end
$scope module dut $end
$var wire 1 l BranchD $end
$var wire 1 m MemtoRegE $end
$var wire 1 n MemtoRegM $end
$var wire 1 o RegWriteE $end
$var wire 1 p RegWriteM $end
$var wire 1 q RegWriteW $end
$var wire 5 y RsD [4:0] $end
$var wire 5 z RsE [4:0] $end
$var wire 5 { RtD [4:0] $end
$var wire 5 | RtE [4:0] $end
$var wire 5 } WriteRegE [4:0] $end
$var wire 5 ~ WriteRegM [4:0] $end
$var wire 1 x WriteRegW $end
$var wire 1 !" branchstall $end
$var wire 1 "" lwstall $end
$var reg 1 k FlushE $end
$var reg 1 j ForwardAD $end
$var reg 2 #" ForwardAE [1:0] $end
$var reg 1 h ForwardBD $end
$var reg 2 $" ForwardBE [1:0] $end
$var reg 1 f StallD $end
$var reg 1 e StallF $end
$upscope $end
$upscope $end
$scope module mux2 $end
$var wire 8 %" d0 [7:0] $end
$var wire 8 &" d1 [7:0] $end
$var wire 1 '" s $end
$var wire 8 (" y [7:0] $end
$upscope $end
$scope module mux_2_5b $end
$var wire 5 )" D0 [4:0] $end
$var wire 5 *" D1 [4:0] $end
$var wire 1 +" sel $end
$var wire 5 ," out [4:0] $end
$upscope $end
$scope module registerFile $end
$var wire 5 -" A1 [4:0] $end
$var wire 5 ." A2 [4:0] $end
$var wire 5 /" A3 [4:0] $end
$var wire 1 0" Clk $end
$var wire 32 1" WD3 [31:0] $end
$var wire 1 2" WE3 $end
$var wire 32 3" RD2 [31:0] $end
$var wire 32 4" RD1 [31:0] $end
$upscope $end
$scope module shift_left_2 $end
$var wire 32 5" shift_in [31:0] $end
$var wire 32 6" shifted_out [31:0] $end
$upscope $end
$enddefinitions $end
#0
$dumpvars
bz00 6"
bz 5"
bx 4"
bx 3"
z2"
bz 1"
z0"
bz /"
bz ."
bz -"
bz ,"
z+"
bz *"
bz )"
bz ("
z'"
bz &"
bz %"
bx $"
bx #"
x""
x!"
bx ~
bx }
bx |
bx {
bx z
bx y
xx
bx w
bx v
bx u
bx t
bx s
bx r
xq
xp
xo
xn
xm
xl
xk
xj
bx i
xh
bx g
xf
xe
bx d
bz c
zb
za
bx `
z_
bz ^
z]
bz \
bx [
bx Z
bz Y
bx X
bx W
bz V
bx U
bx T
xS
xR
xQ
xP
xO
xN
xM
zL
xK
bz J
bz I
bz H
bz G
bx F
bz E
bz D
bx C
bz B
bx A
z@
bz ?
z>
bx =
bz <
bz ;
bx :
bz 9
b0x 8
bx 7
bx 6
bx 5
bz 4
z3
bz 2
bz 1
bz 0
bx /
bz .
z-
bx ,
x+
bx *
b0x )
x(
bz '
bz &
bz %
bz $
bx #
bx "
bz !
$end
