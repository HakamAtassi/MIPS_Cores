`include "MIPS_Single_Cycle.v"



module hazard_unit_tb;

    reg BranchD,MemtoRegE,RegWriteE,RegWriteM,RegWriteW, MemtoRegM;
    reg [4:0] RsD,RtD,RsE,RtE;
    reg [4:0] WriteRegE,WriteRegM,WriteRegW;

    wire StallF,StallD,ForwardAD,ForwardBD,FlushE;
    wire [1:0] ForwardAE,ForwardBE;





    initial begin 
        $display("Starting hazard tb test");
        $dumpfile("hazard_tb.vcd");
        $dumpvars;
    end

        hazard_unit dut(BranchD,RsD,RtD,RsE,RtE,WriteRegE,WriteRegM,MemtoRegE,RegWriteE,MemtoRegM,
        RegWriteM,WriteRegW,RegWriteW,StallF,StallD,ForwardAD,ForwardBD,FlushE,ForwardAE,ForwardBE);


        //Crude testing to ensure the module generally works. 


    /*      PASS
    initial begin
        RsE<=5'b00110;
        WriteRegM<=5'b00110;
        RegWriteM<=1'b1;
        #10;$stop;  
    end
    */

/*          PASS
    initial begin
        RsE<=5'b00110;
        WriteRegW<=5'b00110;
        RegWriteW<=1'b1;
        #10;$stop;  
    end
*/
/*          PASS
    initial begin
        RsE<=5'b00110;
        WriteRegW<=5'b00111;
        RegWriteW<=1'b1;
        #10;$stop;  
    end
*/


//TESTS for SCRB

/*      PASS
    initial begin
        RtE<=5'b00110;
        WriteRegM<=5'b00110;
        RegWriteM<=1'b1;
        #10;$stop;  
    end
*/

/*      PASS
    initial begin
        RtE<=5'b00110;
        WriteRegW<=5'b00110;
        RegWriteW<=1'b1;
        #10;$stop;  
    end
*/

/*      PASS
    initial begin
        RtE<=5'b00110;
        WriteRegW<=5'b00111;
        RegWriteW<=1'b0;
        #10;$stop;  
    end
*/



/***
****
***/
//Check for stalls 
//if LW is performed on an address used in the next instrection (as a source), stall.
//ie. RtE= RsD or RtE=RtD (sources, not destinations) & memtoreg

/*      PASS
    initial begin
        RtE<=5'b00110;
        RsD<=5'b00110;
        MemtoRegE<=1'b1;
        #10;$stop;  
    end
*/

/*      PASS
    initial begin
        RtD<=5'b00110;
        RtE<=5'b00110;
        MemtoRegE<=1'b1;
        #10;$stop;  
    end
*/

/*      PASS    
    initial begin
        RtE<=5'b00110;
        RsD<=5'b00110;
        MemtoRegE<=1'b0;
        #10;$stop;  
    end
*/



/***
****
***/
//ForwardAD/AB tests

/*      PASS
    initial begin
        RsD<=5'b00101;
        WriteRegM<=5'b00101;
        RegWriteM<=1'b1;
        #10;$stop;  
    end
*/

/*      PASS
    initial begin
        RtD<=5'b00101;
        WriteRegM<=5'b00101;
        RegWriteM<=1'b1;
        #10;$stop;  
    end
*/

/***
****
***/
//Stalls with branches


    initial begin
        BranchD<=1'b1;
        RegWriteE<=1'b1;
        WriteRegE<=5'b00101;
        RsD<=5'b00101;
        //branchstall done
        RtE<=5'b00110;
        RsD<=5'b00110;
        MemtoRegE<=1'b1;

        #10;$stop;  
    end



endmodule 
