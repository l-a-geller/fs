/*
 * schoolRISCV - small RISC-V CPU 
 *
 * originally based on Sarah L. Harris MIPS CPU 
 *                   & schoolMIPS project
 * 
 * Copyright(c) 2017-2020 Stanislav Zhelnio 
 *                        Aleksandr Romanov 
 */ 

`include "sr_cpu.vh"

module sr_cpu
(
    input           clk,        // clock
    input           rst_n,      // reset
    input   [ 4:0]  regAddr,    // debug access reg address
    output  [31:0]  regData,    // debug access reg data
    output  [31:0]  imAddr,     // instruction memory address
    input   [31:0]  imData      // instruction memory data
);
    //control wires
    wire        aluZero;
    wire        aluReady;
    wire        pcEnable;
    wire        pcSelect;
    wire        irEnable;
    wire        regWrite;
    wire        aluSrc;
    wire        aluSrcAEnable;
    wire        aluSrcBEnable;
    wire        aluRst;
    wire        aluStart;
    wire        wdSrc;
    wire  [2:0] aluControl;

    //instruction decode wires
    wire [ 6:0] cmdOp;
    wire [ 4:0] rd;
    wire [ 2:0] cmdF3;
    wire [ 4:0] rs1;
    wire [ 4:0] rs2;
    wire [ 6:0] cmdF7;
    wire [31:0] immI;
    wire [31:0] immB;
    wire [31:0] immU;

    //program counter
    wire [31:0] pc;
    wire [31:0] pcBranch = pc + immB;
    wire [31:0] pcPlus4  = pc + 4;
    wire [31:0] pcNext   = pcEnable ? (pcSelect ? pcBranch : pcPlus4) : pc;
    sm_register r_pc(clk ,rst_n, pcNext, pc);

    //program memory access
    assign imAddr = pc >> 2;
    wire [31:0] instr = imData;
    //wire [31:0] instr = 0;

    //reg [31:0] instructionReg;

    //always @ (posedge clk) begin
        //if (PCEnable) programCounter <= nextPC;
        //if (irEnable) instr = imData;
        //if (IR2Enable) i n s t r u c ti o n R e g 2 <= dataOut;
    //end


    //instruction decode
    sr_decode id (
        .instr      ( instr        ),
        .cmdOp      ( cmdOp        ),
        .rd         ( rd           ),
        .cmdF3      ( cmdF3        ),
        .rs1        ( rs1          ),
        .rs2        ( rs2          ),
        .cmdF7      ( cmdF7        ),
        .immI       ( immI         ),
        .immB       ( immB         ),
        .immU       ( immU         ) 
    );

    //register file
    wire [31:0] rd0;
    wire [31:0] rd1;
    wire [31:0] rd2;
    wire [31:0] wd3;

    sm_register_file rf (
        .clk        ( clk          ),
        .a0         ( regAddr      ),
        .a1         ( rs1          ),
        .a2         ( rs2          ),
        .a3         ( rd           ),
        .rd0        ( rd0          ),
        .rd1        ( rd1          ),
        .rd2        ( rd2          ),
        .wd3        ( wd3          ),
        .we3        ( regWrite     )
    );

    assign regData = (regAddr != 0) ? rd0 : pc;

    wire [31:0] srcB = aluSrc ? immI : rd2;
    wire [31:0] aluResult;

    sr_alu alu (
        .clk        ( clk           ),
        .rst        ( aluRst        ),
        .start      ( aluStart      ),
        .srcAEnable ( aluSrcAEnable ),
        .srcBEnable ( aluSrcBEnable ),
        .srcA       ( rd1           ),
        .srcB       ( srcB          ),
        .oper       ( aluControl    ),
        .zero       ( aluZero       ),
        .ready      ( aluReady      ),
        .result     ( aluResult     ) 
    );

    assign wd3 = wdSrc ? immU : aluResult;

    sr_control sm_control (
        .cmdOp      ( cmdOp        ),
        .cmdF3      ( cmdF3        ),
        .cmdF7      ( cmdF7        ),
        .clk        ( clk          ),
        .aluRst     ( aluRst       ),
        .aluStart   ( aluStart     ),
        .aluSrcAEnable ( aluSrcAEnable ),
        .aluSrcBEnable ( aluSrcBEnable ),
        .aluZero    ( aluZero      ),
        .aluReady   ( aluReady     ),
        .pcEnable   ( pcEnable     ),
        .pcSelect   ( pcSelect     ),
        .irEnable   ( irEnable     ),
        .regWrite   ( regWrite     ),
        .aluSrc     ( aluSrc       ),
        .wdSrc      ( wdSrc        ),
        .aluControl ( aluControl   )
    );

endmodule

module sr_decode
(
    input      [31:0] instr,
    output     [ 6:0] cmdOp,
    output     [ 4:0] rd,
    output     [ 2:0] cmdF3,
    output     [ 4:0] rs1,
    output     [ 4:0] rs2,
    output     [ 6:0] cmdF7,
    output reg [31:0] immI,
    output reg [31:0] immB,
    output reg [31:0] immU 
);
    assign cmdOp = instr[ 6: 0];
    assign rd    = instr[11: 7];
    assign cmdF3 = instr[14:12];
    assign rs1   = instr[19:15];
    assign rs2   = instr[24:20];
    assign cmdF7 = instr[31:25];

    // I-immediate
    always @ (*) begin
        immI[10: 0] = instr[30:20];
        immI[31:11] = { 21 {instr[31]} };
    end

    // B-immediate
    always @ (*) begin
        immB[    0] = 1'b0;
        immB[ 4: 1] = instr[11:8];
        immB[10: 5] = instr[30:25];
        immB[   11] = instr[7];
        immB[31:12] = { 20 {instr[31]} };
    end

    // U-immediate
    always @ (*) begin
        immU[11: 0] = 12'b0;
        immU[31:12] = instr[31:12];
    end

endmodule

module sr_control
(
    input      [6:0] cmdOp,
    input      [2:0] cmdF3,
    input      [6:0] cmdF7,
    input            clk,
    output           aluRst,
    output           aluStart,
    input            aluSrcAEnable,
    input            aluSrcBEnable,
    input            aluZero,
    input            aluReady,
    output           pcEnable,
    output           pcSelect,
    output           irEnable,
    output reg       regWrite,
    output reg       aluSrc,
    output reg       wdSrc,
    output reg [2:0] aluControl
);

    reg pcEnableReg;
    reg pcSelectReg;
    reg irEnableReg;
    reg aluRstReg;
    reg aluStartReg;
    reg aluSrcAEnableReg;
    reg aluSrcBEnableReg;
    reg              branch;
    reg              condZero;
    
    assign pcEnable = pcEnableReg;
    assign pcSelect = pcSelectReg;
    assign irEnable = irEnableReg;
    assign aluSrcAEnable = aluSrcAEnableReg;
    assign aluSrcBEnable = aluSrcBEnableReg;
    assign aluRst = aluRstReg;
    assign aluStart = aluStartReg;

  
    parameter loadInstruction = 0;
    parameter decodeInstruction = 1;
    parameter loadAlu = 2;
    parameter compute = 3;
    parameter jump = 4;
    parameter halt = 5;

    reg [3:0] state = loadInstruction;
    reg [3:0] nextstate;

    always @ (posedge clk) state = nextstate;

    always @ (*) begin
        case (state)
            loadInstruction: begin
                aluRstReg = 1;
                aluStartReg = 0;
                pcEnableReg = 1;

                aluSrcAEnableReg = 0;
                aluSrcBEnableReg = 0;
                irEnableReg = 1;

                nextstate = decodeInstruction;
            end

            decodeInstruction: begin

                aluRstReg = 0;
                aluStartReg = 1;
                
                irEnableReg = 0;
                pcEnableReg = 0;

                aluSrcAEnableReg = 0;
                aluSrcBEnableReg = 0;

                branch = 1'b0;
                aluSrc = 1'b0;

                branch      = 1'b0;
                condZero    = 1'b0;
                regWrite    = 1'b0;
              
                wdSrc       = 1'b0;
                aluControl  = `ALU_ADD;

                casez( {cmdF7, cmdF3, cmdOp} )
                    { `RVF7_ADD,  `RVF3_ADD,  `RVOP_ADD  } : begin aluControl = `ALU_ADD; end // begin regWrite = 1'b1; aluControl = `ALU_ADD;  end
                    { `RVF7_OR,   `RVF3_OR,   `RVOP_OR   } : begin aluControl = `ALU_OR;   end
                    //{ `RVF7_SRL,  `RVF3_SRL,  `RVOP_SRL  } : begin aluSrc = 1'b1; aluControl = `ALU_OR;  end
                    //{ `RVF7_SLTU, `RVF3_SLTU, `RVOP_SLTU } : begin regWrite = 1'b1; aluControl = `ALU_SLTU; end
                    //{ `RVF7_SUB,  `RVF3_SUB,  `RVOP_SUB  } : begin regWrite = 1'b1; aluControl = `ALU_SUB;  end

                    { `RVF7_ANY,  `RVF3_ADDI, `RVOP_ADDI } : begin aluSrc = 1'b1; aluControl = `ALU_ADD; end // begin regWrite = 1'b1; aluSrc = 1'b1; aluControl = `ALU_ADD; end
                    { `RVF7_ANY,  `RVF3_ANY,  `RVOP_LUI  } : begin wdSrc  = 1'b1; end

                    { `RVF7_ANY,  `RVF3_BEQ,  `RVOP_BEQ  } : begin  branch = 1'b1; condZero = 1'b1; aluControl = `ALU_SUB; end // begin branch = 1'b1; condZero = 1'b1; aluControl = `ALU_SUB; end
                    //{ `RVF7_ANY,  `RVF3_BNE,  `RVOP_BNE  } : begin branch = 1'b1; aluControl = `ALU_SUB; end

                    { `RVF7_ANY,  `RVF3_BLTU, `RVOP_BLTU } : begin branch = 1'b1; condZero = 0'b0; aluControl = `ALU_SLTU; end
                    
                    { `RVF7_ANY,  `RVF3_ARI, `RVOP_ARI } : begin aluControl = `ALU_ARI; end
                endcase

                nextstate = loadAlu;

            end

            loadAlu: begin // Load registers to alu
                pcEnableReg = 0;
                pcSelectReg = 0;
                irEnableReg = 0;
                aluSrcAEnableReg = 1;
                aluSrcBEnableReg = 1;
                nextstate = compute;
            end

            compute: begin // Compute opeartion

                pcEnableReg = 0;
                pcSelectReg = 0;
                irEnableReg = 0;

                aluSrcAEnableReg = 0;
                aluSrcBEnableReg = 0;

                regWrite = 1'b1;
                nextstate = aluReady ? (branch ? jump : loadInstruction) : compute;
            end

            jump: begin // jump

                pcSelectReg = branch & (aluZero == condZero);

                irEnableReg = 0;
                pcEnableReg = 0;
                nextstate = loadInstruction;
            end

        endcase
    end
endmodule

module sr_alu
(
    input         clk,
    input         rst,
    input         start,
    input         srcAEnable,
    input         srcBEnable,
    input  [31:0] srcA,
    input  [31:0] srcB,
    input  [ 2:0] oper,
    output        zero,
    output        ready,
    output reg [31:0] result
);

    reg [31:0] a;
    reg [31:0] b;

    reg [31:0] root_resReg;
    wire [31:0] root_res;
    wire busy;

    main main(
        .rst_i(rst),
        .clk_i(clk),
        .a_in(srcA[7:0]),
        .b_in(srcB[7:0]),
        .start_i(start),
        .busy_o(busy),
        .y_bo(root_res[15:0])
    );

    always @ (*) begin
        if (srcAEnable) a = srcA;
        if (srcBEnable) b = srcB;
        case (oper)
            default   : result = a + b;
            `ALU_ADD  : result = a + b;
            `ALU_OR   : result = {16'h0000, root_res[15:0]};
            //`ALU_SRL  : result = srcA >> srcB [4:0];
            `ALU_SLTU : result = (srcA < srcB) ? 1 : 0;
            `ALU_SUB  : result = a - b;
            //`ALU_ARI  : result = root_res;
        endcase
    end

    assign ready  = (oper == `ALU_OR) ? ~busy : 1;
    assign zero   = (result == 0);
endmodule

module sm_register_file
(
    input         clk,
    input  [ 4:0] a0,
    input  [ 4:0] a1,
    input  [ 4:0] a2,
    input  [ 4:0] a3,
    output [31:0] rd0,
    output [31:0] rd1,
    output [31:0] rd2,
    input  [31:0] wd3,
    input         we3
);
    reg [31:0] rf [31:0];

    assign rd0 = (a0 != 0) ? rf [a0] : 32'b0;
    assign rd1 = (a1 != 0) ? rf [a1] : 32'b0;
    assign rd2 = (a2 != 0) ? rf [a2] : 32'b0;

    always @ (posedge clk)
        if(we3) rf [a3] <= wd3;
endmodule
