/*===============================================
Author: (add_your_name_here), (add_your_name_here), Eric Tien 
Module: CHIP, reg_file, mulDiv
Description: 
    Single cycle RISC-V processor for simplify instructions.
    Use the submodule mulDiv to simulate mul/div calculations.
===============================================*/

module CHIP(
    clk, rst_n,
    mem_wen_D, mem_addr_D, mem_wdata_D, mem_rdata_D,
    mem_addr_I,mem_rdata_I
);

    input         clk, rst_n ;
    // For mem_D
    output reg    mem_wen_D  ;
    output [31:0] mem_addr_D ;
    output [31:0] mem_wdata_D;
    input  [31:0] mem_rdata_D;
    // For mem_I
    output [31:0] mem_addr_I ;
    input  [31:0] mem_rdata_I;
    
    //---------------------------------------//
    // Do not modify this part!!!            //
    // Exception: You may change wire to reg //
    reg [31:0] PC;
    reg [31:0] PC_nxt;
    reg regWrite;
    reg [4:0] rs1, rs2, rd;
    wire [31:0] rs1_data;
    wire [31:0] rs2_data;
    reg [31:0] rd_data;
    //---------------------------------------//

    // Todo: other wire/reg
    // states
    parameter RUN = 1'd0;
    parameter STALL = 1'd1;

    parameter R_type = 3'd0;
    parameter I_type = 3'd1;
    parameter S_type = 3'd2;
    parameter B_type = 3'd3;
    parameter U_type = 3'd4;
    parameter J_type = 3'd5;

    // control
    reg [2:0] type;
    reg jalr, jal, branch, mem_to_reg, alu_src;
    reg [6:0] opcode;
    reg [2:0] funct3;
    reg [6:0] funct7;

    // alu
    reg [3:0] alu_ctrl;  // TODO: how many bits?
    reg [31:0] alu_out, imm_gen_out;

    // flip-flops
    reg state, state_nxt;

    //---------------------------------------//
    // Do not modify this part!!!            //
    reg_file reg0(                           //
        .clk(clk),                           //
        .rst_n(rst_n),                       //
        .wen(regWrite),                      //
        .a1(rs1),                            //
        .a2(rs2),                            //
        .aw(rd),                             //
        .d(rd_data),                         //
        .q1(rs1_data),                       //
        .q2(rs2_data));                      //
    //---------------------------------------//
    
    // Todo: any combinational/sequential circuit
    //===== Combinational =======================
    // Finite State Machine
    always @(*) begin
        case (state)
        // TODO: detect MUL and handle FSM
            RUN: begin
                state_nxt = state;
            end
            STALL: begin
                state_nxt = state;
            end
            default: begin
                state_nxt = state;
            end
        endcase
    end

    // detect instruction type by opcode
    always @(*) begin
        type = R_type;
    end


    // handle I/O and control signals
    assign mem_addr_I = PC;
    assign mem_addr_D = alu_out;
    assign mem_wdata_D = rs2_data;
    always @(*) begin
        opcode = mem_rdata_I[6:0];
        rd = mem_rdata_I[11:7];
        rs1 = mem_rdata_I[19:15];
        rs2 = mem_rdata_I[24:20];
        func3 = mem_rdata_I[14:12];
        func7 = mem_rdata_I[31:25];

        jalr = 0;
        jal = 0;
        branch = 0;
        mem_to_reg = 0;
        alu_src = 0;
        if (state == RUN) begin
            case (type)
                : 
                default: 
            endcase
        end else begin
            jalr = 0;
            jal = 0;
            branch = 0;
            mem_to_reg = 0;
            alu_src = 0;
        end
    end

    // immediate
    always @(*) begin
        imm_gen_out = 
    end

    // alu
    always @(*) begin
        alu_out = 
    end


    // handle PC
    always @(*) begin
        PC_nxt = PC;
    end

    //===== Sequential ==========================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            PC <= 32'h00010000; // Do not modify this value!!!
            state <= RUN;
        end else begin
            PC <= PC_nxt;
            state <= state_nxt;
        end
    end
endmodule

module reg_file(clk, rst_n, wen, a1, a2, aw, d, q1, q2);
   
    parameter BITS = 32;
    parameter word_depth = 32;
    parameter addr_width = 5; // 2^addr_width >= word_depth
    
    input clk, rst_n, wen; // wen: 0:read | 1:write
    input [BITS-1:0] d;
    input [addr_width-1:0] a1, a2, aw;

    output [BITS-1:0] q1, q2;

    reg [BITS-1:0] mem [0:word_depth-1];
    reg [BITS-1:0] mem_nxt [0:word_depth-1];

    integer i;

    assign q1 = mem[a1];
    assign q2 = mem[a2];

    always @(*) begin
        for (i=0; i<word_depth; i=i+1)
            mem_nxt[i] = (wen && (aw == i)) ? d : mem[i];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mem[0] <= 0;
            for (i=1; i<word_depth; i=i+1) begin
                case(i)
                    32'd2: mem[i] <= 32'hbffffff0;
                    32'd3: mem[i] <= 32'h10008000;
                    default: mem[i] <= 32'h0;
                endcase
            end
        end
        else begin
            mem[0] <= 0;
            for (i=1; i<word_depth; i=i+1)
                mem[i] <= mem_nxt[i];
        end       
    end
endmodule

module mulDiv(clk, rst_n, valid, ready, mode, in_A, in_B, out);

    input         clk, rst_n;
    input         valid, mode; // mode: 0: mulu, 1: divu
    output        ready;
    input  [31:0] in_A, in_B;
    output [63:0] out;

    // Definition of states
    parameter IDLE = 2'b00;
    parameter MUL  = 2'b01;
    parameter DIV  = 2'b10;
    parameter OUT  = 2'b11;

    // flip-flops
    reg  [ 1:0] state, state_nxt;
    reg  [ 4:0] counter, counter_nxt;
    reg  [63:0] shreg, shreg_nxt;
    reg  [31:0] alu_in, alu_in_nxt;

    reg signed [32:0] alu_out;

    //===== Combinational =======================
    assign ready = (state==OUT);
    assign out = shreg;

    // next state logic
    always @(*) begin
        case(state)
            IDLE: begin
                state_nxt = (valid==0)? IDLE : (mode==1)? DIV : MUL;
            end
            MUL: begin
                state_nxt = (counter==5'd31)? OUT : MUL;
            end
            DIV: begin
                state_nxt = (counter==5'd31)? OUT : DIV;
            end
            OUT: begin
                state_nxt = IDLE;
            end
            default: begin
                state_nxt = IDLE;
            end
        endcase

        counter_nxt = (state==MUL | state==DIV)? counter+1 : 5'd0;
    end
    
    // ALU input
    always @(*) begin
        case(state)
            IDLE: begin
                alu_in_nxt = valid? in_B : 0;
            end
            OUT: begin
                alu_in_nxt = 0;
            end
            default: begin
                alu_in_nxt = alu_in;
            end
        endcase
    end

    // ALU output
    always @(*) begin
        case (state)
            MUL: begin
                alu_out = shreg[63:32] + alu_in;
            end
            DIV: begin
                alu_out = shreg[63:32] - alu_in;
            end
            default: begin
                alu_out = 33'd0;
            end
        endcase
    end

    // handle shift registers
    always @(*) begin
        case (state)
            IDLE: begin
                shreg_nxt = valid? mode? {31'd0, in_A, 1'b0} : {32'd0, in_A} : 64'd0;
            end
            MUL: begin
                if (shreg[0]) begin
                    shreg_nxt = {alu_out, shreg[31:1]};
                end else begin
                    shreg_nxt = shreg >> 1;
                end
            end
            DIV: begin
                shreg_nxt = counter<31?
                                alu_out<0? {shreg[62:0], 1'b0} : {alu_out[30:0], shreg[31:0], 1'b1}
                            :
                                alu_out<0? {shreg[63:32], shreg[30:0], 1'b0} : {alu_out[31:0], shreg[30:0], 1'b1};
            end
            default: begin
                shreg_nxt = shreg;
            end
        endcase
    end

    //===== Sequential ==========================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            counter <= 5'd0;
            shreg <= 64'd0;
            alu_in <= 32'd0;
        end else begin
            state <= state_nxt;
            counter <= counter_nxt;
            shreg <= shreg_nxt;
            alu_in <= alu_in_nxt;
        end
    end
endmodule