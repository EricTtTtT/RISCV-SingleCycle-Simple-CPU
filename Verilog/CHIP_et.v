/*======================
Author: Eric Tien
Module: CHIP
Description: 
    RISC-V processor supply for simplify instructions.
======================*/

module CHIP(
    input         clk, rst_n ,
    // for mem_D
    output       mem_wen_D  ,  // mem_wen_D is high, CHIP writes data to D-mem; else, CHIP reads data from D-mem
    output [31:0] mem_addr_D ,  // the specific address to fetch/store data 
    output [31:0] mem_wdata_D,  // data writing to D-mem 
    input  [31:0] mem_rdata_D,  // data reading from D-mem
    // for mem_I
    output [31:0] mem_addr_I ,  // the fetching address of next instruction
    input  [31:0] mem_rdata_I  // instruction reading from I-mem
);
    // control
    wire jalr, jal, branch, mem2reg, mem_write, alu_src, reg_write;
    wire [31:0] inst;
    wire [31:0] m2r_mux_out; // pc

    // registers file
    reg [31:0] registers [0:31];
    wire [31:0] read_data_2, write_data;
    wire signed [31:0] read_data_1;

    // alu
    wire [3:0] alu_ctrl;
    wire signed [31:0] alu_input_2;

    // pc
    reg [31:0] pc, alu_out;
    wire [31:0] pc_next, pc_a4;
    wire signed [31:0] imm_gen_out;

    
//============Combinational====================
    
    // always @(*) begin
    assign inst = {mem_rdata_I[7:0], mem_rdata_I[15:8], mem_rdata_I[23:16], mem_rdata_I[31:24]};

        // processor Output
    assign mem_wen_D = mem_write;
    assign mem_addr_D = alu_out;
    assign mem_wdata_D = {read_data_2[7:0], read_data_2[15:8], read_data_2[23:16], read_data_2[31:24]};
    assign mem_addr_I = pc;
        
        // control
    assign jal  = inst[2] & inst[3];
    assign jalr = inst[2] & (!inst[3]);
    assign branch = inst[6] & (!inst[2]);
    assign mem2reg = !inst[5];
    assign mem_write = (!inst[6]) & inst[5] & (!inst[4]);
    assign alu_src = inst[4] | (inst[6] & (!inst[2]));
    assign reg_write = inst[4] | inst[2] | (!inst[5]);

    // IO
    assign m2r_mux_out = mem2reg? {mem_rdata_D[7:0], mem_rdata_D[15:8], mem_rdata_D[23:16], mem_rdata_D[31:24]} : alu_out;
    
    // imm generator
    assign imm_gen_out[12:0] = inst[5]?
                                inst[3]?
                                    {inst[12], inst[20], inst[30:21], 1'b0}
                                : inst[6]?
                                    inst[2]?
                                        {inst[31], inst[31:20]}
                                    : {inst[31], inst[7], inst[30:25], inst[11:8], 1'b0}
                                : {inst[31], inst[31:25], inst[11:7]}
                            : {inst[31], inst[31:20]};
    assign imm_gen_out[20:13] = inst[3]? {inst[31], inst[19:13]} : {8{imm_gen_out[12]}};
    assign imm_gen_out[31:21] = {11{imm_gen_out[20]}};

    // alu_ctrl
    assign alu_ctrl[0] = inst[4] & inst[14] & inst[13] & (!inst[12]);
    assign alu_ctrl[1] = !(inst[4] & (!inst[3]) & inst[13]);
    assign alu_ctrl[2] = ( (!inst[4]) & (!inst[13]) ) | ( inst[30] & inst[4] );
    assign alu_ctrl[3] = inst[4] & (!inst[14]) & inst[13];

    // registers file
    assign write_data = (jal | jalr)? pc_a4 : m2r_mux_out;
    assign read_data_1 = registers[inst[19:15]];
    assign read_data_2 = registers[inst[24:20]];

    // alu
    assign alu_input_2 = alu_src? read_data_2 : imm_gen_out;  
    always @(*) begin
        case (alu_ctrl)
            4'b0000: alu_out = read_data_1 & alu_input_2;
            4'b0001: alu_out = read_data_1 | alu_input_2;
            4'b0010: alu_out = read_data_1 + alu_input_2;
            4'b0110: alu_out = read_data_1 - alu_input_2;
            4'b1000: alu_out = (read_data_1 < alu_input_2)? 1 : 0;
            default: alu_out = 0;
        endcase
    end 

    // PC
    assign pc_a4 = pc + 4;
    assign pc_next = jalr? (imm_gen_out + (read_data_1))
                        : ( (branch & (alu_out == 0)) | jal )? (imm_gen_out + pc) : pc_a4;

    // end

    //============Sequential====================
    integer i;
    always @(posedge clk) begin
        if (rst_n == 0) begin
            pc <= 0;
            for (i=1; i<32; i=i+1) begin
                registers[i] <= 0;
            end
        end else begin
            pc <= pc_next;
            if (reg_write) begin
                registers[inst[11:7]] <= write_data;
            end
        end
        registers[0] <= 0;
    end
endmodule