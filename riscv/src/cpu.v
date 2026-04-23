`include "defines.vh"

module cpu(
    input  wire                 clk_in,
    input  wire                 rst_in,
    input  wire                 rdy_in,
    input  wire [ 7:0]          mem_din,
    output wire [ 7:0]          mem_dout,
    output wire [31:0]          mem_a,
    output wire                 mem_wr,
    input  wire                 io_buffer_full,
    output wire [31:0]          dbgreg_dout
);

// --- Configuration ---
localparam ROB_SIZE = 32;
localparam ROB_WIDTH = 5;
localparam RS_SIZE = 16;
localparam RS_WIDTH = 4;
localparam LSB_SIZE = 16;
localparam LSB_WIDTH = 4;

// --- Common Data Bus (CDB) ---
reg        cdb_en;
reg [31:0] cdb_val;
reg [ROB_WIDTH-1:0] cdb_rob;
reg [31:0] cdb_pc;
reg        cdb_jump; 
reg        cdb_taken;

// --- Register File ---
reg [31:0] rf_data [31:0];
reg [ROB_WIDTH-1:0] rf_rob [31:0];
reg        rf_busy [31:0];

// --- ROB ---
reg [31:0] rob_val [ROB_SIZE-1:0];
reg [ 4:0] rob_dest [ROB_SIZE-1:0]; 
reg [31:0] rob_pc [ROB_SIZE-1:0];
reg [31:0] rob_jump_pc [ROB_SIZE-1:0];
reg        rob_ready [ROB_SIZE-1:0];
reg        rob_busy [ROB_SIZE-1:0];
reg        rob_is_jump [ROB_SIZE-1:0];
reg        rob_is_store [ROB_SIZE-1:0];
reg        rob_jump_taken [ROB_SIZE-1:0];
reg        rob_predict_taken [ROB_SIZE-1:0];
reg [ROB_WIDTH-1:0] rob_head, rob_tail;

wire rob_full = (rob_head == (rob_tail + 1'b1) && rob_busy[rob_head]);

// --- Reservation Station (RS) ---
reg        rs_busy [RS_SIZE-1:0];
reg [ 6:0] rs_opcode [RS_SIZE-1:0];
reg [ 2:0] rs_funct3 [RS_SIZE-1:0];
reg        rs_funct7 [RS_SIZE-1:0];
reg [31:0] rs_v1 [RS_SIZE-1:0], rs_v2 [RS_SIZE-1:0];
reg [ROB_WIDTH-1:0] rs_q1 [RS_SIZE-1:0], rs_q2 [RS_SIZE-1:0];
reg        rs_has_q1 [RS_SIZE-1:0], rs_has_q2 [RS_SIZE-1:0];
reg [ROB_WIDTH-1:0] rs_rob [RS_SIZE-1:0];
reg [31:0] rs_imm [RS_SIZE-1:0];
reg [31:0] rs_pc [RS_SIZE-1:0];
reg        rs_predict_taken [RS_SIZE-1:0];

// --- Load Store Buffer (LSB) ---
reg        lsb_busy [LSB_SIZE-1:0];
reg [ 6:0] lsb_opcode [LSB_SIZE-1:0];
reg [ 2:0] lsb_funct3 [LSB_SIZE-1:0];
reg [31:0] lsb_v1 [LSB_SIZE-1:0], lsb_v2 [LSB_SIZE-1:0];
reg [ROB_WIDTH-1:0] lsb_q1 [LSB_SIZE-1:0], lsb_q2 [LSB_SIZE-1:0];
reg        lsb_has_q1 [LSB_SIZE-1:0], lsb_has_q2 [LSB_SIZE-1:0];
reg [ROB_WIDTH-1:0] lsb_rob [LSB_SIZE-1:0];
reg [31:0] lsb_imm [LSB_SIZE-1:0];
reg [LSB_WIDTH-1:0] lsb_head, lsb_tail;

// --- Memory Controller (MC) ---
reg [31:0] mc_addr;
reg [ 7:0] mc_dout;
reg        mc_wr;
reg [ 2:0] mc_state;
localparam MC_IDLE = 3'd0, MC_FETCH = 3'd1, MC_LOAD = 3'd2, MC_STORE = 3'd3;
reg [31:0] mc_buf;
reg [ 2:0] mc_byte_cnt;
reg [ROB_WIDTH-1:0] mc_rob;
reg [ 2:0] mc_funct3;

assign mem_a = mc_addr;
assign mem_dout = mc_dout;
assign mem_wr = mc_wr;

// --- IFU & BHT ---
reg [31:0] pc;
reg [31:0] if_inst;
reg        if_valid;
reg [ 1:0] bht [255:0];

assign dbgreg_dout = rf_data[10];

// --- Decoder Helpers ---
wire [6:0] op = if_inst[6:0];
wire [4:0] rd = if_inst[11:7];
wire [2:0] f3 = if_inst[14:12];
wire [4:0] rs1 = if_inst[19:15];
wire [4:0] rs2 = if_inst[24:20];
wire [6:0] f7 = if_inst[31:25];

reg [31:0] imm_val;
always @(*) begin
    case (op)
        `OP_LUI, `OP_AUIPC: imm_val = {if_inst[31:12], 12'b0};
        `OP_JAL: imm_val = {{12{if_inst[31]}}, if_inst[19:12], if_inst[20], if_inst[30:21], 1'b0};
        `OP_JALR, `OP_LOAD, `OP_IMM: imm_val = {{20{if_inst[31]}}, if_inst[31:20]};
        `OP_BRANCH: imm_val = {{20{if_inst[31]}}, if_inst[31], if_inst[7], if_inst[30:25], if_inst[11:8], 1'b0};
        `OP_STORE: imm_val = {{20{if_inst[31]}}, if_inst[31:25], if_inst[11:7]};
        default: imm_val = 0;
    endcase
end

// --- Find Free Slots ---
integer rs_free_idx, lsb_free_idx_v, j;
always @(*) begin
    rs_free_idx = -1;
    for (j = 0; j < RS_SIZE; j = j + 1) if (!rs_busy[j] && rs_free_idx == -1) rs_free_idx = j;
    lsb_free_idx_v = -1;
    if (!((lsb_tail + 1'b1 == lsb_head) || (lsb_tail == LSB_SIZE-1 && lsb_head == 0))) lsb_free_idx_v = lsb_tail;
end

integer i;
always @(posedge clk_in) begin
    if (rst_in) begin
        pc <= 0; rob_head <= 0; rob_tail <= 0; lsb_head <= 0; lsb_tail <= 0;
        mc_state <= MC_IDLE; mc_wr <= 0; if_valid <= 0; cdb_en <= 0;
        for (i = 0; i < 32; i = i + 1) begin rf_data[i] <= 0; rf_busy[i] <= 0; end
        for (i = 0; i < ROB_SIZE; i = i + 1) begin rob_busy[i] <= 0; rob_ready[i] <= 0; end
        for (i = 0; i < RS_SIZE; i = i + 1) begin rs_busy[i] <= 0; end
        for (i = 0; i < LSB_SIZE; i = i + 1) begin lsb_busy[i] <= 0; end
        for (i = 0; i < 256; i = i + 1) begin bht[i] <= 2'b01; end
    end else if (rdy_in) begin
        cdb_en <= 0;

        // --- 1. COMMIT ---
        if (rob_busy[rob_head] && rob_ready[rob_head]) begin
            if (rob_is_store[rob_head]) begin
                if (mc_state == MC_IDLE && lsb_busy[lsb_head] && lsb_rob[lsb_head] == rob_head) begin
                    mc_state <= MC_STORE; mc_addr <= lsb_v1[lsb_head] + lsb_imm[lsb_head];
                    mc_buf <= lsb_v2[lsb_head]; mc_funct3 <= lsb_funct3[lsb_head];
                    mc_byte_cnt <= 0; mc_rob <= rob_head;
                end
            end else if (rob_is_jump[rob_head]) begin
                if (rob_jump_taken[rob_head] != rob_predict_taken[rob_head]) begin
                    pc <= rob_jump_taken[rob_head] ? rob_jump_pc[rob_head] : (rob_pc[rob_head] + 4);
                    rob_head <= 0; rob_tail <= 0; if_valid <= 0; lsb_head <= 0; lsb_tail <= 0; mc_state <= MC_IDLE; mc_wr <= 0;
                    for (i = 0; i < ROB_SIZE; i = i + 1) rob_busy[i] <= 0;
                    for (i = 0; i < RS_SIZE; i = i + 1) rs_busy[i] <= 0;
                    for (i = 0; i < LSB_SIZE; i = i + 1) lsb_busy[i] <= 0;
                    for (i = 0; i < 32; i = i + 1) rf_busy[i] <= 0;
                end else begin
                    rob_busy[rob_head] <= 0; rob_head <= rob_head + 1'b1;
                end
                i = rob_pc[rob_head][9:2];
                if (rob_jump_taken[rob_head]) bht[i] <= (bht[i] == 2'b11) ? 2'b11 : bht[i] + 1'b1;
                else bht[i] <= (bht[i] == 2'b00) ? 2'b00 : bht[i] - 1'b1;
            end else begin
                if (rob_dest[rob_head] != 0) begin
                    rf_data[rob_dest[rob_head]] <= rob_val[rob_head];
                    if (rf_busy[rob_dest[rob_head]] && rf_rob[rob_dest[rob_head]] == rob_head) rf_busy[rob_dest[rob_head]] <= 0;
                end
                rob_busy[rob_head] <= 0; rob_head <= rob_head + 1'b1;
            end
        end

        // --- 2. FETCH ---
        if (!if_valid && mc_state == MC_IDLE && !rob_full) begin
            mc_state <= MC_FETCH; mc_addr <= pc; mc_byte_cnt <= 0;
        end

        // --- 3. ISSUE ---
        if (if_valid && !rob_full) begin
            case (op)
                `OP_LUI, `OP_AUIPC, `OP_JAL, `OP_JALR, `OP_BRANCH, `OP_IMM, `OP_ALU: begin
                    if (rs_free_idx != -1) begin
                        if_valid <= 0;
                        rob_busy[rob_tail] <= 1; rob_ready[rob_tail] <= 0; rob_dest[rob_tail] <= (op == `OP_BRANCH) ? 0 : rd;
                        rob_pc[rob_tail] <= pc; rob_is_jump[rob_tail] <= (op == `OP_BRANCH || op == `OP_JAL || op == `OP_JALR);
                        rob_is_store[rob_tail] <= 0;
                        
                        rs_busy[rs_free_idx] <= 1; rs_opcode[rs_free_idx] <= op; rs_funct3[rs_free_idx] <= f3; rs_funct7[rs_free_idx] <= f7[5];
                        rs_imm[rs_free_idx] <= imm_val; rs_pc[rs_free_idx] <= pc; rs_rob[rs_free_idx] <= rob_tail;
                        
                        if (rf_busy[rs1]) begin
                            if (rob_busy[rf_rob[rs1]] && rob_ready[rf_rob[rs1]]) begin rs_v1[rs_free_idx] <= rob_val[rf_rob[rs1]]; rs_has_q1[rs_free_idx] <= 0; end
                            else begin rs_q1[rs_free_idx] <= rf_rob[rs1]; rs_has_q1[rs_free_idx] <= 1; end
                        end else begin rs_v1[rs_free_idx] <= rf_data[rs1]; rs_has_q1[rs_free_idx] <= 0; end
                        
                        if (rf_busy[rs2]) begin
                            if (rob_busy[rf_rob[rs2]] && rob_ready[rf_rob[rs2]]) begin rs_v2[rs_free_idx] <= rob_val[rf_rob[rs2]]; rs_has_q2[rs_free_idx] <= 0; end
                            else begin rs_q2[rs_free_idx] <= rf_rob[rs2]; rs_has_q2[rs_free_idx] <= 1; end
                        end else begin rs_v2[rs_free_idx] <= rf_data[rs2]; rs_has_q2[rs_free_idx] <= 0; end

                        if (op == `OP_BRANCH) begin
                            rob_predict_taken[rob_tail] <= bht[pc[9:2]][1];
                            pc <= bht[pc[9:2]][1] ? pc + imm_val : pc + 4;
                            rs_predict_taken[rs_free_idx] <= bht[pc[9:2]][1];
                        end else if (op == `OP_JAL) begin
                            pc <= pc + imm_val; rob_val[rob_tail] <= pc + 4; rob_ready[rob_tail] <= 1; rs_busy[rs_free_idx] <= 0;
                        end else pc <= pc + 4;

                        if (op != `OP_BRANCH && op != `OP_JAL && rd != 0) begin rf_busy[rd] <= 1; rf_rob[rd] <= rob_tail; end
                        rob_tail <= rob_tail + 1'b1;
                    end
                end
                `OP_LOAD, `OP_STORE: begin
                    if (lsb_free_idx_v != -1) begin
                        if_valid <= 0; pc <= pc + 4;
                        rob_busy[rob_tail] <= 1; rob_ready[rob_tail] <= 0; rob_dest[rob_tail] <= (op == `OP_STORE) ? 0 : rd;
                        rob_is_jump[rob_tail] <= 0; rob_is_store[rob_tail] <= (op == `OP_STORE);
                        
                        lsb_busy[lsb_tail] <= 1; lsb_opcode[lsb_tail] <= op; lsb_funct3[lsb_tail] <= f3;
                        lsb_imm[lsb_tail] <= imm_val; lsb_rob[lsb_tail] <= rob_tail;
                        
                        if (rf_busy[rs1]) begin
                            if (rob_busy[rf_rob[rs1]] && rob_ready[rf_rob[rs1]]) begin lsb_v1[lsb_tail] <= rob_val[rf_rob[rs1]]; lsb_has_q1[lsb_tail] <= 0; end
                            else begin lsb_q1[lsb_tail] <= rf_rob[rs1]; lsb_has_q1[lsb_tail] <= 1; end
                        end else begin lsb_v1[lsb_tail] <= rf_data[rs1]; lsb_has_q1[lsb_tail] <= 0; end
                        
                        if (rf_busy[rs2]) begin
                            if (rob_busy[rf_rob[rs2]] && rob_ready[rf_rob[rs2]]) begin lsb_v2[lsb_tail] <= rob_val[rf_rob[rs2]]; lsb_has_q2[lsb_tail] <= 0; end
                            else begin lsb_q2[lsb_tail] <= rf_rob[rs2]; lsb_has_q2[lsb_tail] <= 1; end
                        end else begin lsb_v2[lsb_tail] <= rf_data[rs2]; lsb_has_q2[lsb_tail] <= 0; end

                        if (op == `OP_LOAD && rd != 0) begin rf_busy[rd] <= 1; rf_rob[rd] <= rob_tail; end
                        lsb_tail <= lsb_tail + 1'b1; rob_tail <= rob_tail + 1'b1;
                    end
                end
            endcase
        end

        // --- 4. EXECUTE (RS) ---
        rs_free_idx = -1;
        for (i = 0; i < RS_SIZE; i = i + 1) if (rs_busy[i] && !rs_has_q1[i] && !rs_has_q2[i] && rs_free_idx == -1) rs_free_idx = i;
        if (rs_free_idx != -1) begin
            rs_busy[rs_free_idx] <= 0; cdb_en <= 1; cdb_rob <= rs_rob[rs_free_idx];
            case (rs_opcode[rs_free_idx])
                `OP_LUI: cdb_val <= rs_imm[rs_free_idx];
                `OP_AUIPC: cdb_val <= rs_pc[rs_free_idx] + rs_imm[rs_free_idx];
                `OP_JALR: begin cdb_val <= rs_pc[rs_free_idx] + 4; cdb_jump <= 1; cdb_taken <= 1; cdb_pc <= (rs_v1[rs_free_idx] + rs_imm[rs_free_idx]) & ~32'b1; end
                `OP_IMM: begin
                    case (rs_funct3[rs_free_idx])
                        3'b000: cdb_val <= rs_v1[rs_free_idx] + rs_imm[rs_free_idx];
                        3'b010: cdb_val <= ($signed(rs_v1[rs_free_idx]) < $signed(rs_imm[rs_free_idx]));
                        3'b011: cdb_val <= (rs_v1[rs_free_idx] < rs_imm[rs_free_idx]);
                        3'b100: cdb_val <= rs_v1[rs_free_idx] ^ rs_imm[rs_free_idx];
                        3'b110: cdb_val <= rs_v1[rs_free_idx] | rs_imm[rs_free_idx];
                        3'b111: cdb_val <= rs_v1[rs_free_idx] & rs_imm[rs_free_idx];
                        3'b001: cdb_val <= rs_v1[rs_free_idx] << rs_imm[rs_free_idx][4:0];
                        3'b101: cdb_val <= rs_funct7[rs_free_idx] ? ($signed(rs_v1[rs_free_idx]) >>> rs_imm[rs_free_idx][4:0]) : (rs_v1[rs_free_idx] >> rs_imm[rs_free_idx][4:0]);
                    endcase
                end
                `OP_ALU: begin
                    case (rs_funct3[rs_free_idx])
                        3'b000: cdb_val <= rs_funct7[rs_free_idx] ? rs_v1[rs_free_idx] - rs_v2[rs_free_idx] : rs_v1[rs_free_idx] + rs_v2[rs_free_idx];
                        3'b001: cdb_val <= rs_v1[rs_free_idx] << rs_v2[rs_free_idx][4:0];
                        3'b010: cdb_val <= ($signed(rs_v1[rs_free_idx]) < $signed(rs_v2[rs_free_idx]));
                        3'b011: cdb_val <= (rs_v1[rs_free_idx] < rs_v2[rs_free_idx]);
                        3'b100: cdb_val <= rs_v1[rs_free_idx] ^ rs_v2[rs_free_idx];
                        3'b101: cdb_val <= rs_funct7[rs_free_idx] ? ($signed(rs_v1[rs_free_idx]) >>> rs_v2[rs_free_idx][4:0]) : (rs_v1[rs_free_idx] >> rs_v2[rs_free_idx][4:0]);
                        3'b110: cdb_val <= rs_v1[rs_free_idx] | rs_v2[rs_free_idx];
                        3'b111: cdb_val <= rs_v1[rs_free_idx] & rs_v2[rs_free_idx];
                    endcase
                end
                `OP_BRANCH: begin
                    case (rs_funct3[rs_free_idx])
                        `FUNCT3_BEQ:  cdb_taken <= (rs_v1[rs_free_idx] == rs_v2[rs_free_idx]);
                        `FUNCT3_BNE:  cdb_taken <= (rs_v1[rs_free_idx] != rs_v2[rs_free_idx]);
                        `FUNCT3_BLT:  cdb_taken <= ($signed(rs_v1[rs_free_idx]) < $signed(rs_v2[rs_free_idx]));
                        `FUNCT3_BGE:  cdb_taken <= ($signed(rs_v1[rs_free_idx]) >= $signed(rs_v2[rs_free_idx]));
                        `FUNCT3_BLTU: cdb_taken <= (rs_v1[rs_free_idx] < rs_v2[rs_free_idx]);
                        `FUNCT3_BGEU: cdb_taken <= (rs_v1[rs_free_idx] >= rs_v2[rs_free_idx]);
                    endcase
                    cdb_jump <= 1; cdb_pc <= rs_pc[rs_free_idx] + rs_imm[rs_free_idx];
                end
            endcase
        end

        // --- 5. EXECUTE (LSB) ---
        if (mc_state == MC_IDLE && lsb_busy[lsb_head] && !lsb_has_q1[lsb_head] && !lsb_has_q2[lsb_head]) begin
            if (lsb_opcode[lsb_head] == `OP_LOAD) begin
                mc_state <= MC_LOAD; mc_addr <= lsb_v1[lsb_head] + lsb_imm[lsb_head];
                mc_funct3 <= lsb_funct3[lsb_head]; mc_byte_cnt <= 0; mc_rob <= lsb_rob[lsb_head];
            end
        end

        // --- 6. CDB BROADCAST ---
        if (cdb_en) begin
            for (i = 0; i < RS_SIZE; i = i + 1) if (rs_busy[i]) begin
                if (rs_has_q1[i] && rs_q1[i] == cdb_rob) begin rs_v1[i] <= cdb_val; rs_has_q1[i] <= 0; end
                if (rs_has_q2[i] && rs_q2[i] == cdb_rob) begin rs_v2[i] <= cdb_val; rs_has_q2[i] <= 0; end
            end
            for (i = 0; i < LSB_SIZE; i = i + 1) if (lsb_busy[i]) begin
                if (lsb_has_q1[i] && lsb_q1[i] == cdb_rob) begin lsb_v1[i] <= cdb_val; lsb_has_q1[i] <= 0; end
                if (lsb_has_q2[i] && lsb_q2[i] == cdb_rob) begin lsb_v2[i] <= cdb_val; lsb_has_q2[i] <= 0; end
            end
            if (rob_busy[cdb_rob]) begin
                rob_val[cdb_rob] <= cdb_val; rob_ready[cdb_rob] <= 1;
                if (cdb_jump) begin rob_jump_pc[cdb_rob] <= cdb_pc; rob_jump_taken[cdb_rob] <= cdb_taken; end
            end
            cdb_jump <= 0;
        end

        // --- 7. MC LOGIC ---
        case (mc_state)
            MC_FETCH: begin
                mc_buf[mc_byte_cnt*8 +: 8] <= mem_din;
                if (mc_byte_cnt == 3) begin if_inst <= {mem_din, mc_buf[23:0]}; if_valid <= 1; mc_state <= MC_IDLE; end
                else begin mc_byte_cnt <= mc_byte_cnt + 1'b1; mc_addr <= mc_addr + 1'b1; end
            end
            MC_LOAD: begin
                if (mc_addr >= 32'h30000 && io_buffer_full) begin /* Wait */ end
                else begin
                    mc_buf[mc_byte_cnt*8 +: 8] <= mem_din;
                    if ((mc_funct3[1:0] == 2'b00 && mc_byte_cnt == 0) || (mc_funct3[1:0] == 2'b01 && mc_byte_cnt == 1) || (mc_funct3[1:0] == 2'b10 && mc_byte_cnt == 3)) begin
                        cdb_en <= 1; cdb_rob <= mc_rob; mc_state <= MC_IDLE; lsb_busy[lsb_head] <= 0; lsb_head <= lsb_head + 1'b1;
                        case (mc_funct3)
                            `FUNCT3_LB:  cdb_val <= {{24{mem_din[7]}}, mem_din};
                            `FUNCT3_LH:  cdb_val <= {{16{mem_din[7]}}, mem_din, mc_buf[7:0]};
                            `FUNCT3_LW:  cdb_val <= {mem_din, mc_buf[23:0]};
                            `FUNCT3_LBU: cdb_val <= {24'b0, mem_din};
                            `FUNCT3_LHU: cdb_val <= {16'b0, mem_din, mc_buf[7:0]};
                        endcase
                    end else begin mc_byte_cnt <= mc_byte_cnt + 1'b1; mc_addr <= mc_addr + 1'b1; end
                end
            end
            MC_STORE: begin
                if (mc_addr >= 32'h30000 && io_buffer_full) mc_wr <= 0;
                else begin
                    mc_wr <= 1; mc_dout <= mc_buf[mc_byte_cnt*8 +: 8];
                    if ((mc_funct3[1:0] == 2'b00 && mc_byte_cnt == 0) || (mc_funct3[1:0] == 2'b01 && mc_byte_cnt == 1) || (mc_funct3[1:0] == 2'b10 && mc_byte_cnt == 3)) begin
                        mc_state <= MC_IDLE; rob_busy[rob_head] <= 0; rob_head <= rob_head + 1'b1;
                        lsb_busy[lsb_head] <= 0; lsb_head <= lsb_head + 1'b1; mc_wr <= 0;
                    end else begin mc_byte_cnt <= mc_byte_cnt + 1'b1; mc_addr <= mc_addr + 1'b1; end
                end
            end
            default: mc_wr <= 0;
        endcase
    end
end

endmodule
