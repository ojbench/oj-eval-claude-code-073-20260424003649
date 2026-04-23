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
localparam MC_IDLE = 0, MC_FETCH = 1, MC_LOAD = 2, MC_STORE = 3;
reg [31:0] mc_buf;
reg [ 2:0] mc_byte_cnt;
reg [ROB_WIDTH-1:0] mc_rob;

assign mem_a = mc_addr;
assign mem_dout = mc_dout;
assign mem_wr = mc_wr;

// --- IFU & BHT ---
reg [31:0] pc;
reg [31:0] if_inst;
reg        if_valid;
reg [ 1:0] bht [255:0];

assign dbgreg_dout = rf_data[10];

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
        // --- 1. COMMIT ---
        cdb_en <= 0;
        if (rob_busy[rob_head] && rob_ready[rob_head]) begin
            if (rob_is_jump[rob_head]) begin
                if (rob_jump_taken[rob_head] != rob_predict_taken[rob_head]) begin
                    pc <= rob_jump_taken[rob_head] ? rob_jump_pc[rob_head] : (rob_pc[rob_head] + 4);
                    rob_head <= 0; rob_tail <= 0; if_valid <= 0; lsb_head <= 0; lsb_tail <= 0;
                    for (i = 0; i < ROB_SIZE; i = i + 1) rob_busy[i] <= 0;
                    for (i = 0; i < RS_SIZE; i = i + 1) rs_busy[i] <= 0;
                    for (i = 0; i < LSB_SIZE; i = i + 1) lsb_busy[i] <= 0;
                    for (i = 0; i < 32; i = i + 1) rf_busy[i] <= 0;
                end else begin
                    rob_busy[rob_head] <= 0;
                    rob_head <= rob_head + 1'b1;
                end
                i = rob_pc[rob_head][9:2];
                if (rob_jump_taken[rob_head]) bht[i] <= (bht[i] == 2'b11) ? 2'b11 : bht[i] + 1;
                else bht[i] <= (bht[i] == 2'b00) ? 2'b00 : bht[i] - 1;
            end else if (rob_dest[rob_head] != 0) begin
                rf_data[rob_dest[rob_head]] <= rob_val[rob_head];
                if (rf_busy[rob_dest[rob_head]] && rf_rob[rob_dest[rob_head]] == rob_head)
                    rf_busy[rob_dest[rob_head]] <= 0;
                rob_busy[rob_head] <= 0;
                rob_head <= rob_head + 1'b1;
            end else begin
                rob_busy[rob_head] <= 0;
                rob_head <= rob_head + 1'b1;
            end
        end

        // --- 2. FETCH & ISSUE (Skeleton) ---
        if (!if_valid && mc_state == MC_IDLE && !rob_full) begin
            mc_state <= MC_FETCH; mc_addr <= pc; mc_byte_cnt <= 0;
        end
        if (if_valid && !rob_full) begin
            // Basic Issue Logic Placeholder
            if_valid <= 0; pc <= pc + 4;
        end

        // --- 3. EXECUTE (Placeholder) ---
        // CDB broadcast logic
        if (cdb_en) begin
            for (i = 0; i < RS_SIZE; i = i + 1) if (rs_busy[i]) begin
                if (rs_has_q1[i] && rs_q1[i] == cdb_rob) begin rs_v1[i] <= cdb_val; rs_has_q1[i] <= 0; end
                if (rs_has_q2[i] && rs_q2[i] == cdb_rob) begin rs_v2[i] <= cdb_val; rs_has_q2[i] <= 0; end
            end
            for (i = 0; i < LSB_SIZE; i = i + 1) if (lsb_busy[i]) begin
                if (lsb_has_q1[i] && lsb_q1[i] == cdb_rob) begin lsb_v1[i] <= cdb_val; lsb_has_q1[i] <= 0; end
                if (lsb_has_q2[i] && lsb_q2[i] == cdb_rob) begin lsb_v2[i] <= cdb_val; lsb_has_q2[i] <= 0; end
            end
            if (rob_busy[cdb_rob]) begin rob_val[cdb_rob] <= cdb_val; rob_ready[cdb_rob] <= 1; end
        end

        // --- 4. MC LOGIC ---
        case (mc_state)
            MC_FETCH: begin
                mc_buf[mc_byte_cnt*8 +: 8] <= mem_din;
                if (mc_byte_cnt == 3) begin
                    if_inst <= {mem_din, mc_buf[23:0]}; if_valid <= 1; mc_state <= MC_IDLE;
                end else begin
                    mc_byte_cnt <= mc_byte_cnt + 1'b1; mc_addr <= mc_addr + 1'b1;
                end
            end
        endcase
    end
end

endmodule
