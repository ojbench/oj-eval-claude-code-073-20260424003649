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

// Configuration
localparam ROB_SIZE = 16;
localparam ROB_WIDTH = 4;
localparam RS_SIZE = 16;
localparam RS_WIDTH = 4;
localparam LSB_SIZE = 16;
localparam LSB_WIDTH = 4;

// ----------------------------------------------------------------------------
// Internal Signals & Modules
// ----------------------------------------------------------------------------

// --- Common Data Bus (CDB) ---
reg        cdb_en;
reg [31:0] cdb_val;
reg [ROB_WIDTH-1:0] cdb_rob;

// --- Register File ---
reg [31:0] rf_data [31:0];
reg [ROB_WIDTH-1:0] rf_rob [31:0];
reg rf_busy [31:0];

// --- ROB ---
reg [31:0] rob_val [ROB_SIZE-1:0];
reg [31:0] rob_dest [ROB_SIZE-1:0]; // Rd index
reg [31:0] rob_pc [ROB_SIZE-1:0];
reg        rob_ready [ROB_SIZE-1:0];
reg        rob_busy [ROB_SIZE-1:0];
reg [ROB_WIDTH-1:0] rob_head, rob_tail;
wire rob_full = (rob_head == (rob_tail + 1'b1) && rob_busy[rob_head]);

// --- Memory Controller (MC) ---
reg [31:0] mc_addr;
reg [7:0]  mc_dout;
reg        mc_wr;
assign mem_a = mc_addr;
assign mem_dout = mc_dout;
assign mem_wr = mc_wr;

// MC internal state
reg [2:0] mc_state;
localparam MC_IDLE = 0, MC_FETCH = 1, MC_LOAD = 2, MC_STORE = 3;
reg [31:0] mc_buf;
reg [2:0]  mc_byte_cnt;

// --- Instruction Fetch Unit (IFU) ---
reg [31:0] pc;
reg [31:0] if_inst;
reg if_valid;

// ----------------------------------------------------------------------------
// Execution & Control Logic
// ----------------------------------------------------------------------------

assign dbgreg_dout = rf_data[10];

integer i;
always @(posedge clk_in) begin
    if (rst_in) begin
        pc <= 0;
        rob_head <= 0; rob_tail <= 0;
        mc_state <= MC_IDLE; mc_wr <= 0; mc_byte_cnt <= 0;
        if_valid <= 0;
        cdb_en <= 0;
        for (i = 0; i < 32; i = i + 1) begin
            rf_data[i] <= 0;
            rf_busy[i] <= 0;
        end
        for (i = 0; i < ROB_SIZE; i = i + 1) begin
            rob_busy[i] <= 0; rob_ready[i] <= 0;
        end
    end else if (rdy_in) begin
        // --- 1. COMMIT ---
        if (rob_busy[rob_head] && rob_ready[rob_head]) begin
            if (rob_dest[rob_head] != 0) begin
                rf_data[rob_dest[rob_head]] <= rob_val[rob_head];
                if (rf_busy[rob_dest[rob_head]] && rf_rob[rob_dest[rob_head]] == rob_head)
                    rf_busy[rob_dest[rob_head]] <= 0;
            end
            rob_busy[rob_head] <= 0;
            rob_head <= rob_head + 1'b1;
        end

        // --- 2. FETCH ---
        if (!if_valid && mc_state == MC_IDLE) begin
            mc_state <= MC_FETCH;
            mc_addr <= pc;
            mc_byte_cnt <= 0;
        end

        // --- 3. MC LOGIC ---
        case (mc_state)
            MC_IDLE: ;
            MC_FETCH: begin
                mc_buf[mc_byte_cnt*8 +: 8] <= mem_din;
                if (mc_byte_cnt == 3) begin
                    if_inst <= {mem_din, mc_buf[23:0]};
                    if_valid <= 1;
                    mc_state <= MC_IDLE;
                end else begin
                    mc_byte_cnt <= mc_byte_cnt + 1'b1;
                    mc_addr <= mc_addr + 1'b1;
                end
            end
        endcase

        // --- 4. DECODE & ISSUE (Placeholder) ---
        if (if_valid && !rob_full) begin
            // For now, just advance PC to avoid being stuck
            pc <= pc + 4;
            if_valid <= 0;
        end
    end
end

endmodule
