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

// Memory interface signals
reg [31:0] m_addr;
reg [7:0]  m_dout;
reg        m_wr;
assign mem_a = m_addr;
assign mem_dout = m_dout;
assign mem_wr = m_wr;

// Register file
reg [31:0] regs [31:0];
assign dbgreg_dout = regs[10];

// CPU state
reg [31:0] pc;
reg [2:0] state;
localparam S_FETCH = 0, S_DECODE = 1, S_EXEC = 2, S_LOAD = 3, S_STORE = 4;

reg [31:0] inst;
reg [1:0]  byte_idx;

integer i;
always @(posedge clk_in) begin
    if (rst_in) begin
        pc <= 0;
        for (i = 0; i < 32; i = i + 1) regs[i] <= 0;
        state <= S_FETCH;
        byte_idx <= 0;
        m_addr <= 0;
        m_wr <= 0;
    end else if (rdy_in) begin
        case (state)
            S_FETCH: begin
                m_addr <= pc + byte_idx;
                m_wr <= 0;
                if (byte_idx == 0) begin
                    // First byte requested in previous cycle (actually need to wait one cycle)
                end
                // Memory read takes 1 cycle. In S_FETCH, we request bytes.
                // Simplified:
                inst[byte_idx*8 +: 8] <= mem_din;
                if (byte_idx == 3) begin
                    state <= S_DECODE;
                    byte_idx <= 0;
                end else begin
                    byte_idx <= byte_idx + 1'b1;
                end
            end
            S_DECODE: begin
                state <= S_EXEC;
                // Decode logic...
            end
            S_EXEC: begin
                pc <= pc + 4;
                state <= S_FETCH;
                byte_idx <= 0;
            end
        endcase
    end
end

endmodule
