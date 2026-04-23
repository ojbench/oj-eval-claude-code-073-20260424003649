`ifndef DEFINES_VH
`define DEFINES_VH

// Opcode
`define OP_LUI    7'b0110111
`define OP_AUIPC  7'b0010111
`define OP_JAL    7'b1101111
`define OP_JALR   7'b1100111
`define OP_BRANCH 7'b1100011
`define OP_LOAD   7'b0000011
`define OP_STORE  7'b0100011
`define OP_IMM    7'b0010011
`define OP_ALU    7'b0110011

// Branch funct3
`define FUNCT3_BEQ  3'b000
`define FUNCT3_BNE  3'b001
`define FUNCT3_BLT  3'b100
`define FUNCT3_BGE  3'b101
`define FUNCT3_BLTU 3'b110
`define FUNCT3_BGEU 3'b111

// Load/Store funct3
`define FUNCT3_LB  3'b000
`define FUNCT3_LH  3'b001
`define FUNCT3_LW  3'b010
`define FUNCT3_LBU 3'b100
`define FUNCT3_LHU 3'b101
`define FUNCT3_SB  3'b000
`define FUNCT3_SH  3'b001
`define FUNCT3_SW  3'b010

// ALU funct3
`define FUNCT3_ADD_SUB 3'b000
`define FUNCT3_SLL     3'b001
`define FUNCT3_SLT     3'b010
`define FUNCT3_SLTU    3'b011
`define FUNCT3_XOR     3'b100
`define FUNCT3_SRL_SRA 3'b101
`define FUNCT3_OR      3'b110
`define FUNCT3_AND     3'b111

// Reservation Station sizes
`define RS_SIZE 16
`define RS_IDX_WIDTH 4

// ROB size
`define ROB_SIZE 16
`define ROB_IDX_WIDTH 4

// LSB size
`define LSB_SIZE 16
`define LSB_IDX_WIDTH 4

`endif
