//---------------------------------------------------------------------
// regFile Module
//---------------------------------------------------------------------
module regFile (
    input  [63:0] data,      // Data to be written
    input         we,        // Write enable signal
    input         re1,       // Read enable for rs
    input         re2,       // Read enable for rt
    input  [4:0]  rd,        // Write address (destination)
    input  [4:0]  rs,        // Read address 1 (source)
    input  [4:0]  rt,        // Read address 2 (source)
    output reg [63:0] rsOut, // Data output from register rs
    output reg [63:0] rtOut  // Data output from register rt
);
    reg [63:0] registers [31:0];

    // read
    always @(*) begin
        if (re1)
            rsOut = registers[rs];
        else
            rsOut = 64'b0;
        if (re2)
            rtOut = registers[rt];
        else
            rtOut = 64'b0;
    end

    // write
    always @(*) begin
        if (we)
            registers[rd] = data;
    end
endmodule

//---------------------------------------------------------------------
// ALU Module
//---------------------------------------------------------------------
module alu (
    input  [4:0]  opcode,
    input  [63:0] rs,         // Operand 1
    input  [63:0] rt,         // Operand 2
    input  [11:0] L,          // 12-bit literal (for mov)
    output reg [63:0] result  // ALU result
);
    always @(*) begin
        case (opcode)
            // Arithmetic instructions
            5'b11000: result = rs + rt;         // add
            5'b11010: result = rs - rt;         // sub
            5'b11100: result = rs * rt;         // mul
            5'b11101: result = rs / rt;         // div
            // Logical instructions
            5'b00000: result = rs & rt;         // and
            5'b00001: result = rs | rt;         // or
            5'b00010: result = rs ^ rt;         // xor
            5'b00011: result = ~rs;             // not (rt ignored)
            // Shift instructions (using rt as shift amount)
            5'b00100: result = rs >> rt;        // shftr
            5'b00110: result = rs << rt;        // shftl
            // Data movement instructions
            5'b10001: result = rs;              // mov rd, rs
            5'b10010: begin                     // mov rd, L
                        result = 64'b0;
                        result[11:0] = L;    
                     end
            default: result = 64'b0;
        endcase
    end
endmodule

//---------------------------------------------------------------------
// FPU Module
//---------------------------------------------------------------------
module fpu (
    input  [4:0]  opcode,
    input  [63:0] rs,         // Operand 1 (bit pattern)
    input  [63:0] rt,         // Operand 2 (bit pattern)
    input  [11:0] L,          // Literal (if needed)
    output reg [63:0] result  // FPU result (64-bit)
);
    real op1, op2, res_real;
    always @(*) begin
        op1 = $bitstoreal(rs);
        op2 = $bitstoreal(rt);
        case (opcode)
            5'b10100: res_real = op1 + op2; // addf
            5'b10101: res_real = op1 - op2; // subf
            5'b10110: res_real = op1 * op2; // mulf
            5'b10111: res_real = op1 / op2; // divf
            default: res_real = 0.0;
        endcase
        result = $realtobits(res_real);
    end
endmodule


//---------------------------------------------------------------------
// Instruction Decoder Module
//---------------------------------------------------------------------
// Extracts the instruction fields from a 32-bit instruction.
module instruction_decoder(
    input  [31:0] in,       // 32-bit instruction
    output [4:0]  opcode,   // Bits 31:27: opcode
    output [4:0]  rd,       // Bits 26:22: destination register
    output [4:0]  rs,       // Bits 21:17: source register 1
    output [4:0]  rt,       // Bits 16:12: source register 2
    output [11:0] L         // Bits 11:0: literal
);
    assign opcode = in[31:27];
    assign rd     = in[26:22];
    assign rs     = in[21:17];
    assign rt     = in[16:12];
    assign L      = in[11:0];
endmodule



//---------------------------------------------------------------------
// tinker_core Module
//---------------------------------------------------------------------
// Implements the complete combinational datapath.
// The instruction decoder extracts fields from the instruction, the shared
// register file provides operands and receives the computed result, and the 
// ALU or FPU (selected via a big switch statement) computes the result.
module tinker_core(
    input [31:0] instruction  // 32-bit instruction input
);
    // Decoded fields
    wire [4:0] opcode, rd, rs, rt;
    wire [11:0] L;
    instruction_decoder dec (
        .in(instruction),
        .opcode(opcode),
        .rd(rd),
        .rs(rs),
        .rt(rt),
        .L(L)
    );
    
    // Shared register file instance.
    // This regFile is used both for reading register values (rsOut, rtOut)
    // and for writing back the computed result.
    wire [63:0] rsOut, rtOut;
    reg [63:0] final_result;
    regFile reg_file (
        .data(final_result),
        .we(1'b1),      
        .re1(1'b1),
        .re2(1'b1),
        .rd(rd),
        .rs(rs),
        .rt(rt),
        .rsOut(rsOut),
        .rtOut(rtOut)
    );
    
    // Instantiate the ALU and FPU modules.
    // They take the register file's outputs (rsOut and rtOut) as their operands.
    wire [63:0] alu_result;
    alu alu_inst (
        .opcode(opcode),
        .rs(rsOut),
        .rt(rtOut),
        .L(L),
        .result(alu_result)
    );
    
    wire [63:0] fpu_result;
    fpu fpu_inst (
        .opcode(opcode),
        .rs(rsOut),
        .rt(rtOut),
        .L(L),
        .result(fpu_result)
    );
    
    // Big switch statement to select the appropriate result.
    // Floating-point opcodes (10100, 10101, 10110, 10111) use fpu_result;
    // all other opcodes use alu_result.
    always @(*) begin
        case (opcode)
            5'b10100, 5'b10101, 5'b10110, 5'b10111:
                final_result = fpu_result;
            default:
                final_result = alu_result;
        endcase
    end
endmodule
