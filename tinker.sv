//---------------------------------------------------------------------
// instruction_decoder Module
//---------------------------------------------------------------------
module instruction_decoder(
    input  [31:0] in,       // 32-bit instruction
    output [4:0]  opcode,   // Bits [31:27]
    output [4:0]  rd,       // Bits [26:22]
    output [4:0]  rs,       // Bits [21:17]
    output [4:0]  rt,       // Bits [16:12]
    output [11:0] L         // Bits [11:0]
);
    assign opcode = in[31:27];
    assign rd     = in[26:22];
    assign rs     = in[21:17];
    assign rt     = in[16:12];
    assign L      = in[11:0];
endmodule

//---------------------------------------------------------------------
// ALU Module
//---------------------------------------------------------------------
module alu (
    input  [4:0]  opcode,
    input  [63:0] op1,       // First operand
    input  [63:0] op2,       // Second operand
    input  [11:0] L,         // 12-bit literal (for immediate instructions)
    output reg [63:0] result // ALU result
);
    always @(*) begin
        case (opcode)
            5'h18: result = op1 + op2;                   // add
            5'h19: result = op1 + {52'b0, L};             // addi
            5'h1a: result = op1 - op2;                   // sub
            5'h1b: result = op1 - {52'b0, L};             // subi
            5'h1c: result = op1 * op2;                   // mul
            5'h1d: result = op1 / op2;                   // div
            5'h0:  result = op1 & op2;                   // and
            5'h1:  result = op1 | op2;                   // or
            5'h2:  result = op1 ^ op2;                   // xor
            5'h3:  result = ~op1;                        // not (rt ignored)
            5'h4:  result = op1 >> op2;                  // shftr
            5'h5:  result = op1 >> L;                    // shftri
            5'h6:  result = op1 << op2;                  // shftl
            5'h7:  result = op1 << L;                    // shftli
            5'h11: result = op1;                        // mov rd, rs
            5'h12: begin                                 // mov rd, L: update upper 12 bits
                      result = op1;
                      result[63:52] = L;
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
    input  [63:0] rs,         // Operand 1
    input  [63:0] rt,         // Operand 2
    input  [11:0] L,          // Literal
    output reg [63:0] result  // FPU result
);
    real op1, op2, res_real;
    always @(*) begin
        op1 = $bitstoreal(rs);
        op2 = $bitstoreal(rt);
        case (opcode)
            5'h14: res_real = op1 + op2; // addf
            5'h15: res_real = op1 - op2; // subf
            5'h16: res_real = op1 * op2; // mulf
            5'h17: res_real = op1 / op2; // divf
            default: res_real = 0.0;
        endcase
        result = $realtobits(res_real);
    end
endmodule

//---------------------------------------------------------------------
// regFile Module
//---------------------------------------------------------------------
module regFile (
    input         clk,
    input         reset,
    input  [63:0] data_in,   // Data to write
    input         we,        // Write enable
    input  [4:0]  rd,        // Write address
    input  [4:0]  rs,        // Read port 1 address
    input  [4:0]  rt,        // Read port 2 address
    output reg [63:0] rdOut, // Data from register rd
    output reg [63:0] rsOut, // Data from register rs
    output reg [63:0] rtOut  // Data from register rt
);
    // Declaration exactly as specified.
    reg [63:0] registers [0:31];
    integer i;
    
    always @(posedge clk) begin
        if (reset) begin
            for (i = 0; i < 31; i = i + 1)
                registers[i] <= 64'b0;
            registers[31] <= 64'h10000;
        end else begin
            if (we)
                registers[rd] <= data_in;
        end
    end
    
    // Combinational read.
    always @(*) begin
        rdOut = registers[rd];
        rsOut = registers[rs];
        rtOut = registers[rt];
    end
endmodule

//---------------------------------------------------------------------
// memory Module
//---------------------------------------------------------------------
module memory(
   input clk,
   input reset,
   // Fetch interface:
   input  [31:0] fetch_addr,
   output [31:0] fetch_instruction,
   // Data load interface:
   input  [31:0] data_load_addr,
   output [63:0] data_load,
   // Store interface:
   input         store_we,
   input  [31:0] store_addr,
   input  [63:0] store_data
);
    parameter MEM_SIZE = 512*1024;  // 512 KB
    // Declaration exactly as specified.
    reg [7:0] bytes [0:MEM_SIZE-1];
    integer i;
    
    always @(posedge clk) begin
        if (reset) begin
            // Optional memory initialization.
        end
        if (store_we) begin
            bytes[store_addr]     <= store_data[63:56];
            bytes[store_addr + 1] <= store_data[55:48];
            bytes[store_addr + 2] <= store_data[47:40];
            bytes[store_addr + 3] <= store_data[39:32];
            bytes[store_addr + 4] <= store_data[31:24];
            bytes[store_addr + 5] <= store_data[23:16];
            bytes[store_addr + 6] <= store_data[15:8];
            bytes[store_addr + 7] <= store_data[7:0];
        end
    end
    
    // Little-endian ordering for reads.
    assign fetch_instruction = { 
        bytes[fetch_addr+3],
        bytes[fetch_addr+2],
        bytes[fetch_addr+1],
        bytes[fetch_addr]
    };
    assign data_load = { 
        bytes[data_load_addr+7],
        bytes[data_load_addr+6],
        bytes[data_load_addr+5],
        bytes[data_load_addr+4],
        bytes[data_load_addr+3],
        bytes[data_load_addr+2],
        bytes[data_load_addr+1],
        bytes[data_load_addr]
    };
endmodule

//---------------------------------------------------------------------
// fetch Module
//---------------------------------------------------------------------
module fetch(
    input  [31:0] PC,
    input  [31:0] fetch_instruction,
    output [31:0] instruction
);
    assign instruction = fetch_instruction;
endmodule

//---------------------------------------------------------------------
// control Module (Modified: no ALU/FPU instantiation)
//---------------------------------------------------------------------
module control(
    input         clk,
    input         reset,
    input  [31:0] instruction,
    input  [31:0] PC,
    // These come from regFile outputs.
    input  [63:0] opA,         // From regFile.rsOut (contents of register rs, or overridden as needed)
    input  [63:0] opB,         // From regFile.rtOut (contents of register rt, or overridden)
    input  [63:0] opRD,        // From regFile.rdOut (contents of register rd)
    input  [63:0] data_load,   // From memory
    // Results from ALU and FPU (instantiated in tinker_core)
    input  [63:0] alu_result,
    input  [63:0] fpu_result,
    output reg [31:0] next_PC,
    output reg [63:0] exec_result,
    output reg        write_en,
    output reg [4:0]  write_reg,
    // These outputs drive the regFile read ports.
    output reg [4:0]  rf_addrA,
    output reg [4:0]  rf_addrB,
    // Memory store signals.
    output reg        mem_we,
    output reg [31:0] mem_addr,
    output reg [63:0] mem_write_data,
    // Data load address.
    output reg [31:0] data_load_addr
);
    // Decode the instruction.
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
    
    // Set register file read addresses.
    // Default: use rs for opA and rt for opB.
    // Then override for specific opcodes.
    always @(*) begin
         rf_addrA = rs;
         rf_addrB = rt;
         case (opcode)
            // Immediate instructions: use rd for opA.
            5'h19, 5'h1b, 5'h5, 5'h7, 5'h12: rf_addrA = rd;
            // Branch-not-zero: force opB to come from rd.
            5'hb: rf_addrB = rd;
            // Branch instructions (br and brr): use rd for opA.
            5'h8, 5'h9: rf_addrA = rd;
            // Call: use rd for opA and force opB to r31.
            5'hc: begin
                rf_addrA = rd;
                rf_addrB = 5'd31;
            end
            // Return: force opA to r31.
            5'hd: rf_addrA = 5'd31;
            // For brgt (opcode 5'he), do not override so that:
            // opA = registers[rs] and opB = registers[rt]
            default: ; // keep defaults
         endcase
    end
    
    // Main control logic.
    always @(*) begin
         // Default assignments.
         next_PC         = PC + 4;
         exec_result     = 64'b0;
         write_en        = 1'b0;
         write_reg       = rd;
         mem_we          = 1'b0;
         mem_addr        = 32'b0;
         mem_write_data  = 64'b0;
         data_load_addr  = 32'b0;
         
         case (opcode)
            // Integer Arithmetic & Logical Instructions.
            5'h18, 5'h1a, 5'h1c, 5'h1d,
            5'h0, 5'h1, 5'h2, 5'h3, 5'h4, 5'h6,
            5'h19, 5'h1b, 5'h5, 5'h7, 5'h12: begin
                exec_result = alu_result;
                write_en    = 1'b1;
            end
            5'h8: begin // br rd: PC = register[rd]
                next_PC = opA; // opA now comes from register rd (via override in regFile read addresses)
            end
            5'h9: begin // brr rd: PC = PC + register[rd]
                next_PC = PC + opA[31:0];
            end
            5'ha: begin // brr L: PC = PC + sign-extended L
                next_PC = PC + {{20{L[11]}}, L};
            end
            5'hb: begin // brnz rd, rs: if (register[rs] ≠ 0) then PC = register[rd]
                if (opA != 0)
                    next_PC = opB; // opB holds register rd due to override in branch-not-zero case
                else
                    next_PC = PC + 4;
            end
            5'hc: begin // call rd, rs, rt:
                mem_we         = 1'b1;
                mem_addr       = opB - 8;  // opB = register 31 (via override)
                mem_write_data = PC + 4;
                next_PC        = opA;      // opA holds register rd
            end
            5'hd: begin // return:
                data_load_addr = opA - 8;  // opA = register 31 (via override)
                next_PC        = data_load[31:0];
            end
            5'he: begin // brgt rd, rs, rt: if (register[rs] > register[rt]) then PC = register[rd]
                if ($signed(opA) > $signed(opB))
                    next_PC = opRD[31:0]; // opRD (rdOut) holds the value of register rd
                else
                    next_PC = PC + 4;
            end
            5'h10: begin // Load: mov rd, (rs)(L)
                data_load_addr = opA + {20'b0, L}; // opA from register rs
                exec_result    = data_load;
                write_en       = 1'b1;
            end
            5'h11: begin // mov rd, rs
                exec_result = opA;
                write_en    = 1'b1;
            end
            5'h13: begin // Store: mov (rd)(L), rs
                mem_we         = 1'b1;
                mem_addr       = opA + {20'b0, L}; // opA from register rd (base)
                mem_write_data = opB;             // opB from register rs (value)
            end
            5'h14,5'h15,5'h16,5'h17: begin // Floating Point Instructions.
                exec_result = fpu_result;
                write_en    = 1'b1;
            end
            default: next_PC = PC + 4;
         endcase
    end
endmodule

//---------------------------------------------------------------------
// tinker_core Module (Top Level)
//---------------------------------------------------------------------
module tinker_core(
    input clk,
    input reset
);
    reg [31:0] PC;
    
    // Instantiate memory.
    wire [31:0] fetch_instruction;
    wire [63:0] data_load;
    wire [31:0] mem_data_load_addr;
    wire        mem_we;
    wire [31:0] mem_store_addr;
    wire [63:0] mem_store_data;
    
    memory memory_inst(
        .clk(clk),
        .reset(reset),
        .fetch_addr(PC),
        .fetch_instruction(fetch_instruction),
        .data_load_addr(mem_data_load_addr),
        .data_load(data_load),
        .store_we(mem_we),
        .store_addr(mem_store_addr),
        .store_data(mem_store_data)
    );
    
    // Instantiate fetch.
    wire [31:0] instruction;
    fetch fetch_inst(
        .PC(PC),
        .fetch_instruction(fetch_instruction),
        .instruction(instruction)
    );
    
    // Instantiate regFile.
    wire [4:0] rf_addrA;
    wire [4:0] rf_addrB;
    wire [63:0] opA, opB;
    wire [63:0] rdOut;
    reg [63:0] write_data;
    reg write_en;
    reg [4:0] write_reg;
    
    regFile reg_file_inst(
        .clk(clk),
        .reset(reset),
        .data_in(write_data),
        .we(write_en),
        .rd(write_reg),
        .rs(rf_addrA),
        .rt(rf_addrB),
        .rdOut(rdOut),
        .rsOut(opA),
        .rtOut(opB)
    );
    
    // Instantiate instruction_decoder.
    wire [4:0] dec_opcode, dec_rd, dec_rs, dec_rt;
    wire [11:0] dec_L;
    instruction_decoder dec_inst(
        .in(instruction),
        .opcode(dec_opcode),
        .rd(dec_rd),
        .rs(dec_rs),
        .rt(dec_rt),
        .L(dec_L)
    );
    
    // The ALU and FPU are now instantiated here in tinker_core.
    wire [63:0] alu_result;
    alu alu_inst(
        .opcode(dec_opcode),
        .op1(opA),       // Use opA as first operand
        .op2(opB),       // Use opB as second operand
        .L(dec_L),
        .result(alu_result)
    );
    
    wire [63:0] fpu_result;
    fpu fpu_inst(
        .opcode(dec_opcode),
        .rs(opA),
        .rt(opB),
        .L(dec_L),
        .result(fpu_result)
    );
    
    // Instantiate control module.
    // Now control receives alu_result and fpu_result as inputs.
    wire [31:0] next_PC;
    wire [63:0] exec_result;
    wire ctrl_write_en;
    wire [4:0] ctrl_write_reg;
    wire [4:0] ctrl_rf_addrA;
    wire [4:0] ctrl_rf_addrB;
    wire ctrl_mem_we;
    wire [31:0] ctrl_mem_addr;
    wire [63:0] ctrl_mem_write_data;
    wire [31:0] ctrl_data_load_addr;
    
    control ctrl_inst(
        .clk(clk),
        .reset(reset),
        .instruction(instruction),
        .PC(PC),
        .opA(opA),
        .opB(opB),
        .opRD(rdOut),
        .data_load(data_load),
        .alu_result(alu_result),
        .fpu_result(fpu_result),
        .next_PC(next_PC),
        .exec_result(exec_result),
        .write_en(ctrl_write_en),
        .write_reg(ctrl_write_reg),
        .rf_addrA(ctrl_rf_addrA),
        .rf_addrB(ctrl_rf_addrB),
        .mem_we(ctrl_mem_we),
        .mem_addr(ctrl_mem_addr),
        .mem_write_data(ctrl_mem_write_data),
        .data_load_addr(ctrl_data_load_addr)
    );
    
    // Connect control outputs to regFile and memory.
    assign rf_addrA = ctrl_rf_addrA;
    assign rf_addrB = ctrl_rf_addrB;
    
    always @(*) begin
        write_data = exec_result;
        write_en   = ctrl_write_en;
        write_reg  = ctrl_write_reg;
    end
    
    assign mem_we             = ctrl_mem_we;
    assign mem_store_addr     = ctrl_mem_addr;
    assign mem_store_data     = ctrl_mem_write_data;
    assign mem_data_load_addr = ctrl_data_load_addr;
    
    // Update PC.
    always @(posedge clk) begin
        if (reset)
            PC <= 32'h2000;
        else
            PC <= next_PC;
    end
endmodule
