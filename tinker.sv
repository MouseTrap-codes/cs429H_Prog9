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
    input  [11:0] L,         // 12-bit literal/immediate
    output reg [63:0] result // Result
);
    always @(*) begin
        case (opcode)
            // Integer arithmetic
            5'h18: result = op1 + op2;                   // add
            5'h19: result = op1 + {52'b0, L};             // addi
            5'h1a: result = op1 - op2;                   // sub
            5'h1b: result = op1 - {52'b0, L};             // subi
            5'h1c: result = op1 * op2;                   // mul
            5'h1d: result = op1 / op2;                   // div
            // Logical operations
            5'h0:  result = op1 & op2;                   // and
            5'h1:  result = op1 | op2;                   // or
            5'h2:  result = op1 ^ op2;                   // xor
            5'h3:  result = ~op1;                        // not (rt ignored)
            // Shift operations
            5'h4:  result = op1 >> op2;                  // shftr
            5'h5:  result = op1 >> L;                    // shftri
            5'h6:  result = op1 << op2;                  // shftl
            5'h7:  result = op1 << L;                    // shftli
            // Data movement
            5'h11: result = op1;                        // mov rd, rs
            5'h12: begin                                 // mov rd, L: update lower 12 bits
                      result = op1;
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
    input  [4:0]  rs,        // Read address 1
    input  [4:0]  rt,        // Read address 2
    output reg [63:0] rsOut, // Data out port A
    output reg [63:0] rtOut  // Data out port B
);
    // Declaration exactly as specified:
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
    // Declaration exactly as specified:
    reg [7:0] bytes [0:MEM_SIZE-1];
    integer i;
    
    always @(posedge clk) begin
        if (reset) begin
            // Optional memory initialization.
        end
        if (store_we) begin
            // Store 64-bit word into 8 consecutive bytes (big-endian storage for writes is fine
            // if the testbench writes in little-endian format into memory).
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
    
    // IMPORTANT: Change the ordering for reads to little-endian.
    // Combinational read for instruction fetch (4 bytes, little-endian):
    assign fetch_instruction = { 
        bytes[fetch_addr+3],
        bytes[fetch_addr+2],
        bytes[fetch_addr+1],
        bytes[fetch_addr]
    };
    // Combinational read for data load (8 bytes, little-endian):
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
// control Module
//---------------------------------------------------------------------
module control(
    input         clk,
    input         reset,
    input  [31:0] instruction,
    input  [31:0] PC,
    input  [63:0] opA,         // Data from regFile port A
    input  [63:0] opB,         // Data from regFile port B
    input  [63:0] data_load,   // Data loaded from memory
    output reg [31:0] next_PC,
    output reg [63:0] exec_result,
    output reg        write_en,
    output reg [4:0]  write_reg,
    // Register file read addresses:
    output reg [4:0]  rf_addrA,
    output reg [4:0]  rf_addrB,
    // Memory store signals:
    output reg        mem_we,
    output reg [31:0] mem_addr,
    output reg [63:0] mem_write_data,
    // Data load address for load instructions:
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
    
    // Instantiate ALU and FPU.
    wire [63:0] alu_out;
    alu alu_inst (
       .opcode(opcode),
       .op1(opA),
       .op2(opB),
       .L(L),
       .result(alu_out)
    );
    
    wire [63:0] fpu_out;
    fpu fpu_inst (
       .opcode(opcode),
       .rs(opA),
       .rt(opB),
       .L(L),
       .result(fpu_out)
    );
    
    // Set register file read addresses.
    // Default: use rs for opA and rt for opB.
    // Then override for specific opcodes.
    always @(*) begin
         rf_addrA = rs;
         rf_addrB = rt;
         case (opcode)
            // Immediate instructions: op1 should come from rd.
            5'h19, 5'h1b, 5'h5, 5'h7, 5'h12:
                rf_addrA = rd;
            // Branch-not-zero: branch target comes from rd.
            5'hb:
                rf_addrB = rd;
            // Branch instructions that jump to a register address.
            5'h8, 5'h9:
                rf_addrA = rd;
            // Call: use rd for jump target and force second port to r31.
            5'hc: begin
                rf_addrA = rd;
                rf_addrB = 5'd31;
            end
            // Return: force first port to r31.
            5'hd:
                rf_addrA = 5'd31;
            // brgt: branch target from rd.
            5'he:
                // keep default
            // Store (mov (rd)(L), rs): use rd as base and rs as value.
            5'h13: begin
                rf_addrA = rd;
                rf_addrB = rs;
            end
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
            // Integer Arithmetic & Logical Instructions:
            5'h18, 5'h1a, 5'h1c, 5'h1d,
            5'h0, 5'h1, 5'h2, 5'h3, 5'h4, 5'h6,
            5'h19, 5'h1b, 5'h5, 5'h7, 5'h12: begin
                exec_result = alu_out;
                write_en    = 1'b1;
            end
            // Branch instructions:
            5'h8: begin // br rd: PC = register[rd]
                next_PC = opA; // opA = register rd
            end
            5'h9: begin // brr rd: PC = PC + register[rd]
                next_PC = PC + opA[31:0];
            end
            5'ha: begin // brr L: PC = PC + sign-extended L
                next_PC = PC + {{20{L[11]}}, L};
            end
            5'hb: begin // brnz rd, rs: if (register[rs] != 0) then PC = register[rd]
                if (opA != 0)
                    next_PC = opB; // opB = register rd
                else
                    next_PC = PC + 4;
            end
            5'hc: begin // call rd, rs, rt:
                mem_we         = 1'b1;
                mem_addr       = opB - 8;  // opB = register 31
                mem_write_data = PC + 4;
                next_PC        = opA;      // opA = register rd
            end
            5'hd: begin // return:
                // For return, read r31 and then set load address to (r31 - 8)
                data_load_addr = opA - 8;  // opA = register 31
                next_PC        = data_load[31:0];
            end
            5'he: begin // brgt rd, rs, rt: if (register[rs] > register[rt]) then PC = register[rd]
                if ($signed(opA) > $signed(opB))
                    // Access the branch target directly from the register file's array.
                    next_PC = tinker_core.reg_file.registers[rd][63:32];
                else
                    next_PC = PC + 4;
            end
            // Data Movement Instructions:
            5'h10: begin // mov rd, (rs)(L): load 64-bit word from memory.
                data_load_addr = opA + {20'b0, L}; // opA = register rs
                exec_result    = data_load;
                write_en       = 1'b1;
            end
            5'h11: begin // mov rd, rs: move register.
                exec_result = opA;
                write_en    = 1'b1;
            end
            5'h13: begin // mov (rd)(L), rs: store 64-bit word.
                mem_we         = 1'b1;
                mem_addr       = opA + {20'b0, L}; // opA = register rd (base)
                mem_write_data = opB;             // opB = register rs (value)
            end
            // Floating Point Instructions:
            5'h14, 5'h15, 5'h16, 5'h17: begin
                exec_result = fpu_out;
                write_en    = 1'b1;
            end
            default: begin
                next_PC = PC + 4;
            end
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
    // Program Counter (PC) register.
    reg [31:0] PC;
    
    // Instantiate memory module.
    // Instance name is "memory" (as expected by the testbench).
    wire [31:0] fetch_instruction;
    wire [63:0] data_load;
    wire [31:0] mem_data_load_addr;
    wire        mem_we;
    wire [31:0] mem_store_addr;
    wire [63:0] mem_store_data;
    
    memory memory (
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
    
    // Instantiate fetch module.
    wire [31:0] instruction;
    fetch fetch_inst (
        .PC(PC),
        .fetch_instruction(fetch_instruction),
        .instruction(instruction)
    );
    
    // Instantiate register file (instance name: reg_file).
    wire [4:0]  rf_addrA;
    wire [4:0]  rf_addrB;
    wire [63:0] opA, opB;
    reg  [63:0] write_data;
    reg         write_en;
    reg  [4:0]  write_reg;
    
    regFile reg_file (
        .clk(clk),
        .reset(reset),
        .data_in(write_data),
        .we(write_en),
        .rd(write_reg),
        .rs(rf_addrA),
        .rt(rf_addrB),
        .rsOut(opA),
        .rtOut(opB)
    );
    
    // Instantiate control module.
    wire [31:0] next_PC;
    wire [63:0] exec_result;
    wire        ctrl_write_en;
    wire [4:0]  ctrl_write_reg;
    wire [4:0]  ctrl_rf_addrA;
    wire [4:0]  ctrl_rf_addrB;
    wire        ctrl_mem_we;
    wire [31:0] ctrl_mem_addr;
    wire [63:0] ctrl_mem_write_data;
    wire [31:0] ctrl_data_load_addr;
    
    control ctrl_inst (
        .clk(clk),
        .reset(reset),
        .instruction(instruction),
        .PC(PC),
        .opA(opA),
        .opB(opB),
        .data_load(data_load),
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
    
    // Connect control outputs to register file and memory.
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
    
    // PC update.
    always @(posedge clk) begin
        if (reset)
            PC <= 32'h2000;
        else
            PC <= next_PC;
    end
endmodule
