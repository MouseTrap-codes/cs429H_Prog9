//---------------------------------------------------------------------
// instruction_decoder Module
//---------------------------------------------------------------------
// Extracts the various fields from a 32–bit instruction.
module instruction_decoder(
    input  [31:0] in,       // 32–bit instruction
    output [4:0]  opcode,   // Bits [31:27]: opcode
    output [4:0]  rd,       // Bits [26:22]: destination register
    output [4:0]  rs,       // Bits [21:17]: source register 1
    output [4:0]  rt,       // Bits [16:12]: source register 2
    output [11:0] L         // Bits [11:0]: literal/immediate
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
// Performs integer arithmetic, logical and shift operations. For immediate
// instructions (addi, subi, shftri, shftli, mov rd, L) the literal L is used.
module alu (
    input  [4:0]  opcode,
    input  [63:0] op1,       // First operand (selected via control)
    input  [63:0] op2,       // Second operand (from register file)
    input  [11:0] L,         // 12–bit literal/immediate
    output reg [63:0] result // ALU result
);
    always @(*) begin
        case (opcode)
            // Integer arithmetic
            5'h18: result = op1 + op2;                   // add: rd ← rs + rt
            5'h19: result = op1 + {52'b0, L};             // addi: rd ← rd + L
            5'h1a: result = op1 - op2;                   // sub: rd ← rs - rt
            5'h1b: result = op1 - {52'b0, L};             // subi: rd ← rd - L
            5'h1c: result = op1 * op2;                   // mul: rd ← rs * rt
            5'h1d: result = op1 / op2;                   // div: rd ← rs / rt
            // Logical operations
            5'h0:  result = op1 & op2;                   // and: rd ← rs & rt
            5'h1:  result = op1 | op2;                   // or:  rd ← rs | rt
            5'h2:  result = op1 ^ op2;                   // xor: rd ← rs ^ rt
            5'h3:  result = ~op1;                        // not: rd ← ~rs  (rt ignored)
            // Shifts
            5'h4:  result = op1 >> op2;                  // shftr: rd ← rs >> rt
            5'h5:  result = op1 >> L;                    // shftri: rd ← rd >> L
            5'h6:  result = op1 << op2;                  // shftl: rd ← rs << rt
            5'h7:  result = op1 << L;                    // shftli: rd ← rd << L
            // Data movement
            5'h11: result = op1;                        // mov rd, rs: rd ← rs
            5'h12: begin                                 // mov rd, L: set upper 12 bits of rd
                      result = op1;                     // keep lower 52 bits
                      result[63:52] = L;
                   end
            default: result = 64'b0;
        endcase
    end
endmodule

//---------------------------------------------------------------------
// FPU Module
//---------------------------------------------------------------------
// Performs double–precision floating point operations.
module fpu (
    input  [4:0]  opcode,
    input  [63:0] rs,         // First operand (as bit–pattern)
    input  [63:0] rt,         // Second operand (as bit–pattern)
    input  [11:0] L,          // Literal if needed
    output reg [63:0] result  // 64–bit floating–point result
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
// A synchronous register file with two read ports. The registers are stored
// in a packed array exactly as specified. On reset, all registers are cleared
// except r31 which is set to 64'h10000.
module regFile (
    input         clk,
    input         reset,
    input  [63:0] data_in,   // Data to write
    input         we,        // Write enable
    input  [4:0]  rd,        // Write address (destination)
    input  [4:0]  rs,        // Read address 1
    input  [4:0]  rt,        // Read address 2
    output reg [63:0] rsOut, // Data output from port A
    output reg [63:0] rtOut  // Data output from port B
);
    // Do not change the following declaration:
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
// Implements a 512KB memory using a packed array named "bytes" (exactly as
// specified). This version provides three interfaces:
// 1) An instruction fetch interface (32–bit instruction read)
// 2) A data load interface (64–bit word read)
// 3) A store interface (64–bit word write)
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
    parameter MEM_SIZE = 512*1024;  // 512 KB of memory
    // Do not change the following declaration:
    reg [7:0] bytes [0:MEM_SIZE-1];
    integer i;

    // (Optional) Memory initialization can be added here.
    always @(posedge clk) begin
        if (reset) begin
            // Optionally clear memory here.
            // for (i = 0; i < MEM_SIZE; i = i + 1)
            //     bytes[i] <= 8'b0;
        end
        // Store operation: write an 8–byte (64–bit) word into consecutive bytes (big–endian).
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

    // Combinational read for instruction fetch: fetch 4 bytes.
    assign fetch_instruction = { bytes[fetch_addr],
                                 bytes[fetch_addr+1],
                                 bytes[fetch_addr+2],
                                 bytes[fetch_addr+3] };
    // Combinational read for data load: read 8 bytes.
    assign data_load = { bytes[data_load_addr],
                         bytes[data_load_addr+1],
                         bytes[data_load_addr+2],
                         bytes[data_load_addr+3],
                         bytes[data_load_addr+4],
                         bytes[data_load_addr+5],
                         bytes[data_load_addr+6],
                         bytes[data_load_addr+7] };
endmodule

//---------------------------------------------------------------------
// fetch Module
//---------------------------------------------------------------------
// A separate fetch module. Here it is a thin wrapper that simply forwards
// the instruction fetched from memory. (This module can be expanded in the future.)
module fetch(
    input  [31:0] PC,
    input  [31:0] fetch_instruction, // from memory interface
    output [31:0] instruction
);
    assign instruction = fetch_instruction;
endmodule

//---------------------------------------------------------------------
// control Module
//---------------------------------------------------------------------
// Implements the execution and control logic. It instantiates the instruction
// decoder, ALU and FPU, selects the proper register file read addresses, computes
// the next PC (handling branches/jumps) and generates the register and memory
// write signals. For instructions that load from memory, the data_load value (from
// the memory module) is used.
module control(
    input         clk,
    input         reset,
    input  [31:0] instruction,
    input  [31:0] PC,
    input  [63:0] opA,         // Value from register file (port A)
    input  [63:0] opB,         // Value from register file (port B)
    input  [63:0] data_load,   // 64–bit word read from memory (for loads)
    output reg [31:0] next_PC,
    output reg [63:0] exec_result,
    output reg        write_en,
    output reg [4:0]  write_reg,
    // Outputs for selecting register file read ports:
    output reg [4:0]  rf_addrA,
    output reg [4:0]  rf_addrB,
    // Memory store signals:
    output reg        mem_we,
    output reg [31:0] mem_addr,
    output reg [63:0] mem_write_data,
    // Memory load address for data movement load instructions:
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
    
    // Set the register file read addresses.
    // By default, use rs and rt. For immediate instructions (e.g. addi, subi,
    // shftri, shftli, mov rd, L) use rd as the first operand.
    always @(*) begin
         rf_addrA = rs;
         rf_addrB = rt;
         if ((opcode == 5'h19) || (opcode == 5'h1b) ||
             (opcode == 5'h5)  || (opcode == 5'h7)  ||
             (opcode == 5'h12))
             rf_addrA = rd;
         // For brnz, the branch condition comes from rs and branch target from rd.
         if (opcode == 5'hb)
             rf_addrB = rd;
         // For branch instructions that use a register target (br, brr),
         // override to use rd.
         if ((opcode == 5'h8) || (opcode == 5'h9))
             rf_addrA = rd;
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
            // ---------------------------
            // Integer Arithmetic & Logical Instructions:
            // (Both register–register and immediate forms.)
            // ---------------------------
            5'h18, 5'h1a, 5'h1c, 5'h1d,
            5'h0, 5'h1, 5'h2, 5'h3, 5'h4, 5'h6,
            5'h19, 5'h1b, 5'h5, 5'h7, 5'h12: begin
                exec_result = alu_out;
                write_en    = 1'b1;
            end
            // ---------------------------
            // Control Instructions:
            // ---------------------------
            5'h8: begin // br rd: jump to the address contained in register rd.
                next_PC = opA;
            end
            5'h9: begin // brr rd: PC = PC + register[rd]
                next_PC = PC + opA[31:0];
            end
            5'ha: begin // brr L: PC = PC + sign–extended L
                next_PC = PC + {{20{L[11]}}, L};
            end
            5'hb: begin // brnz rd, rs: if opA (from rs) ≠ 0, then PC = opB (from rd), else PC+4.
                if (opA != 0)
                    next_PC = opB;
                else
                    next_PC = PC + 4;
            end
            5'hc: begin // call rd, rs, rt:
                // Save PC+4 into memory at (r31 – 8) and jump.
                // (Here we assume that one of the register file ports provides r31.)
                mem_we         = 1'b1;
                mem_addr       = opB - 8;      // (Assumes opB comes from r31.)
                mem_write_data = PC + 4;
                next_PC        = opA;          // Jump to address in register rd.
            end
            5'hd: begin // return: PC = Mem[r31 – 8]
                next_PC = data_load[31:0];     // Assume the loaded word holds the target address.
            end
            5'he: begin // brgt rd, rs, rt: if (opA > opB) then branch.
                if ($signed(opA) > $signed(opB))
                    // Due to only two read ports, we approximate by using rd as an immediate target.
                    next_PC = {27'b0, rd};
                else
                    next_PC = PC + 4;
            end
            // ---------------------------
            // Data Movement Instructions:
            // ---------------------------
            5'h10: begin // mov rd, (rs)(L): load a 64–bit word from memory.
                data_load_addr = opA + {20'b0, L};
                exec_result    = data_load;
                write_en       = 1'b1;
            end
            5'h11: begin // mov rd, rs: simple register move.
                exec_result = opA;
                write_en    = 1'b1;
            end
            5'h13: begin // mov (rd)(L), rs: store a 64–bit word into memory.
                mem_we         = 1'b1;
                mem_addr       = opA + {20'b0, L};
                mem_write_data = opB;
            end
            // ---------------------------
            // Floating Point Instructions:
            // ---------------------------
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
// The top-level module instantiates memory, the fetch module, the register
// file, and the control module. On reset, the PC is initialized to 32'h2000.
// The regFile instance is named "reg_file" and its internal registers are stored
// in a packed array as specified. The memory instance (named "mem_inst") holds the
// 512KB of memory in the packed array "bytes".
module tinker_core(
    input clk,
    input reset
);
    // Program Counter (PC) register.
    reg [31:0] PC;
    
    // ----------------------------------------------------------
    // Instantiate the memory module.
    // The memory provides three interfaces:
    // - fetch_addr: for instruction fetch (4 bytes)
    // - data_load_addr: for data load instructions (8 bytes)
    // - store interface: for data store instructions
    // ----------------------------------------------------------
    wire [31:0] fetch_instruction;
    wire [63:0] data_load;
    wire [31:0] mem_data_load_addr;
    wire        mem_we;
    wire [31:0] mem_store_addr;
    wire [63:0] mem_store_data;
    
    memory mem_inst (
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
    
    // ----------------------------------------------------------
    // Instantiate the fetch module.
    // (A simple wrapper that forwards the fetched instruction.)
    // ----------------------------------------------------------
    wire [31:0] instruction;
    fetch fetch_inst (
        .PC(PC),
        .fetch_instruction(fetch_instruction),
        .instruction(instruction)
    );
    
    // ----------------------------------------------------------
    // Instantiate the register file.
    // Instance name: reg_file. It provides two read ports.
    // ----------------------------------------------------------
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
    
    // ----------------------------------------------------------
    // Instantiate the control module.
    // The control module receives the current instruction, PC, and the
    // register file outputs (opA and opB) and also the memory data load value.
    // It outputs the next PC, the execution result (to be written back),
    // register file read addresses, and memory store signals.
    // ----------------------------------------------------------
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
    
    // Connect the control module outputs to the register file.
    assign rf_addrA = ctrl_rf_addrA;
    assign rf_addrB = ctrl_rf_addrB;
    
    // Drive the writeback signals to the register file.
    always @(*) begin
        write_data = exec_result;
        write_en   = ctrl_write_en;
        write_reg  = ctrl_write_reg;
    end
    
    // Connect memory store and data load addresses.
    assign mem_we          = ctrl_mem_we;
    assign mem_store_addr  = ctrl_mem_addr;
    assign mem_store_data  = ctrl_mem_write_data;
    assign mem_data_load_addr = ctrl_data_load_addr;
    
    // ----------------------------------------------------------
    // PC Update (Synchronous)
    // ----------------------------------------------------------
    always @(posedge clk) begin
        if (reset)
            PC <= 32'h2000;
        else
            PC <= next_PC;
    end
endmodule
