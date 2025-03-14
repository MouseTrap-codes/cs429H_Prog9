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
            5'h18: result = op1 + op2;                   // add
            5'h19: result = op1 + {52'b0, L};             // addi
            5'h1a: result = op1 - op2;                   // sub
            5'h1b: result = op1 - {52'b0, L};             // subi
            5'h1c: result = op1 * op2;                   // mul
            5'h1d: result = op1 / op2;                   // div
            5'h0:  result = op1 & op2;                   // and
            5'h1:  result = op1 | op2;                   // or
            5'h2:  result = op1 ^ op2;                   // xor
            5'h3:  result = ~op1;                        // not
            5'h4:  result = op1 >> op2;                  // shftr
            5'h5:  result = op1 >> L;                    // shftri
            5'h6:  result = op1 << op2;                  // shftl
            5'h7:  result = op1 << L;                    // shftli
            5'h11: result = op1;                        // mov rd, rs
            5'h12: begin
                      result = op1;
                      result[63:52] = L;                // mov rd, L: update upper 12 bits
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
// regFile Module (with extra branch read port)
//---------------------------------------------------------------------
module regFile (
    input         clk,
    input         reset,
    input  [63:0] data_in,     // Data to write
    input         we,          // Write enable
    input  [4:0]  rd,          // Write address
    input  [4:0]  rs,          // Read port 1 address
    input  [4:0]  rt,          // Read port 2 address
    input  [4:0]  branch_addr, // Address to read branch target
    output reg [63:0] rsOut,   // Data out port A
    output reg [63:0] rtOut,   // Data out port B
    output [63:0] branch_out   // Branch target value = registers[branch_addr]
);
    // Exactly as specified.
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
    
    // Two standard read ports.
    always @(*) begin
        rsOut = registers[rs];
        rtOut = registers[rt];
    end
    
    // Extra continuous output for branch target.
    assign branch_out = registers[branch_addr];
endmodule

//---------------------------------------------------------------------
// memory Module
//---------------------------------------------------------------------
module memory(
   input clk,
   input reset,
   input  [31:0] fetch_addr,        // Instruction fetch address
   output [31:0] fetch_instruction,   // 32-bit instruction read (little-endian)
   input  [31:0] data_load_addr,      // Data load address
   output [63:0] data_load,           // 64-bit word loaded (little-endian)
   input         store_we,            // Write enable for store
   input  [31:0] store_addr,          // Store address
   input  [63:0] store_data           // Data to store
);
    parameter MEM_SIZE = 512*1024;  // 512 KB
    reg [7:0] bytes [0:MEM_SIZE-1];
    integer i;
    
    always @(posedge clk) begin
        if (reset) begin
            // do nothing id
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
    
    // Little-endian read for fetch (instruction) and data load.
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
// control Module (Modified to use branch_target from regFile)
//---------------------------------------------------------------------
module control(
    input         clk,
    input         reset,
    input  [31:0] instruction,
    input  [31:0] PC,
    input  [63:0] opA,         // From regFile port A (contents of register rs)
    input  [63:0] opB,         // From regFile port B (contents of register rt)
    input  [63:0] data_load,   // From memory
    input  [63:0] branch_target, // From regFile.branch_out (contents of register rd)
    output reg [31:0] next_PC,
    output reg [63:0] exec_result,
    output reg        write_en,
    output reg [4:0]  write_reg,
    // These outputs drive the regFile's two standard read ports.
    output reg [4:0]  rf_addrA,
    output reg [4:0]  rf_addrB,
    // Memory store signals.
    output reg        mem_we,
    output reg [31:0] mem_addr,
    output reg [63:0] mem_write_data,
    // Data load address (for load instructions)
    output reg [31:0] data_load_addr,
    // New: output branch_addr_out to tell regFile which register to use for branch target.
    output reg [4:0] branch_addr_out
);
    // Decode instruction.
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
    
    // Set the regFile read addresses.
    // Default: opA from rs, opB from rt.
    // Override for immediate, branch, call, etc.
    always @(*) begin
        rf_addrA = rs;
        rf_addrB = rt;
        case (opcode)
            // Immediate instructions: opA from rd.
            5'h19, 5'h1b, 5'h5, 5'h7, 5'h12: rf_addrA = rd;
            // Branch-not-zero: force opB to come from rd.
            5'hb: rf_addrB = rd;
            // Branch instructions (br and brr): opA from rd.
            5'h8, 5'h9: rf_addrA = rd;
            // Call: opA from rd, opB forced to r31.
            5'hc: begin
                rf_addrA = rd;
                rf_addrB = 5'd31;
            end
            // Return: force opA to r31.
            5'hd: rf_addrA = 5'd31;
            // Store (mov (rd)(L), rs): opA from rd, opB from rs.
            5'h13: begin
                rf_addrA = rd;
                rf_addrB = rs;
            end
            // For brgt (opcode 5'he), do not override (so opA = registers[rs], opB = registers[rt]).
            default: ; // keep defaults
        endcase
        // For brgt we want to read register rd.
        branch_addr_out = rd;
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
            // Integer arithmetic and logical.
            5'h18, 5'h1a, 5'h1c, 5'h1d,
            5'h0, 5'h1, 5'h2, 5'h3, 5'h4, 5'h6,
            5'h19, 5'h1b, 5'h5, 5'h7, 5'h12: begin
                exec_result = alu_out;
                write_en    = 1'b1;
            end
            5'h8: next_PC = opA;                   // br rd: jump to register rd.
            5'h9: next_PC = PC + opA[31:0];          // brr rd: PC = PC + register rd.
            5'ha: next_PC = PC + {{20{L[11]}}, L};    // brr L: PC = PC + sign-extended L.
            5'hb: begin                            // brnz rd, rs: if (register rs != 0) then PC = register rd.
                if (opA != 0)
                    next_PC = opB;
                else
                    next_PC = PC + 4;
            end
            5'hc: begin                            // call rd, rs, rt:
                mem_we         = 1'b1;
                mem_addr       = opB - 8;         // opB = r31
                mem_write_data = PC + 4;
                next_PC        = opA;             // opA = register rd.
            end
            5'hd: begin                            // return: PC = Mem[r31 - 8]
                data_load_addr = opA - 8;         // opA = r31
                next_PC        = data_load[31:0];
            end
            5'he: begin                            // brgt rd, rs, rt: if (register rs > register rt) then PC = register rd.
                if ($signed(opA) > $signed(opB))
                    next_PC = branch_target[31:0];  // branch_target from regFile (i.e. registers[rd])
                else
                    next_PC = PC + 4;
            end
            5'h10: begin                           // Load: mov rd, (rs)(L)
                data_load_addr = opA + {20'b0, L};  // opA = register rs
                exec_result    = data_load;
                write_en       = 1'b1;
            end
            5'h11: begin                           // mov rd, rs.
                exec_result = opA;
                write_en    = 1'b1;
            end
            5'h13: begin                           // Store: mov (rd)(L), rs.
                mem_we         = 1'b1;
                mem_addr       = opA + {20'b0, L};  // opA = register rd
                mem_write_data = opB;              // opB = register rs
            end
            5'h14, 5'h15, 5'h16, 5'h17: begin       // Floating point.
                exec_result = fpu_out;
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
    
    // Instantiate fetch.
    wire [31:0] instruction;
    fetch fetch_inst (
        .PC(PC),
        .fetch_instruction(fetch_instruction),
        .instruction(instruction)
    );
    
    // Instantiate regFile with extra branch port.
    wire [4:0]  rf_addrA;
    wire [4:0]  rf_addrB;
    wire [63:0] opA, opB;
    wire [63:0] branch_target; // This will hold registers[rd]
    reg  [63:0] write_data;
    reg         write_en;
    reg  [4:0]  write_reg;
    wire [4:0]  ctrl_branch_addr; // from control module
    
    regFile reg_file (
        .clk(clk),
        .reset(reset),
        .data_in(write_data),
        .we(write_en),
        .rd(write_reg),
        .rs(rf_addrA),
        .rt(rf_addrB),
        .branch_addr(ctrl_branch_addr), // extra branch read address
        .rsOut(opA),
        .rtOut(opB),
        .branch_out(branch_target)       // extra branch output
    );
    
    // Instantiate control.
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
        .branch_target(branch_target), // value from regFile[rd]
        .next_PC(next_PC),
        .exec_result(exec_result),
        .write_en(ctrl_write_en),
        .write_reg(ctrl_write_reg),
        .rf_addrA(ctrl_rf_addrA),
        .rf_addrB(ctrl_rf_addrB),
        .mem_we(ctrl_mem_we),
        .mem_addr(ctrl_mem_addr),
        .mem_write_data(ctrl_mem_write_data),
        .data_load_addr(ctrl_data_load_addr),
        .branch_addr_out(ctrl_branch_addr)  // decoded rd for branch read
    );
    
    // Connect regFile read addresses.
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
