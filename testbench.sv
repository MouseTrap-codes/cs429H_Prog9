module tinker_core_tb;
  reg clk;
  reg reset;

  // Instantiate the top-level module.
  tinker_core uut (
    .clk(clk),
    .reset(reset)
  );

  // Clock generation: 10 time units period.
  initial begin
    clk = 0;
    forever #5 clk = ~clk;
  end

  // Helper task to write a 32-bit instruction to memory at a given address.
  task write_instr;
    input [31:0] addr;
    input [31:0] instr;
    begin
      uut.memory.bytes[addr]     = instr[7:0];
      uut.memory.bytes[addr + 1] = instr[15:8];
      uut.memory.bytes[addr + 2] = instr[23:16];
      uut.memory.bytes[addr + 3] = instr[31:24];
    end
  endtask

  // The test program:
  // We use consecutive addresses starting at 32'h2000.
  // (Keep in mind that immediates are 12 bits and registers are referenced by number.)
  initial begin
    integer addr;
    reg [31:0] instr;
    // Wait for reset.
    reset = 1;
    #20;
    reset = 0;
    #10;
    
    addr = 32'h2000;

    // ----- ALU & Immediate Integer Instructions -----
    // 1. addi r1, 10         ; opcode 0x19, r1 = 10
    instr = {5'h19, 5'd1, 5'd0, 5'd0, 12'd10};
    write_instr(addr, instr); addr = addr + 4;
    
    // 2. addi r2, 20         ; r2 = 20
    instr = {5'h19, 5'd2, 5'd0, 5'd0, 12'd20};
    write_instr(addr, instr); addr = addr + 4;
    
    // 3. add r3, r1, r2       ; opcode 0x18, r3 = r1 + r2 = 10+20 = 30
    instr = {5'h18, 5'd3, 5'd1, 5'd2, 12'd0};
    write_instr(addr, instr); addr = addr + 4;
    
    // 4. sub r4, r2, r1       ; opcode 0x1A, r4 = 20 - 10 = 10
    instr = {5'h1a, 5'd4, 5'd2, 5'd1, 12'd0};
    write_instr(addr, instr); addr = addr + 4;
    
    // 5. subi r1, 5           ; opcode 0x1B, r1 = r1 - 5 = 10-5 = 5
    instr = {5'h1b, 5'd1, 5'd0, 5'd0, 12'd5};
    write_instr(addr, instr); addr = addr + 4;
    
    // 6. mul r5, r1, r2       ; opcode 0x1C, r5 = 5 * 20 = 100
    instr = {5'h1c, 5'd5, 5'd1, 5'd2, 12'd0};
    write_instr(addr, instr); addr = addr + 4;
    
    // 7. div r6, r2, r1       ; opcode 0x1D, r6 = 20 / 5 = 4
    instr = {5'h1d, 5'd6, 5'd2, 5'd1, 12'd0};
    write_instr(addr, instr); addr = addr + 4;
    
    // 8. and r7, r1, r2       ; opcode 0x0, r7 = 5 & 20
    instr = {5'h0, 5'd7, 5'd1, 5'd2, 12'd0};
    write_instr(addr, instr); addr = addr + 4;
    
    // 9. or r8, r1, r2        ; opcode 0x1, r8 = 5 | 20
    instr = {5'h1, 5'd8, 5'd1, 5'd2, 12'd0};
    write_instr(addr, instr); addr = addr + 4;
    
    // 10. xor r9, r1, r2      ; opcode 0x2, r9 = 5 ^ 20
    instr = {5'h2, 5'd9, 5'd1, 5'd2, 12'd0};
    write_instr(addr, instr); addr = addr + 4;
    
    // 11. not r10, r1        ; opcode 0x3, r10 = ~r1 (i.e. ~5)
    instr = {5'h3, 5'd10, 5'd1, 5'd0, 12'd0};
    write_instr(addr, instr); addr = addr + 4;
    
    // 12. shftr r11, r2, r1   ; opcode 0x4, r11 = r2 >> r1 = 20 >> 5 (should be 0)
    instr = {5'h4, 5'd11, 5'd2, 5'd1, 12'd0};
    write_instr(addr, instr); addr = addr + 4;
    
    // 13. addi r12, 1024      ; set r12 = 1024 for shifting test (opcode 0x19)
    instr = {5'h19, 5'd12, 5'd0, 5'd0, 12'd1024};
    write_instr(addr, instr); addr = addr + 4;
    
    // 14. shftri r12, 2       ; opcode 0x5, r12 = 1024 >> 2 = 256
    instr = {5'h5, 5'd12, 5'd0, 5'd0, 12'd2};
    write_instr(addr, instr); addr = addr + 4;
    
    // 15. shftl r13, r1, r2   ; opcode 0x6, r13 = r1 << r2 = 5 << 20
    instr = {5'h6, 5'd13, 5'd1, 5'd2, 12'd0};
    write_instr(addr, instr); addr = addr + 4;
    
    // 16. addi r14, 1         ; set r14 = 1 (for shift left immediate)
    instr = {5'h19, 5'd14, 5'd0, 5'd0, 12'd1};
    write_instr(addr, instr); addr = addr + 4;
    
    // 17. shftli r14, 3       ; opcode 0x7, r14 = 1 << 3 = 8
    instr = {5'h7, 5'd14, 5'd0, 5'd0, 12'd3};
    write_instr(addr, instr); addr = addr + 4;
    
    // 18. mov r15, r1         ; opcode 0x11, r15 = r1 = 5
    instr = {5'h11, 5'd15, 5'd1, 5'd0, 12'd0};
    write_instr(addr, instr); addr = addr + 4;
    
    // 19. mov r16, L          ; opcode 0x12, r16 gets literal 123 in lower 12 bits
    instr = {5'h12, 5'd16, 5'd0, 5'd0, 12'd123};
    write_instr(addr, instr); addr = addr + 4;
    
    // ----- FPU Instructions (assumes registers r1 and r2 contain 5 and 20, interpreted as doubles) -----
    // 20. addf r17, r1, r2     ; opcode 0x14, r17 = 5.0 + 20.0 = 25.0
    instr = {5'h14, 5'd17, 5'd1, 5'd2, 12'd0};
    write_instr(addr, instr); addr = addr + 4;
    
    // 21. subf r18, r2, r1     ; opcode 0x15, r18 = 20.0 - 5.0 = 15.0
    instr = {5'h15, 5'd18, 5'd2, 5'd1, 12'd0};
    write_instr(addr, instr); addr = addr + 4;
    
    // 22. mulf r19, r1, r2     ; opcode 0x16, r19 = 5.0 * 20.0 = 100.0
    instr = {5'h16, 5'd19, 5'd1, 5'd2, 12'd0};
    write_instr(addr, instr); addr = addr + 4;
    
    // 23. divf r20, r2, r1     ; opcode 0x17, r20 = 20.0 / 5.0 = 4.0
    instr = {5'h17, 5'd20, 5'd2, 5'd1, 12'd0};
    write_instr(addr, instr); addr = addr + 4;
    
    // ----- Data Movement Instructions -----
    // 24. mov (r22)(L), r21    ; opcode 0x13, store r21 into memory at (r22 + 0)
    // First, set r21 = 789.
    instr = {5'h19, 5'd21, 5'd0, 5'd0, 12'd789};
    write_instr(addr, instr); addr = addr + 4;
    // Use r22 as base address for store; later we will load from that location.
    // For this test, we will assume r22 already holds an address. So we set r22 = 256.
    instr = {5'h19, 5'd22, 5'd0, 5'd0, 12'd256};
    write_instr(addr, instr); addr = addr + 4;
    // Now store: mov (r22)(L) with L = 0.
    instr = {5'h13, 5'd22, 5'd21, 5'd0, 12'd0};
    write_instr(addr, instr); addr = addr + 4;
    // 25. mov r23, (r22)(L)   ; opcode 0x10, load from memory at address in r22 + 0 into r23.
    instr = {5'h10, 5'd23, 5'd22, 5'd0, 12'd0};
    write_instr(addr, instr); addr = addr + 4;
    
    // ----- Branch Instructions (for coverage, we “neutralize” the branch targets) -----
    // For these tests we set branch target registers equal to PC+4 so that the branch becomes a no-op.
    // 26. (br) opcode 0x8: br r25, so set r25 = (current PC + 4).
    // For test purposes, load r25 with 0 (we cannot load full PC, so we expect the branch to be “neutral”).
    instr = {5'h19, 5'd25, 5'd0, 5'd0, 12'd0};
    write_instr(addr, instr); addr = addr + 4;
    // Now br r25 (opcode 0x8): PC = register[r25]
    instr = {5'h8, 5'd25, 5'd0, 5'd0, 12'd0};
    write_instr(addr, instr); addr = addr + 4;
    
    // 27. brr r26: opcode 0x9, PC = PC + register[r26]. Set r26 = 4.
    instr = {5'h19, 5'd26, 5'd0, 5'd0, 12'd4};
    write_instr(addr, instr); addr = addr + 4;
    instr = {5'h9, 5'd26, 5'd0, 5'd0, 12'd0};
    write_instr(addr, instr); addr = addr + 4;
    
    // 28. brr L: opcode 0xA, PC = PC + sign-extended L; use L = 4.
    instr = {5'hA, 5'd0, 5'd0, 5'd0, 12'd4};
    write_instr(addr, instr); addr = addr + 4;
    
    // 29. brnz r27, r1: opcode 0xB. Since r1 (5) is nonzero, branch to register[r27].
    // For neutrality, set r27 = 0.
    instr = {5'h19, 5'd27, 5'd0, 5'd0, 12'd0};
    write_instr(addr, instr); addr = addr + 4;
    instr = {5'hB, 5'd27, 5'd1, 5'd0, 12'd0};
    write_instr(addr, instr); addr = addr + 4;
    
    // 30. call r28, r29, r30: opcode 0xC.
    // (Call writes return address to memory using r31; here we simply include the instruction.)
    instr = {5'hC, 5'd28, 5'd29, 5'd30, 12'd0};
    write_instr(addr, instr); addr = addr + 4;
    
    // 31. return: opcode 0xD.
    instr = {5'hD, 5'd0, 5'd0, 5'd0, 12'd0};
    write_instr(addr, instr); addr = addr + 4;
    
    // 32. brgt r29, r1, r2: opcode 0xE.
    // Since 5 is not greater than 20, the condition is false and PC should advance normally.
    instr = {5'hE, 5'd29, 5'd1, 5'd2, 12'd0};
    write_instr(addr, instr); addr = addr + 4;
    
    // Let the simulation run long enough for all instructions to be executed.
    #1000;
    
    // Display register values (accessing hierarchical register file for verification)
    $display("===== Final Register Values =====");
    $display("r1  (addi):       %d", uut.reg_file.registers[1]);
    $display("r2  (addi):       %d", uut.reg_file.registers[2]);
    $display("r3  (add):        %d", uut.reg_file.registers[3]);
    $display("r4  (sub):        %d", uut.reg_file.registers[4]);
    $display("r5  (mul):        %d", uut.reg_file.registers[5]);
    $display("r6  (div):        %d", uut.reg_file.registers[6]);
    $display("r7  (and):        %d", uut.reg_file.registers[7]);
    $display("r8  (or):         %d", uut.reg_file.registers[8]);
    $display("r9  (xor):        %d", uut.reg_file.registers[9]);
    $display("r10 (not):        %h", uut.reg_file.registers[10]);
    $display("r11 (shftr):      %d", uut.reg_file.registers[11]);
    $display("r12 (shftri):     %d", uut.reg_file.registers[12]);
    $display("r13 (shftl):      %d", uut.reg_file.registers[13]);
    $display("r14 (shftli):     %d", uut.reg_file.registers[14]);
    $display("r15 (mov r,rs):   %d", uut.reg_file.registers[15]);
    $display("r16 (mov rd,L):   %d", uut.reg_file.registers[16]);
    $display("r17 (addf):       %h", uut.reg_file.registers[17]);
    $display("r18 (subf):       %h", uut.reg_file.registers[18]);
    $display("r19 (mulf):       %h", uut.reg_file.registers[19]);
    $display("r20 (divf):       %h", uut.reg_file.registers[20]);
    $display("r21 (data imm):   %d", uut.reg_file.registers[21]);
    $display("r22 (store base): %d", uut.reg_file.registers[22]);
    $display("r23 (data load):  %d", uut.reg_file.registers[23]);
    $display("r24 (branch test):%d", uut.reg_file.registers[24]);
    $display("r25 (br target):  %d", uut.reg_file.registers[25]);
    $display("r26 (brr add):    %d", uut.reg_file.registers[26]);
    $display("r27 (brnz target):%d", uut.reg_file.registers[27]);
    $display("r28 (call target):%d", uut.reg_file.registers[28]);
    $display("r29 (brgt target):%d", uut.reg_file.registers[29]);
    $display("r30:              %d", uut.reg_file.registers[30]);
    $display("r31 (init value): %h", uut.reg_file.registers[31]);
    
    $finish;
  end

endmodule
