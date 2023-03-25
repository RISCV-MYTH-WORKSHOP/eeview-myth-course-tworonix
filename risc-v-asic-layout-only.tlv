\m4_TLV_version 1d -p verilog --bestsv --noline: tl-x.org

// Only for layout with SiliconCompiler!
// Instruction decoding is incorrect - ?== (don't care x bits comparison) had to be removed to pass through their compiler
// Plus some other hacks to get it to work

\SV
   // This code can be found in: https://github.com/stevehoover/RISC-V_MYTH_Workshop
   
   m4_include_lib(['https://raw.githubusercontent.com/BalaDhinesh/RISC-V_MYTH_Workshop/master/tlv_lib/risc-v_shell_lib.tlv'])

\SV
  // m4_makerchip_module   // (Expanded in Nav-TLV pane.)
//  module riscv(input clk, input reset, input [31:0] idata0, idata1, idata2, idata3, idata4, idata5, idata6, idata7, idata8, idata9, idata10, idata11, idata12, idata13, idata14, idata15, idata16, idata17, idata18, idata19, idata20, idata21, idata22, idata23, idata24, idata25, idata26, idata27, idata28, idata29, idata30, idata31, output reg [31:0] reg0, reg1, reg2, reg3, reg4, reg5, reg6, reg7, reg8, reg9, reg10, reg11, reg12, reg13, reg14, reg15, reg16, reg17, reg18, reg19, reg20, reg21, reg22, reg23, reg24, reg25, reg26, reg27, reg28, reg29, reg30, reg31);

//  module riscv(input clk, input reset, output reg [31:0] reg0, reg1);
module riscv(input wire clk, input wire reset, input wire [31:0] cyc_cnt, output wire passed, output wire failed, input wire mem_ext, input wire [31:0] imem_rd_data, output wire [31:0] imem_rd_addr, input wire [31:0] dmem_rd_data, output wire [1:0] dmem_mode, output wire [31:0] dmem_addr, output wire dmem_wr_en);



//\SV_plus
   // The program in an instruction memory.
   logic [31:0] instrs [0:16-1];
   //logic [40*8-1:0] instr_strs [0:16];
   assign instrs[0] = {12'b1011, 5'd0, 3'b110, 5'd10, 7'b0010011}; assign instrs[1] = {12'b1001, 5'd0, 3'b110, 5'd11, 7'b0010011}; assign instrs[2] = {1'b0, 10'b0000001010, 1'b0, 8'b00000000, 5'd1, 7'b1101111}; assign instrs[3] = {12'b1100011, 5'd0, 3'b000, 5'd5, 7'b0010011}; assign instrs[4] = {1'b0, 6'b000000, 5'd5, 5'd10, 3'b000, 4'b0100, 1'b0, 7'b1100011}; assign instrs[5] = {7'b0000000, 5'd0, 5'd6, 3'b000, 5'd0, 7'b0110011}; assign instrs[6] = {7'b0000000, 5'd0, 5'd7, 3'b000, 5'd0, 7'b0110011}; assign instrs[7] = {12'b000000000000, 5'd0, 3'b110, 5'd5, 7'b0010011}; assign instrs[8] = {12'b000000000001, 5'd11, 3'b111, 5'd6, 7'b0010011}; assign instrs[9] = {6'b010000, 6'b1, 5'd11, 3'b101, 5'd11, 7'b0010011}; assign instrs[10] = {1'b0, 6'b000000, 5'd0, 5'd6, 3'b000, 4'b0100, 1'b0, 7'b1100011}; assign instrs[11] = {7'b0000000, 5'd10, 5'd5, 3'b000, 5'd5, 7'b0110011}; assign instrs[12] = {6'b000000, 6'b1, 5'd10, 3'b001, 5'd10, 7'b0010011}; assign instrs[13] = {1'b1, 6'b111111, 5'd0, 5'd11, 3'b001, 4'b0110, 1'b1, 7'b1100011}; assign instrs[14] = {12'b000000000000, 5'd5, 3'b000, 5'd10, 7'b0010011}; assign instrs[15] = {12'b000000000000, 5'd1, 3'b000, 5'd0, 7'b1100111}; 


\TLV

   // /====================\
   // | Sum 1 to 9 Program |
   // \====================/
   //
   // Program for MYTH Workshop to test RV32I
   // Add 1,2,3,...,9 (in that order).
   //
   // Regs:
   //  r10 (a0): In: 0, Out: final sum
   //  r12 (a2): 10
   //  r13 (a3): 1..10
   //  r14 (a4): Sum
   // 
   // External to function:
//   m4_asm(ADD, r10, r0, r0)             // Initialize r10 (a0) to 0.
   // Function:
//   m4_asm(ADD, r14, r10, r0)            // Initialize sum register a4 with 0x0
//   m4_asm(ADDI, r12, r10, 1010)         // Store count of 10 in register a2.
//   m4_asm(ADD, r13, r10, r0)            // Initialize intermediate sum register a3 with 0
   // Loop:
//   m4_asm(ADD, r14, r13, r14)           // Incremental addition
//   m4_asm(ADDI, r13, r13, 1)            // Increment intermediate register by 1
//   m4_asm(BLT, r13, r12, 1111111111000) // If a3 is less than a2, branch to label named <loop>
//   m4_asm(ADD, r10, r14, r0)            // Store final result to register a0 so that it can be read by main program
   
   // Optional:
   // m4_asm(JAL, r7, 00000000000000000000) // Done. Jump to itself (infinite loop). (Up to 20-bit signed immediate plus implicit 0 bit (unlike JALR) provides byte address; last immediate bit should also be 0)
  // m4_define_hier(['M4_IMEM'], M4_NUM_INSTRS)
     m4_define_hier(['M4_IMEM'], 2)

   |cpu
      @0
         $reset = *reset;
         
         //extra inputs/outputs
         *imem_rd_addr = $imem_rd_addr[31:0];
         // end extra

         $valid_taken_br_debug = >>3$valid_taken_br;
         $br_tgt_pc_debug[31:0] = >>3$br_tgt_pc;
         `BOGUS_USE($valid_taken_br_debug $br_tgt_pc_debug);
         
         $pc[31:0] = >>3$reset ? 0 :
                     >>3$valid_taken_br ? >>3$br_tgt_pc :
                     >>3$valid_load ? >>3$inc_pc :
                     >>3$valid_jump ? >>3$jalr_tgt_pc :
                     >>1$inc_pc[31:0];
         
         $imem_rd_en = ! $reset;
         //$imem_rd_addr[M4_IMEM_INDEX_CNT-1:0] = $pc[M4_IMEM_INDEX_CNT+1:2];
         $imem_rd_addr[31:0] = $pc[31:2]; //TODO errors in vivado 
         
         //TODO
         //$ld_data[31:0] = $reset ? 32'b0 : >>1$ld_data[31:0];
      @1
         $inc_pc[31:0] = $pc + 4;
         $instr[31:0] = *mem_ext ? *imem_rd_data : $imem_rd_data[31:0];
         
         $is_i_instr = $instr[6:2] == 5'b00001 ||
                       $instr[6:2] == 5'b00110 ||
                       $instr[6:2] == 5'b11001 ||
                       $instr[6:2] == 5'b11100;
         $is_r_instr = $instr[6:2] == 5'b01110 ||
                       $instr[6:2] == 5'b01011 ||
                       $instr[6:2] == 5'b10100;
         $is_s_instr = $instr[6:2] == 5'b01001;
         $is_b_instr = $instr[6:2] == 5'b11000;
         $is_j_instr = $instr[6:2] == 5'b11011;
         $is_u_instr = $instr[6:2] == 5'b01101;
         
         $imm[31:0] = $is_i_instr ? { {21{$instr[31]}}, $instr[30:20] } :
                      $is_s_instr ? { {21{$instr[31]}}, $instr[30:25], $instr[11:8], $instr[7] } :
                      $is_b_instr ? { {20{$instr[31]}}, $instr[7], $instr[30:25], $instr[11:8], 1'b0 } :
                      $is_u_instr ? { $instr[31:12], 12'b0 } :
                      $is_j_instr ? { {12{$instr[31]}}, $instr[19:12], $instr[20], $instr[30:21], 1'b0 } :
                      32'b0;
         
         //slide 11
         //$funct7_valid = $is_r_instr;
         //?$funct7_valid
         $funct7[6:0] = $instr[31:25];
         $funct3_valid = $is_r_instr || $is_i_instr || $is_s_instr || $is_b_instr;
         ?$funct3_valid
            $funct3[2:0] = $instr[14:12];
         $rs1_valid = $funct3_valid;
         //?$rs1_valid
         $rs1[4:0] = $instr[19:15];
         $rs2_valid = $is_r_instr || $is_s_instr || $is_b_instr;
         //?$rs2_valid
         $rs2[4:0] = $instr[24:20];
         $rd_valid = $is_r_instr || $is_i_instr || $is_u_instr || $is_j_instr;
         //?$rd_valid
         $rd[4:0] = $instr[11:7];
         $opcode[6:0] = $instr[6:0];
         
         $dec_bits[10:0] = {$funct7[5], $funct3, $opcode};
         $is_beq  = $dec_bits == 11'b1_000_1100011;
         $is_bne  = $dec_bits == 11'b1_001_1100011;
         $is_blt  = $dec_bits == 11'b1_100_1100011;
         $is_bge  = $dec_bits == 11'b1_101_1100011;
         $is_bltu = $dec_bits == 11'b1_110_1100011;
         $is_bgeu = $dec_bits == 11'b1_111_1100011;
         $is_addi = $dec_bits == 11'b1_000_0010011;
         $is_add  = $dec_bits == 11'b0_000_0110011;
         //slide 44
         $is_lui  = $dec_bits == 11'b1_111_0110111;
         $is_auipc = $dec_bits == 11'b1_111_0010111;
         $is_jal  = $dec_bits == 11'b1_111_1101111;
         $is_jalr = $dec_bits == 11'b1_000_1100111;
         $is_load = $dec_bits == 11'b1_111_0000011; // all loads
         $is_sb   = $dec_bits == 11'b1_000_0100011;
         $is_sh   = $dec_bits == 11'b1_001_0100011;
         $is_sw   = $dec_bits == 11'b1_010_0100011;
         //addi above
         $is_slti = $dec_bits == 11'b1_010_0010011;
         $is_sltiu = $dec_bits == 11'b1_011_0010011;
         $is_xori = $dec_bits == 11'b1_100_0010011;
         $is_ori  = $dec_bits == 11'b1_110_0010011;
         $is_andi = $dec_bits == 11'b1_111_0010011;
         $is_slli = $dec_bits == 11'b0_001_0010011;
         $is_srli = $dec_bits == 11'b0_101_0010011;
         $is_srai = $dec_bits == 11'b1_101_0010011;
         //add already above
         $is_sub  = $dec_bits == 11'b1_000_0110011;
         $is_sll  = $dec_bits == 11'b0_001_0110011;
         $is_slt  = $dec_bits == 11'b0_010_0110011;
         $is_sltu = $dec_bits == 11'b0_011_0110011;
         $is_xor  = $dec_bits == 11'b0_100_0110011;
         $is_srl  = $dec_bits == 11'b0_101_0110011;
         $is_sra  = $dec_bits == 11'b1_101_0110011;
         $is_or   = $dec_bits == 11'b0_110_0110011;
         $is_and  = $dec_bits == 11'b0_111_0110011;
         $is_jump = $is_jal || $is_jalr;
         
      @2   
         // register file
         $rf_rd_en1 = $rs1_valid;
         ?$rs1_valid
            $rf_rd_index1[4:0] = $rs1;
         $rf_rd_en2 = $rs2_valid;
         ?$rs2_valid
            $rf_rd_index2[4:0] = $rs2;
         
         // branches, address only         
         $br_tgt_pc[31:0] = $pc + $imm;
      @3   
         // connect alu slide 17, plus register bypass slide 39
         $src1_value[31:0] = >>1$rf_wr_en && >>1$rd == $rs1 ?
                              >>1$result : $rf_rd_data1;
         $src2_value[31:0] = >>1$rf_wr_en && >>1$rd == $rs2 ?
                              >>1$result : $rf_rd_data2;
      
         $sltu_rslt  = $src1_value < $src2_value;
         $sltiu_rslt = $src1_value < $imm;
         
         $result[31:0] = ($is_addi || $is_load || $is_s_instr) ? $src1_value + $imm :
                         $is_add ? $src1_value + $src2_value :
                         //slide 45
                         $is_andi ? $src1_value & $imm :
                         $is_ori  ? $src1_value | $imm :
                         $is_xori ? $src1_value ^ $imm :
                         $is_slli ? $src1_value << $imm[5:0] :
                         $is_srli ? $src1_value >> $imm[5:0] :
                         $is_and  ? $src1_value & $src2_value :
                         $is_or   ? $src1_value | $src2_value :
                         $is_xor  ? $src1_value ^ $src2_value :
                         $is_sub  ? $src1_value - $src2_value :
                         $is_sll  ? $src1_value << $src2_value[4:0] :
                         $is_srl  ? $src1_value >> $src2_value[4:0] :
                         $is_sltu ? $sltu_rslt :
                         $is_sltiu ? $sltiu_rslt :
                         $is_lui  ? {$imm[31:12], 12'b0} :
                         $is_auipc ? $pc + $imm :
                         $is_jal  ? $pc + 4 :
                         $is_jalr ? $pc + 4 :
                         // signed operations, not native to verilog
                         $is_srai ? { {32{$src1_value[31]}}, $src1_value} >> $imm[4:0] :
                         $is_slt  ? (($src1_value[31] == $src2_value[31]) ? $sltu_rslt : {31'b0,$src1_value[31]}) :
                         $is_slti ? (($src1_value[31] == $imm[31]) ? $sltiu_rslt : {31'b0,$src1_value[31]}) :
                         $is_sra  ? { {32{$src1_value[31]}}, $src1_value} >> $src2_value[4:0] :
                         32'bx;
         
         `BOGUS_USE($is_sw $is_sh $is_sb)
         //register file write, slide 20, plus data load slide 49
         $rf_wr_en = ($rd_valid && $rd != 0 && $valid) ||  //register index zero is read-only
                     (>>2$is_load && >>2$valid);
         //?$rd_valid                   
         $rf_wr_data[31:0] = $valid ? $result : >>2$ld_data[31:0];
         $rf_wr_index[4:0] = $valid ? $rd : >>2$rd;
         
         //branches, slide 21
         $taken_br = $is_beq  ?  $src1_value == $src2_value :
                     $is_bne  ?  $src1_value != $src2_value :
                     $is_blt  ? ($src1_value <  $src2_value) ^ ($src1_value[31] != $src2_value[31]) :
                     $is_bge  ? ($src1_value >= $src2_value) ^ ($src1_value[31] != $src2_value[31]) :
                     $is_bltu ?  $src1_value <  $src2_value :
                     $is_bgeu ?  $src1_value >= $src2_value :
                     $is_jal  ?  1'b1 :
                     1'b0;
         $valid_taken_br = $valid && $taken_br;
         $valid_jump = $valid && $is_jump;
         $valid_load = $valid && $is_load;
         //taken branch invalidates the pipeline, slide 42
         $valid = (! >>1$valid_taken_br) && (! >>2$valid_taken_br) &&
                  (! >>1$valid_load) && (! >>2$valid_load) &&   // TODO should it check $valid load
                  (! >>1$valid_jump) && (! >>2$valid_jump);  //slide 53
         
         $jalr_tgt_pc[31:0] = $src1_value + $imm;  // slide 53
         
         //$debug3_blt_taken = ($src1_value <  $src2_value) ^ ($src1_value[31] != $src2_value[31]);
         //$debug3_is_blt = $is_blt;
         
      @4
         // data memory, slide 51
         $dmem_wr_en = $is_s_instr && $valid;
         $dmem_addr[3:0] = $result[5:2];
         $dmem_wr_data[31:0] = $src2_value;
         $dmem_rd_en = $is_load;
         
         //extra inputs/outputs
         *dmem_addr = $dmem_addr;
         *dmem_wr_en = $dmem_wr_en;
         //*dmem_mode = $dmem_mode;
         //end extra
         
      @5
         $ld_data[31:0] = *mem_ext ? *dmem_rd_data : $dmem_rd_data[31:0];
         
//paste end


      // YOUR CODE HERE
      // ...

      // Note: Because of the magic we are using for visualisation, if visualisation is enabled below,
      //       be sure to avoid having unassigned signals (which you might be using for random inputs)
      //       other than those specifically expected in the labs. You'll get strange errors for these.

   
   // Assert these to end simulation (before Makerchip cycle limit).
   //*passed = *cyc_cnt > 40;
   //*failed = 1'b0;
   
   // Macro instantiations for:
   //  o instruction memory
   //  o register file
   //  o data memory
   //  o CPU visualization
   |cpu
      m4+imem(@1)    // Args: (read stage)
      m4+rf(@2, @3)  // Args: (read stage, write stage) - if equal, no register bypass is required
      m4+dmem(@4)    // Args: (read/write stage)
  //    m4+myth_fpga(@0)  // Uncomment to run on fpga

   //m4+cpu_viz(@4)    // For visualisation, argument should be at least equal to the last stage of CPU logic. @4 would work for all labs.
\SV
   endmodule

