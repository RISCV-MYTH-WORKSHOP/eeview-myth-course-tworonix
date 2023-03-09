\m4_TLV_version 1d: tl-x.org
\SV
   // This code can be found in: https://github.com/stevehoover/RISC-V_MYTH_Workshop
   
   m4_include_lib(['https://raw.githubusercontent.com/stevehoover/RISC-V_MYTH_Workshop/ecba3769fff373ef6b8f66b3347e8940c859792d/tlv_lib/calculator_shell_lib.tlv'])

\SV
   m4_makerchip_module   // (Expanded in Nav-TLV pane.)

\TLV
   // Tomasz Woroniecki 07 March 2023
   |calc
      @0
         $reset = *reset;
      @1   
         $valid[0] = $reset ? 1'b0 : >>1$valid[0] + 1'b1;
         $valid_or_reset = $valid || $reset;
      ?$valid_or_reset
         @1
            $val1[31:0] = >>2$out[31:0];
            $val2[31:0] = $rand2[3:0];

            $sum[31:0]  = $val1[31:0] + $val2[31:0];
            $diff[31:0] = $val1[31:0] - $val2[31:0];
            $prod[31:0] = $val1[31:0] * $val2[31:0];
            $quot[31:0] = $val1[31:0] / $val2[31:0];
            
            $op[2:0] = $rand3[2:0];  //the op should be here together with other inputs?

         @2
            $out[31:0] =
                  $reset ? 32'b0 :
                  ($op[2:0] == 3'b000) ? $sum :
                  ($op[2:0] == 3'b001) ? $diff :
                  ($op[2:0] == 3'b010) ? $prod :
                  ($op[2:0] == 3'b011) ? $quot :
                  ($op[2:0] == 3'b100) ? $recall :
                  $RETAIN; // when mem operation or undefined - hold value

            $mem[31:0] = $reset ? 0 :
                  ($op[2:0] == 3'b101) ? >>2$out
                  : $RETAIN;
            $recall[31:0] = >>2$mem;

   m4+cal_viz(@4) // Arg: Pipeline stage represented by viz, should be atleast equal to last stage of CALCULATOR logic.     
   
   // Assert these to end simulation (before Makerchip cycle limit).
   *passed = *cyc_cnt > 40;
   *failed = 1'b0;
   

\SV
   endmodule
