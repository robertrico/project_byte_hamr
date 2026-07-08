// GENERATED FILE - do not edit.
// Source: ~/Development/intel-8008-vhdl @ 2ff4659, entity b8008_top
// Cmd: ghdl --synth --std=08 --out=verilog -gCLK_FREQ_HZ=25000000 b8008_top
// Personality: defaults (ROM 4KB @ 0x0000, RAM 12KB @ 0x1000, monitor map)
module io_buffer_Brtl
  (input  [7:0] external_data_in,
   output [7:0] external_data_out,
   output external_data_oe,
   input  [7:0] internal_bus_in,
   output [7:0] internal_bus_out,
   output internal_bus_oe,
   input  enable,
   input  direction);
  wire n3658;
  wire n3659;
  wire n3660;
  assign external_data_out = internal_bus_in; //(module output)
  assign external_data_oe = n3660; //(module output)
  assign internal_bus_out = external_data_in; //(module output)
  assign internal_bus_oe = n3659; //(module output)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/io_buffer.vhdl:48:37  */
  assign n3658 = ~direction;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/io_buffer.vhdl:48:32  */
  assign n3659 = enable & n3658;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/io_buffer.vhdl:52:33  */
  assign n3660 = enable & direction;
endmodule

module instruction_register_Brtl
  (input  clk,
   input  phi1_falling,
   input  reset,
   input  [7:0] internal_bus_in,
   output [7:0] internal_bus_out,
   output internal_bus_oe,
   input  load_ir,
   input  output_ir,
   output ir_bit_7,
   output ir_bit_6,
   output ir_bit_5,
   output ir_bit_4,
   output ir_bit_3,
   output ir_bit_2,
   output ir_bit_1,
   output ir_bit_0);
  reg [7:0] ir;
  wire n3638;
  wire n3644;
  wire n3645;
  wire n3646;
  wire n3647;
  wire n3648;
  wire n3649;
  wire n3650;
  wire n3651;
  wire [7:0] n3652;
  reg [7:0] n3653;
  assign internal_bus_out = ir; //(module output)
  assign internal_bus_oe = output_ir; //(module output)
  assign ir_bit_7 = n3644; //(module output)
  assign ir_bit_6 = n3645; //(module output)
  assign ir_bit_5 = n3646; //(module output)
  assign ir_bit_4 = n3647; //(module output)
  assign ir_bit_3 = n3648; //(module output)
  assign ir_bit_2 = n3649; //(module output)
  assign ir_bit_1 = n3650; //(module output)
  assign ir_bit_0 = n3651; //(module output)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_register.vhdl:55:12  */
  always @*
    ir = n3653; // (isignal)
  initial
    ir = 8'b00000000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_register.vhdl:66:35  */
  assign n3638 = load_ir & phi1_falling;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_register.vhdl:78:19  */
  assign n3644 = ir[7]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_register.vhdl:79:19  */
  assign n3645 = ir[6]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_register.vhdl:80:19  */
  assign n3646 = ir[5]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_register.vhdl:81:19  */
  assign n3647 = ir[4]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_register.vhdl:82:19  */
  assign n3648 = ir[3]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_register.vhdl:83:19  */
  assign n3649 = ir[2]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_register.vhdl:84:19  */
  assign n3650 = ir[1]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_register.vhdl:85:19  */
  assign n3651 = ir[0]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_register.vhdl:65:9  */
  assign n3652 = n3638 ? internal_bus_in : ir;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_register.vhdl:65:9  */
  always @(posedge clk or posedge reset)
    if (reset)
      n3653 <= 8'b00000000;
    else
      n3653 <= n3652;
endmodule

module condition_flags_Brtl
  (input  clk,
   input  phi2_rising,
   input  reset,
   input  flag_carry_in,
   input  flag_zero_in,
   input  flag_sign_in,
   input  flag_parity_in,
   input  update_flags,
   input  carry_only,
   input  [1:0] condition_code,
   input  test_true,
   input  eval_condition,
   input  output_flags,
   output [7:0] internal_bus_out,
   output internal_bus_oe,
   output condition_met,
   output flag_carry,
   output flag_zero,
   output flag_sign,
   output flag_parity);
  reg carry_ff;
  reg zero_ff;
  reg sign_ff;
  reg parity_ff;
  wire n3566;
  wire n3567;
  wire n3572;
  wire n3573;
  wire n3574;
  wire [4:0] n3589;
  wire [5:0] n3590;
  wire [6:0] n3591;
  wire [7:0] n3592;
  wire n3598;
  wire n3600;
  wire n3602;
  wire n3604;
  wire [3:0] n3605;
  reg n3607;
  wire n3608;
  wire n3609;
  wire n3612;
  wire n3616;
  reg n3617;
  wire n3618;
  reg n3619;
  wire n3620;
  reg n3621;
  wire n3622;
  reg n3623;
  assign internal_bus_out = n3592; //(module output)
  assign internal_bus_oe = output_flags; //(module output)
  assign condition_met = n3612; //(module output)
  assign flag_carry = carry_ff; //(module output)
  assign flag_zero = zero_ff; //(module output)
  assign flag_sign = sign_ff; //(module output)
  assign flag_parity = parity_ff; //(module output)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/condition_flags.vhdl:82:12  */
  always @*
    carry_ff = n3617; // (isignal)
  initial
    carry_ff = 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/condition_flags.vhdl:83:12  */
  always @*
    zero_ff = n3619; // (isignal)
  initial
    zero_ff = 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/condition_flags.vhdl:84:12  */
  always @*
    sign_ff = n3621; // (isignal)
  initial
    sign_ff = 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/condition_flags.vhdl:85:12  */
  always @*
    parity_ff = n3623; // (isignal)
  initial
    parity_ff = 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/condition_flags.vhdl:104:34  */
  assign n3566 = update_flags & phi2_rising;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/condition_flags.vhdl:106:31  */
  assign n3567 = ~carry_only;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/condition_flags.vhdl:104:13  */
  assign n3572 = n3567 & n3566;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/condition_flags.vhdl:104:13  */
  assign n3573 = n3567 & n3566;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/condition_flags.vhdl:104:13  */
  assign n3574 = n3567 & n3566;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/condition_flags.vhdl:128:32  */
  assign n3589 = {4'b0000, parity_ff};
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/condition_flags.vhdl:128:44  */
  assign n3590 = {n3589, sign_ff};
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/condition_flags.vhdl:128:54  */
  assign n3591 = {n3590, zero_ff};
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/condition_flags.vhdl:128:64  */
  assign n3592 = {n3591, carry_ff};
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/condition_flags.vhdl:144:17  */
  assign n3598 = condition_code == 2'b00;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/condition_flags.vhdl:147:17  */
  assign n3600 = condition_code == 2'b01;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/condition_flags.vhdl:150:17  */
  assign n3602 = condition_code == 2'b10;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/condition_flags.vhdl:153:17  */
  assign n3604 = condition_code == 2'b11;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/condition_flags.vhdl:143:13  */
  assign n3605 = {n3604, n3602, n3600, n3598};
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/condition_flags.vhdl:143:13  */
  always @*
    case (n3605)
      4'b1000: n3607 = parity_ff;
      4'b0100: n3607 = sign_ff;
      4'b0010: n3607 = zero_ff;
      4'b0001: n3607 = carry_ff;
      default: n3607 = 1'b0;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/condition_flags.vhdl:164:37  */
  assign n3608 = ~n3607;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/condition_flags.vhdl:161:13  */
  assign n3609 = test_true ? n3607 : n3608;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/condition_flags.vhdl:140:9  */
  assign n3612 = eval_condition ? n3609 : 1'b1;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/condition_flags.vhdl:103:9  */
  assign n3616 = n3566 ? flag_carry_in : carry_ff;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/condition_flags.vhdl:103:9  */
  always @(posedge clk or posedge reset)
    if (reset)
      n3617 <= 1'b0;
    else
      n3617 <= n3616;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/condition_flags.vhdl:103:9  */
  assign n3618 = n3572 ? flag_zero_in : zero_ff;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/condition_flags.vhdl:103:9  */
  always @(posedge clk or posedge reset)
    if (reset)
      n3619 <= 1'b0;
    else
      n3619 <= n3618;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/condition_flags.vhdl:103:9  */
  assign n3620 = n3573 ? flag_sign_in : sign_ff;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/condition_flags.vhdl:103:9  */
  always @(posedge clk or posedge reset)
    if (reset)
      n3621 <= 1'b0;
    else
      n3621 <= n3620;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/condition_flags.vhdl:103:9  */
  assign n3622 = n3574 ? flag_parity_in : parity_ff;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/condition_flags.vhdl:103:9  */
  always @(posedge clk or posedge reset)
    if (reset)
      n3623 <= 1'b0;
    else
      n3623 <= n3622;
endmodule

module alu_Brtl
  (input  clk,
   input  phi2_rising,
   input  [7:0] accumulator_in,
   input  [7:0] reg_b_in,
   input  [2:0] opcode,
   input  is_inr_dcr,
   input  is_rotate,
   input  carry_in,
   input  enable,
   input  output_result,
   output [7:0] internal_bus_out,
   output internal_bus_oe,
   output [8:0] result,
   output flag_carry,
   output flag_zero,
   output flag_sign,
   output flag_parity);
  reg [8:0] result_latched;
  reg enable_prev;
  wire [8:0] result_internal;
  wire n3388;
  wire n3389;
  wire n3392;
  localparam [8:0] n3393 = 9'b000000000;
  wire [7:0] n3394;
  wire n3395;
  wire [6:0] n3396;
  wire n3397;
  wire n3399;
  wire n3400;
  wire [6:0] n3401;
  wire n3402;
  wire n3404;
  wire n3405;
  wire [6:0] n3406;
  wire n3408;
  wire n3409;
  wire [6:0] n3410;
  wire n3412;
  wire [3:0] n3413;
  wire n3414;
  wire n3415;
  reg n3417;
  wire [5:0] n3418;
  wire [5:0] n3419;
  wire [5:0] n3420;
  wire [5:0] n3421;
  reg [5:0] n3423;
  wire n3424;
  wire n3425;
  reg n3427;
  reg n3429;
  wire [7:0] n3430;
  wire [7:0] n3432;
  wire [8:0] n3434;
  wire [8:0] n3436;
  wire [8:0] n3437;
  wire n3439;
  wire [8:0] n3441;
  wire [8:0] n3443;
  wire [8:0] n3444;
  wire [8:0] n3445;
  wire [8:0] n3446;
  wire n3448;
  wire [8:0] n3450;
  wire [8:0] n3452;
  wire [8:0] n3453;
  wire n3455;
  wire [8:0] n3457;
  wire [8:0] n3459;
  wire [8:0] n3460;
  wire [8:0] n3461;
  wire [8:0] n3462;
  wire n3464;
  wire [7:0] n3465;
  wire [8:0] n3467;
  wire n3469;
  wire [7:0] n3470;
  wire [8:0] n3472;
  wire n3474;
  wire [7:0] n3475;
  wire [8:0] n3477;
  wire n3479;
  wire [8:0] n3481;
  wire [8:0] n3483;
  wire [8:0] n3484;
  wire n3486;
  wire [7:0] n3487;
  reg [8:0] n3489;
  wire [8:0] n3490;
  wire [8:0] n3491;
  wire n3500;
  wire [8:0] n3512;
  wire [7:0] n3514;
  wire n3515;
  wire n3516;
  wire n3517;
  wire n3518;
  wire [7:0] n3521;
  wire n3523;
  wire n3524;
  wire n3525;
  wire n3527;
  wire n3528;
  wire n3530;
  wire n3531;
  wire n3532;
  wire n3533;
  wire n3534;
  wire n3535;
  wire n3536;
  wire n3537;
  wire n3538;
  wire n3539;
  wire n3540;
  wire n3541;
  wire n3542;
  wire n3543;
  wire n3544;
  wire n3545;
  wire n3546;
  wire [8:0] n3548;
  reg [8:0] n3549;
  wire n3550;
  reg n3551;
  assign internal_bus_out = n3514; //(module output)
  assign internal_bus_oe = output_result; //(module output)
  assign result = result_internal; //(module output)
  assign flag_carry = n3516; //(module output)
  assign flag_zero = n3525; //(module output)
  assign flag_sign = n3528; //(module output)
  assign flag_parity = n3546; //(module output)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:99:12  */
  always @*
    result_latched = n3549; // (isignal)
  initial
    result_latched = 9'b000000000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:102:12  */
  always @*
    enable_prev = n3551; // (isignal)
  initial
    enable_prev = 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:105:12  */
  assign result_internal = n3512; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:121:45  */
  assign n3388 = ~enable_prev;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:121:29  */
  assign n3389 = n3388 & enable;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:124:17  */
  assign n3392 = carry_in ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:112:18  */
  assign n3394 = n3393[8:1]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:134:61  */
  assign n3395 = accumulator_in[7]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:135:70  */
  assign n3396 = accumulator_in[6:0]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:136:61  */
  assign n3397 = accumulator_in[7]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:132:25  */
  assign n3399 = opcode == 3'b000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:140:61  */
  assign n3400 = accumulator_in[0]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:141:70  */
  assign n3401 = accumulator_in[7:1]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:142:61  */
  assign n3402 = accumulator_in[0]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:138:25  */
  assign n3404 = opcode == 3'b001;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:146:61  */
  assign n3405 = accumulator_in[7]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:147:70  */
  assign n3406 = accumulator_in[6:0]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:144:25  */
  assign n3408 = opcode == 3'b010;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:152:61  */
  assign n3409 = accumulator_in[0]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:153:70  */
  assign n3410 = accumulator_in[7:1]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:150:25  */
  assign n3412 = opcode == 3'b011;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:131:21  */
  assign n3413 = {n3412, n3408, n3404, n3399};
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:141:70  */
  assign n3414 = n3401[0]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:153:70  */
  assign n3415 = n3410[0]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:131:21  */
  always @*
    case (n3413)
      4'b1000: n3417 = n3415;
      4'b0100: n3417 = carry_in;
      4'b0010: n3417 = n3414;
      4'b0001: n3417 = n3397;
      default: n3417 = 1'b0;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:135:70  */
  assign n3418 = n3396[5:0]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:141:70  */
  assign n3419 = n3401[6:1]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:147:70  */
  assign n3420 = n3406[5:0]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:153:70  */
  assign n3421 = n3410[6:1]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:131:21  */
  always @*
    case (n3413)
      4'b1000: n3423 = n3421;
      4'b0100: n3423 = n3420;
      4'b0010: n3423 = n3419;
      4'b0001: n3423 = n3418;
      default: n3423 = 6'b000000;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:135:70  */
  assign n3424 = n3396[6]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:147:70  */
  assign n3425 = n3406[6]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:131:21  */
  always @*
    case (n3413)
      4'b1000: n3427 = carry_in;
      4'b0100: n3427 = n3425;
      4'b0010: n3427 = n3402;
      4'b0001: n3427 = n3424;
      default: n3427 = 1'b0;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:131:21  */
  always @*
    case (n3413)
      4'b1000: n3429 = n3409;
      4'b0100: n3429 = n3405;
      4'b0010: n3429 = n3400;
      4'b0001: n3429 = n3395;
      default: n3429 = 1'b0;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:161:21  */
  assign n3430 = is_inr_dcr ? reg_b_in : accumulator_in;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:161:21  */
  assign n3432 = is_inr_dcr ? 8'b00000001 : reg_b_in;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:175:74  */
  assign n3434 = {1'b0, n3430};
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:175:101  */
  assign n3436 = {1'b0, n3432};
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:175:86  */
  assign n3437 = n3434 + n3436;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:173:25  */
  assign n3439 = opcode == 3'b000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:178:74  */
  assign n3441 = {1'b0, n3430};
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:178:101  */
  assign n3443 = {1'b0, n3432};
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:178:86  */
  assign n3444 = n3441 + n3443;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:112:18  */
  assign n3445 = {n3394, n3392};
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:178:113  */
  assign n3446 = n3444 + n3445;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:177:25  */
  assign n3448 = opcode == 3'b001;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:182:74  */
  assign n3450 = {1'b0, n3430};
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:182:101  */
  assign n3452 = {1'b0, n3432};
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:182:86  */
  assign n3453 = n3450 - n3452;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:180:25  */
  assign n3455 = opcode == 3'b010;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:185:74  */
  assign n3457 = {1'b0, n3430};
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:185:101  */
  assign n3459 = {1'b0, n3432};
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:185:86  */
  assign n3460 = n3457 - n3459;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:112:18  */
  assign n3461 = {n3394, n3392};
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:185:113  */
  assign n3462 = n3460 - n3461;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:184:25  */
  assign n3464 = opcode == 3'b011;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:188:60  */
  assign n3465 = n3430 & n3432;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:188:48  */
  assign n3467 = {1'b0, n3465};
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:187:25  */
  assign n3469 = opcode == 3'b100;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:191:60  */
  assign n3470 = n3430 ^ n3432;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:191:48  */
  assign n3472 = {1'b0, n3470};
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:190:25  */
  assign n3474 = opcode == 3'b101;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:194:60  */
  assign n3475 = n3430 | n3432;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:194:48  */
  assign n3477 = {1'b0, n3475};
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:193:25  */
  assign n3479 = opcode == 3'b110;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:198:74  */
  assign n3481 = {1'b0, n3430};
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:198:101  */
  assign n3483 = {1'b0, n3432};
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:198:86  */
  assign n3484 = n3481 - n3483;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:196:25  */
  assign n3486 = opcode == 3'b111;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:172:21  */
  assign n3487 = {n3486, n3479, n3474, n3469, n3464, n3455, n3448, n3439};
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:172:21  */
  always @*
    case (n3487)
      8'b10000000: n3489 = n3484;
      8'b01000000: n3489 = n3477;
      8'b00100000: n3489 = n3472;
      8'b00010000: n3489 = n3467;
      8'b00001000: n3489 = n3462;
      8'b00000100: n3489 = n3453;
      8'b00000010: n3489 = n3446;
      8'b00000001: n3489 = n3437;
      default: n3489 = 9'b000000000;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:129:17  */
  assign n3490 = {n3429, n3427, n3423, n3417};
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:129:17  */
  assign n3491 = is_rotate ? n3490 : n3489;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:117:29  */
  assign n3500 = n3389 & phi2_rising;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:211:39  */
  assign n3512 = enable ? result_latched : 9'b000000000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:217:40  */
  assign n3514 = result_internal[7:0]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:227:56  */
  assign n3515 = is_inr_dcr & enable;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:227:37  */
  assign n3516 = n3515 ? carry_in : n3518;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:228:34  */
  assign n3517 = result_internal[8]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:227:78  */
  assign n3518 = enable ? n3517 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:232:60  */
  assign n3521 = result_internal[7:0]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:232:73  */
  assign n3523 = n3521 == 8'b00000000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:232:41  */
  assign n3524 = n3523 & enable;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:232:22  */
  assign n3525 = n3524 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:235:33  */
  assign n3527 = result_internal[7]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:235:37  */
  assign n3528 = enable ? n3527 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:239:40  */
  assign n3530 = result_internal[0]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:239:63  */
  assign n3531 = result_internal[1]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:239:44  */
  assign n3532 = n3530 ^ n3531;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:239:86  */
  assign n3533 = result_internal[2]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:239:67  */
  assign n3534 = n3532 ^ n3533;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:239:109  */
  assign n3535 = result_internal[3]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:239:90  */
  assign n3536 = n3534 ^ n3535;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:240:40  */
  assign n3537 = result_internal[4]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:239:113  */
  assign n3538 = n3536 ^ n3537;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:240:63  */
  assign n3539 = result_internal[5]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:240:44  */
  assign n3540 = n3538 ^ n3539;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:240:86  */
  assign n3541 = result_internal[6]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:240:67  */
  assign n3542 = n3540 ^ n3541;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:240:109  */
  assign n3543 = result_internal[7]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:240:90  */
  assign n3544 = n3542 ^ n3543;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:239:20  */
  assign n3545 = ~n3544;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:241:20  */
  assign n3546 = enable ? n3545 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:117:9  */
  assign n3548 = n3500 ? n3491 : result_latched;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:117:9  */
  always @(posedge clk)
    n3549 <= n3548;
  initial
    n3549 = 9'b000000000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:117:9  */
  assign n3550 = phi2_rising ? enable : enable_prev;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/alu.vhdl:117:9  */
  always @(posedge clk)
    n3551 <= n3550;
  initial
    n3551 = 1'b0;
endmodule

module temp_registers_Brtl
  (input  clk,
   input  phi2_rising,
   input  reset,
   input  load_reg_a,
   input  load_reg_b,
   input  output_reg_a,
   input  output_reg_b,
   input  [7:0] internal_bus_in,
   output [7:0] internal_bus_out,
   output internal_bus_oe,
   output [7:0] reg_a_out,
   output [7:0] reg_b_out);
  reg [7:0] reg_a;
  reg [7:0] reg_b;
  wire [7:0] n3347;
  wire n3348;
  wire n3352;
  wire n3361;
  wire [7:0] n3367;
  reg [7:0] n3368;
  wire [7:0] n3369;
  reg [7:0] n3370;
  assign internal_bus_out = n3347; //(module output)
  assign internal_bus_oe = n3348; //(module output)
  assign reg_a_out = reg_a; //(module output)
  assign reg_b_out = reg_b; //(module output)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/temp_registers.vhdl:54:12  */
  always @*
    reg_a = n3368; // (isignal)
  initial
    reg_a = 8'b00000000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/temp_registers.vhdl:55:12  */
  always @*
    reg_b = n3370; // (isignal)
  initial
    reg_b = 8'b00000000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/temp_registers.vhdl:65:31  */
  assign n3347 = output_reg_a ? reg_a : reg_b;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/temp_registers.vhdl:66:38  */
  assign n3348 = output_reg_a | output_reg_b;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/temp_registers.vhdl:74:34  */
  assign n3352 = load_reg_a & phi2_rising;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/temp_registers.vhdl:87:34  */
  assign n3361 = load_reg_b & phi2_rising;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/temp_registers.vhdl:73:9  */
  assign n3367 = n3352 ? internal_bus_in : reg_a;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/temp_registers.vhdl:73:9  */
  always @(posedge clk or posedge reset)
    if (reset)
      n3368 <= 8'b00000000;
    else
      n3368 <= n3367;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/temp_registers.vhdl:86:9  */
  assign n3369 = n3361 ? internal_bus_in : reg_b;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/temp_registers.vhdl:86:9  */
  always @(posedge clk or posedge reset)
    if (reset)
      n3370 <= 8'b00000000;
    else
      n3370 <= n3369;
endmodule

module register_alu_control_Brtl
  (input  clk,
   input  phi2_rising,
   input  status_s0,
   input  status_s1,
   input  status_s2,
   input  instr_is_alu_op,
   input  instr_uses_temp_regs,
   input  instr_needs_immediate,
   input  instr_writes_reg,
   input  instr_is_write,
   input  instr_is_io,
   input  [1:0] current_cycle,
   input  state_half,
   input  interrupt,
   output load_reg_a,
   output load_reg_b,
   output alu_enable,
   output update_flags,
   output output_reg_a,
   output output_reg_b,
   output output_result,
   output output_flags);
  wire state_is_t2;
  wire state_is_t3;
  wire state_is_t4;
  wire state_is_t5;
  wire n3194;
  wire n3195;
  wire n3196;
  wire n3197;
  wire n3198;
  wire n3201;
  wire n3202;
  wire n3203;
  wire n3204;
  wire n3205;
  wire n3208;
  wire n3209;
  wire n3210;
  wire n3213;
  wire n3214;
  wire n3215;
  wire n3216;
  wire [31:0] n3225;
  wire n3227;
  wire n3228;
  wire n3229;
  wire [31:0] n3231;
  wire n3233;
  wire n3234;
  wire n3235;
  wire n3236;
  wire n3237;
  wire [31:0] n3239;
  wire n3241;
  wire n3242;
  wire n3243;
  wire n3244;
  wire n3245;
  wire n3246;
  wire [31:0] n3256;
  wire n3258;
  wire n3259;
  wire n3260;
  wire n3261;
  wire n3264;
  wire [31:0] n3265;
  wire n3267;
  wire n3268;
  wire n3269;
  wire [31:0] n3270;
  wire n3272;
  wire n3273;
  wire n3274;
  wire n3275;
  wire n3276;
  wire n3279;
  wire n3280;
  wire [31:0] n3281;
  wire n3283;
  wire n3284;
  wire n3285;
  wire [31:0] n3286;
  wire n3288;
  wire n3289;
  wire n3290;
  wire n3291;
  wire n3292;
  localparam n3300 = 1'b0;
  wire [31:0] n3302;
  wire n3304;
  wire n3305;
  wire n3306;
  wire n3307;
  wire n3308;
  wire n3309;
  wire n3310;
  wire n3311;
  wire n3312;
  wire [31:0] n3314;
  wire n3316;
  wire n3317;
  wire n3318;
  wire n3319;
  wire n3320;
  wire [31:0] n3322;
  wire n3324;
  wire n3325;
  wire n3326;
  wire n3327;
  wire n3328;
  wire n3329;
  wire n3330;
  wire [31:0] n3332;
  wire n3334;
  wire n3335;
  wire n3336;
  wire n3337;
  wire n3339;
  localparam n3340 = 1'b0;
  assign load_reg_a = n3261; //(module output)
  assign load_reg_b = n3229; //(module output)
  assign alu_enable = n3276; //(module output)
  assign update_flags = n3292; //(module output)
  assign output_reg_a = n3300; //(module output)
  assign output_reg_b = n3312; //(module output)
  assign output_result = n3339; //(module output)
  assign output_flags = n3340; //(module output)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:74:12  */
  assign state_is_t2 = n3198; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:75:12  */
  assign state_is_t3 = n3205; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:76:12  */
  assign state_is_t4 = n3210; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:77:12  */
  assign state_is_t5 = n3216; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:84:61  */
  assign n3194 = ~status_s1;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:84:47  */
  assign n3195 = n3194 & status_s2;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:84:81  */
  assign n3196 = ~status_s0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:84:67  */
  assign n3197 = n3196 & n3195;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:84:25  */
  assign n3198 = n3197 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:85:41  */
  assign n3201 = ~status_s2;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:85:61  */
  assign n3202 = ~status_s1;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:85:47  */
  assign n3203 = n3202 & n3201;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:85:67  */
  assign n3204 = status_s0 & n3203;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:85:25  */
  assign n3205 = n3204 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:86:47  */
  assign n3208 = status_s1 & status_s2;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:86:67  */
  assign n3209 = status_s0 & n3208;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:86:25  */
  assign n3210 = n3209 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:87:61  */
  assign n3213 = ~status_s1;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:87:47  */
  assign n3214 = n3213 & status_s2;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:87:67  */
  assign n3215 = status_s0 & n3214;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:87:25  */
  assign n3216 = n3215 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:113:65  */
  assign n3225 = {30'b0, current_cycle};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:113:65  */
  assign n3227 = n3225 == 32'b00000000000000000000000000000000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:113:47  */
  assign n3228 = n3227 & state_is_t3;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:113:23  */
  assign n3229 = n3228 ? 1'b1 : n3237;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:114:65  */
  assign n3231 = {30'b0, current_cycle};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:114:65  */
  assign n3233 = n3231 == 32'b00000000000000000000000000000001;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:114:47  */
  assign n3234 = n3233 & state_is_t3;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:114:101  */
  assign n3235 = instr_uses_temp_regs | instr_needs_immediate;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:114:69  */
  assign n3236 = n3235 & n3234;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:113:70  */
  assign n3237 = n3236 ? 1'b1 : n3246;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:115:65  */
  assign n3239 = {30'b0, current_cycle};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:115:65  */
  assign n3241 = n3239 == 32'b00000000000000000000000000000000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:115:47  */
  assign n3242 = n3241 & state_is_t4;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:115:69  */
  assign n3243 = instr_uses_temp_regs & n3242;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:115:126  */
  assign n3244 = ~instr_needs_immediate;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:115:100  */
  assign n3245 = n3244 & n3243;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:114:134  */
  assign n3246 = n3245 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:136:65  */
  assign n3256 = {30'b0, current_cycle};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:136:65  */
  assign n3258 = n3256 == 32'b00000000000000000000000000000010;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:136:47  */
  assign n3259 = n3258 & state_is_t3;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:136:69  */
  assign n3260 = instr_uses_temp_regs & n3259;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:136:23  */
  assign n3261 = n3260 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:144:47  */
  assign n3264 = instr_is_alu_op & state_is_t5;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:145:45  */
  assign n3265 = {30'b0, current_cycle};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:145:45  */
  assign n3267 = n3265 == 32'b00000000000000000000000000000000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:145:75  */
  assign n3268 = ~instr_needs_immediate;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:145:49  */
  assign n3269 = n3268 & n3267;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:146:45  */
  assign n3270 = {30'b0, current_cycle};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:146:45  */
  assign n3272 = n3270 == 32'b00000000000000000000000000000001;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:146:49  */
  assign n3273 = instr_needs_immediate & n3272;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:145:82  */
  assign n3274 = n3269 | n3273;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:144:73  */
  assign n3275 = n3274 & n3264;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:144:23  */
  assign n3276 = n3275 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:152:49  */
  assign n3279 = state_half & state_is_t5;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:152:70  */
  assign n3280 = instr_is_alu_op & n3279;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:153:47  */
  assign n3281 = {30'b0, current_cycle};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:153:47  */
  assign n3283 = n3281 == 32'b00000000000000000000000000000000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:153:77  */
  assign n3284 = ~instr_needs_immediate;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:153:51  */
  assign n3285 = n3284 & n3283;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:154:47  */
  assign n3286 = {30'b0, current_cycle};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:154:47  */
  assign n3288 = n3286 == 32'b00000000000000000000000000000001;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:154:51  */
  assign n3289 = instr_needs_immediate & n3288;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:153:84  */
  assign n3290 = n3285 | n3289;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:152:96  */
  assign n3291 = n3290 & n3280;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:152:25  */
  assign n3292 = n3291 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:184:68  */
  assign n3302 = {30'b0, current_cycle};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:184:68  */
  assign n3304 = n3302 == 32'b00000000000000000000000000000000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:184:50  */
  assign n3305 = n3304 & state_is_t5;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:184:72  */
  assign n3306 = instr_writes_reg & n3305;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:184:99  */
  assign n3307 = instr_uses_temp_regs & n3306;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:185:48  */
  assign n3308 = ~instr_is_alu_op;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:184:130  */
  assign n3309 = n3308 & n3307;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:185:80  */
  assign n3310 = ~instr_needs_immediate;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:185:54  */
  assign n3311 = n3310 & n3309;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:184:26  */
  assign n3312 = n3311 ? 1'b1 : n3320;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:186:68  */
  assign n3314 = {30'b0, current_cycle};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:186:68  */
  assign n3316 = n3314 == 32'b00000000000000000000000000000010;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:186:50  */
  assign n3317 = n3316 & state_is_t3;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:186:72  */
  assign n3318 = instr_is_write & n3317;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:186:97  */
  assign n3319 = instr_needs_immediate & n3318;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:185:87  */
  assign n3320 = n3319 ? 1'b1 : n3330;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:187:68  */
  assign n3322 = {30'b0, current_cycle};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:187:68  */
  assign n3324 = n3322 == 32'b00000000000000000000000000000001;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:187:50  */
  assign n3325 = n3324 & state_is_t5;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:187:72  */
  assign n3326 = instr_writes_reg & n3325;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:188:48  */
  assign n3327 = ~instr_is_alu_op;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:187:99  */
  assign n3328 = n3327 & n3326;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:188:54  */
  assign n3329 = instr_needs_immediate & n3328;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:186:130  */
  assign n3330 = n3329 ? 1'b1 : n3337;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:189:68  */
  assign n3332 = {30'b0, current_cycle};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:189:68  */
  assign n3334 = n3332 == 32'b00000000000000000000000000000001;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:189:50  */
  assign n3335 = n3334 & state_is_t2;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:189:72  */
  assign n3336 = instr_is_io & n3335;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:188:87  */
  assign n3337 = n3336 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_alu_control.vhdl:194:34  */
  assign n3339 = state_is_t5 & instr_is_alu_op;
endmodule

module register_file_Brtl
  (input  clk,
   input  phi2_rising,
   input  reset,
   input  [7:0] data_in,
   output [7:0] data_out,
   input  enable_a,
   input  enable_b,
   input  enable_c,
   input  enable_d,
   input  enable_e,
   input  enable_h,
   input  enable_l,
   input  read_enable,
   input  write_enable,
   output [7:0] accumulator_out,
   output [7:0] debug_reg_a,
   output [7:0] debug_reg_b,
   output [7:0] debug_reg_c,
   output [7:0] debug_reg_d,
   output [7:0] debug_reg_e,
   output [7:0] debug_reg_h,
   output [7:0] debug_reg_l);
  reg [7:0] reg_a;
  reg [7:0] reg_b;
  reg [7:0] reg_c;
  reg [7:0] reg_d;
  reg [7:0] reg_e;
  reg [7:0] reg_h;
  reg [7:0] reg_l;
  wire n3112;
  wire n3120;
  wire n3121;
  wire n3122;
  wire n3123;
  wire n3124;
  wire n3125;
  wire n3126;
  wire n3149;
  wire [7:0] n3150;
  wire n3151;
  wire [7:0] n3152;
  wire n3153;
  wire [7:0] n3154;
  wire n3155;
  wire [7:0] n3156;
  wire n3157;
  wire [7:0] n3158;
  wire n3159;
  wire [7:0] n3160;
  wire n3161;
  wire [7:0] n3162;
  wire [7:0] n3164;
  reg [7:0] n3165;
  wire [7:0] n3166;
  reg [7:0] n3167;
  wire [7:0] n3168;
  reg [7:0] n3169;
  wire [7:0] n3170;
  reg [7:0] n3171;
  wire [7:0] n3172;
  reg [7:0] n3173;
  wire [7:0] n3174;
  reg [7:0] n3175;
  wire [7:0] n3176;
  reg [7:0] n3177;
  assign data_out = n3150; //(module output)
  assign accumulator_out = reg_a; //(module output)
  assign debug_reg_a = reg_a; //(module output)
  assign debug_reg_b = reg_b; //(module output)
  assign debug_reg_c = reg_c; //(module output)
  assign debug_reg_d = reg_d; //(module output)
  assign debug_reg_e = reg_e; //(module output)
  assign debug_reg_h = reg_h; //(module output)
  assign debug_reg_l = reg_l; //(module output)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_file.vhdl:70:12  */
  always @*
    reg_a = n3165; // (isignal)
  initial
    reg_a = 8'b00000000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_file.vhdl:71:12  */
  always @*
    reg_b = n3167; // (isignal)
  initial
    reg_b = 8'b00000000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_file.vhdl:72:12  */
  always @*
    reg_c = n3169; // (isignal)
  initial
    reg_c = 8'b00000000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_file.vhdl:73:12  */
  always @*
    reg_d = n3171; // (isignal)
  initial
    reg_d = 8'b00000000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_file.vhdl:74:12  */
  always @*
    reg_e = n3173; // (isignal)
  initial
    reg_e = 8'b00000000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_file.vhdl:75:12  */
  always @*
    reg_h = n3175; // (isignal)
  initial
    reg_h = 8'b00000000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_file.vhdl:76:12  */
  always @*
    reg_l = n3177; // (isignal)
  initial
    reg_l = 8'b00000000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_file.vhdl:92:34  */
  assign n3112 = write_enable & phi2_rising;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_file.vhdl:92:13  */
  assign n3120 = enable_a & n3112;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_file.vhdl:92:13  */
  assign n3121 = enable_b & n3112;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_file.vhdl:92:13  */
  assign n3122 = enable_c & n3112;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_file.vhdl:92:13  */
  assign n3123 = enable_d & n3112;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_file.vhdl:92:13  */
  assign n3124 = enable_e & n3112;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_file.vhdl:92:13  */
  assign n3125 = enable_h & n3112;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_file.vhdl:92:13  */
  assign n3126 = enable_l & n3112;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_file.vhdl:114:47  */
  assign n3149 = enable_a & read_enable;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_file.vhdl:114:23  */
  assign n3150 = n3149 ? reg_a : n3152;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_file.vhdl:115:47  */
  assign n3151 = enable_b & read_enable;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_file.vhdl:114:67  */
  assign n3152 = n3151 ? reg_b : n3154;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_file.vhdl:116:47  */
  assign n3153 = enable_c & read_enable;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_file.vhdl:115:67  */
  assign n3154 = n3153 ? reg_c : n3156;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_file.vhdl:117:47  */
  assign n3155 = enable_d & read_enable;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_file.vhdl:116:67  */
  assign n3156 = n3155 ? reg_d : n3158;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_file.vhdl:118:47  */
  assign n3157 = enable_e & read_enable;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_file.vhdl:117:67  */
  assign n3158 = n3157 ? reg_e : n3160;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_file.vhdl:119:47  */
  assign n3159 = enable_h & read_enable;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_file.vhdl:118:67  */
  assign n3160 = n3159 ? reg_h : n3162;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_file.vhdl:120:47  */
  assign n3161 = enable_l & read_enable;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_file.vhdl:119:67  */
  assign n3162 = n3161 ? reg_l : 8'b00000000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_file.vhdl:91:9  */
  assign n3164 = n3120 ? data_in : reg_a;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_file.vhdl:91:9  */
  always @(posedge clk or posedge reset)
    if (reset)
      n3165 <= 8'b00000000;
    else
      n3165 <= n3164;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_file.vhdl:91:9  */
  assign n3166 = n3121 ? data_in : reg_b;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_file.vhdl:91:9  */
  always @(posedge clk or posedge reset)
    if (reset)
      n3167 <= 8'b00000000;
    else
      n3167 <= n3166;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_file.vhdl:91:9  */
  assign n3168 = n3122 ? data_in : reg_c;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_file.vhdl:91:9  */
  always @(posedge clk or posedge reset)
    if (reset)
      n3169 <= 8'b00000000;
    else
      n3169 <= n3168;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_file.vhdl:91:9  */
  assign n3170 = n3123 ? data_in : reg_d;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_file.vhdl:91:9  */
  always @(posedge clk or posedge reset)
    if (reset)
      n3171 <= 8'b00000000;
    else
      n3171 <= n3170;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_file.vhdl:91:9  */
  assign n3172 = n3124 ? data_in : reg_e;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_file.vhdl:91:9  */
  always @(posedge clk or posedge reset)
    if (reset)
      n3173 <= 8'b00000000;
    else
      n3173 <= n3172;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_file.vhdl:91:9  */
  assign n3174 = n3125 ? data_in : reg_h;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_file.vhdl:91:9  */
  always @(posedge clk or posedge reset)
    if (reset)
      n3175 <= 8'b00000000;
    else
      n3175 <= n3174;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_file.vhdl:91:9  */
  assign n3176 = n3126 ? data_in : reg_l;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/register_file.vhdl:91:9  */
  always @(posedge clk or posedge reset)
    if (reset)
      n3177 <= 8'b00000000;
    else
      n3177 <= n3176;
endmodule

module scratchpad_decoder_Brtl
  (input  [2:0] addr_in,
   input  read_enable,
   input  write_enable,
   output enable_a,
   output enable_b,
   output enable_c,
   output enable_d,
   output enable_e,
   output enable_h,
   output enable_l,
   output enable_m,
   output read_out,
   output write_out);
  wire n3026;
  wire n3028;
  wire n3030;
  wire n3032;
  wire n3034;
  wire n3036;
  wire n3038;
  wire n3040;
  wire n3042;
  wire [7:0] n3043;
  reg n3046;
  reg n3049;
  reg n3052;
  reg n3055;
  reg n3058;
  reg n3061;
  reg n3064;
  reg n3067;
  wire n3069;
  wire n3072;
  wire n3075;
  wire n3078;
  wire n3081;
  wire n3084;
  wire n3087;
  wire n3090;
  assign enable_a = n3069; //(module output)
  assign enable_b = n3072; //(module output)
  assign enable_c = n3075; //(module output)
  assign enable_d = n3078; //(module output)
  assign enable_e = n3081; //(module output)
  assign enable_h = n3084; //(module output)
  assign enable_l = n3087; //(module output)
  assign enable_m = n3090; //(module output)
  assign read_out = read_enable; //(module output)
  assign write_out = write_enable; //(module output)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/scratchpad_decoder.vhdl:67:30  */
  assign n3026 = read_enable | write_enable;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/scratchpad_decoder.vhdl:69:17  */
  assign n3028 = addr_in == 3'b000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/scratchpad_decoder.vhdl:70:17  */
  assign n3030 = addr_in == 3'b001;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/scratchpad_decoder.vhdl:71:17  */
  assign n3032 = addr_in == 3'b010;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/scratchpad_decoder.vhdl:72:17  */
  assign n3034 = addr_in == 3'b011;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/scratchpad_decoder.vhdl:73:17  */
  assign n3036 = addr_in == 3'b100;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/scratchpad_decoder.vhdl:74:17  */
  assign n3038 = addr_in == 3'b101;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/scratchpad_decoder.vhdl:75:17  */
  assign n3040 = addr_in == 3'b110;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/scratchpad_decoder.vhdl:76:17  */
  assign n3042 = addr_in == 3'b111;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/scratchpad_decoder.vhdl:68:13  */
  assign n3043 = {n3042, n3040, n3038, n3036, n3034, n3032, n3030, n3028};
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/scratchpad_decoder.vhdl:68:13  */
  always @*
    case (n3043)
      8'b10000000: n3046 = 1'b0;
      8'b01000000: n3046 = 1'b0;
      8'b00100000: n3046 = 1'b0;
      8'b00010000: n3046 = 1'b0;
      8'b00001000: n3046 = 1'b0;
      8'b00000100: n3046 = 1'b0;
      8'b00000010: n3046 = 1'b0;
      8'b00000001: n3046 = 1'b1;
      default: n3046 = 1'b0;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/scratchpad_decoder.vhdl:68:13  */
  always @*
    case (n3043)
      8'b10000000: n3049 = 1'b0;
      8'b01000000: n3049 = 1'b0;
      8'b00100000: n3049 = 1'b0;
      8'b00010000: n3049 = 1'b0;
      8'b00001000: n3049 = 1'b0;
      8'b00000100: n3049 = 1'b0;
      8'b00000010: n3049 = 1'b1;
      8'b00000001: n3049 = 1'b0;
      default: n3049 = 1'b0;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/scratchpad_decoder.vhdl:68:13  */
  always @*
    case (n3043)
      8'b10000000: n3052 = 1'b0;
      8'b01000000: n3052 = 1'b0;
      8'b00100000: n3052 = 1'b0;
      8'b00010000: n3052 = 1'b0;
      8'b00001000: n3052 = 1'b0;
      8'b00000100: n3052 = 1'b1;
      8'b00000010: n3052 = 1'b0;
      8'b00000001: n3052 = 1'b0;
      default: n3052 = 1'b0;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/scratchpad_decoder.vhdl:68:13  */
  always @*
    case (n3043)
      8'b10000000: n3055 = 1'b0;
      8'b01000000: n3055 = 1'b0;
      8'b00100000: n3055 = 1'b0;
      8'b00010000: n3055 = 1'b0;
      8'b00001000: n3055 = 1'b1;
      8'b00000100: n3055 = 1'b0;
      8'b00000010: n3055 = 1'b0;
      8'b00000001: n3055 = 1'b0;
      default: n3055 = 1'b0;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/scratchpad_decoder.vhdl:68:13  */
  always @*
    case (n3043)
      8'b10000000: n3058 = 1'b0;
      8'b01000000: n3058 = 1'b0;
      8'b00100000: n3058 = 1'b0;
      8'b00010000: n3058 = 1'b1;
      8'b00001000: n3058 = 1'b0;
      8'b00000100: n3058 = 1'b0;
      8'b00000010: n3058 = 1'b0;
      8'b00000001: n3058 = 1'b0;
      default: n3058 = 1'b0;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/scratchpad_decoder.vhdl:68:13  */
  always @*
    case (n3043)
      8'b10000000: n3061 = 1'b0;
      8'b01000000: n3061 = 1'b0;
      8'b00100000: n3061 = 1'b1;
      8'b00010000: n3061 = 1'b0;
      8'b00001000: n3061 = 1'b0;
      8'b00000100: n3061 = 1'b0;
      8'b00000010: n3061 = 1'b0;
      8'b00000001: n3061 = 1'b0;
      default: n3061 = 1'b0;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/scratchpad_decoder.vhdl:68:13  */
  always @*
    case (n3043)
      8'b10000000: n3064 = 1'b0;
      8'b01000000: n3064 = 1'b1;
      8'b00100000: n3064 = 1'b0;
      8'b00010000: n3064 = 1'b0;
      8'b00001000: n3064 = 1'b0;
      8'b00000100: n3064 = 1'b0;
      8'b00000010: n3064 = 1'b0;
      8'b00000001: n3064 = 1'b0;
      default: n3064 = 1'b0;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/scratchpad_decoder.vhdl:68:13  */
  always @*
    case (n3043)
      8'b10000000: n3067 = 1'b1;
      8'b01000000: n3067 = 1'b0;
      8'b00100000: n3067 = 1'b0;
      8'b00010000: n3067 = 1'b0;
      8'b00001000: n3067 = 1'b0;
      8'b00000100: n3067 = 1'b0;
      8'b00000010: n3067 = 1'b0;
      8'b00000001: n3067 = 1'b0;
      default: n3067 = 1'b0;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/scratchpad_decoder.vhdl:67:9  */
  assign n3069 = n3026 ? n3046 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/scratchpad_decoder.vhdl:67:9  */
  assign n3072 = n3026 ? n3049 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/scratchpad_decoder.vhdl:67:9  */
  assign n3075 = n3026 ? n3052 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/scratchpad_decoder.vhdl:67:9  */
  assign n3078 = n3026 ? n3055 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/scratchpad_decoder.vhdl:67:9  */
  assign n3081 = n3026 ? n3058 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/scratchpad_decoder.vhdl:67:9  */
  assign n3084 = n3026 ? n3061 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/scratchpad_decoder.vhdl:67:9  */
  assign n3087 = n3026 ? n3064 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/scratchpad_decoder.vhdl:67:9  */
  assign n3090 = n3026 ? n3067 : 1'b0;
endmodule

module stack_memory_Brtl
  (input  clk,
   input  phi1_rising,
   input  reset,
   input  [2:0] sp_in,
   input  \control[increment_lower] ,
   input  \control[increment_upper] ,
   input  \control[load] ,
   input  \control[hold] ,
   input  [13:0] data_in,
   output [13:0] addr_out,
   output carry_out);
  wire [3:0] n2822;
  reg [111:0] stack;
  reg carry_flag;
  wire [13:0] slot;
  wire [13:0] slot_incremented_lower;
  wire [13:0] slot_incremented_upper;
  wire next_carry;
  wire [2:0] n2829;
  wire [7:0] n2836;
  wire [5:0] n2837;
  wire [7:0] n2839;
  wire [13:0] n2840;
  wire [5:0] n2842;
  wire [13:0] n2843;
  wire n2845;
  wire n2848;
  wire n2850;
  wire [13:0] n2851;
  wire n2852;
  wire [13:0] n2853;
  wire n2854;
  wire [13:0] n2855;
  wire n2856;
  wire n2857;
  wire n2863;
  wire [2:0] n2865;
  wire n2868;
  wire [2:0] n2870;
  wire n2873;
  wire [2:0] n2875;
  wire [7:0] n2878;
  wire n2880;
  wire n2883;
  wire [111:0] n2884;
  wire n2885;
  wire [111:0] n2886;
  wire n2888;
  wire [111:0] n2889;
  wire n2891;
  wire [111:0] n2904;
  reg [111:0] n2905;
  wire n2906;
  reg n2907;
  wire [13:0] n2908;
  wire n2909;
  wire n2910;
  wire n2911;
  wire n2912;
  wire n2913;
  wire n2914;
  wire n2915;
  wire n2916;
  wire n2917;
  wire n2918;
  wire n2919;
  wire n2920;
  wire n2921;
  wire n2922;
  wire n2923;
  wire n2924;
  wire n2925;
  wire n2926;
  wire [13:0] n2927;
  wire [13:0] n2928;
  wire [13:0] n2929;
  wire [13:0] n2930;
  wire [13:0] n2931;
  wire [13:0] n2932;
  wire [13:0] n2933;
  wire [13:0] n2934;
  wire [13:0] n2935;
  wire [13:0] n2936;
  wire [13:0] n2937;
  wire [13:0] n2938;
  wire [13:0] n2939;
  wire [13:0] n2940;
  wire [13:0] n2941;
  wire [13:0] n2942;
  wire [111:0] n2943;
  wire n2944;
  wire n2945;
  wire n2946;
  wire n2947;
  wire n2948;
  wire n2949;
  wire n2950;
  wire n2951;
  wire n2952;
  wire n2953;
  wire n2954;
  wire n2955;
  wire n2956;
  wire n2957;
  wire n2958;
  wire n2959;
  wire n2960;
  wire n2961;
  wire [13:0] n2962;
  wire [13:0] n2963;
  wire [13:0] n2964;
  wire [13:0] n2965;
  wire [13:0] n2966;
  wire [13:0] n2967;
  wire [13:0] n2968;
  wire [13:0] n2969;
  wire [13:0] n2970;
  wire [13:0] n2971;
  wire [13:0] n2972;
  wire [13:0] n2973;
  wire [13:0] n2974;
  wire [13:0] n2975;
  wire [13:0] n2976;
  wire [13:0] n2977;
  wire [111:0] n2978;
  wire n2979;
  wire n2980;
  wire n2981;
  wire n2982;
  wire n2983;
  wire n2984;
  wire n2985;
  wire n2986;
  wire n2987;
  wire n2988;
  wire n2989;
  wire n2990;
  wire n2991;
  wire n2992;
  wire n2993;
  wire n2994;
  wire n2995;
  wire n2996;
  wire [13:0] n2997;
  wire [13:0] n2998;
  wire [13:0] n2999;
  wire [13:0] n3000;
  wire [13:0] n3001;
  wire [13:0] n3002;
  wire [13:0] n3003;
  wire [13:0] n3004;
  wire [13:0] n3005;
  wire [13:0] n3006;
  wire [13:0] n3007;
  wire [13:0] n3008;
  wire [13:0] n3009;
  wire [13:0] n3010;
  wire [13:0] n3011;
  wire [13:0] n3012;
  wire [111:0] n3013;
  assign addr_out = n2851; //(module output)
  assign carry_out = n2857; //(module output)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:32:8  */
  assign n2822 = {\control[hold] , \control[load] , \control[increment_upper] , \control[increment_lower] };
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:64:12  */
  always @*
    stack = n2905; // (isignal)
  initial
    stack = 112'b0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:67:12  */
  always @*
    carry_flag = n2907; // (isignal)
  initial
    carry_flag = 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:69:12  */
  assign slot = n2908; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:70:12  */
  assign slot_incremented_lower = n2840; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:71:12  */
  assign slot_incremented_upper = n2843; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:72:12  */
  assign next_carry = n2848; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:77:19  */
  assign n2829 = 3'b111 - sp_in;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:84:25  */
  assign n2836 = slot[7:0]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:85:25  */
  assign n2837 = slot[13:8]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:87:56  */
  assign n2839 = n2836 + 8'b00000001;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:87:44  */
  assign n2840 = {n2837, n2839};
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:88:45  */
  assign n2842 = n2837 + 6'b000001;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:88:50  */
  assign n2843 = {n2842, n2836};
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:90:21  */
  assign n2845 = n2836 == 8'b11111111;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:90:9  */
  assign n2848 = n2845 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:98:53  */
  assign n2850 = n2822[2]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:98:40  */
  assign n2851 = n2850 ? data_in : n2853;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:99:53  */
  assign n2852 = n2822[1]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:98:64  */
  assign n2853 = n2852 ? slot_incremented_upper : n2855;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:100:53  */
  assign n2854 = n2822[0]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:99:75  */
  assign n2855 = n2854 ? slot_incremented_lower : slot;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:103:42  */
  assign n2856 = n2822[0]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:103:29  */
  assign n2857 = n2856 ? next_carry : carry_flag;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:115:28  */
  assign n2863 = n2822[2]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:116:27  */
  assign n2865 = 3'b111 - sp_in;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:119:31  */
  assign n2868 = n2822[1]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:120:27  */
  assign n2870 = 3'b111 - sp_in;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:123:31  */
  assign n2873 = n2822[0]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:124:27  */
  assign n2875 = 3'b111 - sp_in;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:125:37  */
  assign n2878 = slot[7:0]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:125:51  */
  assign n2880 = n2878 == 8'b11111111;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:125:21  */
  assign n2883 = n2880 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:123:17  */
  assign n2884 = n2873 ? n3013 : stack;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:123:17  */
  assign n2885 = n2873 ? n2883 : carry_flag;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:119:17  */
  assign n2886 = n2868 ? n2978 : n2884;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:119:17  */
  assign n2888 = n2868 ? 1'b0 : n2885;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:115:17  */
  assign n2889 = n2863 ? n2943 : n2886;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:115:17  */
  assign n2891 = n2863 ? 1'b0 : n2888;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:112:9  */
  assign n2904 = phi1_rising ? n2889 : stack;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:112:9  */
  always @(posedge clk or posedge reset)
    if (reset)
      n2905 <= 112'b0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000;
    else
      n2905 <= n2904;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:112:9  */
  assign n2906 = phi1_rising ? n2891 : carry_flag;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:112:9  */
  always @(posedge clk or posedge reset)
    if (reset)
      n2907 <= 1'b0;
    else
      n2907 <= n2906;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:77:19  */
  assign n2908 = stack[n2829 * 14 +: 14]; //(Bmux)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:116:21  */
  assign n2909 = n2865[2]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:116:21  */
  assign n2910 = ~n2909;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:116:21  */
  assign n2911 = n2865[1]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:116:21  */
  assign n2912 = ~n2911;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:116:21  */
  assign n2913 = n2910 & n2912;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:116:21  */
  assign n2914 = n2910 & n2911;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:116:21  */
  assign n2915 = n2909 & n2912;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:116:21  */
  assign n2916 = n2909 & n2911;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:116:21  */
  assign n2917 = n2865[0]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:116:21  */
  assign n2918 = ~n2917;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:116:21  */
  assign n2919 = n2913 & n2918;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:116:21  */
  assign n2920 = n2913 & n2917;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:116:21  */
  assign n2921 = n2914 & n2918;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:116:21  */
  assign n2922 = n2914 & n2917;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:116:21  */
  assign n2923 = n2915 & n2918;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:116:21  */
  assign n2924 = n2915 & n2917;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:116:21  */
  assign n2925 = n2916 & n2918;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:116:21  */
  assign n2926 = n2916 & n2917;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:116:21  */
  assign n2927 = stack[13:0]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:116:21  */
  assign n2928 = n2919 ? data_in : n2927;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:116:21  */
  assign n2929 = stack[27:14]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:116:21  */
  assign n2930 = n2920 ? data_in : n2929;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:116:21  */
  assign n2931 = stack[41:28]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:116:21  */
  assign n2932 = n2921 ? data_in : n2931;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:116:21  */
  assign n2933 = stack[55:42]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:116:21  */
  assign n2934 = n2922 ? data_in : n2933;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:116:21  */
  assign n2935 = stack[69:56]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:116:21  */
  assign n2936 = n2923 ? data_in : n2935;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:116:21  */
  assign n2937 = stack[83:70]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:116:21  */
  assign n2938 = n2924 ? data_in : n2937;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:116:21  */
  assign n2939 = stack[97:84]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:116:21  */
  assign n2940 = n2925 ? data_in : n2939;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:116:21  */
  assign n2941 = stack[111:98]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:116:21  */
  assign n2942 = n2926 ? data_in : n2941;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:116:21  */
  assign n2943 = {n2942, n2940, n2938, n2936, n2934, n2932, n2930, n2928};
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:120:21  */
  assign n2944 = n2870[2]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:120:21  */
  assign n2945 = ~n2944;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:120:21  */
  assign n2946 = n2870[1]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:120:21  */
  assign n2947 = ~n2946;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:120:21  */
  assign n2948 = n2945 & n2947;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:120:21  */
  assign n2949 = n2945 & n2946;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:120:21  */
  assign n2950 = n2944 & n2947;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:120:21  */
  assign n2951 = n2944 & n2946;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:120:21  */
  assign n2952 = n2870[0]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:120:21  */
  assign n2953 = ~n2952;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:120:21  */
  assign n2954 = n2948 & n2953;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:120:21  */
  assign n2955 = n2948 & n2952;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:120:21  */
  assign n2956 = n2949 & n2953;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:120:21  */
  assign n2957 = n2949 & n2952;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:120:21  */
  assign n2958 = n2950 & n2953;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:120:21  */
  assign n2959 = n2950 & n2952;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:120:21  */
  assign n2960 = n2951 & n2953;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:120:21  */
  assign n2961 = n2951 & n2952;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:120:21  */
  assign n2962 = stack[13:0]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:120:21  */
  assign n2963 = n2954 ? slot_incremented_upper : n2962;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:120:21  */
  assign n2964 = stack[27:14]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:120:21  */
  assign n2965 = n2955 ? slot_incremented_upper : n2964;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:120:21  */
  assign n2966 = stack[41:28]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:120:21  */
  assign n2967 = n2956 ? slot_incremented_upper : n2966;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:120:21  */
  assign n2968 = stack[55:42]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:120:21  */
  assign n2969 = n2957 ? slot_incremented_upper : n2968;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:120:21  */
  assign n2970 = stack[69:56]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:120:21  */
  assign n2971 = n2958 ? slot_incremented_upper : n2970;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:120:21  */
  assign n2972 = stack[83:70]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:120:21  */
  assign n2973 = n2959 ? slot_incremented_upper : n2972;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:120:21  */
  assign n2974 = stack[97:84]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:120:21  */
  assign n2975 = n2960 ? slot_incremented_upper : n2974;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:120:21  */
  assign n2976 = stack[111:98]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:120:21  */
  assign n2977 = n2961 ? slot_incremented_upper : n2976;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:120:21  */
  assign n2978 = {n2977, n2975, n2973, n2971, n2969, n2967, n2965, n2963};
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:124:21  */
  assign n2979 = n2875[2]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:124:21  */
  assign n2980 = ~n2979;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:124:21  */
  assign n2981 = n2875[1]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:124:21  */
  assign n2982 = ~n2981;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:124:21  */
  assign n2983 = n2980 & n2982;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:124:21  */
  assign n2984 = n2980 & n2981;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:124:21  */
  assign n2985 = n2979 & n2982;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:124:21  */
  assign n2986 = n2979 & n2981;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:124:21  */
  assign n2987 = n2875[0]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:124:21  */
  assign n2988 = ~n2987;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:124:21  */
  assign n2989 = n2983 & n2988;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:124:21  */
  assign n2990 = n2983 & n2987;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:124:21  */
  assign n2991 = n2984 & n2988;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:124:21  */
  assign n2992 = n2984 & n2987;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:124:21  */
  assign n2993 = n2985 & n2988;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:124:21  */
  assign n2994 = n2985 & n2987;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:124:21  */
  assign n2995 = n2986 & n2988;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:124:21  */
  assign n2996 = n2986 & n2987;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:124:21  */
  assign n2997 = stack[13:0]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:124:21  */
  assign n2998 = n2989 ? slot_incremented_lower : n2997;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:124:21  */
  assign n2999 = stack[27:14]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:124:21  */
  assign n3000 = n2990 ? slot_incremented_lower : n2999;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:124:21  */
  assign n3001 = stack[41:28]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:124:21  */
  assign n3002 = n2991 ? slot_incremented_lower : n3001;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:124:21  */
  assign n3003 = stack[55:42]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:124:21  */
  assign n3004 = n2992 ? slot_incremented_lower : n3003;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:124:21  */
  assign n3005 = stack[69:56]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:124:21  */
  assign n3006 = n2993 ? slot_incremented_lower : n3005;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:124:21  */
  assign n3007 = stack[83:70]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:124:21  */
  assign n3008 = n2994 ? slot_incremented_lower : n3007;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:124:21  */
  assign n3009 = stack[97:84]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:124:21  */
  assign n3010 = n2995 ? slot_incremented_lower : n3009;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:124:21  */
  assign n3011 = stack[111:98]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:124:21  */
  assign n3012 = n2996 ? slot_incremented_lower : n3011;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_memory.vhdl:124:21  */
  assign n3013 = {n3012, n3010, n3008, n3006, n3004, n3002, n3000, n2998};
endmodule

module stack_pointer_Brtl
  (input  clk,
   input  phi1_rising,
   input  reset,
   input  stack_push,
   input  stack_pop,
   output [2:0] sp_out);
  reg [2:0] sp;
  wire [2:0] n2810;
  wire [2:0] n2812;
  wire [2:0] n2813;
  wire [2:0] n2814;
  wire [2:0] n2820;
  reg [2:0] n2821;
  assign sp_out = sp; //(module output)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_pointer.vhdl:42:12  */
  always @*
    sp = n2821; // (isignal)
  initial
    sp = 3'b000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_pointer.vhdl:55:30  */
  assign n2810 = sp + 3'b001;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_pointer.vhdl:59:30  */
  assign n2812 = sp - 3'b001;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_pointer.vhdl:57:17  */
  assign n2813 = stack_pop ? n2812 : sp;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_pointer.vhdl:53:17  */
  assign n2814 = stack_push ? n2810 : n2813;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_pointer.vhdl:51:9  */
  assign n2820 = phi1_rising ? n2814 : sp;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/stack_pointer.vhdl:51:9  */
  always @(posedge clk or posedge reset)
    if (reset)
      n2821 <= 3'b000;
    else
      n2821 <= n2820;
endmodule

module mem_mux_refresh_Brtl
  (input  [13:0] pc_addr,
   input  [13:0] stack_addr,
   input  [7:0] reg_a,
   input  [7:0] reg_b,
   input  [2:0] rst_vector,
   input  [7:0] regfile_data_out,
   output [7:0] regfile_data_in,
   input  [7:0] internal_bus_in,
   output [7:0] internal_bus_out,
   output internal_bus_oe,
   input  select_pc,
   input  select_stack,
   input  pc_load_from_regs,
   input  pc_load_from_stack,
   input  pc_load_from_rst,
   input  regfile_to_bus,
   input  bus_to_regfile,
   output [13:0] pc_data_in);
  wire [13:0] n2793;
  wire [10:0] n2795;
  wire [13:0] n2797;
  wire [13:0] n2798;
  wire [5:0] n2799;
  wire [13:0] n2800;
  wire [7:0] n2801;
  assign regfile_data_in = n2801; //(module output)
  assign internal_bus_out = regfile_data_out; //(module output)
  assign internal_bus_oe = regfile_to_bus; //(module output)
  assign pc_data_in = n2793; //(module output)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/mem_mux_refresh.vhdl:69:30  */
  assign n2793 = pc_load_from_stack ? stack_addr : n2798;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/mem_mux_refresh.vhdl:70:37  */
  assign n2795 = {8'b00000000, rst_vector};
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/mem_mux_refresh.vhdl:70:60  */
  assign n2797 = {n2795, 3'b000};
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/mem_mux_refresh.vhdl:69:60  */
  assign n2798 = pc_load_from_rst ? n2797 : n2800;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/mem_mux_refresh.vhdl:71:33  */
  assign n2799 = reg_a[5:0]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/mem_mux_refresh.vhdl:71:46  */
  assign n2800 = {n2799, reg_b};
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/mem_mux_refresh.vhdl:80:40  */
  assign n2801 = bus_to_regfile ? internal_bus_in : 8'b00000000;
endmodule

module ahl_pointer_Brtl
  (input  state_t1,
   input  state_t2,
   input  [1:0] current_cycle,
   input  [1:0] next_cycle,
   input  instr_is_mem_indirect,
   input  instr_needs_address,
   output [2:0] ahl_select,
   output ahl_active);
  wire [1:0] n2766;
  wire [1:0] n2767;
  wire [31:0] n2768;
  wire [31:0] n2769;
  wire n2770;
  wire n2771;
  wire [2:0] n2774;
  wire n2777;
  wire [2:0] n2779;
  wire n2781;
  wire [2:0] n2783;
  wire n2786;
  assign ahl_select = n2783; //(module output)
  assign ahl_active = n2786; //(module output)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/ahl_pointer.vhdl:75:9  */
  assign n2766 = instr_needs_address ? 2'b10 : 2'b01;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/ahl_pointer.vhdl:82:9  */
  assign n2767 = state_t1 ? next_cycle : current_cycle;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/ahl_pointer.vhdl:90:28  */
  assign n2768 = {30'b0, n2767};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/ahl_pointer.vhdl:90:28  */
  assign n2769 = {30'b0, n2766};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/ahl_pointer.vhdl:90:28  */
  assign n2770 = n2768 == n2769;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/ahl_pointer.vhdl:90:39  */
  assign n2771 = instr_is_mem_indirect & n2770;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/ahl_pointer.vhdl:95:13  */
  assign n2774 = state_t2 ? 3'b101 : 3'b000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/ahl_pointer.vhdl:95:13  */
  assign n2777 = state_t2 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/ahl_pointer.vhdl:91:13  */
  assign n2779 = state_t1 ? 3'b110 : n2774;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/ahl_pointer.vhdl:91:13  */
  assign n2781 = state_t1 ? 1'b1 : n2777;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/ahl_pointer.vhdl:90:9  */
  assign n2783 = n2771 ? n2779 : 3'b000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/ahl_pointer.vhdl:90:9  */
  assign n2786 = n2771 ? n2781 : 1'b0;
endmodule

module memory_io_control_Brtl
  (input  clk,
   input  phi1_rising,
   input  reset,
   input  state_t1,
   input  state_t2,
   input  state_t3,
   input  state_t4,
   input  state_t5,
   input  state_t1i,
   input  state_stopped,
   input  state_half,
   input  status_s0,
   input  status_s1,
   input  status_s2,
   input  [1:0] cycle_type,
   input  [1:0] current_cycle,
   input  [1:0] next_cycle,
   input  advance_state,
   input  instr_is_hlt_flag,
   input  instr_needs_immediate,
   input  instr_needs_address,
   input  instr_is_io,
   input  instr_is_write,
   input  [2:0] instr_sss_field,
   input  [2:0] instr_ddd_field,
   input  instr_is_alu,
   input  instr_is_call,
   input  instr_is_ret,
   input  instr_is_rst,
   input  instr_writes_reg,
   input  instr_reads_reg,
   input  instr_is_mem_indirect,
   input  eval_condition,
   input  condition_met,
   input  interrupt_pending,
   input  ready_status,
   output ir_load,
   output ir_output_enable,
   output io_buffer_enable,
   output io_buffer_direction,
   output [2:0] addr_select_sss,
   output [2:0] addr_select_ddd,
   output [2:0] scratchpad_select,
   output scratchpad_read,
   output scratchpad_write,
   output memory_read,
   output memory_write,
   output memory_refresh,
   output regfile_to_bus,
   output bus_to_regfile,
   output select_pc,
   output select_stack,
   output pc_load_from_regs,
   output pc_load_from_stack,
   output pc_load_from_rst,
   output refresh_increment,
   output stack_addr_select,
   output stack_push,
   output stack_pop,
   output stack_read,
   output stack_write,
   output pc_increment_lower,
   output pc_increment_upper,
   input  pc_carry_in,
   input  [7:0] pc_lower_byte,
   output pc_load,
   output pc_hold);
  reg prev_state_t2;
  reg prev_state_t3;
  reg prev_state_t4;
  reg prev_state_t5;
  reg suppress_pc_inc_next_cycle;
  reg ir_loaded_from_interrupt;
  wire n2180;
  wire n2182;
  wire n2183;
  wire n2185;
  wire n2186;
  wire n2187;
  wire n2188;
  wire n2189;
  wire [31:0] n2190;
  wire n2192;
  wire n2193;
  wire n2194;
  wire n2195;
  wire n2196;
  wire n2197;
  wire n2198;
  wire n2199;
  wire n2200;
  wire n2201;
  wire n2202;
  wire n2203;
  wire n2204;
  wire n2205;
  wire [31:0] n2206;
  wire n2208;
  wire n2209;
  wire n2210;
  wire n2211;
  wire n2212;
  wire n2213;
  wire [31:0] n2214;
  wire n2216;
  wire n2217;
  wire n2218;
  wire n2219;
  wire n2220;
  wire n2221;
  wire n2222;
  wire n2223;
  wire n2224;
  wire [31:0] n2225;
  wire n2227;
  wire n2228;
  wire n2229;
  wire [31:0] n2230;
  wire n2232;
  wire n2233;
  wire n2235;
  wire n2237;
  wire n2238;
  wire [31:0] n2239;
  wire n2241;
  wire n2242;
  wire n2244;
  wire n2245;
  wire n2246;
  wire n2248;
  wire n2270;
  wire n2271;
  wire n2272;
  wire n2273;
  wire n2274;
  wire n2277;
  wire n2279;
  wire n2280;
  wire n2283;
  wire [31:0] n2285;
  wire n2287;
  wire n2288;
  wire n2289;
  wire n2290;
  wire n2291;
  wire n2292;
  wire n2295;
  wire n2298;
  wire n2299;
  wire n2301;
  wire [31:0] n2303;
  wire n2305;
  wire n2306;
  wire n2307;
  wire [31:0] n2308;
  wire n2310;
  wire n2311;
  wire n2312;
  wire n2313;
  wire n2316;
  wire n2319;
  wire n2322;
  wire n2325;
  wire [31:0] n2326;
  wire n2328;
  wire n2329;
  wire n2331;
  wire n2333;
  wire n2335;
  wire n2337;
  wire [31:0] n2338;
  wire n2340;
  wire n2341;
  wire n2342;
  wire [31:0] n2343;
  wire n2345;
  wire n2346;
  wire n2347;
  wire n2348;
  wire n2351;
  wire n2354;
  wire n2357;
  wire n2360;
  wire [31:0] n2361;
  wire n2363;
  wire n2364;
  wire n2366;
  wire n2368;
  wire [31:0] n2369;
  wire n2371;
  wire n2373;
  wire n2374;
  wire [2:0] n2376;
  wire n2379;
  wire n2382;
  wire [2:0] n2384;
  wire n2386;
  wire n2388;
  wire n2389;
  wire n2390;
  wire n2391;
  wire n2394;
  wire n2396;
  wire n2398;
  wire [31:0] n2399;
  wire n2401;
  wire n2402;
  wire [2:0] n2403;
  wire n2405;
  wire n2407;
  wire n2409;
  wire n2410;
  wire n2411;
  wire n2414;
  wire [2:0] n2416;
  wire n2418;
  wire n2420;
  wire n2422;
  wire [3:0] n2423;
  reg n2425;
  reg n2431;
  reg n2436;
  reg [2:0] n2437;
  reg n2438;
  reg n2442;
  reg n2445;
  reg n2446;
  wire [31:0] n2447;
  wire n2449;
  wire n2450;
  wire n2451;
  wire n2452;
  wire n2454;
  wire n2455;
  wire [2:0] n2456;
  wire n2457;
  wire n2458;
  wire n2459;
  wire n2460;
  wire n2463;
  wire n2464;
  wire n2467;
  wire n2468;
  wire n2469;
  wire n2470;
  wire n2471;
  wire n2472;
  wire [2:0] n2474;
  wire n2477;
  wire n2480;
  wire [2:0] n2482;
  wire n2484;
  wire n2486;
  wire n2488;
  wire [2:0] n2490;
  wire n2492;
  wire n2494;
  wire n2496;
  wire n2498;
  wire [2:0] n2499;
  wire n2501;
  wire n2503;
  wire n2505;
  wire n2507;
  wire [31:0] n2508;
  wire n2510;
  wire n2511;
  wire n2514;
  wire n2517;
  wire [31:0] n2518;
  wire n2520;
  wire n2521;
  wire n2522;
  wire n2523;
  wire n2526;
  wire n2528;
  wire n2530;
  wire n2532;
  wire n2534;
  wire n2536;
  wire [2:0] n2538;
  wire n2539;
  wire n2540;
  wire n2541;
  wire n2543;
  wire [31:0] n2544;
  wire n2546;
  wire n2547;
  wire n2548;
  wire n2549;
  wire n2550;
  wire [2:0] n2552;
  wire n2555;
  wire n2558;
  wire [31:0] n2559;
  wire n2561;
  wire n2562;
  wire n2563;
  wire n2564;
  wire [2:0] n2565;
  wire n2567;
  wire n2569;
  wire [31:0] n2570;
  wire n2572;
  wire n2573;
  wire n2574;
  wire n2575;
  wire n2576;
  wire [2:0] n2577;
  wire n2579;
  wire n2581;
  wire [31:0] n2582;
  wire n2584;
  wire n2585;
  wire n2586;
  wire n2587;
  wire n2588;
  wire n2589;
  wire n2590;
  wire [2:0] n2591;
  wire n2593;
  wire n2595;
  wire [31:0] n2596;
  wire n2598;
  wire n2599;
  wire n2600;
  wire n2601;
  wire n2604;
  wire n2607;
  wire n2609;
  wire n2612;
  wire n2614;
  wire n2616;
  wire [2:0] n2618;
  wire n2620;
  wire n2622;
  wire n2624;
  wire n2626;
  wire n2628;
  wire n2630;
  wire [2:0] n2631;
  wire n2633;
  wire n2635;
  wire n2637;
  wire n2639;
  wire n2641;
  wire n2643;
  wire n2645;
  wire n2647;
  wire n2648;
  wire n2649;
  wire n2651;
  wire [2:0] n2652;
  wire n2653;
  wire n2655;
  wire n2657;
  wire n2659;
  wire n2660;
  wire n2662;
  wire n2664;
  wire n2666;
  wire n2668;
  wire n2670;
  wire n2672;
  wire n2673;
  wire n2674;
  wire [2:0] n2676;
  wire n2677;
  wire n2679;
  wire n2681;
  wire n2683;
  wire n2684;
  wire n2686;
  wire n2688;
  wire n2690;
  wire n2692;
  wire n2694;
  wire n2696;
  wire n2698;
  wire n2700;
  wire [2:0] n2703;
  wire n2705;
  wire n2708;
  wire n2711;
  wire n2714;
  wire n2716;
  wire n2719;
  wire n2722;
  wire n2725;
  wire n2728;
  wire n2731;
  localparam n2734 = 1'b0;
  localparam [2:0] n2735 = 3'b000;
  localparam [2:0] n2736 = 3'b000;
  localparam n2737 = 1'b0;
  localparam n2738 = 1'b1;
  localparam n2739 = 1'b0;
  localparam n2740 = 1'b0;
  localparam n2741 = 1'b0;
  localparam n2742 = 1'b0;
  localparam n2743 = 1'b0;
  localparam n2744 = 1'b0;
  localparam n2745 = 1'b0;
  wire n2746;
  reg n2747;
  wire n2748;
  reg n2749;
  wire n2750;
  reg n2751;
  wire n2752;
  reg n2753;
  wire n2754;
  reg n2755;
  wire n2756;
  reg n2757;
  assign ir_load = n2696; //(module output)
  assign ir_output_enable = n2734; //(module output)
  assign io_buffer_enable = n2698; //(module output)
  assign io_buffer_direction = n2700; //(module output)
  assign addr_select_sss = n2735; //(module output)
  assign addr_select_ddd = n2736; //(module output)
  assign scratchpad_select = n2703; //(module output)
  assign scratchpad_read = n2705; //(module output)
  assign scratchpad_write = n2708; //(module output)
  assign memory_read = n2711; //(module output)
  assign memory_write = n2714; //(module output)
  assign memory_refresh = n2737; //(module output)
  assign regfile_to_bus = n2716; //(module output)
  assign bus_to_regfile = n2719; //(module output)
  assign select_pc = n2738; //(module output)
  assign select_stack = n2739; //(module output)
  assign pc_load_from_regs = n2722; //(module output)
  assign pc_load_from_stack = n2740; //(module output)
  assign pc_load_from_rst = n2725; //(module output)
  assign refresh_increment = n2741; //(module output)
  assign stack_addr_select = n2742; //(module output)
  assign stack_push = n2728; //(module output)
  assign stack_pop = n2731; //(module output)
  assign stack_read = n2743; //(module output)
  assign stack_write = n2744; //(module output)
  assign pc_increment_lower = n2277; //(module output)
  assign pc_increment_upper = n2283; //(module output)
  assign pc_load = n2301; //(module output)
  assign pc_hold = n2745; //(module output)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:158:12  */
  always @*
    prev_state_t2 = n2747; // (isignal)
  initial
    prev_state_t2 = 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:159:12  */
  always @*
    prev_state_t3 = n2749; // (isignal)
  initial
    prev_state_t3 = 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:160:12  */
  always @*
    prev_state_t4 = n2751; // (isignal)
  initial
    prev_state_t4 = 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:161:12  */
  always @*
    prev_state_t5 = n2753; // (isignal)
  initial
    prev_state_t5 = 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:172:12  */
  always @*
    suppress_pc_inc_next_cycle = n2755; // (isignal)
  initial
    suppress_pc_inc_next_cycle = 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:177:12  */
  always @*
    ir_loaded_from_interrupt = n2757; // (isignal)
  initial
    ir_loaded_from_interrupt = 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:197:32  */
  assign n2180 = state_half & state_t1i;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:197:13  */
  assign n2182 = n2180 ? 1'b1 : ir_loaded_from_interrupt;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:203:36  */
  assign n2183 = ir_loaded_from_interrupt & prev_state_t4;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:203:13  */
  assign n2185 = n2183 ? 1'b0 : n2182;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:218:55  */
  assign n2186 = ~advance_state;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:218:37  */
  assign n2187 = n2186 & prev_state_t5;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:219:55  */
  assign n2188 = ~advance_state;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:219:37  */
  assign n2189 = n2188 & prev_state_t3;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:219:79  */
  assign n2190 = {30'b0, current_cycle};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:219:79  */
  assign n2192 = n2190 == 32'b00000000000000000000000000000000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:219:61  */
  assign n2193 = n2192 & n2189;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:220:46  */
  assign n2194 = instr_needs_immediate | instr_needs_address;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:219:83  */
  assign n2195 = n2194 & n2193;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:221:50  */
  assign n2196 = instr_is_write & instr_is_mem_indirect;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:222:42  */
  assign n2197 = ~instr_needs_address;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:221:75  */
  assign n2198 = n2197 & n2196;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:222:64  */
  assign n2199 = ~instr_is_io;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:222:48  */
  assign n2200 = n2199 & n2198;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:221:17  */
  assign n2201 = ~n2200;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:220:76  */
  assign n2202 = n2201 & n2195;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:218:62  */
  assign n2203 = n2187 | n2202;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:223:55  */
  assign n2204 = ~advance_state;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:223:37  */
  assign n2205 = n2204 & prev_state_t3;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:223:79  */
  assign n2206 = {30'b0, current_cycle};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:223:79  */
  assign n2208 = n2206 == 32'b00000000000000000000000000000001;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:223:61  */
  assign n2209 = n2208 & n2205;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:223:83  */
  assign n2210 = instr_needs_address & n2209;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:222:72  */
  assign n2211 = n2203 | n2210;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:225:55  */
  assign n2212 = ~advance_state;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:225:37  */
  assign n2213 = n2212 & prev_state_t4;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:225:79  */
  assign n2214 = {30'b0, current_cycle};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:225:79  */
  assign n2216 = n2214 == 32'b00000000000000000000000000000000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:225:61  */
  assign n2217 = n2216 & n2213;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:225:83  */
  assign n2218 = instr_is_mem_indirect & n2217;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:226:45  */
  assign n2219 = instr_is_write & n2218;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:227:37  */
  assign n2220 = ~instr_needs_address;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:226:70  */
  assign n2221 = n2220 & n2219;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:227:59  */
  assign n2222 = ~instr_is_io;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:227:43  */
  assign n2223 = n2222 & n2221;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:224:44  */
  assign n2224 = n2211 | n2223;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:237:38  */
  assign n2225 = {30'b0, current_cycle};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:237:38  */
  assign n2227 = n2225 == 32'b00000000000000000000000000000000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:237:66  */
  assign n2228 = ~instr_needs_address;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:237:42  */
  assign n2229 = n2228 & n2227;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:241:41  */
  assign n2230 = {30'b0, current_cycle};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:241:41  */
  assign n2232 = n2230 == 32'b00000000000000000000000000000001;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:241:45  */
  assign n2233 = instr_needs_address & n2232;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:241:21  */
  assign n2235 = n2233 ? 1'b1 : suppress_pc_inc_next_cycle;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:237:21  */
  assign n2237 = n2229 ? 1'b1 : n2235;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:236:17  */
  assign n2238 = instr_is_mem_indirect ? n2237 : suppress_pc_inc_next_cycle;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:248:56  */
  assign n2239 = {30'b0, current_cycle};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:248:56  */
  assign n2241 = n2239 == 32'b00000000000000000000000000000000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:248:38  */
  assign n2242 = n2241 & instr_is_io;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:248:17  */
  assign n2244 = n2242 ? 1'b1 : n2238;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:218:13  */
  assign n2245 = n2224 ? n2244 : suppress_pc_inc_next_cycle;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:254:36  */
  assign n2246 = suppress_pc_inc_next_cycle & prev_state_t2;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:254:13  */
  assign n2248 = n2246 ? 1'b0 : n2245;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:342:31  */
  assign n2270 = state_half & state_t1;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:342:66  */
  assign n2271 = ~state_t1i;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:342:52  */
  assign n2272 = n2271 & n2270;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:343:43  */
  assign n2273 = ~suppress_pc_inc_next_cycle;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:342:72  */
  assign n2274 = n2273 & n2272;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:342:13  */
  assign n2277 = n2274 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:349:31  */
  assign n2279 = state_half & state_t2;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:349:52  */
  assign n2280 = pc_carry_in & n2279;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:349:13  */
  assign n2283 = n2280 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:366:34  */
  assign n2285 = {30'b0, current_cycle};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:366:34  */
  assign n2287 = n2285 == 32'b00000000000000000000000000000010;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:366:38  */
  assign n2288 = instr_needs_address & n2287;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:366:87  */
  assign n2289 = ~instr_is_write;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:366:68  */
  assign n2290 = n2289 & n2288;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:369:39  */
  assign n2291 = ~eval_condition;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:369:45  */
  assign n2292 = n2291 | condition_met;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:369:21  */
  assign n2295 = n2292 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:381:17  */
  assign n2298 = instr_is_rst ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:366:17  */
  assign n2299 = n2290 ? n2295 : n2298;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:365:13  */
  assign n2301 = state_t5 ? n2299 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:401:29  */
  assign n2303 = {30'b0, next_cycle};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:401:29  */
  assign n2305 = n2303 == 32'b00000000000000000000000000000001;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:401:57  */
  assign n2306 = ~instr_needs_address;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:401:33  */
  assign n2307 = n2306 & n2305;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:402:29  */
  assign n2308 = {30'b0, next_cycle};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:402:29  */
  assign n2310 = n2308 == 32'b00000000000000000000000000000010;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:402:33  */
  assign n2311 = instr_needs_address & n2310;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:401:64  */
  assign n2312 = n2307 | n2311;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:400:44  */
  assign n2313 = n2312 & instr_is_mem_indirect;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:400:13  */
  assign n2316 = n2313 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:400:13  */
  assign n2319 = n2313 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:400:13  */
  assign n2322 = n2313 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:400:13  */
  assign n2325 = n2313 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:413:49  */
  assign n2326 = {30'b0, next_cycle};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:413:49  */
  assign n2328 = n2326 == 32'b00000000000000000000000000000001;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:413:34  */
  assign n2329 = n2328 & instr_is_io;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:413:13  */
  assign n2331 = n2329 ? 1'b1 : n2316;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:413:13  */
  assign n2333 = n2329 ? 1'b1 : n2319;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:413:13  */
  assign n2335 = n2329 ? 1'b1 : n2322;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:413:13  */
  assign n2337 = n2329 ? 1'b1 : n2325;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:431:32  */
  assign n2338 = {30'b0, current_cycle};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:431:32  */
  assign n2340 = n2338 == 32'b00000000000000000000000000000001;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:431:60  */
  assign n2341 = ~instr_needs_address;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:431:36  */
  assign n2342 = n2341 & n2340;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:432:32  */
  assign n2343 = {30'b0, current_cycle};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:432:32  */
  assign n2345 = n2343 == 32'b00000000000000000000000000000010;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:432:36  */
  assign n2346 = instr_needs_address & n2345;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:431:67  */
  assign n2347 = n2342 | n2346;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:430:44  */
  assign n2348 = n2347 & instr_is_mem_indirect;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:430:13  */
  assign n2351 = n2348 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:430:13  */
  assign n2354 = n2348 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:430:13  */
  assign n2357 = n2348 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:430:13  */
  assign n2360 = n2348 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:444:52  */
  assign n2361 = {30'b0, current_cycle};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:444:52  */
  assign n2363 = n2361 == 32'b00000000000000000000000000000001;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:444:34  */
  assign n2364 = n2363 & instr_is_io;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:444:13  */
  assign n2366 = n2364 ? 1'b1 : n2351;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:444:13  */
  assign n2368 = n2364 ? 1'b1 : n2354;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:457:30  */
  assign n2369 = {30'b0, current_cycle};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:457:30  */
  assign n2371 = n2369 == 32'b00000000000000000000000000000000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:457:49  */
  assign n2373 = cycle_type != 2'b00;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:457:34  */
  assign n2374 = n2373 & n2371;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:458:17  */
  assign n2376 = instr_reads_reg ? instr_sss_field : 3'b000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:458:17  */
  assign n2379 = instr_reads_reg ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:458:17  */
  assign n2382 = instr_reads_reg ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:457:13  */
  assign n2384 = n2374 ? n2376 : 3'b000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:457:13  */
  assign n2386 = n2374 ? n2379 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:457:13  */
  assign n2388 = n2374 ? n2382 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:477:38  */
  assign n2389 = ~state_stopped;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:477:73  */
  assign n2390 = ~ir_loaded_from_interrupt;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:477:44  */
  assign n2391 = n2390 & n2389;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:477:21  */
  assign n2394 = n2391 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:469:17  */
  assign n2396 = cycle_type == 2'b00;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:487:17  */
  assign n2398 = cycle_type == 2'b01;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:505:38  */
  assign n2399 = {30'b0, current_cycle};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:505:38  */
  assign n2401 = n2399 == 32'b00000000000000000000000000000010;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:505:42  */
  assign n2402 = instr_needs_immediate & n2401;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:505:21  */
  assign n2403 = n2402 ? n2384 : instr_sss_field;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:505:21  */
  assign n2405 = n2402 ? n2386 : 1'b1;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:505:21  */
  assign n2407 = n2402 ? n2388 : 1'b1;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:495:17  */
  assign n2409 = cycle_type == 2'b11;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:521:67  */
  assign n2410 = ~instr_writes_reg;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:521:46  */
  assign n2411 = n2410 & instr_reads_reg;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:521:21  */
  assign n2414 = n2411 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:521:21  */
  assign n2416 = n2411 ? 3'b000 : n2384;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:521:21  */
  assign n2418 = n2411 ? 1'b1 : n2386;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:521:21  */
  assign n2420 = n2411 ? 1'b1 : n2388;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:516:17  */
  assign n2422 = cycle_type == 2'b10;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:468:13  */
  assign n2423 = {n2422, n2409, n2398, n2396};
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:468:13  */
  always @*
    case (n2423)
      4'b1000: n2425 = 1'b0;
      4'b0100: n2425 = 1'b0;
      4'b0010: n2425 = 1'b0;
      4'b0001: n2425 = n2394;
      default: n2425 = 1'b0;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:468:13  */
  always @*
    case (n2423)
      4'b1000: n2431 = 1'b1;
      4'b0100: n2431 = 1'b1;
      4'b0010: n2431 = 1'b1;
      4'b0001: n2431 = 1'b1;
      default: n2431 = 1'b0;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:468:13  */
  always @*
    case (n2423)
      4'b1000: n2436 = n2414;
      4'b0100: n2436 = 1'b1;
      4'b0010: n2436 = 1'b0;
      4'b0001: n2436 = 1'b0;
      default: n2436 = 1'b0;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:468:13  */
  always @*
    case (n2423)
      4'b1000: n2437 = n2416;
      4'b0100: n2437 = n2403;
      4'b0010: n2437 = n2384;
      4'b0001: n2437 = n2384;
      default: n2437 = n2384;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:468:13  */
  always @*
    case (n2423)
      4'b1000: n2438 = n2418;
      4'b0100: n2438 = n2405;
      4'b0010: n2438 = n2386;
      4'b0001: n2438 = n2386;
      default: n2438 = n2386;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:468:13  */
  always @*
    case (n2423)
      4'b1000: n2442 = 1'b0;
      4'b0100: n2442 = 1'b0;
      4'b0010: n2442 = 1'b1;
      4'b0001: n2442 = 1'b1;
      default: n2442 = 1'b0;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:468:13  */
  always @*
    case (n2423)
      4'b1000: n2445 = 1'b0;
      4'b0100: n2445 = 1'b1;
      4'b0010: n2445 = 1'b0;
      4'b0001: n2445 = 1'b0;
      default: n2445 = 1'b0;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:468:13  */
  always @*
    case (n2423)
      4'b1000: n2446 = n2420;
      4'b0100: n2446 = n2407;
      4'b0010: n2446 = n2388;
      4'b0001: n2446 = n2388;
      default: n2446 = n2388;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:543:30  */
  assign n2447 = {30'b0, current_cycle};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:543:30  */
  assign n2449 = n2447 == 32'b00000000000000000000000000000000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:545:65  */
  assign n2450 = ~instr_needs_immediate;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:545:39  */
  assign n2451 = n2450 & instr_is_alu;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:545:71  */
  assign n2452 = instr_reads_reg & n2451;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:550:67  */
  assign n2454 = instr_ddd_field != 3'b000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:550:47  */
  assign n2455 = n2454 & instr_writes_reg;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:550:21  */
  assign n2456 = n2455 ? instr_ddd_field : instr_sss_field;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:559:62  */
  assign n2457 = ~eval_condition;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:559:68  */
  assign n2458 = n2457 | condition_met;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:559:42  */
  assign n2459 = n2458 & instr_is_ret;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:564:35  */
  assign n2460 = ~state_half;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:564:21  */
  assign n2463 = n2460 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:572:35  */
  assign n2464 = ~state_half;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:572:21  */
  assign n2467 = n2464 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:575:46  */
  assign n2468 = instr_reads_reg & instr_writes_reg;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:575:89  */
  assign n2469 = ~instr_is_alu;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:575:72  */
  assign n2470 = n2469 & n2468;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:575:121  */
  assign n2471 = ~instr_needs_immediate;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:575:95  */
  assign n2472 = n2471 & n2470;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:575:17  */
  assign n2474 = n2472 ? instr_sss_field : 3'b000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:575:17  */
  assign n2477 = n2472 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:575:17  */
  assign n2480 = n2472 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:568:17  */
  assign n2482 = instr_is_rst ? 3'b000 : n2474;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:568:17  */
  assign n2484 = instr_is_rst ? 1'b0 : n2477;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:568:17  */
  assign n2486 = instr_is_rst ? 1'b0 : n2480;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:568:17  */
  assign n2488 = instr_is_rst ? n2467 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:559:17  */
  assign n2490 = n2459 ? 3'b000 : n2482;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:559:17  */
  assign n2492 = n2459 ? 1'b0 : n2484;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:559:17  */
  assign n2494 = n2459 ? 1'b0 : n2486;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:559:17  */
  assign n2496 = n2459 ? 1'b0 : n2488;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:559:17  */
  assign n2498 = n2459 ? n2463 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:545:17  */
  assign n2499 = n2452 ? n2456 : n2490;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:545:17  */
  assign n2501 = n2452 ? 1'b1 : n2492;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:545:17  */
  assign n2503 = n2452 ? 1'b1 : n2494;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:545:17  */
  assign n2505 = n2452 ? 1'b0 : n2496;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:545:17  */
  assign n2507 = n2452 ? 1'b0 : n2498;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:585:33  */
  assign n2508 = {30'b0, current_cycle};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:585:33  */
  assign n2510 = n2508 == 32'b00000000000000000000000000000001;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:592:39  */
  assign n2511 = instr_needs_immediate & instr_is_alu;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:592:17  */
  assign n2514 = n2511 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:592:17  */
  assign n2517 = n2511 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:599:33  */
  assign n2518 = {30'b0, current_cycle};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:599:33  */
  assign n2520 = n2518 == 32'b00000000000000000000000000000010;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:605:60  */
  assign n2521 = ~eval_condition;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:605:66  */
  assign n2522 = n2521 | condition_met;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:605:40  */
  assign n2523 = n2522 & instr_is_call;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:610:21  */
  assign n2526 = state_half ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:605:17  */
  assign n2528 = n2523 ? n2526 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:599:13  */
  assign n2530 = n2520 ? n2528 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:585:13  */
  assign n2532 = n2510 ? n2514 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:585:13  */
  assign n2534 = n2510 ? n2517 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:585:13  */
  assign n2536 = n2510 ? 1'b0 : n2530;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:543:13  */
  assign n2538 = n2449 ? n2499 : 3'b000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:543:13  */
  assign n2539 = n2449 ? n2501 : n2532;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:543:13  */
  assign n2540 = n2449 ? n2503 : n2534;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:543:13  */
  assign n2541 = n2449 ? n2505 : n2536;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:543:13  */
  assign n2543 = n2449 ? n2507 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:624:30  */
  assign n2544 = {30'b0, current_cycle};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:624:30  */
  assign n2546 = n2544 == 32'b00000000000000000000000000000000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:624:34  */
  assign n2547 = instr_is_alu & n2546;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:624:57  */
  assign n2548 = instr_writes_reg & n2547;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:624:110  */
  assign n2549 = ~instr_needs_immediate;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:624:84  */
  assign n2550 = n2549 & n2548;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:624:13  */
  assign n2552 = n2550 ? instr_ddd_field : 3'b000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:624:13  */
  assign n2555 = n2550 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:624:13  */
  assign n2558 = n2550 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:634:30  */
  assign n2559 = {30'b0, current_cycle};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:634:30  */
  assign n2561 = n2559 == 32'b00000000000000000000000000000001;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:634:34  */
  assign n2562 = instr_is_alu & n2561;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:634:57  */
  assign n2563 = instr_writes_reg & n2562;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:634:84  */
  assign n2564 = instr_needs_immediate & n2563;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:634:13  */
  assign n2565 = n2564 ? instr_ddd_field : n2552;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:634:13  */
  assign n2567 = n2564 ? 1'b1 : n2555;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:634:13  */
  assign n2569 = n2564 ? 1'b1 : n2558;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:646:30  */
  assign n2570 = {30'b0, current_cycle};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:646:30  */
  assign n2572 = n2570 == 32'b00000000000000000000000000000001;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:646:34  */
  assign n2573 = instr_writes_reg & n2572;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:646:78  */
  assign n2574 = ~instr_is_alu;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:646:61  */
  assign n2575 = n2574 & n2573;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:646:84  */
  assign n2576 = instr_needs_immediate & n2575;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:646:13  */
  assign n2577 = n2576 ? instr_ddd_field : n2565;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:646:13  */
  assign n2579 = n2576 ? 1'b1 : n2567;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:646:13  */
  assign n2581 = n2576 ? 1'b1 : n2569;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:656:30  */
  assign n2582 = {30'b0, current_cycle};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:656:30  */
  assign n2584 = n2582 == 32'b00000000000000000000000000000000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:656:34  */
  assign n2585 = instr_writes_reg & n2584;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:656:61  */
  assign n2586 = instr_reads_reg & n2585;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:657:29  */
  assign n2587 = ~instr_is_alu;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:656:87  */
  assign n2588 = n2587 & n2586;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:657:61  */
  assign n2589 = ~instr_needs_immediate;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:657:35  */
  assign n2590 = n2589 & n2588;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:656:13  */
  assign n2591 = n2590 ? instr_ddd_field : n2577;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:656:13  */
  assign n2593 = n2590 ? 1'b1 : n2579;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:656:13  */
  assign n2595 = n2590 ? 1'b1 : n2581;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:667:30  */
  assign n2596 = {30'b0, current_cycle};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:667:30  */
  assign n2598 = n2596 == 32'b00000000000000000000000000000010;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:667:34  */
  assign n2599 = instr_needs_address & n2598;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:667:83  */
  assign n2600 = ~instr_is_write;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:667:64  */
  assign n2601 = n2600 & n2599;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:667:13  */
  assign n2604 = n2601 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:681:13  */
  assign n2607 = instr_is_rst ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:685:9  */
  assign n2609 = state_t1i ? state_half : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:685:9  */
  assign n2612 = state_t1i ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:619:9  */
  assign n2614 = state_t5 ? 1'b0 : n2609;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:619:9  */
  assign n2616 = state_t5 ? 1'b0 : n2612;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:619:9  */
  assign n2618 = state_t5 ? n2591 : 3'b000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:619:9  */
  assign n2620 = state_t5 ? n2593 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:619:9  */
  assign n2622 = state_t5 ? n2595 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:619:9  */
  assign n2624 = state_t5 ? n2604 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:619:9  */
  assign n2626 = state_t5 ? n2607 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:538:9  */
  assign n2628 = state_t4 ? 1'b0 : n2614;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:538:9  */
  assign n2630 = state_t4 ? 1'b0 : n2616;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:538:9  */
  assign n2631 = state_t4 ? n2538 : n2618;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:538:9  */
  assign n2633 = state_t4 ? n2539 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:538:9  */
  assign n2635 = state_t4 ? 1'b0 : n2620;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:538:9  */
  assign n2637 = state_t4 ? n2540 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:538:9  */
  assign n2639 = state_t4 ? 1'b0 : n2622;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:538:9  */
  assign n2641 = state_t4 ? 1'b0 : n2624;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:538:9  */
  assign n2643 = state_t4 ? 1'b0 : n2626;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:538:9  */
  assign n2645 = state_t4 ? n2541 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:538:9  */
  assign n2647 = state_t4 ? n2543 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:450:9  */
  assign n2648 = state_t3 ? n2425 : n2628;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:450:9  */
  assign n2649 = state_t3 ? n2431 : n2630;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:450:9  */
  assign n2651 = state_t3 ? n2436 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:450:9  */
  assign n2652 = state_t3 ? n2437 : n2631;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:450:9  */
  assign n2653 = state_t3 ? n2438 : n2633;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:450:9  */
  assign n2655 = state_t3 ? 1'b0 : n2635;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:450:9  */
  assign n2657 = state_t3 ? n2442 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:450:9  */
  assign n2659 = state_t3 ? n2445 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:450:9  */
  assign n2660 = state_t3 ? n2446 : n2637;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:450:9  */
  assign n2662 = state_t3 ? 1'b0 : n2639;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:450:9  */
  assign n2664 = state_t3 ? 1'b0 : n2641;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:450:9  */
  assign n2666 = state_t3 ? 1'b0 : n2643;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:450:9  */
  assign n2668 = state_t3 ? 1'b0 : n2645;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:450:9  */
  assign n2670 = state_t3 ? 1'b0 : n2647;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:422:9  */
  assign n2672 = state_t2 ? 1'b0 : n2648;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:422:9  */
  assign n2673 = state_t2 ? n2366 : n2649;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:422:9  */
  assign n2674 = state_t2 ? n2368 : n2651;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:422:9  */
  assign n2676 = state_t2 ? 3'b000 : n2652;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:422:9  */
  assign n2677 = state_t2 ? n2357 : n2653;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:422:9  */
  assign n2679 = state_t2 ? 1'b0 : n2655;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:422:9  */
  assign n2681 = state_t2 ? 1'b0 : n2657;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:422:9  */
  assign n2683 = state_t2 ? 1'b0 : n2659;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:422:9  */
  assign n2684 = state_t2 ? n2360 : n2660;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:422:9  */
  assign n2686 = state_t2 ? 1'b0 : n2662;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:422:9  */
  assign n2688 = state_t2 ? 1'b0 : n2664;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:422:9  */
  assign n2690 = state_t2 ? 1'b0 : n2666;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:422:9  */
  assign n2692 = state_t2 ? 1'b0 : n2668;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:422:9  */
  assign n2694 = state_t2 ? 1'b0 : n2670;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:388:9  */
  assign n2696 = state_t1 ? 1'b0 : n2672;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:388:9  */
  assign n2698 = state_t1 ? n2331 : n2673;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:388:9  */
  assign n2700 = state_t1 ? n2333 : n2674;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:388:9  */
  assign n2703 = state_t1 ? 3'b000 : n2676;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:388:9  */
  assign n2705 = state_t1 ? n2335 : n2677;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:388:9  */
  assign n2708 = state_t1 ? 1'b0 : n2679;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:388:9  */
  assign n2711 = state_t1 ? 1'b0 : n2681;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:388:9  */
  assign n2714 = state_t1 ? 1'b0 : n2683;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:388:9  */
  assign n2716 = state_t1 ? n2337 : n2684;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:388:9  */
  assign n2719 = state_t1 ? 1'b0 : n2686;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:388:9  */
  assign n2722 = state_t1 ? 1'b0 : n2688;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:388:9  */
  assign n2725 = state_t1 ? 1'b0 : n2690;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:388:9  */
  assign n2728 = state_t1 ? 1'b0 : n2692;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:388:9  */
  assign n2731 = state_t1 ? 1'b0 : n2694;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:195:9  */
  assign n2746 = phi1_rising ? state_t2 : prev_state_t2;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:195:9  */
  always @(posedge clk or posedge reset)
    if (reset)
      n2747 <= 1'b0;
    else
      n2747 <= n2746;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:195:9  */
  assign n2748 = phi1_rising ? state_t3 : prev_state_t3;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:195:9  */
  always @(posedge clk or posedge reset)
    if (reset)
      n2749 <= 1'b0;
    else
      n2749 <= n2748;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:195:9  */
  assign n2750 = phi1_rising ? state_t4 : prev_state_t4;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:195:9  */
  always @(posedge clk or posedge reset)
    if (reset)
      n2751 <= 1'b0;
    else
      n2751 <= n2750;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:195:9  */
  assign n2752 = phi1_rising ? state_t5 : prev_state_t5;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:195:9  */
  always @(posedge clk or posedge reset)
    if (reset)
      n2753 <= 1'b0;
    else
      n2753 <= n2752;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:195:9  */
  assign n2754 = phi1_rising ? n2248 : suppress_pc_inc_next_cycle;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:195:9  */
  always @(posedge clk or posedge reset)
    if (reset)
      n2755 <= 1'b0;
    else
      n2755 <= n2754;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:195:9  */
  assign n2756 = phi1_rising ? n2185 : ir_loaded_from_interrupt;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/memory_io_control.vhdl:195:9  */
  always @(posedge clk or posedge reset)
    if (reset)
      n2757 <= 1'b0;
    else
      n2757 <= n2756;
endmodule

module instruction_decoder_Brtl
  (input  [7:0] instruction_byte,
   output instr_needs_immediate,
   output instr_needs_address,
   output instr_is_io,
   output instr_is_write,
   output [2:0] instr_sss_field,
   output [2:0] instr_ddd_field,
   output instr_is_alu,
   output instr_is_call,
   output instr_is_ret,
   output instr_is_rst,
   output instr_is_hlt,
   output instr_writes_reg,
   output instr_reads_reg,
   output instr_is_mem_indirect,
   output instr_uses_temp_regs,
   output instr_is_inr_dcr,
   output instr_is_binary_alu,
   output instr_is_rotate,
   output instr_needs_t4t5,
   output [2:0] rst_vector,
   output [1:0] condition_code,
   output test_true,
   output eval_condition,
   output transition_to_stopped);
  wire [1:0] n1712;
  wire [2:0] n1713;
  wire [2:0] n1714;
  wire n1718;
  wire n1720;
  wire n1721;
  wire n1723;
  wire n1725;
  wire n1726;
  wire n1727;
  wire n1730;
  wire n1733;
  wire [1:0] n1734;
  wire n1736;
  wire n1737;
  wire n1739;
  wire [2:0] n1741;
  wire n1744;
  wire n1747;
  wire n1750;
  wire n1753;
  wire n1756;
  wire n1759;
  wire n1761;
  wire n1763;
  wire n1765;
  wire [1:0] n1766;
  wire n1767;
  wire n1769;
  wire n1771;
  wire n1774;
  wire n1776;
  wire n1778;
  wire n1780;
  wire n1783;
  wire n1786;
  wire n1789;
  wire n1791;
  wire n1794;
  wire n1796;
  wire n1798;
  wire [7:0] n1799;
  reg n1803;
  reg n1805;
  reg n1807;
  reg [2:0] n1809;
  reg [2:0] n1812;
  reg n1817;
  reg n1821;
  reg n1824;
  reg n1828;
  reg n1833;
  reg n1834;
  reg n1839;
  reg n1842;
  reg n1845;
  reg n1848;
  reg n1856;
  reg [2:0] n1858;
  reg [1:0] n1860;
  reg n1862;
  reg n1865;
  wire n1867;
  wire n1869;
  wire n1871;
  wire [2:0] n1872;
  wire [2:0] n1873;
  wire n1875;
  wire n1877;
  wire n1879;
  wire n1882;
  wire n1884;
  wire n1886;
  wire n1887;
  wire n1889;
  wire n1891;
  wire n1893;
  wire n1895;
  wire n1897;
  wire [2:0] n1899;
  wire [1:0] n1901;
  wire n1903;
  wire n1905;
  wire n1908;
  wire n1910;
  wire n1911;
  wire [1:0] n1912;
  wire n1914;
  wire [2:0] n1916;
  wire [2:0] n1918;
  wire n1921;
  wire n1924;
  wire n1927;
  wire n1929;
  wire [1:0] n1930;
  wire n1931;
  wire n1933;
  wire [1:0] n1934;
  wire n1935;
  wire n1937;
  wire [1:0] n1938;
  wire n1940;
  wire n1943;
  wire n1945;
  wire n1947;
  wire [1:0] n1949;
  wire n1951;
  wire n1954;
  wire n1956;
  wire [1:0] n1957;
  wire n1958;
  wire n1960;
  wire n1963;
  wire n1966;
  wire n1969;
  wire [2:0] n1970;
  wire n1971;
  wire n1973;
  wire n1975;
  wire n1977;
  wire n1980;
  wire n1982;
  wire [1:0] n1984;
  wire n1986;
  wire n1988;
  wire n1990;
  wire n1992;
  wire n1995;
  wire n1997;
  wire n2000;
  wire n2002;
  wire n2004;
  wire n2006;
  wire n2008;
  wire n2011;
  wire n2014;
  wire n2017;
  wire n2020;
  wire n2023;
  wire n2025;
  wire n2027;
  wire n2029;
  wire n2032;
  wire n2034;
  wire n2036;
  wire n2038;
  wire n2040;
  wire n2043;
  wire n2045;
  wire n2047;
  wire n2049;
  wire n2051;
  wire n2054;
  wire n2056;
  wire [3:0] n2057;
  reg n2059;
  reg n2062;
  reg n2065;
  reg n2068;
  reg [2:0] n2070;
  reg [2:0] n2072;
  reg n2075;
  reg n2078;
  reg n2081;
  reg n2084;
  reg n2087;
  reg n2090;
  reg n2094;
  reg n2096;
  reg n2099;
  reg n2102;
  reg n2106;
  reg n2109;
  reg n2113;
  reg [2:0] n2116;
  reg [1:0] n2119;
  reg n2122;
  reg n2125;
  reg n2128;
  assign instr_needs_immediate = n2059; //(module output)
  assign instr_needs_address = n2062; //(module output)
  assign instr_is_io = n2065; //(module output)
  assign instr_is_write = n2068; //(module output)
  assign instr_sss_field = n2070; //(module output)
  assign instr_ddd_field = n2072; //(module output)
  assign instr_is_alu = n2075; //(module output)
  assign instr_is_call = n2078; //(module output)
  assign instr_is_ret = n2081; //(module output)
  assign instr_is_rst = n2084; //(module output)
  assign instr_is_hlt = n2087; //(module output)
  assign instr_writes_reg = n2090; //(module output)
  assign instr_reads_reg = n2094; //(module output)
  assign instr_is_mem_indirect = n2096; //(module output)
  assign instr_uses_temp_regs = n2099; //(module output)
  assign instr_is_inr_dcr = n2102; //(module output)
  assign instr_is_binary_alu = n2106; //(module output)
  assign instr_is_rotate = n2109; //(module output)
  assign instr_needs_t4t5 = n2113; //(module output)
  assign rst_vector = n2116; //(module output)
  assign condition_code = n2119; //(module output)
  assign test_true = n2122; //(module output)
  assign eval_condition = n2125; //(module output)
  assign transition_to_stopped = n2128; //(module output)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:70:11  */
  assign n1712 = instruction_byte[7:6]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:129:29  */
  assign n1713 = instruction_byte[5:3]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:72:11  */
  assign n1714 = instruction_byte[2:0]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:109:20  */
  assign n1718 = n1714 == 3'b111;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:109:38  */
  assign n1720 = n1713 == 3'b111;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:109:28  */
  assign n1721 = n1718 | n1720;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:109:58  */
  assign n1723 = n1712 == 2'b10;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:109:74  */
  assign n1725 = n1712 == 2'b11;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:109:65  */
  assign n1726 = n1723 | n1725;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:109:47  */
  assign n1727 = n1726 & n1721;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:109:9  */
  assign n1730 = n1727 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:119:27  */
  assign n1733 = n1713 == 3'b000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:119:45  */
  assign n1734 = instruction_byte[2:1]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:119:58  */
  assign n1736 = n1734 == 2'b00;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:119:35  */
  assign n1737 = n1736 & n1733;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:129:39  */
  assign n1739 = n1713 != 3'b000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:129:29  */
  assign n1741 = n1739 ? 3'b000 : n1714;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:129:29  */
  assign n1744 = n1739 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:129:29  */
  assign n1747 = n1739 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:129:29  */
  assign n1750 = n1739 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:129:29  */
  assign n1753 = n1739 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:129:29  */
  assign n1756 = n1739 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:129:29  */
  assign n1759 = n1739 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:126:25  */
  assign n1761 = n1714 == 3'b000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:140:21  */
  assign n1763 = n1714 == 3'b001;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:151:21  */
  assign n1765 = n1714 == 3'b010;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:170:49  */
  assign n1766 = instruction_byte[4:3]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:171:44  */
  assign n1767 = instruction_byte[5]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:163:21  */
  assign n1769 = n1714 == 3'b011;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:184:35  */
  assign n1771 = n1713 == 3'b111;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:184:25  */
  assign n1774 = n1771 ? 1'b0 : 1'b1;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:173:21  */
  assign n1776 = n1714 == 3'b100;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:193:21  */
  assign n1778 = n1714 == 3'b101;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:203:35  */
  assign n1780 = n1713 == 3'b111;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:203:25  */
  assign n1783 = n1780 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:203:25  */
  assign n1786 = n1780 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:203:25  */
  assign n1789 = n1780 ? 1'b0 : 1'b1;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:203:25  */
  assign n1791 = n1780 ? 1'b1 : n1730;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:203:25  */
  assign n1794 = n1780 ? 1'b0 : 1'b1;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:200:21  */
  assign n1796 = n1714 == 3'b110;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:223:21  */
  assign n1798 = n1714 == 3'b111;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:125:21  */
  assign n1799 = {n1798, n1796, n1778, n1776, n1769, n1765, n1763, n1761};
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:125:21  */
  always @*
    case (n1799)
      8'b10000000: n1803 = 1'b0;
      8'b01000000: n1803 = 1'b1;
      8'b00100000: n1803 = 1'b0;
      8'b00010000: n1803 = 1'b1;
      8'b00001000: n1803 = 1'b0;
      8'b00000100: n1803 = 1'b0;
      8'b00000010: n1803 = 1'b0;
      8'b00000001: n1803 = 1'b0;
      default: n1803 = 1'b0;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:125:21  */
  always @*
    case (n1799)
      8'b10000000: n1805 = 1'b0;
      8'b01000000: n1805 = n1783;
      8'b00100000: n1805 = 1'b0;
      8'b00010000: n1805 = 1'b0;
      8'b00001000: n1805 = 1'b0;
      8'b00000100: n1805 = 1'b0;
      8'b00000010: n1805 = 1'b0;
      8'b00000001: n1805 = 1'b0;
      default: n1805 = 1'b0;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:125:21  */
  always @*
    case (n1799)
      8'b10000000: n1807 = 1'b0;
      8'b01000000: n1807 = n1786;
      8'b00100000: n1807 = 1'b0;
      8'b00010000: n1807 = 1'b0;
      8'b00001000: n1807 = 1'b0;
      8'b00000100: n1807 = 1'b0;
      8'b00000010: n1807 = 1'b0;
      8'b00000001: n1807 = 1'b0;
      default: n1807 = 1'b0;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:125:21  */
  always @*
    case (n1799)
      8'b10000000: n1809 = n1714;
      8'b01000000: n1809 = n1714;
      8'b00100000: n1809 = n1714;
      8'b00010000: n1809 = n1713;
      8'b00001000: n1809 = n1714;
      8'b00000100: n1809 = n1713;
      8'b00000010: n1809 = 3'b010;
      8'b00000001: n1809 = n1741;
      default: n1809 = n1714;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:125:21  */
  always @*
    case (n1799)
      8'b10000000: n1812 = n1713;
      8'b01000000: n1812 = n1713;
      8'b00100000: n1812 = n1713;
      8'b00010000: n1812 = 3'b000;
      8'b00001000: n1812 = n1713;
      8'b00000100: n1812 = 3'b000;
      8'b00000010: n1812 = n1713;
      8'b00000001: n1812 = n1713;
      default: n1812 = n1713;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:125:21  */
  always @*
    case (n1799)
      8'b10000000: n1817 = 1'b0;
      8'b01000000: n1817 = 1'b0;
      8'b00100000: n1817 = 1'b0;
      8'b00010000: n1817 = 1'b1;
      8'b00001000: n1817 = 1'b0;
      8'b00000100: n1817 = 1'b1;
      8'b00000010: n1817 = 1'b1;
      8'b00000001: n1817 = n1744;
      default: n1817 = 1'b0;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:125:21  */
  always @*
    case (n1799)
      8'b10000000: n1821 = 1'b1;
      8'b01000000: n1821 = 1'b0;
      8'b00100000: n1821 = 1'b0;
      8'b00010000: n1821 = 1'b0;
      8'b00001000: n1821 = 1'b1;
      8'b00000100: n1821 = 1'b0;
      8'b00000010: n1821 = 1'b0;
      8'b00000001: n1821 = 1'b0;
      default: n1821 = 1'b0;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:125:21  */
  always @*
    case (n1799)
      8'b10000000: n1824 = 1'b0;
      8'b01000000: n1824 = 1'b0;
      8'b00100000: n1824 = 1'b1;
      8'b00010000: n1824 = 1'b0;
      8'b00001000: n1824 = 1'b0;
      8'b00000100: n1824 = 1'b0;
      8'b00000010: n1824 = 1'b0;
      8'b00000001: n1824 = 1'b0;
      default: n1824 = 1'b0;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:125:21  */
  always @*
    case (n1799)
      8'b10000000: n1828 = 1'b0;
      8'b01000000: n1828 = n1789;
      8'b00100000: n1828 = 1'b0;
      8'b00010000: n1828 = n1774;
      8'b00001000: n1828 = 1'b0;
      8'b00000100: n1828 = 1'b1;
      8'b00000010: n1828 = 1'b1;
      8'b00000001: n1828 = n1747;
      default: n1828 = 1'b0;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:125:21  */
  always @*
    case (n1799)
      8'b10000000: n1833 = 1'b0;
      8'b01000000: n1833 = 1'b0;
      8'b00100000: n1833 = 1'b0;
      8'b00010000: n1833 = 1'b1;
      8'b00001000: n1833 = 1'b0;
      8'b00000100: n1833 = 1'b1;
      8'b00000010: n1833 = 1'b1;
      8'b00000001: n1833 = n1750;
      default: n1833 = 1'b0;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:125:21  */
  always @*
    case (n1799)
      8'b10000000: n1834 = n1730;
      8'b01000000: n1834 = n1791;
      8'b00100000: n1834 = n1730;
      8'b00010000: n1834 = n1730;
      8'b00001000: n1834 = n1730;
      8'b00000100: n1834 = n1730;
      8'b00000010: n1834 = n1730;
      8'b00000001: n1834 = n1730;
      default: n1834 = n1730;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:125:21  */
  always @*
    case (n1799)
      8'b10000000: n1839 = 1'b0;
      8'b01000000: n1839 = 1'b0;
      8'b00100000: n1839 = 1'b0;
      8'b00010000: n1839 = 1'b0;
      8'b00001000: n1839 = 1'b0;
      8'b00000100: n1839 = 1'b1;
      8'b00000010: n1839 = 1'b1;
      8'b00000001: n1839 = n1753;
      default: n1839 = 1'b0;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:125:21  */
  always @*
    case (n1799)
      8'b10000000: n1842 = 1'b0;
      8'b01000000: n1842 = 1'b0;
      8'b00100000: n1842 = 1'b0;
      8'b00010000: n1842 = 1'b0;
      8'b00001000: n1842 = 1'b0;
      8'b00000100: n1842 = 1'b0;
      8'b00000010: n1842 = 1'b1;
      8'b00000001: n1842 = n1756;
      default: n1842 = 1'b0;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:125:21  */
  always @*
    case (n1799)
      8'b10000000: n1845 = 1'b0;
      8'b01000000: n1845 = 1'b0;
      8'b00100000: n1845 = 1'b0;
      8'b00010000: n1845 = 1'b1;
      8'b00001000: n1845 = 1'b0;
      8'b00000100: n1845 = 1'b0;
      8'b00000010: n1845 = 1'b0;
      8'b00000001: n1845 = 1'b0;
      default: n1845 = 1'b0;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:125:21  */
  always @*
    case (n1799)
      8'b10000000: n1848 = 1'b0;
      8'b01000000: n1848 = 1'b0;
      8'b00100000: n1848 = 1'b0;
      8'b00010000: n1848 = 1'b0;
      8'b00001000: n1848 = 1'b0;
      8'b00000100: n1848 = 1'b1;
      8'b00000010: n1848 = 1'b0;
      8'b00000001: n1848 = 1'b0;
      default: n1848 = 1'b0;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:125:21  */
  always @*
    case (n1799)
      8'b10000000: n1856 = 1'b1;
      8'b01000000: n1856 = n1794;
      8'b00100000: n1856 = 1'b1;
      8'b00010000: n1856 = 1'b1;
      8'b00001000: n1856 = 1'b1;
      8'b00000100: n1856 = 1'b1;
      8'b00000010: n1856 = 1'b1;
      8'b00000001: n1856 = n1759;
      default: n1856 = 1'b0;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:125:21  */
  always @*
    case (n1799)
      8'b10000000: n1858 = 3'b000;
      8'b01000000: n1858 = 3'b000;
      8'b00100000: n1858 = n1713;
      8'b00010000: n1858 = 3'b000;
      8'b00001000: n1858 = 3'b000;
      8'b00000100: n1858 = 3'b000;
      8'b00000010: n1858 = 3'b000;
      8'b00000001: n1858 = 3'b000;
      default: n1858 = 3'b000;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:125:21  */
  always @*
    case (n1799)
      8'b10000000: n1860 = 2'b00;
      8'b01000000: n1860 = 2'b00;
      8'b00100000: n1860 = 2'b00;
      8'b00010000: n1860 = 2'b00;
      8'b00001000: n1860 = n1766;
      8'b00000100: n1860 = 2'b00;
      8'b00000010: n1860 = 2'b00;
      8'b00000001: n1860 = 2'b00;
      default: n1860 = 2'b00;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:125:21  */
  always @*
    case (n1799)
      8'b10000000: n1862 = 1'b0;
      8'b01000000: n1862 = 1'b0;
      8'b00100000: n1862 = 1'b0;
      8'b00010000: n1862 = 1'b0;
      8'b00001000: n1862 = n1767;
      8'b00000100: n1862 = 1'b0;
      8'b00000010: n1862 = 1'b0;
      8'b00000001: n1862 = 1'b0;
      default: n1862 = 1'b0;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:125:21  */
  always @*
    case (n1799)
      8'b10000000: n1865 = 1'b0;
      8'b01000000: n1865 = 1'b0;
      8'b00100000: n1865 = 1'b0;
      8'b00010000: n1865 = 1'b0;
      8'b00001000: n1865 = 1'b1;
      8'b00000100: n1865 = 1'b0;
      8'b00000010: n1865 = 1'b0;
      8'b00000001: n1865 = 1'b0;
      default: n1865 = 1'b0;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:119:17  */
  assign n1867 = n1737 ? 1'b0 : n1803;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:119:17  */
  assign n1869 = n1737 ? 1'b0 : n1805;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:119:17  */
  assign n1871 = n1737 ? 1'b0 : n1807;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:119:17  */
  assign n1872 = n1737 ? n1714 : n1809;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:119:17  */
  assign n1873 = n1737 ? n1713 : n1812;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:119:17  */
  assign n1875 = n1737 ? 1'b0 : n1817;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:119:17  */
  assign n1877 = n1737 ? 1'b0 : n1821;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:119:17  */
  assign n1879 = n1737 ? 1'b0 : n1824;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:119:17  */
  assign n1882 = n1737 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:119:17  */
  assign n1884 = n1737 ? 1'b0 : n1828;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:119:17  */
  assign n1886 = n1737 ? 1'b0 : n1833;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:119:17  */
  assign n1887 = n1737 ? n1730 : n1834;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:119:17  */
  assign n1889 = n1737 ? 1'b0 : n1839;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:119:17  */
  assign n1891 = n1737 ? 1'b0 : n1842;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:119:17  */
  assign n1893 = n1737 ? 1'b0 : n1845;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:119:17  */
  assign n1895 = n1737 ? 1'b0 : n1848;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:119:17  */
  assign n1897 = n1737 ? 1'b0 : n1856;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:119:17  */
  assign n1899 = n1737 ? 3'b000 : n1858;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:119:17  */
  assign n1901 = n1737 ? 2'b00 : n1860;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:119:17  */
  assign n1903 = n1737 ? 1'b0 : n1862;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:119:17  */
  assign n1905 = n1737 ? 1'b0 : n1865;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:119:17  */
  assign n1908 = n1737 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:117:13  */
  assign n1910 = n1712 == 2'b00;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:237:26  */
  assign n1911 = instruction_byte[0]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:244:40  */
  assign n1912 = instruction_byte[5:4]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:244:53  */
  assign n1914 = n1912 == 2'b00;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:244:21  */
  assign n1916 = n1914 ? n1714 : 3'b000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:237:17  */
  assign n1918 = n1971 ? 3'b000 : n1713;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:244:21  */
  assign n1921 = n1914 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:244:21  */
  assign n1924 = n1914 ? 1'b0 : 1'b1;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:244:21  */
  assign n1927 = n1914 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:268:31  */
  assign n1929 = n1714 == 3'b000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:273:49  */
  assign n1930 = instruction_byte[4:3]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:274:44  */
  assign n1931 = instruction_byte[5]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:279:34  */
  assign n1933 = n1714 == 3'b010;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:285:49  */
  assign n1934 = instruction_byte[4:3]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:286:44  */
  assign n1935 = instruction_byte[5]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:288:34  */
  assign n1937 = n1714 == 3'b100;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:293:33  */
  assign n1938 = instruction_byte[2:1]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:293:46  */
  assign n1940 = n1938 == 2'b11;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:293:21  */
  assign n1943 = n1940 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:288:21  */
  assign n1945 = n1937 ? 1'b0 : n1943;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:279:21  */
  assign n1947 = n1933 ? 1'b1 : n1945;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:279:21  */
  assign n1949 = n1933 ? n1934 : 2'b00;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:279:21  */
  assign n1951 = n1933 ? n1935 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:279:21  */
  assign n1954 = n1933 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:268:21  */
  assign n1956 = n1929 ? 1'b0 : n1947;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:268:21  */
  assign n1957 = n1929 ? n1930 : n1949;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:268:21  */
  assign n1958 = n1929 ? n1931 : n1951;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:268:21  */
  assign n1960 = n1929 ? 1'b1 : n1954;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:237:17  */
  assign n1963 = n1911 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:237:17  */
  assign n1966 = n1911 ? 1'b0 : 1'b1;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:237:17  */
  assign n1969 = n1911 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:237:17  */
  assign n1970 = n1911 ? n1916 : n1714;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:237:17  */
  assign n1971 = n1914 & n1911;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:237:17  */
  assign n1973 = n1911 ? 1'b0 : n1956;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:237:17  */
  assign n1975 = n1911 ? n1921 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:237:17  */
  assign n1977 = n1911 ? n1924 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:237:17  */
  assign n1980 = n1911 ? 1'b0 : 1'b1;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:237:17  */
  assign n1982 = n1911 ? n1927 : 1'b1;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:237:17  */
  assign n1984 = n1911 ? 2'b00 : n1957;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:237:17  */
  assign n1986 = n1911 ? 1'b0 : n1958;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:237:17  */
  assign n1988 = n1911 ? 1'b0 : n1960;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:236:13  */
  assign n1990 = n1712 == 2'b01;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:309:27  */
  assign n1992 = n1713 == 3'b111;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:309:17  */
  assign n1995 = n1992 ? 1'b0 : 1'b1;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:316:27  */
  assign n1997 = n1714 == 3'b111;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:316:17  */
  assign n2000 = n1997 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:302:13  */
  assign n2002 = n1712 == 2'b10;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:329:37  */
  assign n2004 = instruction_byte == 8'b11111111;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:333:30  */
  assign n2006 = n1714 == 3'b111;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:341:30  */
  assign n2008 = n1713 == 3'b111;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:341:17  */
  assign n2011 = n2008 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:341:17  */
  assign n2014 = n2008 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:341:17  */
  assign n2017 = n2008 ? 1'b0 : 1'b1;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:341:17  */
  assign n2020 = n2008 ? 1'b0 : 1'b1;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:341:17  */
  assign n2023 = n2008 ? 1'b0 : 1'b1;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:333:17  */
  assign n2025 = n2006 ? 1'b1 : n2011;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:333:17  */
  assign n2027 = n2006 ? 1'b0 : n2014;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:333:17  */
  assign n2029 = n2006 ? 1'b1 : n2017;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:333:17  */
  assign n2032 = n2006 ? 1'b0 : 1'b1;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:333:17  */
  assign n2034 = n2006 ? 1'b0 : n2020;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:333:17  */
  assign n2036 = n2006 ? 1'b1 : n2023;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:329:17  */
  assign n2038 = n2004 ? 1'b0 : n2025;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:329:17  */
  assign n2040 = n2004 ? 1'b0 : n2027;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:329:17  */
  assign n2043 = n2004 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:329:17  */
  assign n2045 = n2004 ? 1'b0 : n2029;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:329:17  */
  assign n2047 = n2004 ? 1'b0 : n2032;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:329:17  */
  assign n2049 = n2004 ? 1'b0 : n2034;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:329:17  */
  assign n2051 = n2004 ? 1'b0 : n2036;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:329:17  */
  assign n2054 = n2004 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:328:13  */
  assign n2056 = n1712 == 2'b11;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:113:9  */
  assign n2057 = {n2056, n2002, n1990, n1910};
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:113:9  */
  always @*
    case (n2057)
      4'b1000: n2059 = n2038;
      4'b0100: n2059 = n2000;
      4'b0010: n2059 = n1963;
      4'b0001: n2059 = n1867;
      default: n2059 = 1'b0;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:113:9  */
  always @*
    case (n2057)
      4'b1000: n2062 = 1'b0;
      4'b0100: n2062 = 1'b0;
      4'b0010: n2062 = n1966;
      4'b0001: n2062 = n1869;
      default: n2062 = 1'b0;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:113:9  */
  always @*
    case (n2057)
      4'b1000: n2065 = 1'b0;
      4'b0100: n2065 = 1'b0;
      4'b0010: n2065 = n1969;
      4'b0001: n2065 = 1'b0;
      default: n2065 = 1'b0;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:113:9  */
  always @*
    case (n2057)
      4'b1000: n2068 = n2040;
      4'b0100: n2068 = 1'b0;
      4'b0010: n2068 = 1'b0;
      4'b0001: n2068 = n1871;
      default: n2068 = 1'b0;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:113:9  */
  always @*
    case (n2057)
      4'b1000: n2070 = n1714;
      4'b0100: n2070 = n1714;
      4'b0010: n2070 = n1970;
      4'b0001: n2070 = n1872;
      default: n2070 = n1714;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:113:9  */
  always @*
    case (n2057)
      4'b1000: n2072 = n1713;
      4'b0100: n2072 = 3'b000;
      4'b0010: n2072 = n1918;
      4'b0001: n2072 = n1873;
      default: n2072 = n1713;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:113:9  */
  always @*
    case (n2057)
      4'b1000: n2075 = 1'b0;
      4'b0100: n2075 = 1'b1;
      4'b0010: n2075 = 1'b0;
      4'b0001: n2075 = n1875;
      default: n2075 = 1'b0;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:113:9  */
  always @*
    case (n2057)
      4'b1000: n2078 = 1'b0;
      4'b0100: n2078 = 1'b0;
      4'b0010: n2078 = n1973;
      4'b0001: n2078 = 1'b0;
      default: n2078 = 1'b0;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:113:9  */
  always @*
    case (n2057)
      4'b1000: n2081 = 1'b0;
      4'b0100: n2081 = 1'b0;
      4'b0010: n2081 = 1'b0;
      4'b0001: n2081 = n1877;
      default: n2081 = 1'b0;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:113:9  */
  always @*
    case (n2057)
      4'b1000: n2084 = 1'b0;
      4'b0100: n2084 = 1'b0;
      4'b0010: n2084 = 1'b0;
      4'b0001: n2084 = n1879;
      default: n2084 = 1'b0;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:113:9  */
  always @*
    case (n2057)
      4'b1000: n2087 = n2043;
      4'b0100: n2087 = 1'b0;
      4'b0010: n2087 = 1'b0;
      4'b0001: n2087 = n1882;
      default: n2087 = 1'b0;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:113:9  */
  always @*
    case (n2057)
      4'b1000: n2090 = n2045;
      4'b0100: n2090 = n1995;
      4'b0010: n2090 = n1975;
      4'b0001: n2090 = n1884;
      default: n2090 = 1'b0;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:113:9  */
  always @*
    case (n2057)
      4'b1000: n2094 = n2047;
      4'b0100: n2094 = 1'b1;
      4'b0010: n2094 = n1977;
      4'b0001: n2094 = n1886;
      default: n2094 = 1'b0;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:113:9  */
  always @*
    case (n2057)
      4'b1000: n2096 = n1730;
      4'b0100: n2096 = n1730;
      4'b0010: n2096 = n1730;
      4'b0001: n2096 = n1887;
      default: n2096 = n1730;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:113:9  */
  always @*
    case (n2057)
      4'b1000: n2099 = n2049;
      4'b0100: n2099 = 1'b1;
      4'b0010: n2099 = n1980;
      4'b0001: n2099 = n1889;
      default: n2099 = 1'b0;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:113:9  */
  always @*
    case (n2057)
      4'b1000: n2102 = 1'b0;
      4'b0100: n2102 = 1'b0;
      4'b0010: n2102 = 1'b0;
      4'b0001: n2102 = n1891;
      default: n2102 = 1'b0;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:113:9  */
  always @*
    case (n2057)
      4'b1000: n2106 = 1'b0;
      4'b0100: n2106 = 1'b1;
      4'b0010: n2106 = 1'b0;
      4'b0001: n2106 = n1893;
      default: n2106 = 1'b0;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:113:9  */
  always @*
    case (n2057)
      4'b1000: n2109 = 1'b0;
      4'b0100: n2109 = 1'b0;
      4'b0010: n2109 = 1'b0;
      4'b0001: n2109 = n1895;
      default: n2109 = 1'b0;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:113:9  */
  always @*
    case (n2057)
      4'b1000: n2113 = n2051;
      4'b0100: n2113 = 1'b1;
      4'b0010: n2113 = n1982;
      4'b0001: n2113 = n1897;
      default: n2113 = 1'b0;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:113:9  */
  always @*
    case (n2057)
      4'b1000: n2116 = 3'b000;
      4'b0100: n2116 = 3'b000;
      4'b0010: n2116 = 3'b000;
      4'b0001: n2116 = n1899;
      default: n2116 = 3'b000;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:113:9  */
  always @*
    case (n2057)
      4'b1000: n2119 = 2'b00;
      4'b0100: n2119 = 2'b00;
      4'b0010: n2119 = n1984;
      4'b0001: n2119 = n1901;
      default: n2119 = 2'b00;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:113:9  */
  always @*
    case (n2057)
      4'b1000: n2122 = 1'b0;
      4'b0100: n2122 = 1'b0;
      4'b0010: n2122 = n1986;
      4'b0001: n2122 = n1903;
      default: n2122 = 1'b0;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:113:9  */
  always @*
    case (n2057)
      4'b1000: n2125 = 1'b0;
      4'b0100: n2125 = 1'b0;
      4'b0010: n2125 = n1988;
      4'b0001: n2125 = n1905;
      default: n2125 = 1'b0;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/instruction_decoder.vhdl:113:9  */
  always @*
    case (n2057)
      4'b1000: n2128 = n2054;
      4'b0100: n2128 = 1'b0;
      4'b0010: n2128 = 1'b0;
      4'b0001: n2128 = n1908;
      default: n2128 = 1'b0;
    endcase
endmodule

module machine_cycle_control_Brtl
  (input  clk,
   input  phi1_rising,
   input  reset,
   input  state_t1,
   input  state_t2,
   input  state_t3,
   input  state_t4,
   input  state_t5,
   input  state_t1i,
   input  state_half,
   input  instr_needs_immediate,
   input  instr_needs_address,
   input  instr_is_io,
   input  instr_is_write,
   input  instr_is_hlt,
   input  instr_needs_t4t5,
   input  instr_is_mem_indirect,
   input  eval_condition,
   input  condition_met,
   output advance_state,
   output cycle_done,
   output instr_is_hlt_flag,
   output [1:0] cycle_type,
   output [1:0] current_cycle,
   output [1:0] next_cycle);
  reg [1:0] cycle_count;
  wire needs_cycle_2;
  wire needs_cycle_3;
  reg advance_latch;
  reg cycle_done_latch;
  wire is_lmr;
  reg [1:0] cycle_type_latch;
  reg instr_is_hlt_latch;
  wire needs_t4t5_this_cycle;
  reg prev_state_t1;
  reg prev_state_t2;
  reg prev_state_t3;
  reg prev_state_t4;
  reg prev_state_t5;
  reg prev_state_t1i;
  wire t1_rising;
  wire t2_rising;
  wire t3_rising;
  wire t4_rising;
  wire t5_rising;
  wire t1i_rising;
  wire n1422;
  wire [1:0] n1424;
  wire [31:0] n1426;
  wire n1428;
  wire n1429;
  wire [1:0] n1430;
  wire [31:0] n1432;
  wire n1434;
  wire n1435;
  wire [1:0] n1436;
  wire [31:0] n1439;
  wire n1441;
  wire n1442;
  wire n1443;
  wire n1444;
  wire n1445;
  wire n1446;
  wire [31:0] n1448;
  wire n1450;
  wire n1451;
  wire n1452;
  wire [31:0] n1454;
  wire n1456;
  wire n1457;
  wire n1458;
  wire n1459;
  wire n1460;
  wire n1461;
  wire n1463;
  wire n1464;
  wire n1465;
  wire n1466;
  wire n1467;
  wire n1468;
  wire n1469;
  wire n1470;
  wire n1471;
  wire n1472;
  wire n1473;
  wire n1474;
  wire n1475;
  wire n1476;
  wire n1477;
  wire n1478;
  wire n1479;
  wire [31:0] n1484;
  wire n1486;
  wire [31:0] n1487;
  wire n1489;
  wire n1490;
  wire n1491;
  wire [31:0] n1492;
  wire n1494;
  wire n1495;
  wire n1496;
  wire n1497;
  wire [1:0] n1500;
  wire [1:0] n1502;
  wire [1:0] n1504;
  wire n1506;
  wire [31:0] n1507;
  wire n1509;
  wire n1510;
  wire n1511;
  wire [31:0] n1512;
  wire n1514;
  wire n1515;
  wire n1516;
  wire [31:0] n1517;
  wire n1519;
  wire n1520;
  wire [31:0] n1521;
  wire n1523;
  wire n1524;
  wire n1526;
  wire n1528;
  wire n1529;
  wire n1531;
  wire n1533;
  wire n1534;
  wire n1535;
  wire n1536;
  wire [31:0] n1537;
  wire n1539;
  wire n1540;
  wire n1541;
  wire n1542;
  wire n1543;
  wire n1544;
  wire n1545;
  wire [31:0] n1546;
  wire n1548;
  wire n1549;
  wire n1550;
  wire n1551;
  wire n1553;
  wire n1555;
  wire n1556;
  wire n1557;
  wire [31:0] n1558;
  wire n1560;
  wire n1561;
  wire n1562;
  wire n1563;
  wire n1564;
  wire n1565;
  wire [31:0] n1566;
  wire n1568;
  wire n1569;
  wire n1571;
  wire n1573;
  wire n1574;
  wire n1575;
  wire [31:0] n1576;
  wire n1578;
  wire n1579;
  wire n1580;
  wire [31:0] n1581;
  wire n1583;
  wire n1584;
  wire n1585;
  wire n1586;
  wire [31:0] n1587;
  wire n1589;
  wire n1590;
  wire n1591;
  wire n1593;
  wire n1594;
  wire n1595;
  wire n1596;
  wire n1597;
  wire n1598;
  wire n1599;
  wire n1600;
  wire n1601;
  wire n1603;
  wire n1605;
  wire n1607;
  wire n1609;
  wire n1611;
  wire n1613;
  wire [31:0] n1614;
  wire n1616;
  wire n1617;
  wire [31:0] n1618;
  wire n1620;
  wire n1621;
  wire [1:0] n1624;
  wire [1:0] n1626;
  wire [1:0] n1627;
  wire [1:0] n1629;
  wire n1633;
  wire [1:0] n1664;
  reg [1:0] n1665;
  wire n1666;
  reg n1667;
  wire n1668;
  wire n1669;
  wire n1670;
  reg n1671;
  wire [1:0] n1672;
  reg [1:0] n1673;
  wire n1674;
  reg n1675;
  wire n1676;
  reg n1677;
  wire n1678;
  reg n1679;
  wire n1680;
  reg n1681;
  wire n1682;
  reg n1683;
  wire n1684;
  reg n1685;
  wire n1686;
  reg n1687;
  assign advance_state = advance_latch; //(module output)
  assign cycle_done = cycle_done_latch; //(module output)
  assign instr_is_hlt_flag = instr_is_hlt_latch; //(module output)
  assign cycle_type = cycle_type_latch; //(module output)
  assign current_cycle = cycle_count; //(module output)
  assign next_cycle = n1424; //(module output)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:70:12  */
  always @*
    cycle_count = n1665; // (isignal)
  initial
    cycle_count = 2'b00;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:73:12  */
  assign needs_cycle_2 = n1422; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:74:12  */
  assign needs_cycle_3 = instr_needs_address; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:77:12  */
  always @*
    advance_latch = n1667; // (isignal)
  initial
    advance_latch = 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:80:12  */
  always @*
    cycle_done_latch = n1671; // (isignal)
  initial
    cycle_done_latch = 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:84:12  */
  assign is_lmr = n1467; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:87:12  */
  always @*
    cycle_type_latch = n1673; // (isignal)
  initial
    cycle_type_latch = 2'b00;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:90:12  */
  always @*
    instr_is_hlt_latch = n1675; // (isignal)
  initial
    instr_is_hlt_latch = 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:93:12  */
  assign needs_t4t5_this_cycle = n1446; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:96:12  */
  always @*
    prev_state_t1 = n1677; // (isignal)
  initial
    prev_state_t1 = 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:97:12  */
  always @*
    prev_state_t2 = n1679; // (isignal)
  initial
    prev_state_t2 = 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:98:12  */
  always @*
    prev_state_t3 = n1681; // (isignal)
  initial
    prev_state_t3 = 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:99:12  */
  always @*
    prev_state_t4 = n1683; // (isignal)
  initial
    prev_state_t4 = 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:100:12  */
  always @*
    prev_state_t5 = n1685; // (isignal)
  initial
    prev_state_t5 = 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:101:12  */
  always @*
    prev_state_t1i = n1687; // (isignal)
  initial
    prev_state_t1i = 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:104:12  */
  assign t1_rising = n1469; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:105:12  */
  assign t2_rising = n1471; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:106:12  */
  assign t3_rising = n1473; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:107:12  */
  assign t4_rising = n1475; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:108:12  */
  assign t5_rising = n1477; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:109:12  */
  assign t1i_rising = n1479; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:117:44  */
  assign n1422 = instr_needs_immediate | instr_needs_address;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:123:21  */
  assign n1424 = state_t1i ? 2'b00 : n1430;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:124:39  */
  assign n1426 = {30'b0, cycle_count};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:124:39  */
  assign n1428 = n1426 == 32'b00000000000000000000000000000000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:124:43  */
  assign n1429 = needs_cycle_2 & n1428;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:123:42  */
  assign n1430 = n1429 ? 2'b01 : n1436;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:125:39  */
  assign n1432 = {30'b0, cycle_count};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:125:39  */
  assign n1434 = n1432 == 32'b00000000000000000000000000000001;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:125:43  */
  assign n1435 = needs_cycle_3 & n1434;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:124:68  */
  assign n1436 = n1435 ? 2'b10 : 2'b00;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:129:52  */
  assign n1439 = {30'b0, cycle_count};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:129:52  */
  assign n1441 = n1439 == 32'b00000000000000000000000000000000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:129:56  */
  assign n1442 = instr_needs_t4t5 & n1441;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:130:78  */
  assign n1443 = ~eval_condition;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:130:60  */
  assign n1444 = condition_met | n1443;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:129:83  */
  assign n1445 = n1444 & n1442;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:129:34  */
  assign n1446 = n1445 ? 1'b1 : n1452;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:131:52  */
  assign n1448 = {30'b0, cycle_count};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:131:52  */
  assign n1450 = n1448 == 32'b00000000000000000000000000000001;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:131:56  */
  assign n1451 = instr_needs_t4t5 & n1450;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:130:86  */
  assign n1452 = n1451 ? 1'b1 : n1461;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:132:52  */
  assign n1454 = {30'b0, cycle_count};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:132:52  */
  assign n1456 = n1454 == 32'b00000000000000000000000000000010;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:132:56  */
  assign n1457 = instr_needs_t4t5 & n1456;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:133:78  */
  assign n1458 = ~eval_condition;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:133:60  */
  assign n1459 = condition_met | n1458;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:132:83  */
  assign n1460 = n1459 & n1457;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:131:84  */
  assign n1461 = n1460 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:137:37  */
  assign n1463 = instr_is_mem_indirect & instr_is_write;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:138:16  */
  assign n1464 = ~instr_needs_address;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:137:56  */
  assign n1465 = n1463 & n1464;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:138:46  */
  assign n1466 = ~instr_is_io;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:138:41  */
  assign n1467 = n1465 & n1466;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:147:33  */
  assign n1468 = ~prev_state_t1;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:147:29  */
  assign n1469 = state_t1 & n1468;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:148:33  */
  assign n1470 = ~prev_state_t2;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:148:29  */
  assign n1471 = state_t2 & n1470;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:149:33  */
  assign n1472 = ~prev_state_t3;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:149:29  */
  assign n1473 = state_t3 & n1472;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:150:33  */
  assign n1474 = ~prev_state_t4;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:150:29  */
  assign n1475 = state_t4 & n1474;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:151:33  */
  assign n1476 = ~prev_state_t5;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:151:29  */
  assign n1477 = state_t5 & n1476;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:152:33  */
  assign n1478 = ~prev_state_t1i;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:152:29  */
  assign n1479 = state_t1i & n1478;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:179:32  */
  assign n1484 = {30'b0, cycle_count};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:179:32  */
  assign n1486 = n1484 == 32'b00000000000000000000000000000000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:186:37  */
  assign n1487 = {30'b0, cycle_count};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:186:37  */
  assign n1489 = n1487 == 32'b00000000000000000000000000000001;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:186:65  */
  assign n1490 = ~instr_needs_address;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:186:41  */
  assign n1491 = n1490 & n1489;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:187:37  */
  assign n1492 = {30'b0, cycle_count};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:187:37  */
  assign n1494 = n1492 == 32'b00000000000000000000000000000010;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:187:41  */
  assign n1495 = instr_needs_address & n1494;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:186:72  */
  assign n1496 = n1491 | n1495;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:185:44  */
  assign n1497 = n1496 & instr_is_write;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:185:17  */
  assign n1500 = n1497 ? 2'b11 : 2'b01;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:182:17  */
  assign n1502 = instr_is_io ? 2'b10 : n1500;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:179:17  */
  assign n1504 = n1486 ? 2'b00 : n1502;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:211:42  */
  assign n1506 = ~needs_t4t5_this_cycle;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:212:34  */
  assign n1507 = {30'b0, cycle_count};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:212:34  */
  assign n1509 = n1507 == 32'b00000000000000000000000000000001;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:212:56  */
  assign n1510 = ~needs_cycle_3;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:212:38  */
  assign n1511 = n1510 & n1509;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:213:34  */
  assign n1512 = {30'b0, cycle_count};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:213:34  */
  assign n1514 = n1512 == 32'b00000000000000000000000000000010;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:212:63  */
  assign n1515 = n1511 | n1514;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:211:48  */
  assign n1516 = n1515 & n1506;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:216:58  */
  assign n1517 = {30'b0, cycle_count};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:216:58  */
  assign n1519 = n1517 == 32'b00000000000000000000000000000000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:216:42  */
  assign n1520 = n1519 & instr_is_hlt;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:220:35  */
  assign n1521 = {30'b0, cycle_count};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:220:35  */
  assign n1523 = n1521 == 32'b00000000000000000000000000000001;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:220:39  */
  assign n1524 = needs_cycle_3 & n1523;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:220:17  */
  assign n1526 = n1524 ? 1'b1 : cycle_done_latch;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:216:17  */
  assign n1528 = n1520 ? 1'b1 : advance_latch;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:216:17  */
  assign n1529 = n1520 ? cycle_done_latch : n1526;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:216:17  */
  assign n1531 = n1520 ? 1'b1 : instr_is_hlt_latch;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:211:17  */
  assign n1533 = n1516 ? 1'b1 : n1528;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:211:17  */
  assign n1534 = n1516 ? cycle_done_latch : n1529;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:211:17  */
  assign n1535 = n1516 ? instr_is_hlt_latch : n1531;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:227:34  */
  assign n1536 = state_half & state_t3;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:232:32  */
  assign n1537 = {30'b0, cycle_count};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:232:32  */
  assign n1539 = n1537 == 32'b00000000000000000000000000000000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:232:54  */
  assign n1540 = ~needs_cycle_2;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:232:36  */
  assign n1541 = n1540 & n1539;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:233:42  */
  assign n1542 = ~needs_t4t5_this_cycle;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:232:60  */
  assign n1543 = n1542 & n1541;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:233:65  */
  assign n1544 = ~instr_is_hlt;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:233:48  */
  assign n1545 = n1544 & n1543;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:238:35  */
  assign n1546 = {30'b0, cycle_count};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:238:35  */
  assign n1548 = n1546 == 32'b00000000000000000000000000000000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:238:39  */
  assign n1549 = needs_cycle_2 & n1548;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:238:74  */
  assign n1550 = ~is_lmr;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:238:63  */
  assign n1551 = n1550 & n1549;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:238:17  */
  assign n1553 = n1551 ? 1'b1 : cycle_done_latch;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:232:17  */
  assign n1555 = n1545 ? 1'b1 : advance_latch;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:232:17  */
  assign n1556 = n1545 ? cycle_done_latch : n1553;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:245:33  */
  assign n1557 = ~instr_is_hlt;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:245:55  */
  assign n1558 = {30'b0, cycle_count};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:245:55  */
  assign n1560 = n1558 == 32'b00000000000000000000000000000000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:245:39  */
  assign n1561 = n1560 & n1557;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:246:42  */
  assign n1562 = ~needs_t4t5_this_cycle;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:245:59  */
  assign n1563 = n1562 & n1561;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:246:66  */
  assign n1564 = ~needs_cycle_2;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:246:48  */
  assign n1565 = n1564 & n1563;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:250:35  */
  assign n1566 = {30'b0, cycle_count};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:250:35  */
  assign n1568 = n1566 == 32'b00000000000000000000000000000000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:250:39  */
  assign n1569 = is_lmr & n1568;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:250:17  */
  assign n1571 = n1569 ? 1'b1 : cycle_done_latch;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:245:17  */
  assign n1573 = n1565 ? 1'b1 : advance_latch;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:245:17  */
  assign n1574 = n1565 ? cycle_done_latch : n1571;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:258:33  */
  assign n1575 = ~instr_is_hlt;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:259:34  */
  assign n1576 = {30'b0, cycle_count};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:259:34  */
  assign n1578 = n1576 == 32'b00000000000000000000000000000000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:259:56  */
  assign n1579 = ~needs_cycle_2;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:259:38  */
  assign n1580 = n1579 & n1578;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:260:34  */
  assign n1581 = {30'b0, cycle_count};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:260:34  */
  assign n1583 = n1581 == 32'b00000000000000000000000000000001;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:260:56  */
  assign n1584 = ~needs_cycle_3;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:260:38  */
  assign n1585 = n1584 & n1583;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:259:63  */
  assign n1586 = n1580 | n1585;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:261:34  */
  assign n1587 = {30'b0, cycle_count};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:261:34  */
  assign n1589 = n1587 == 32'b00000000000000000000000000000010;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:260:63  */
  assign n1590 = n1586 | n1589;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:258:39  */
  assign n1591 = n1590 & n1575;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:256:13  */
  assign n1593 = n1594 ? 1'b1 : advance_latch;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:256:13  */
  assign n1594 = n1591 & t5_rising;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:244:13  */
  assign n1595 = t4_rising ? n1573 : n1593;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:244:13  */
  assign n1596 = t4_rising ? n1574 : cycle_done_latch;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:227:13  */
  assign n1597 = n1536 ? n1555 : n1595;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:227:13  */
  assign n1598 = n1536 ? n1556 : n1596;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:209:13  */
  assign n1599 = t3_rising ? n1533 : n1597;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:209:13  */
  assign n1600 = t3_rising ? n1534 : n1598;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:209:13  */
  assign n1601 = t3_rising ? n1535 : instr_is_hlt_latch;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:203:13  */
  assign n1603 = t1_rising ? 1'b0 : n1599;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:203:13  */
  assign n1605 = t1_rising ? 1'b0 : n1600;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:203:13  */
  assign n1607 = t1_rising ? 1'b0 : n1601;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:197:13  */
  assign n1609 = t1i_rising ? 1'b0 : n1603;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:197:13  */
  assign n1611 = t1i_rising ? 1'b0 : n1605;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:197:13  */
  assign n1613 = t1i_rising ? 1'b0 : n1607;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:272:32  */
  assign n1614 = {30'b0, cycle_count};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:272:32  */
  assign n1616 = n1614 == 32'b00000000000000000000000000000000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:272:36  */
  assign n1617 = needs_cycle_2 & n1616;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:274:35  */
  assign n1618 = {30'b0, cycle_count};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:274:35  */
  assign n1620 = n1618 == 32'b00000000000000000000000000000001;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:274:39  */
  assign n1621 = needs_cycle_3 & n1620;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:274:17  */
  assign n1624 = n1621 ? 2'b10 : 2'b00;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:272:17  */
  assign n1626 = n1617 ? 2'b01 : n1624;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:270:13  */
  assign n1627 = t1_rising ? n1626 : cycle_count;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:268:13  */
  assign n1629 = t1i_rising ? 2'b00 : n1627;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:168:32  */
  assign n1633 = t2_rising & phi1_rising;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:168:9  */
  assign n1664 = phi1_rising ? n1629 : cycle_count;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:168:9  */
  always @(posedge clk or posedge reset)
    if (reset)
      n1665 <= 2'b00;
    else
      n1665 <= n1664;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:168:9  */
  assign n1666 = phi1_rising ? n1609 : advance_latch;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:168:9  */
  always @(posedge clk or posedge reset)
    if (reset)
      n1667 <= 1'b0;
    else
      n1667 <= n1666;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:80:12  */
  assign n1668 = ~reset;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:80:12  */
  assign n1669 = phi1_rising & n1668;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:168:9  */
  assign n1670 = n1669 ? n1611 : cycle_done_latch;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:168:9  */
  always @(posedge clk)
    n1671 <= n1670;
  initial
    n1671 = 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:168:9  */
  assign n1672 = n1633 ? n1504 : cycle_type_latch;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:168:9  */
  always @(posedge clk or posedge reset)
    if (reset)
      n1673 <= 2'b00;
    else
      n1673 <= n1672;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:168:9  */
  assign n1674 = phi1_rising ? n1613 : instr_is_hlt_latch;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:168:9  */
  always @(posedge clk or posedge reset)
    if (reset)
      n1675 <= 1'b0;
    else
      n1675 <= n1674;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:168:9  */
  assign n1676 = phi1_rising ? state_t1 : prev_state_t1;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:168:9  */
  always @(posedge clk or posedge reset)
    if (reset)
      n1677 <= 1'b0;
    else
      n1677 <= n1676;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:168:9  */
  assign n1678 = phi1_rising ? state_t2 : prev_state_t2;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:168:9  */
  always @(posedge clk or posedge reset)
    if (reset)
      n1679 <= 1'b0;
    else
      n1679 <= n1678;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:168:9  */
  assign n1680 = phi1_rising ? state_t3 : prev_state_t3;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:168:9  */
  always @(posedge clk or posedge reset)
    if (reset)
      n1681 <= 1'b0;
    else
      n1681 <= n1680;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:168:9  */
  assign n1682 = phi1_rising ? state_t4 : prev_state_t4;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:168:9  */
  always @(posedge clk or posedge reset)
    if (reset)
      n1683 <= 1'b0;
    else
      n1683 <= n1682;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:168:9  */
  assign n1684 = phi1_rising ? state_t5 : prev_state_t5;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:168:9  */
  always @(posedge clk or posedge reset)
    if (reset)
      n1685 <= 1'b0;
    else
      n1685 <= n1684;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:168:9  */
  assign n1686 = phi1_rising ? state_t1i : prev_state_t1i;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/machine_cycle_control.vhdl:168:9  */
  always @(posedge clk or posedge reset)
    if (reset)
      n1687 <= 1'b0;
    else
      n1687 <= n1686;
endmodule

module interrupt_ready_ff_Brtl
  (input  clk,
   input  phi2_rising,
   input  reset,
   input  int_request,
   input  int_clear,
   input  ready_in,
   output interrupt_pending,
   output ready_status);
  reg int_ff;
  reg ready_ff;
  wire n1386;
  wire n1388;
  wire n1401;
  reg n1402;
  wire n1403;
  reg n1404;
  assign interrupt_pending = int_ff; //(module output)
  assign ready_status = ready_ff; //(module output)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/interrupt_ready_ff.vhdl:45:12  */
  always @*
    int_ff = n1402; // (isignal)
  initial
    int_ff = 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/interrupt_ready_ff.vhdl:46:12  */
  always @*
    ready_ff = n1404; // (isignal)
  initial
    ready_ff = 1'b1;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/interrupt_ready_ff.vhdl:58:13  */
  assign n1386 = int_request ? 1'b1 : int_ff;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/interrupt_ready_ff.vhdl:56:13  */
  assign n1388 = int_clear ? 1'b0 : n1386;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/interrupt_ready_ff.vhdl:55:9  */
  assign n1401 = phi2_rising ? n1388 : int_ff;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/interrupt_ready_ff.vhdl:55:9  */
  always @(posedge clk or posedge reset)
    if (reset)
      n1402 <= 1'b0;
    else
      n1402 <= n1401;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/interrupt_ready_ff.vhdl:69:9  */
  assign n1403 = phi2_rising ? ready_in : ready_ff;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/interrupt_ready_ff.vhdl:69:9  */
  always @(posedge clk or posedge reset)
    if (reset)
      n1404 <= 1'b1;
    else
      n1404 <= n1403;
endmodule

module state_timing_generator_Brtl
  (input  clk,
   input  phi2_falling,
   input  reset,
   input  advance_state,
   input  cycle_done,
   input  interrupt_pending,
   input  ready,
   input  instr_is_hlt_flag,
   input  transition_to_stopped,
   output state_t1,
   output state_t2,
   output state_t3,
   output state_t4,
   output state_t5,
   output state_t1i,
   output state_stopped,
   output state_half,
   output status_s0,
   output status_s1,
   output status_s2);
  reg [2:0] current_state;
  wire [2:0] next_state;
  reg cycle_count;
  wire n1223;
  wire n1224;
  wire n1228;
  wire n1229;
  wire n1233;
  wire n1234;
  wire n1238;
  wire n1239;
  wire n1243;
  wire n1244;
  wire n1248;
  wire n1249;
  wire n1253;
  wire n1254;
  wire n1258;
  wire n1260;
  wire n1261;
  wire n1263;
  wire n1264;
  wire n1266;
  wire n1267;
  wire n1268;
  wire n1272;
  wire n1274;
  wire n1275;
  wire n1277;
  wire n1278;
  wire n1280;
  wire n1281;
  wire n1282;
  wire n1286;
  wire n1288;
  wire n1289;
  wire n1291;
  wire n1292;
  wire n1294;
  wire n1295;
  wire n1296;
  wire [2:0] n1301;
  wire n1303;
  wire [2:0] n1305;
  wire n1307;
  wire [2:0] n1309;
  wire n1311;
  wire n1312;
  wire [2:0] n1315;
  wire [2:0] n1316;
  wire n1318;
  wire n1319;
  wire [2:0] n1321;
  wire n1323;
  wire [2:0] n1326;
  wire [2:0] n1329;
  wire [2:0] n1330;
  wire [2:0] n1332;
  wire [2:0] n1333;
  wire n1335;
  wire [2:0] n1338;
  wire [2:0] n1341;
  wire [2:0] n1342;
  wire [2:0] n1343;
  wire n1345;
  wire n1346;
  wire [2:0] n1349;
  wire [2:0] n1350;
  wire n1352;
  wire [7:0] n1353;
  reg [2:0] n1355;
  wire n1361;
  wire [2:0] n1362;
  wire n1365;
  wire [2:0] n1373;
  reg [2:0] n1374;
  wire n1375;
  reg n1376;
  assign state_t1 = n1224; //(module output)
  assign state_t2 = n1229; //(module output)
  assign state_t3 = n1234; //(module output)
  assign state_t4 = n1239; //(module output)
  assign state_t5 = n1244; //(module output)
  assign state_t1i = n1249; //(module output)
  assign state_stopped = n1254; //(module output)
  assign state_half = cycle_count; //(module output)
  assign status_s0 = n1268; //(module output)
  assign status_s1 = n1282; //(module output)
  assign status_s2 = n1296; //(module output)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:61:12  */
  always @*
    current_state = n1374; // (isignal)
  initial
    current_state = 3'b000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:62:12  */
  assign next_state = n1355; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:65:12  */
  always @*
    cycle_count = n1376; // (isignal)
  initial
    cycle_count = 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:70:45  */
  assign n1223 = current_state == 3'b001;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:70:26  */
  assign n1224 = n1223 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:71:45  */
  assign n1228 = current_state == 3'b010;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:71:26  */
  assign n1229 = n1228 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:72:45  */
  assign n1233 = current_state == 3'b100;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:72:26  */
  assign n1234 = n1233 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:73:45  */
  assign n1238 = current_state == 3'b101;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:73:26  */
  assign n1239 = n1238 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:74:45  */
  assign n1243 = current_state == 3'b110;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:74:26  */
  assign n1244 = n1243 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:75:45  */
  assign n1248 = current_state == 3'b111;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:75:26  */
  assign n1249 = n1248 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:76:45  */
  assign n1253 = current_state == 3'b000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:76:26  */
  assign n1254 = n1253 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:91:42  */
  assign n1258 = current_state == 3'b000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:91:71  */
  assign n1260 = current_state == 3'b100;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:91:54  */
  assign n1261 = n1258 | n1260;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:91:95  */
  assign n1263 = current_state == 3'b101;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:91:78  */
  assign n1264 = n1261 | n1263;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:91:119  */
  assign n1266 = current_state == 3'b110;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:91:102  */
  assign n1267 = n1264 | n1266;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:91:22  */
  assign n1268 = n1267 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:92:42  */
  assign n1272 = current_state == 3'b000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:92:71  */
  assign n1274 = current_state == 3'b001;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:92:54  */
  assign n1275 = n1272 | n1274;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:92:95  */
  assign n1277 = current_state == 3'b101;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:92:78  */
  assign n1278 = n1275 | n1277;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:92:119  */
  assign n1280 = current_state == 3'b111;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:92:102  */
  assign n1281 = n1278 | n1280;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:92:22  */
  assign n1282 = n1281 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:93:42  */
  assign n1286 = current_state == 3'b010;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:93:66  */
  assign n1288 = current_state == 3'b101;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:93:49  */
  assign n1289 = n1286 | n1288;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:93:90  */
  assign n1291 = current_state == 3'b110;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:93:73  */
  assign n1292 = n1289 | n1291;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:93:114  */
  assign n1294 = current_state == 3'b111;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:93:97  */
  assign n1295 = n1292 | n1294;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:93:22  */
  assign n1296 = n1295 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:105:17  */
  assign n1301 = interrupt_pending ? 3'b111 : current_state;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:102:13  */
  assign n1303 = current_state == 3'b000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:112:17  */
  assign n1305 = cycle_count ? 3'b010 : current_state;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:110:13  */
  assign n1307 = current_state == 3'b001;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:118:17  */
  assign n1309 = cycle_count ? 3'b010 : current_state;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:116:13  */
  assign n1311 = current_state == 3'b111;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:127:30  */
  assign n1312 = ~ready;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:127:21  */
  assign n1315 = n1312 ? 3'b011 : 3'b100;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:126:17  */
  assign n1316 = cycle_count ? n1315 : current_state;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:122:13  */
  assign n1318 = current_state == 3'b010;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:136:38  */
  assign n1319 = ready & cycle_count;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:136:17  */
  assign n1321 = n1319 ? 3'b100 : current_state;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:134:13  */
  assign n1323 = current_state == 3'b011;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:149:25  */
  assign n1326 = interrupt_pending ? 3'b111 : 3'b001;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:157:21  */
  assign n1329 = cycle_done ? 3'b001 : 3'b101;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:147:21  */
  assign n1330 = advance_state ? n1326 : n1329;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:144:21  */
  assign n1332 = transition_to_stopped ? 3'b000 : n1330;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:142:17  */
  assign n1333 = cycle_count ? n1332 : current_state;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:140:13  */
  assign n1335 = current_state == 3'b100;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:170:25  */
  assign n1338 = interrupt_pending ? 3'b111 : 3'b001;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:175:21  */
  assign n1341 = cycle_done ? 3'b001 : 3'b110;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:168:21  */
  assign n1342 = advance_state ? n1338 : n1341;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:167:17  */
  assign n1343 = cycle_count ? n1342 : current_state;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:165:13  */
  assign n1345 = current_state == 3'b101;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:192:48  */
  assign n1346 = advance_state & interrupt_pending;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:192:21  */
  assign n1349 = n1346 ? 3'b111 : 3'b001;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:191:17  */
  assign n1350 = cycle_count ? n1349 : current_state;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:185:13  */
  assign n1352 = current_state == 3'b110;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:101:9  */
  assign n1353 = {n1352, n1345, n1335, n1323, n1318, n1311, n1307, n1303};
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:101:9  */
  always @*
    case (n1353)
      8'b10000000: n1355 = n1350;
      8'b01000000: n1355 = n1343;
      8'b00100000: n1355 = n1333;
      8'b00010000: n1355 = n1321;
      8'b00001000: n1355 = n1316;
      8'b00000100: n1355 = n1309;
      8'b00000010: n1355 = n1305;
      8'b00000001: n1355 = n1301;
      default: n1355 = 3'bX;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:213:28  */
  assign n1361 = ~cycle_count;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:213:13  */
  assign n1362 = n1361 ? current_state : next_state;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:213:13  */
  assign n1365 = n1361 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:210:9  */
  assign n1373 = phi2_falling ? n1362 : current_state;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:210:9  */
  always @(posedge clk or posedge reset)
    if (reset)
      n1374 <= 3'b000;
    else
      n1374 <= n1373;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:210:9  */
  assign n1375 = phi2_falling ? n1365 : cycle_count;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/state_timing_generator.vhdl:210:9  */
  always @(posedge clk or posedge reset)
    if (reset)
      n1376 <= 1'b0;
    else
      n1376 <= n1375;
endmodule

module phase_clocks_Brtl_25000000
  (input  clk_in,
   input  reset,
   input  run_enable,
   output phi1,
   output phi2,
   output sync,
   output phi1_rising,
   output phi1_falling,
   output phi2_rising,
   output phi2_falling);
  reg [6:0] counter;
  reg [1:0] current_phase;
  reg phi1_next;
  reg phi2_next;
  reg sync_toggle;
  reg phi1_reg;
  reg phi2_reg;
  reg phi1_prev;
  reg phi2_prev;
  wire n1079;
  wire n1080;
  wire n1081;
  wire n1082;
  wire n1083;
  wire n1084;
  wire n1085;
  wire n1086;
  wire [31:0] n1091;
  wire n1093;
  wire [31:0] n1094;
  wire [31:0] n1096;
  wire [6:0] n1097;
  wire [6:0] n1099;
  wire [1:0] n1101;
  wire n1103;
  wire n1105;
  wire n1107;
  wire [31:0] n1108;
  wire n1110;
  wire [31:0] n1111;
  wire [31:0] n1113;
  wire [6:0] n1114;
  wire [6:0] n1116;
  wire [1:0] n1118;
  wire n1120;
  wire n1122;
  wire n1124;
  wire [31:0] n1125;
  wire n1127;
  wire [31:0] n1128;
  wire [31:0] n1130;
  wire [6:0] n1131;
  wire [6:0] n1133;
  wire [1:0] n1135;
  wire n1137;
  wire n1139;
  wire n1141;
  wire [31:0] n1142;
  wire n1144;
  wire n1145;
  wire [31:0] n1146;
  wire [31:0] n1148;
  wire [6:0] n1149;
  wire [6:0] n1151;
  wire [1:0] n1153;
  wire n1155;
  wire n1157;
  wire n1158;
  wire n1160;
  wire [3:0] n1161;
  reg [6:0] n1163;
  reg [1:0] n1165;
  reg n1167;
  reg n1169;
  reg n1171;
  wire n1188;
  reg n1189;
  wire [6:0] n1190;
  reg [6:0] n1191;
  wire [1:0] n1192;
  reg [1:0] n1193;
  wire n1194;
  reg n1195;
  wire n1196;
  reg n1197;
  wire n1198;
  reg n1199;
  wire n1200;
  reg n1201;
  wire n1202;
  reg n1203;
  wire n1204;
  reg n1205;
  wire n1206;
  reg n1207;
  assign phi1 = phi1_reg; //(module output)
  assign phi2 = phi2_reg; //(module output)
  assign sync = n1189; //(module output)
  assign phi1_rising = n1080; //(module output)
  assign phi1_falling = n1082; //(module output)
  assign phi2_rising = n1084; //(module output)
  assign phi2_falling = n1086; //(module output)
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:71:12  */
  always @*
    counter = n1191; // (isignal)
  initial
    counter = 7'b0000000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:72:12  */
  always @*
    current_phase = n1193; // (isignal)
  initial
    current_phase = 2'b00;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:75:12  */
  always @*
    phi1_next = n1195; // (isignal)
  initial
    phi1_next = 1'b1;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:76:12  */
  always @*
    phi2_next = n1197; // (isignal)
  initial
    phi2_next = 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:81:12  */
  always @*
    sync_toggle = n1199; // (isignal)
  initial
    sync_toggle = 1'b1;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:84:12  */
  always @*
    phi1_reg = n1201; // (isignal)
  initial
    phi1_reg = 1'b1;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:85:12  */
  always @*
    phi2_reg = n1203; // (isignal)
  initial
    phi2_reg = 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:86:12  */
  always @*
    phi1_prev = n1205; // (isignal)
  initial
    phi1_prev = 1'b1;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:87:12  */
  always @*
    phi2_prev = n1207; // (isignal)
  initial
    phi2_prev = 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:118:34  */
  assign n1079 = ~phi1_prev;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:118:30  */
  assign n1080 = phi1_reg & n1079;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:119:35  */
  assign n1081 = ~phi1_reg;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:119:31  */
  assign n1082 = phi1_prev & n1081;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:120:34  */
  assign n1083 = ~phi2_prev;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:120:30  */
  assign n1084 = phi2_reg & n1083;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:121:35  */
  assign n1085 = ~phi2_reg;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:121:31  */
  assign n1086 = phi2_prev & n1085;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:138:32  */
  assign n1091 = {25'b0, counter};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:138:32  */
  assign n1093 = n1091 == 32'b00000000000000000000000000010011;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:144:44  */
  assign n1094 = {25'b0, counter};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:144:44  */
  assign n1096 = n1094 + 32'b00000000000000000000000000000001;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:144:36  */
  assign n1097 = n1096[6:0];  // trunc
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:138:21  */
  assign n1099 = n1093 ? 7'b0000000 : n1097;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:138:21  */
  assign n1101 = n1093 ? 2'b11 : current_phase;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:138:21  */
  assign n1103 = n1093 ? 1'b0 : phi1_next;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:138:21  */
  assign n1105 = n1093 ? 1'b0 : phi2_next;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:137:17  */
  assign n1107 = current_phase == 2'b00;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:149:32  */
  assign n1108 = {25'b0, counter};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:149:32  */
  assign n1110 = n1108 == 32'b00000000000000000000000000001001;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:155:44  */
  assign n1111 = {25'b0, counter};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:155:44  */
  assign n1113 = n1111 + 32'b00000000000000000000000000000001;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:155:36  */
  assign n1114 = n1113[6:0];  // trunc
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:149:21  */
  assign n1116 = n1110 ? 7'b0000000 : n1114;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:149:21  */
  assign n1118 = n1110 ? 2'b01 : current_phase;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:149:21  */
  assign n1120 = n1110 ? 1'b0 : phi1_next;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:149:21  */
  assign n1122 = n1110 ? 1'b1 : phi2_next;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:148:17  */
  assign n1124 = current_phase == 2'b11;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:160:32  */
  assign n1125 = {25'b0, counter};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:160:32  */
  assign n1127 = n1125 == 32'b00000000000000000000000000001110;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:166:44  */
  assign n1128 = {25'b0, counter};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:166:44  */
  assign n1130 = n1128 + 32'b00000000000000000000000000000001;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:166:36  */
  assign n1131 = n1130[6:0];  // trunc
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:160:21  */
  assign n1133 = n1127 ? 7'b0000000 : n1131;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:160:21  */
  assign n1135 = n1127 ? 2'b10 : current_phase;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:160:21  */
  assign n1137 = n1127 ? 1'b0 : phi1_next;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:160:21  */
  assign n1139 = n1127 ? 1'b0 : phi2_next;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:159:17  */
  assign n1141 = current_phase == 2'b01;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:171:32  */
  assign n1142 = {25'b0, counter};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:171:32  */
  assign n1144 = n1142 == 32'b00000000000000000000000000001001;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:177:40  */
  assign n1145 = ~sync_toggle;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:179:44  */
  assign n1146 = {25'b0, counter};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:179:44  */
  assign n1148 = n1146 + 32'b00000000000000000000000000000001;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:179:36  */
  assign n1149 = n1148[6:0];  // trunc
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:171:21  */
  assign n1151 = n1144 ? 7'b0000000 : n1149;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:171:21  */
  assign n1153 = n1144 ? 2'b00 : current_phase;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:171:21  */
  assign n1155 = n1144 ? 1'b1 : phi1_next;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:171:21  */
  assign n1157 = n1144 ? 1'b0 : phi2_next;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:171:21  */
  assign n1158 = n1144 ? n1145 : sync_toggle;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:170:17  */
  assign n1160 = current_phase == 2'b10;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:135:13  */
  assign n1161 = {n1160, n1141, n1124, n1107};
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:135:13  */
  always @*
    case (n1161)
      4'b1000: n1163 = n1151;
      4'b0100: n1163 = n1133;
      4'b0010: n1163 = n1116;
      4'b0001: n1163 = n1099;
      default: n1163 = 7'bX;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:135:13  */
  always @*
    case (n1161)
      4'b1000: n1165 = n1153;
      4'b0100: n1165 = n1135;
      4'b0010: n1165 = n1118;
      4'b0001: n1165 = n1101;
      default: n1165 = 2'bX;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:135:13  */
  always @*
    case (n1161)
      4'b1000: n1167 = n1155;
      4'b0100: n1167 = n1137;
      4'b0010: n1167 = n1120;
      4'b0001: n1167 = n1103;
      default: n1167 = 1'bX;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:135:13  */
  always @*
    case (n1161)
      4'b1000: n1169 = n1157;
      4'b0100: n1169 = n1139;
      4'b0010: n1169 = n1122;
      4'b0001: n1169 = n1105;
      default: n1169 = 1'bX;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:135:13  */
  always @*
    case (n1161)
      4'b1000: n1171 = n1158;
      4'b0100: n1171 = sync_toggle;
      4'b0010: n1171 = sync_toggle;
      4'b0001: n1171 = sync_toggle;
      default: n1171 = 1'bX;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:100:9  */
  assign n1188 = run_enable ? sync_toggle : n1189;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:100:9  */
  always @(posedge clk_in or posedge reset)
    if (reset)
      n1189 <= 1'b1;
    else
      n1189 <= n1188;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:134:9  */
  assign n1190 = run_enable ? n1163 : counter;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:134:9  */
  always @(posedge clk_in or posedge reset)
    if (reset)
      n1191 <= 7'b0000000;
    else
      n1191 <= n1190;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:134:9  */
  assign n1192 = run_enable ? n1165 : current_phase;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:134:9  */
  always @(posedge clk_in or posedge reset)
    if (reset)
      n1193 <= 2'b00;
    else
      n1193 <= n1192;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:134:9  */
  assign n1194 = run_enable ? n1167 : phi1_next;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:134:9  */
  always @(posedge clk_in or posedge reset)
    if (reset)
      n1195 <= 1'b1;
    else
      n1195 <= n1194;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:134:9  */
  assign n1196 = run_enable ? n1169 : phi2_next;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:134:9  */
  always @(posedge clk_in or posedge reset)
    if (reset)
      n1197 <= 1'b0;
    else
      n1197 <= n1196;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:134:9  */
  assign n1198 = run_enable ? n1171 : sync_toggle;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:134:9  */
  always @(posedge clk_in or posedge reset)
    if (reset)
      n1199 <= 1'b1;
    else
      n1199 <= n1198;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:100:9  */
  assign n1200 = run_enable ? phi1_next : phi1_reg;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:100:9  */
  always @(posedge clk_in or posedge reset)
    if (reset)
      n1201 <= 1'b1;
    else
      n1201 <= n1200;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:100:9  */
  assign n1202 = run_enable ? phi2_next : phi2_reg;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:100:9  */
  always @(posedge clk_in or posedge reset)
    if (reset)
      n1203 <= 1'b0;
    else
      n1203 <= n1202;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:100:9  */
  assign n1204 = run_enable ? phi1_reg : phi1_prev;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:100:9  */
  always @(posedge clk_in or posedge reset)
    if (reset)
      n1205 <= 1'b1;
    else
      n1205 <= n1204;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:100:9  */
  assign n1206 = run_enable ? phi2_reg : phi2_prev;
  /* /Users/hambook/Development/intel-8008-vhdl/src/components/phase_clocks.vhdl:100:9  */
  always @(posedge clk_in or posedge reset)
    if (reset)
      n1207 <= 1'b0;
    else
      n1207 <= n1206;
endmodule

module address_decoder_Brtl_0_4095_4096_16383
  (input  [13:0] address,
   output rom_sel,
   output ram_sel,
   output rom_cs_n,
   output ram_cs_n);
  wire [13:0] addr_int;
  wire rom_hit;
  wire ram_hit;
  wire [31:0] n1018;
  wire n1020;
  wire [31:0] n1021;
  wire n1023;
  wire n1024;
  wire n1025;
  wire [31:0] n1028;
  wire n1030;
  wire [31:0] n1031;
  wire n1033;
  wire n1034;
  wire n1035;
  wire n1037;
  wire n1038;
  assign rom_sel = rom_hit; //(module output)
  assign ram_sel = ram_hit; //(module output)
  assign rom_cs_n = n1037; //(module output)
  assign ram_cs_n = n1038; //(module output)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/address_decoder.vhdl:36:12  */
  assign addr_int = address; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/address_decoder.vhdl:37:12  */
  assign rom_hit = n1025; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/address_decoder.vhdl:38:12  */
  assign ram_hit = n1035; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/address_decoder.vhdl:43:34  */
  assign n1018 = {18'b0, addr_int};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/address_decoder.vhdl:43:34  */
  assign n1020 = $signed(n1018) >= $signed(32'b00000000000000000000000000000000);
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/address_decoder.vhdl:43:59  */
  assign n1021 = {18'b0, addr_int};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/address_decoder.vhdl:43:59  */
  assign n1023 = $signed(n1021) <= $signed(32'b00000000000000000000111111111111);
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/address_decoder.vhdl:43:46  */
  assign n1024 = n1023 & n1020;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/address_decoder.vhdl:43:20  */
  assign n1025 = n1024 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/address_decoder.vhdl:44:34  */
  assign n1028 = {18'b0, addr_int};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/address_decoder.vhdl:44:34  */
  assign n1030 = $signed(n1028) >= $signed(32'b00000000000000000001000000000000);
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/address_decoder.vhdl:44:59  */
  assign n1031 = {18'b0, addr_int};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/address_decoder.vhdl:44:59  */
  assign n1033 = $signed(n1031) <= $signed(32'b00000000000000000011111111111111);
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/address_decoder.vhdl:44:46  */
  assign n1034 = n1033 & n1030;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/address_decoder.vhdl:44:20  */
  assign n1035 = n1034 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/address_decoder.vhdl:48:17  */
  assign n1037 = ~rom_hit;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/address_decoder.vhdl:49:17  */
  assign n1038 = ~ram_hit;
endmodule

module ram_sync_Brtl_14_da39a3ee5e6b4b0d3255bfef95601890afd80709
  (input  clk,
   input  [13:0] addr,
   input  [7:0] data_in,
   output [7:0] data_out,
   input  rw_n,
   input  cs_n);
  wire n987;
  wire n988;
  wire n989;
  reg [7:0] n1010; // mem_rd
  assign data_out = n1010; //(module output)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/ram_sync.vhdl:69:21  */
  assign n987 = ~cs_n;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/ram_sync.vhdl:69:36  */
  assign n988 = ~rw_n;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/ram_sync.vhdl:69:27  */
  assign n989 = n988 & n987;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/ram_sync.vhdl:70:21  */
  reg [7:0] ram[16383:0] ; // memory
  initial begin
    ram[16383] = 8'b00000000;
    ram[16382] = 8'b00000000;
    ram[16381] = 8'b00000000;
    ram[16380] = 8'b00000000;
    ram[16379] = 8'b00000000;
    ram[16378] = 8'b00000000;
    ram[16377] = 8'b00000000;
    ram[16376] = 8'b00000000;
    ram[16375] = 8'b00000000;
    ram[16374] = 8'b00000000;
    ram[16373] = 8'b00000000;
    ram[16372] = 8'b00000000;
    ram[16371] = 8'b00000000;
    ram[16370] = 8'b00000000;
    ram[16369] = 8'b00000000;
    ram[16368] = 8'b00000000;
    ram[16367] = 8'b00000000;
    ram[16366] = 8'b00000000;
    ram[16365] = 8'b00000000;
    ram[16364] = 8'b00000000;
    ram[16363] = 8'b00000000;
    ram[16362] = 8'b00000000;
    ram[16361] = 8'b00000000;
    ram[16360] = 8'b00000000;
    ram[16359] = 8'b00000000;
    ram[16358] = 8'b00000000;
    ram[16357] = 8'b00000000;
    ram[16356] = 8'b00000000;
    ram[16355] = 8'b00000000;
    ram[16354] = 8'b00000000;
    ram[16353] = 8'b00000000;
    ram[16352] = 8'b00000000;
    ram[16351] = 8'b00000000;
    ram[16350] = 8'b00000000;
    ram[16349] = 8'b00000000;
    ram[16348] = 8'b00000000;
    ram[16347] = 8'b00000000;
    ram[16346] = 8'b00000000;
    ram[16345] = 8'b00000000;
    ram[16344] = 8'b00000000;
    ram[16343] = 8'b00000000;
    ram[16342] = 8'b00000000;
    ram[16341] = 8'b00000000;
    ram[16340] = 8'b00000000;
    ram[16339] = 8'b00000000;
    ram[16338] = 8'b00000000;
    ram[16337] = 8'b00000000;
    ram[16336] = 8'b00000000;
    ram[16335] = 8'b00000000;
    ram[16334] = 8'b00000000;
    ram[16333] = 8'b00000000;
    ram[16332] = 8'b00000000;
    ram[16331] = 8'b00000000;
    ram[16330] = 8'b00000000;
    ram[16329] = 8'b00000000;
    ram[16328] = 8'b00000000;
    ram[16327] = 8'b00000000;
    ram[16326] = 8'b00000000;
    ram[16325] = 8'b00000000;
    ram[16324] = 8'b00000000;
    ram[16323] = 8'b00000000;
    ram[16322] = 8'b00000000;
    ram[16321] = 8'b00000000;
    ram[16320] = 8'b00000000;
    ram[16319] = 8'b00000000;
    ram[16318] = 8'b00000000;
    ram[16317] = 8'b00000000;
    ram[16316] = 8'b00000000;
    ram[16315] = 8'b00000000;
    ram[16314] = 8'b00000000;
    ram[16313] = 8'b00000000;
    ram[16312] = 8'b00000000;
    ram[16311] = 8'b00000000;
    ram[16310] = 8'b00000000;
    ram[16309] = 8'b00000000;
    ram[16308] = 8'b00000000;
    ram[16307] = 8'b00000000;
    ram[16306] = 8'b00000000;
    ram[16305] = 8'b00000000;
    ram[16304] = 8'b00000000;
    ram[16303] = 8'b00000000;
    ram[16302] = 8'b00000000;
    ram[16301] = 8'b00000000;
    ram[16300] = 8'b00000000;
    ram[16299] = 8'b00000000;
    ram[16298] = 8'b00000000;
    ram[16297] = 8'b00000000;
    ram[16296] = 8'b00000000;
    ram[16295] = 8'b00000000;
    ram[16294] = 8'b00000000;
    ram[16293] = 8'b00000000;
    ram[16292] = 8'b00000000;
    ram[16291] = 8'b00000000;
    ram[16290] = 8'b00000000;
    ram[16289] = 8'b00000000;
    ram[16288] = 8'b00000000;
    ram[16287] = 8'b00000000;
    ram[16286] = 8'b00000000;
    ram[16285] = 8'b00000000;
    ram[16284] = 8'b00000000;
    ram[16283] = 8'b00000000;
    ram[16282] = 8'b00000000;
    ram[16281] = 8'b00000000;
    ram[16280] = 8'b00000000;
    ram[16279] = 8'b00000000;
    ram[16278] = 8'b00000000;
    ram[16277] = 8'b00000000;
    ram[16276] = 8'b00000000;
    ram[16275] = 8'b00000000;
    ram[16274] = 8'b00000000;
    ram[16273] = 8'b00000000;
    ram[16272] = 8'b00000000;
    ram[16271] = 8'b00000000;
    ram[16270] = 8'b00000000;
    ram[16269] = 8'b00000000;
    ram[16268] = 8'b00000000;
    ram[16267] = 8'b00000000;
    ram[16266] = 8'b00000000;
    ram[16265] = 8'b00000000;
    ram[16264] = 8'b00000000;
    ram[16263] = 8'b00000000;
    ram[16262] = 8'b00000000;
    ram[16261] = 8'b00000000;
    ram[16260] = 8'b00000000;
    ram[16259] = 8'b00000000;
    ram[16258] = 8'b00000000;
    ram[16257] = 8'b00000000;
    ram[16256] = 8'b00000000;
    ram[16255] = 8'b00000000;
    ram[16254] = 8'b00000000;
    ram[16253] = 8'b00000000;
    ram[16252] = 8'b00000000;
    ram[16251] = 8'b00000000;
    ram[16250] = 8'b00000000;
    ram[16249] = 8'b00000000;
    ram[16248] = 8'b00000000;
    ram[16247] = 8'b00000000;
    ram[16246] = 8'b00000000;
    ram[16245] = 8'b00000000;
    ram[16244] = 8'b00000000;
    ram[16243] = 8'b00000000;
    ram[16242] = 8'b00000000;
    ram[16241] = 8'b00000000;
    ram[16240] = 8'b00000000;
    ram[16239] = 8'b00000000;
    ram[16238] = 8'b00000000;
    ram[16237] = 8'b00000000;
    ram[16236] = 8'b00000000;
    ram[16235] = 8'b00000000;
    ram[16234] = 8'b00000000;
    ram[16233] = 8'b00000000;
    ram[16232] = 8'b00000000;
    ram[16231] = 8'b00000000;
    ram[16230] = 8'b00000000;
    ram[16229] = 8'b00000000;
    ram[16228] = 8'b00000000;
    ram[16227] = 8'b00000000;
    ram[16226] = 8'b00000000;
    ram[16225] = 8'b00000000;
    ram[16224] = 8'b00000000;
    ram[16223] = 8'b00000000;
    ram[16222] = 8'b00000000;
    ram[16221] = 8'b00000000;
    ram[16220] = 8'b00000000;
    ram[16219] = 8'b00000000;
    ram[16218] = 8'b00000000;
    ram[16217] = 8'b00000000;
    ram[16216] = 8'b00000000;
    ram[16215] = 8'b00000000;
    ram[16214] = 8'b00000000;
    ram[16213] = 8'b00000000;
    ram[16212] = 8'b00000000;
    ram[16211] = 8'b00000000;
    ram[16210] = 8'b00000000;
    ram[16209] = 8'b00000000;
    ram[16208] = 8'b00000000;
    ram[16207] = 8'b00000000;
    ram[16206] = 8'b00000000;
    ram[16205] = 8'b00000000;
    ram[16204] = 8'b00000000;
    ram[16203] = 8'b00000000;
    ram[16202] = 8'b00000000;
    ram[16201] = 8'b00000000;
    ram[16200] = 8'b00000000;
    ram[16199] = 8'b00000000;
    ram[16198] = 8'b00000000;
    ram[16197] = 8'b00000000;
    ram[16196] = 8'b00000000;
    ram[16195] = 8'b00000000;
    ram[16194] = 8'b00000000;
    ram[16193] = 8'b00000000;
    ram[16192] = 8'b00000000;
    ram[16191] = 8'b00000000;
    ram[16190] = 8'b00000000;
    ram[16189] = 8'b00000000;
    ram[16188] = 8'b00000000;
    ram[16187] = 8'b00000000;
    ram[16186] = 8'b00000000;
    ram[16185] = 8'b00000000;
    ram[16184] = 8'b00000000;
    ram[16183] = 8'b00000000;
    ram[16182] = 8'b00000000;
    ram[16181] = 8'b00000000;
    ram[16180] = 8'b00000000;
    ram[16179] = 8'b00000000;
    ram[16178] = 8'b00000000;
    ram[16177] = 8'b00000000;
    ram[16176] = 8'b00000000;
    ram[16175] = 8'b00000000;
    ram[16174] = 8'b00000000;
    ram[16173] = 8'b00000000;
    ram[16172] = 8'b00000000;
    ram[16171] = 8'b00000000;
    ram[16170] = 8'b00000000;
    ram[16169] = 8'b00000000;
    ram[16168] = 8'b00000000;
    ram[16167] = 8'b00000000;
    ram[16166] = 8'b00000000;
    ram[16165] = 8'b00000000;
    ram[16164] = 8'b00000000;
    ram[16163] = 8'b00000000;
    ram[16162] = 8'b00000000;
    ram[16161] = 8'b00000000;
    ram[16160] = 8'b00000000;
    ram[16159] = 8'b00000000;
    ram[16158] = 8'b00000000;
    ram[16157] = 8'b00000000;
    ram[16156] = 8'b00000000;
    ram[16155] = 8'b00000000;
    ram[16154] = 8'b00000000;
    ram[16153] = 8'b00000000;
    ram[16152] = 8'b00000000;
    ram[16151] = 8'b00000000;
    ram[16150] = 8'b00000000;
    ram[16149] = 8'b00000000;
    ram[16148] = 8'b00000000;
    ram[16147] = 8'b00000000;
    ram[16146] = 8'b00000000;
    ram[16145] = 8'b00000000;
    ram[16144] = 8'b00000000;
    ram[16143] = 8'b00000000;
    ram[16142] = 8'b00000000;
    ram[16141] = 8'b00000000;
    ram[16140] = 8'b00000000;
    ram[16139] = 8'b00000000;
    ram[16138] = 8'b00000000;
    ram[16137] = 8'b00000000;
    ram[16136] = 8'b00000000;
    ram[16135] = 8'b00000000;
    ram[16134] = 8'b00000000;
    ram[16133] = 8'b00000000;
    ram[16132] = 8'b00000000;
    ram[16131] = 8'b00000000;
    ram[16130] = 8'b00000000;
    ram[16129] = 8'b00000000;
    ram[16128] = 8'b00000000;
    ram[16127] = 8'b00000000;
    ram[16126] = 8'b00000000;
    ram[16125] = 8'b00000000;
    ram[16124] = 8'b00000000;
    ram[16123] = 8'b00000000;
    ram[16122] = 8'b00000000;
    ram[16121] = 8'b00000000;
    ram[16120] = 8'b00000000;
    ram[16119] = 8'b00000000;
    ram[16118] = 8'b00000000;
    ram[16117] = 8'b00000000;
    ram[16116] = 8'b00000000;
    ram[16115] = 8'b00000000;
    ram[16114] = 8'b00000000;
    ram[16113] = 8'b00000000;
    ram[16112] = 8'b00000000;
    ram[16111] = 8'b00000000;
    ram[16110] = 8'b00000000;
    ram[16109] = 8'b00000000;
    ram[16108] = 8'b00000000;
    ram[16107] = 8'b00000000;
    ram[16106] = 8'b00000000;
    ram[16105] = 8'b00000000;
    ram[16104] = 8'b00000000;
    ram[16103] = 8'b00000000;
    ram[16102] = 8'b00000000;
    ram[16101] = 8'b00000000;
    ram[16100] = 8'b00000000;
    ram[16099] = 8'b00000000;
    ram[16098] = 8'b00000000;
    ram[16097] = 8'b00000000;
    ram[16096] = 8'b00000000;
    ram[16095] = 8'b00000000;
    ram[16094] = 8'b00000000;
    ram[16093] = 8'b00000000;
    ram[16092] = 8'b00000000;
    ram[16091] = 8'b00000000;
    ram[16090] = 8'b00000000;
    ram[16089] = 8'b00000000;
    ram[16088] = 8'b00000000;
    ram[16087] = 8'b00000000;
    ram[16086] = 8'b00000000;
    ram[16085] = 8'b00000000;
    ram[16084] = 8'b00000000;
    ram[16083] = 8'b00000000;
    ram[16082] = 8'b00000000;
    ram[16081] = 8'b00000000;
    ram[16080] = 8'b00000000;
    ram[16079] = 8'b00000000;
    ram[16078] = 8'b00000000;
    ram[16077] = 8'b00000000;
    ram[16076] = 8'b00000000;
    ram[16075] = 8'b00000000;
    ram[16074] = 8'b00000000;
    ram[16073] = 8'b00000000;
    ram[16072] = 8'b00000000;
    ram[16071] = 8'b00000000;
    ram[16070] = 8'b00000000;
    ram[16069] = 8'b00000000;
    ram[16068] = 8'b00000000;
    ram[16067] = 8'b00000000;
    ram[16066] = 8'b00000000;
    ram[16065] = 8'b00000000;
    ram[16064] = 8'b00000000;
    ram[16063] = 8'b00000000;
    ram[16062] = 8'b00000000;
    ram[16061] = 8'b00000000;
    ram[16060] = 8'b00000000;
    ram[16059] = 8'b00000000;
    ram[16058] = 8'b00000000;
    ram[16057] = 8'b00000000;
    ram[16056] = 8'b00000000;
    ram[16055] = 8'b00000000;
    ram[16054] = 8'b00000000;
    ram[16053] = 8'b00000000;
    ram[16052] = 8'b00000000;
    ram[16051] = 8'b00000000;
    ram[16050] = 8'b00000000;
    ram[16049] = 8'b00000000;
    ram[16048] = 8'b00000000;
    ram[16047] = 8'b00000000;
    ram[16046] = 8'b00000000;
    ram[16045] = 8'b00000000;
    ram[16044] = 8'b00000000;
    ram[16043] = 8'b00000000;
    ram[16042] = 8'b00000000;
    ram[16041] = 8'b00000000;
    ram[16040] = 8'b00000000;
    ram[16039] = 8'b00000000;
    ram[16038] = 8'b00000000;
    ram[16037] = 8'b00000000;
    ram[16036] = 8'b00000000;
    ram[16035] = 8'b00000000;
    ram[16034] = 8'b00000000;
    ram[16033] = 8'b00000000;
    ram[16032] = 8'b00000000;
    ram[16031] = 8'b00000000;
    ram[16030] = 8'b00000000;
    ram[16029] = 8'b00000000;
    ram[16028] = 8'b00000000;
    ram[16027] = 8'b00000000;
    ram[16026] = 8'b00000000;
    ram[16025] = 8'b00000000;
    ram[16024] = 8'b00000000;
    ram[16023] = 8'b00000000;
    ram[16022] = 8'b00000000;
    ram[16021] = 8'b00000000;
    ram[16020] = 8'b00000000;
    ram[16019] = 8'b00000000;
    ram[16018] = 8'b00000000;
    ram[16017] = 8'b00000000;
    ram[16016] = 8'b00000000;
    ram[16015] = 8'b00000000;
    ram[16014] = 8'b00000000;
    ram[16013] = 8'b00000000;
    ram[16012] = 8'b00000000;
    ram[16011] = 8'b00000000;
    ram[16010] = 8'b00000000;
    ram[16009] = 8'b00000000;
    ram[16008] = 8'b00000000;
    ram[16007] = 8'b00000000;
    ram[16006] = 8'b00000000;
    ram[16005] = 8'b00000000;
    ram[16004] = 8'b00000000;
    ram[16003] = 8'b00000000;
    ram[16002] = 8'b00000000;
    ram[16001] = 8'b00000000;
    ram[16000] = 8'b00000000;
    ram[15999] = 8'b00000000;
    ram[15998] = 8'b00000000;
    ram[15997] = 8'b00000000;
    ram[15996] = 8'b00000000;
    ram[15995] = 8'b00000000;
    ram[15994] = 8'b00000000;
    ram[15993] = 8'b00000000;
    ram[15992] = 8'b00000000;
    ram[15991] = 8'b00000000;
    ram[15990] = 8'b00000000;
    ram[15989] = 8'b00000000;
    ram[15988] = 8'b00000000;
    ram[15987] = 8'b00000000;
    ram[15986] = 8'b00000000;
    ram[15985] = 8'b00000000;
    ram[15984] = 8'b00000000;
    ram[15983] = 8'b00000000;
    ram[15982] = 8'b00000000;
    ram[15981] = 8'b00000000;
    ram[15980] = 8'b00000000;
    ram[15979] = 8'b00000000;
    ram[15978] = 8'b00000000;
    ram[15977] = 8'b00000000;
    ram[15976] = 8'b00000000;
    ram[15975] = 8'b00000000;
    ram[15974] = 8'b00000000;
    ram[15973] = 8'b00000000;
    ram[15972] = 8'b00000000;
    ram[15971] = 8'b00000000;
    ram[15970] = 8'b00000000;
    ram[15969] = 8'b00000000;
    ram[15968] = 8'b00000000;
    ram[15967] = 8'b00000000;
    ram[15966] = 8'b00000000;
    ram[15965] = 8'b00000000;
    ram[15964] = 8'b00000000;
    ram[15963] = 8'b00000000;
    ram[15962] = 8'b00000000;
    ram[15961] = 8'b00000000;
    ram[15960] = 8'b00000000;
    ram[15959] = 8'b00000000;
    ram[15958] = 8'b00000000;
    ram[15957] = 8'b00000000;
    ram[15956] = 8'b00000000;
    ram[15955] = 8'b00000000;
    ram[15954] = 8'b00000000;
    ram[15953] = 8'b00000000;
    ram[15952] = 8'b00000000;
    ram[15951] = 8'b00000000;
    ram[15950] = 8'b00000000;
    ram[15949] = 8'b00000000;
    ram[15948] = 8'b00000000;
    ram[15947] = 8'b00000000;
    ram[15946] = 8'b00000000;
    ram[15945] = 8'b00000000;
    ram[15944] = 8'b00000000;
    ram[15943] = 8'b00000000;
    ram[15942] = 8'b00000000;
    ram[15941] = 8'b00000000;
    ram[15940] = 8'b00000000;
    ram[15939] = 8'b00000000;
    ram[15938] = 8'b00000000;
    ram[15937] = 8'b00000000;
    ram[15936] = 8'b00000000;
    ram[15935] = 8'b00000000;
    ram[15934] = 8'b00000000;
    ram[15933] = 8'b00000000;
    ram[15932] = 8'b00000000;
    ram[15931] = 8'b00000000;
    ram[15930] = 8'b00000000;
    ram[15929] = 8'b00000000;
    ram[15928] = 8'b00000000;
    ram[15927] = 8'b00000000;
    ram[15926] = 8'b00000000;
    ram[15925] = 8'b00000000;
    ram[15924] = 8'b00000000;
    ram[15923] = 8'b00000000;
    ram[15922] = 8'b00000000;
    ram[15921] = 8'b00000000;
    ram[15920] = 8'b00000000;
    ram[15919] = 8'b00000000;
    ram[15918] = 8'b00000000;
    ram[15917] = 8'b00000000;
    ram[15916] = 8'b00000000;
    ram[15915] = 8'b00000000;
    ram[15914] = 8'b00000000;
    ram[15913] = 8'b00000000;
    ram[15912] = 8'b00000000;
    ram[15911] = 8'b00000000;
    ram[15910] = 8'b00000000;
    ram[15909] = 8'b00000000;
    ram[15908] = 8'b00000000;
    ram[15907] = 8'b00000000;
    ram[15906] = 8'b00000000;
    ram[15905] = 8'b00000000;
    ram[15904] = 8'b00000000;
    ram[15903] = 8'b00000000;
    ram[15902] = 8'b00000000;
    ram[15901] = 8'b00000000;
    ram[15900] = 8'b00000000;
    ram[15899] = 8'b00000000;
    ram[15898] = 8'b00000000;
    ram[15897] = 8'b00000000;
    ram[15896] = 8'b00000000;
    ram[15895] = 8'b00000000;
    ram[15894] = 8'b00000000;
    ram[15893] = 8'b00000000;
    ram[15892] = 8'b00000000;
    ram[15891] = 8'b00000000;
    ram[15890] = 8'b00000000;
    ram[15889] = 8'b00000000;
    ram[15888] = 8'b00000000;
    ram[15887] = 8'b00000000;
    ram[15886] = 8'b00000000;
    ram[15885] = 8'b00000000;
    ram[15884] = 8'b00000000;
    ram[15883] = 8'b00000000;
    ram[15882] = 8'b00000000;
    ram[15881] = 8'b00000000;
    ram[15880] = 8'b00000000;
    ram[15879] = 8'b00000000;
    ram[15878] = 8'b00000000;
    ram[15877] = 8'b00000000;
    ram[15876] = 8'b00000000;
    ram[15875] = 8'b00000000;
    ram[15874] = 8'b00000000;
    ram[15873] = 8'b00000000;
    ram[15872] = 8'b00000000;
    ram[15871] = 8'b00000000;
    ram[15870] = 8'b00000000;
    ram[15869] = 8'b00000000;
    ram[15868] = 8'b00000000;
    ram[15867] = 8'b00000000;
    ram[15866] = 8'b00000000;
    ram[15865] = 8'b00000000;
    ram[15864] = 8'b00000000;
    ram[15863] = 8'b00000000;
    ram[15862] = 8'b00000000;
    ram[15861] = 8'b00000000;
    ram[15860] = 8'b00000000;
    ram[15859] = 8'b00000000;
    ram[15858] = 8'b00000000;
    ram[15857] = 8'b00000000;
    ram[15856] = 8'b00000000;
    ram[15855] = 8'b00000000;
    ram[15854] = 8'b00000000;
    ram[15853] = 8'b00000000;
    ram[15852] = 8'b00000000;
    ram[15851] = 8'b00000000;
    ram[15850] = 8'b00000000;
    ram[15849] = 8'b00000000;
    ram[15848] = 8'b00000000;
    ram[15847] = 8'b00000000;
    ram[15846] = 8'b00000000;
    ram[15845] = 8'b00000000;
    ram[15844] = 8'b00000000;
    ram[15843] = 8'b00000000;
    ram[15842] = 8'b00000000;
    ram[15841] = 8'b00000000;
    ram[15840] = 8'b00000000;
    ram[15839] = 8'b00000000;
    ram[15838] = 8'b00000000;
    ram[15837] = 8'b00000000;
    ram[15836] = 8'b00000000;
    ram[15835] = 8'b00000000;
    ram[15834] = 8'b00000000;
    ram[15833] = 8'b00000000;
    ram[15832] = 8'b00000000;
    ram[15831] = 8'b00000000;
    ram[15830] = 8'b00000000;
    ram[15829] = 8'b00000000;
    ram[15828] = 8'b00000000;
    ram[15827] = 8'b00000000;
    ram[15826] = 8'b00000000;
    ram[15825] = 8'b00000000;
    ram[15824] = 8'b00000000;
    ram[15823] = 8'b00000000;
    ram[15822] = 8'b00000000;
    ram[15821] = 8'b00000000;
    ram[15820] = 8'b00000000;
    ram[15819] = 8'b00000000;
    ram[15818] = 8'b00000000;
    ram[15817] = 8'b00000000;
    ram[15816] = 8'b00000000;
    ram[15815] = 8'b00000000;
    ram[15814] = 8'b00000000;
    ram[15813] = 8'b00000000;
    ram[15812] = 8'b00000000;
    ram[15811] = 8'b00000000;
    ram[15810] = 8'b00000000;
    ram[15809] = 8'b00000000;
    ram[15808] = 8'b00000000;
    ram[15807] = 8'b00000000;
    ram[15806] = 8'b00000000;
    ram[15805] = 8'b00000000;
    ram[15804] = 8'b00000000;
    ram[15803] = 8'b00000000;
    ram[15802] = 8'b00000000;
    ram[15801] = 8'b00000000;
    ram[15800] = 8'b00000000;
    ram[15799] = 8'b00000000;
    ram[15798] = 8'b00000000;
    ram[15797] = 8'b00000000;
    ram[15796] = 8'b00000000;
    ram[15795] = 8'b00000000;
    ram[15794] = 8'b00000000;
    ram[15793] = 8'b00000000;
    ram[15792] = 8'b00000000;
    ram[15791] = 8'b00000000;
    ram[15790] = 8'b00000000;
    ram[15789] = 8'b00000000;
    ram[15788] = 8'b00000000;
    ram[15787] = 8'b00000000;
    ram[15786] = 8'b00000000;
    ram[15785] = 8'b00000000;
    ram[15784] = 8'b00000000;
    ram[15783] = 8'b00000000;
    ram[15782] = 8'b00000000;
    ram[15781] = 8'b00000000;
    ram[15780] = 8'b00000000;
    ram[15779] = 8'b00000000;
    ram[15778] = 8'b00000000;
    ram[15777] = 8'b00000000;
    ram[15776] = 8'b00000000;
    ram[15775] = 8'b00000000;
    ram[15774] = 8'b00000000;
    ram[15773] = 8'b00000000;
    ram[15772] = 8'b00000000;
    ram[15771] = 8'b00000000;
    ram[15770] = 8'b00000000;
    ram[15769] = 8'b00000000;
    ram[15768] = 8'b00000000;
    ram[15767] = 8'b00000000;
    ram[15766] = 8'b00000000;
    ram[15765] = 8'b00000000;
    ram[15764] = 8'b00000000;
    ram[15763] = 8'b00000000;
    ram[15762] = 8'b00000000;
    ram[15761] = 8'b00000000;
    ram[15760] = 8'b00000000;
    ram[15759] = 8'b00000000;
    ram[15758] = 8'b00000000;
    ram[15757] = 8'b00000000;
    ram[15756] = 8'b00000000;
    ram[15755] = 8'b00000000;
    ram[15754] = 8'b00000000;
    ram[15753] = 8'b00000000;
    ram[15752] = 8'b00000000;
    ram[15751] = 8'b00000000;
    ram[15750] = 8'b00000000;
    ram[15749] = 8'b00000000;
    ram[15748] = 8'b00000000;
    ram[15747] = 8'b00000000;
    ram[15746] = 8'b00000000;
    ram[15745] = 8'b00000000;
    ram[15744] = 8'b00000000;
    ram[15743] = 8'b00000000;
    ram[15742] = 8'b00000000;
    ram[15741] = 8'b00000000;
    ram[15740] = 8'b00000000;
    ram[15739] = 8'b00000000;
    ram[15738] = 8'b00000000;
    ram[15737] = 8'b00000000;
    ram[15736] = 8'b00000000;
    ram[15735] = 8'b00000000;
    ram[15734] = 8'b00000000;
    ram[15733] = 8'b00000000;
    ram[15732] = 8'b00000000;
    ram[15731] = 8'b00000000;
    ram[15730] = 8'b00000000;
    ram[15729] = 8'b00000000;
    ram[15728] = 8'b00000000;
    ram[15727] = 8'b00000000;
    ram[15726] = 8'b00000000;
    ram[15725] = 8'b00000000;
    ram[15724] = 8'b00000000;
    ram[15723] = 8'b00000000;
    ram[15722] = 8'b00000000;
    ram[15721] = 8'b00000000;
    ram[15720] = 8'b00000000;
    ram[15719] = 8'b00000000;
    ram[15718] = 8'b00000000;
    ram[15717] = 8'b00000000;
    ram[15716] = 8'b00000000;
    ram[15715] = 8'b00000000;
    ram[15714] = 8'b00000000;
    ram[15713] = 8'b00000000;
    ram[15712] = 8'b00000000;
    ram[15711] = 8'b00000000;
    ram[15710] = 8'b00000000;
    ram[15709] = 8'b00000000;
    ram[15708] = 8'b00000000;
    ram[15707] = 8'b00000000;
    ram[15706] = 8'b00000000;
    ram[15705] = 8'b00000000;
    ram[15704] = 8'b00000000;
    ram[15703] = 8'b00000000;
    ram[15702] = 8'b00000000;
    ram[15701] = 8'b00000000;
    ram[15700] = 8'b00000000;
    ram[15699] = 8'b00000000;
    ram[15698] = 8'b00000000;
    ram[15697] = 8'b00000000;
    ram[15696] = 8'b00000000;
    ram[15695] = 8'b00000000;
    ram[15694] = 8'b00000000;
    ram[15693] = 8'b00000000;
    ram[15692] = 8'b00000000;
    ram[15691] = 8'b00000000;
    ram[15690] = 8'b00000000;
    ram[15689] = 8'b00000000;
    ram[15688] = 8'b00000000;
    ram[15687] = 8'b00000000;
    ram[15686] = 8'b00000000;
    ram[15685] = 8'b00000000;
    ram[15684] = 8'b00000000;
    ram[15683] = 8'b00000000;
    ram[15682] = 8'b00000000;
    ram[15681] = 8'b00000000;
    ram[15680] = 8'b00000000;
    ram[15679] = 8'b00000000;
    ram[15678] = 8'b00000000;
    ram[15677] = 8'b00000000;
    ram[15676] = 8'b00000000;
    ram[15675] = 8'b00000000;
    ram[15674] = 8'b00000000;
    ram[15673] = 8'b00000000;
    ram[15672] = 8'b00000000;
    ram[15671] = 8'b00000000;
    ram[15670] = 8'b00000000;
    ram[15669] = 8'b00000000;
    ram[15668] = 8'b00000000;
    ram[15667] = 8'b00000000;
    ram[15666] = 8'b00000000;
    ram[15665] = 8'b00000000;
    ram[15664] = 8'b00000000;
    ram[15663] = 8'b00000000;
    ram[15662] = 8'b00000000;
    ram[15661] = 8'b00000000;
    ram[15660] = 8'b00000000;
    ram[15659] = 8'b00000000;
    ram[15658] = 8'b00000000;
    ram[15657] = 8'b00000000;
    ram[15656] = 8'b00000000;
    ram[15655] = 8'b00000000;
    ram[15654] = 8'b00000000;
    ram[15653] = 8'b00000000;
    ram[15652] = 8'b00000000;
    ram[15651] = 8'b00000000;
    ram[15650] = 8'b00000000;
    ram[15649] = 8'b00000000;
    ram[15648] = 8'b00000000;
    ram[15647] = 8'b00000000;
    ram[15646] = 8'b00000000;
    ram[15645] = 8'b00000000;
    ram[15644] = 8'b00000000;
    ram[15643] = 8'b00000000;
    ram[15642] = 8'b00000000;
    ram[15641] = 8'b00000000;
    ram[15640] = 8'b00000000;
    ram[15639] = 8'b00000000;
    ram[15638] = 8'b00000000;
    ram[15637] = 8'b00000000;
    ram[15636] = 8'b00000000;
    ram[15635] = 8'b00000000;
    ram[15634] = 8'b00000000;
    ram[15633] = 8'b00000000;
    ram[15632] = 8'b00000000;
    ram[15631] = 8'b00000000;
    ram[15630] = 8'b00000000;
    ram[15629] = 8'b00000000;
    ram[15628] = 8'b00000000;
    ram[15627] = 8'b00000000;
    ram[15626] = 8'b00000000;
    ram[15625] = 8'b00000000;
    ram[15624] = 8'b00000000;
    ram[15623] = 8'b00000000;
    ram[15622] = 8'b00000000;
    ram[15621] = 8'b00000000;
    ram[15620] = 8'b00000000;
    ram[15619] = 8'b00000000;
    ram[15618] = 8'b00000000;
    ram[15617] = 8'b00000000;
    ram[15616] = 8'b00000000;
    ram[15615] = 8'b00000000;
    ram[15614] = 8'b00000000;
    ram[15613] = 8'b00000000;
    ram[15612] = 8'b00000000;
    ram[15611] = 8'b00000000;
    ram[15610] = 8'b00000000;
    ram[15609] = 8'b00000000;
    ram[15608] = 8'b00000000;
    ram[15607] = 8'b00000000;
    ram[15606] = 8'b00000000;
    ram[15605] = 8'b00000000;
    ram[15604] = 8'b00000000;
    ram[15603] = 8'b00000000;
    ram[15602] = 8'b00000000;
    ram[15601] = 8'b00000000;
    ram[15600] = 8'b00000000;
    ram[15599] = 8'b00000000;
    ram[15598] = 8'b00000000;
    ram[15597] = 8'b00000000;
    ram[15596] = 8'b00000000;
    ram[15595] = 8'b00000000;
    ram[15594] = 8'b00000000;
    ram[15593] = 8'b00000000;
    ram[15592] = 8'b00000000;
    ram[15591] = 8'b00000000;
    ram[15590] = 8'b00000000;
    ram[15589] = 8'b00000000;
    ram[15588] = 8'b00000000;
    ram[15587] = 8'b00000000;
    ram[15586] = 8'b00000000;
    ram[15585] = 8'b00000000;
    ram[15584] = 8'b00000000;
    ram[15583] = 8'b00000000;
    ram[15582] = 8'b00000000;
    ram[15581] = 8'b00000000;
    ram[15580] = 8'b00000000;
    ram[15579] = 8'b00000000;
    ram[15578] = 8'b00000000;
    ram[15577] = 8'b00000000;
    ram[15576] = 8'b00000000;
    ram[15575] = 8'b00000000;
    ram[15574] = 8'b00000000;
    ram[15573] = 8'b00000000;
    ram[15572] = 8'b00000000;
    ram[15571] = 8'b00000000;
    ram[15570] = 8'b00000000;
    ram[15569] = 8'b00000000;
    ram[15568] = 8'b00000000;
    ram[15567] = 8'b00000000;
    ram[15566] = 8'b00000000;
    ram[15565] = 8'b00000000;
    ram[15564] = 8'b00000000;
    ram[15563] = 8'b00000000;
    ram[15562] = 8'b00000000;
    ram[15561] = 8'b00000000;
    ram[15560] = 8'b00000000;
    ram[15559] = 8'b00000000;
    ram[15558] = 8'b00000000;
    ram[15557] = 8'b00000000;
    ram[15556] = 8'b00000000;
    ram[15555] = 8'b00000000;
    ram[15554] = 8'b00000000;
    ram[15553] = 8'b00000000;
    ram[15552] = 8'b00000000;
    ram[15551] = 8'b00000000;
    ram[15550] = 8'b00000000;
    ram[15549] = 8'b00000000;
    ram[15548] = 8'b00000000;
    ram[15547] = 8'b00000000;
    ram[15546] = 8'b00000000;
    ram[15545] = 8'b00000000;
    ram[15544] = 8'b00000000;
    ram[15543] = 8'b00000000;
    ram[15542] = 8'b00000000;
    ram[15541] = 8'b00000000;
    ram[15540] = 8'b00000000;
    ram[15539] = 8'b00000000;
    ram[15538] = 8'b00000000;
    ram[15537] = 8'b00000000;
    ram[15536] = 8'b00000000;
    ram[15535] = 8'b00000000;
    ram[15534] = 8'b00000000;
    ram[15533] = 8'b00000000;
    ram[15532] = 8'b00000000;
    ram[15531] = 8'b00000000;
    ram[15530] = 8'b00000000;
    ram[15529] = 8'b00000000;
    ram[15528] = 8'b00000000;
    ram[15527] = 8'b00000000;
    ram[15526] = 8'b00000000;
    ram[15525] = 8'b00000000;
    ram[15524] = 8'b00000000;
    ram[15523] = 8'b00000000;
    ram[15522] = 8'b00000000;
    ram[15521] = 8'b00000000;
    ram[15520] = 8'b00000000;
    ram[15519] = 8'b00000000;
    ram[15518] = 8'b00000000;
    ram[15517] = 8'b00000000;
    ram[15516] = 8'b00000000;
    ram[15515] = 8'b00000000;
    ram[15514] = 8'b00000000;
    ram[15513] = 8'b00000000;
    ram[15512] = 8'b00000000;
    ram[15511] = 8'b00000000;
    ram[15510] = 8'b00000000;
    ram[15509] = 8'b00000000;
    ram[15508] = 8'b00000000;
    ram[15507] = 8'b00000000;
    ram[15506] = 8'b00000000;
    ram[15505] = 8'b00000000;
    ram[15504] = 8'b00000000;
    ram[15503] = 8'b00000000;
    ram[15502] = 8'b00000000;
    ram[15501] = 8'b00000000;
    ram[15500] = 8'b00000000;
    ram[15499] = 8'b00000000;
    ram[15498] = 8'b00000000;
    ram[15497] = 8'b00000000;
    ram[15496] = 8'b00000000;
    ram[15495] = 8'b00000000;
    ram[15494] = 8'b00000000;
    ram[15493] = 8'b00000000;
    ram[15492] = 8'b00000000;
    ram[15491] = 8'b00000000;
    ram[15490] = 8'b00000000;
    ram[15489] = 8'b00000000;
    ram[15488] = 8'b00000000;
    ram[15487] = 8'b00000000;
    ram[15486] = 8'b00000000;
    ram[15485] = 8'b00000000;
    ram[15484] = 8'b00000000;
    ram[15483] = 8'b00000000;
    ram[15482] = 8'b00000000;
    ram[15481] = 8'b00000000;
    ram[15480] = 8'b00000000;
    ram[15479] = 8'b00000000;
    ram[15478] = 8'b00000000;
    ram[15477] = 8'b00000000;
    ram[15476] = 8'b00000000;
    ram[15475] = 8'b00000000;
    ram[15474] = 8'b00000000;
    ram[15473] = 8'b00000000;
    ram[15472] = 8'b00000000;
    ram[15471] = 8'b00000000;
    ram[15470] = 8'b00000000;
    ram[15469] = 8'b00000000;
    ram[15468] = 8'b00000000;
    ram[15467] = 8'b00000000;
    ram[15466] = 8'b00000000;
    ram[15465] = 8'b00000000;
    ram[15464] = 8'b00000000;
    ram[15463] = 8'b00000000;
    ram[15462] = 8'b00000000;
    ram[15461] = 8'b00000000;
    ram[15460] = 8'b00000000;
    ram[15459] = 8'b00000000;
    ram[15458] = 8'b00000000;
    ram[15457] = 8'b00000000;
    ram[15456] = 8'b00000000;
    ram[15455] = 8'b00000000;
    ram[15454] = 8'b00000000;
    ram[15453] = 8'b00000000;
    ram[15452] = 8'b00000000;
    ram[15451] = 8'b00000000;
    ram[15450] = 8'b00000000;
    ram[15449] = 8'b00000000;
    ram[15448] = 8'b00000000;
    ram[15447] = 8'b00000000;
    ram[15446] = 8'b00000000;
    ram[15445] = 8'b00000000;
    ram[15444] = 8'b00000000;
    ram[15443] = 8'b00000000;
    ram[15442] = 8'b00000000;
    ram[15441] = 8'b00000000;
    ram[15440] = 8'b00000000;
    ram[15439] = 8'b00000000;
    ram[15438] = 8'b00000000;
    ram[15437] = 8'b00000000;
    ram[15436] = 8'b00000000;
    ram[15435] = 8'b00000000;
    ram[15434] = 8'b00000000;
    ram[15433] = 8'b00000000;
    ram[15432] = 8'b00000000;
    ram[15431] = 8'b00000000;
    ram[15430] = 8'b00000000;
    ram[15429] = 8'b00000000;
    ram[15428] = 8'b00000000;
    ram[15427] = 8'b00000000;
    ram[15426] = 8'b00000000;
    ram[15425] = 8'b00000000;
    ram[15424] = 8'b00000000;
    ram[15423] = 8'b00000000;
    ram[15422] = 8'b00000000;
    ram[15421] = 8'b00000000;
    ram[15420] = 8'b00000000;
    ram[15419] = 8'b00000000;
    ram[15418] = 8'b00000000;
    ram[15417] = 8'b00000000;
    ram[15416] = 8'b00000000;
    ram[15415] = 8'b00000000;
    ram[15414] = 8'b00000000;
    ram[15413] = 8'b00000000;
    ram[15412] = 8'b00000000;
    ram[15411] = 8'b00000000;
    ram[15410] = 8'b00000000;
    ram[15409] = 8'b00000000;
    ram[15408] = 8'b00000000;
    ram[15407] = 8'b00000000;
    ram[15406] = 8'b00000000;
    ram[15405] = 8'b00000000;
    ram[15404] = 8'b00000000;
    ram[15403] = 8'b00000000;
    ram[15402] = 8'b00000000;
    ram[15401] = 8'b00000000;
    ram[15400] = 8'b00000000;
    ram[15399] = 8'b00000000;
    ram[15398] = 8'b00000000;
    ram[15397] = 8'b00000000;
    ram[15396] = 8'b00000000;
    ram[15395] = 8'b00000000;
    ram[15394] = 8'b00000000;
    ram[15393] = 8'b00000000;
    ram[15392] = 8'b00000000;
    ram[15391] = 8'b00000000;
    ram[15390] = 8'b00000000;
    ram[15389] = 8'b00000000;
    ram[15388] = 8'b00000000;
    ram[15387] = 8'b00000000;
    ram[15386] = 8'b00000000;
    ram[15385] = 8'b00000000;
    ram[15384] = 8'b00000000;
    ram[15383] = 8'b00000000;
    ram[15382] = 8'b00000000;
    ram[15381] = 8'b00000000;
    ram[15380] = 8'b00000000;
    ram[15379] = 8'b00000000;
    ram[15378] = 8'b00000000;
    ram[15377] = 8'b00000000;
    ram[15376] = 8'b00000000;
    ram[15375] = 8'b00000000;
    ram[15374] = 8'b00000000;
    ram[15373] = 8'b00000000;
    ram[15372] = 8'b00000000;
    ram[15371] = 8'b00000000;
    ram[15370] = 8'b00000000;
    ram[15369] = 8'b00000000;
    ram[15368] = 8'b00000000;
    ram[15367] = 8'b00000000;
    ram[15366] = 8'b00000000;
    ram[15365] = 8'b00000000;
    ram[15364] = 8'b00000000;
    ram[15363] = 8'b00000000;
    ram[15362] = 8'b00000000;
    ram[15361] = 8'b00000000;
    ram[15360] = 8'b00000000;
    ram[15359] = 8'b00000000;
    ram[15358] = 8'b00000000;
    ram[15357] = 8'b00000000;
    ram[15356] = 8'b00000000;
    ram[15355] = 8'b00000000;
    ram[15354] = 8'b00000000;
    ram[15353] = 8'b00000000;
    ram[15352] = 8'b00000000;
    ram[15351] = 8'b00000000;
    ram[15350] = 8'b00000000;
    ram[15349] = 8'b00000000;
    ram[15348] = 8'b00000000;
    ram[15347] = 8'b00000000;
    ram[15346] = 8'b00000000;
    ram[15345] = 8'b00000000;
    ram[15344] = 8'b00000000;
    ram[15343] = 8'b00000000;
    ram[15342] = 8'b00000000;
    ram[15341] = 8'b00000000;
    ram[15340] = 8'b00000000;
    ram[15339] = 8'b00000000;
    ram[15338] = 8'b00000000;
    ram[15337] = 8'b00000000;
    ram[15336] = 8'b00000000;
    ram[15335] = 8'b00000000;
    ram[15334] = 8'b00000000;
    ram[15333] = 8'b00000000;
    ram[15332] = 8'b00000000;
    ram[15331] = 8'b00000000;
    ram[15330] = 8'b00000000;
    ram[15329] = 8'b00000000;
    ram[15328] = 8'b00000000;
    ram[15327] = 8'b00000000;
    ram[15326] = 8'b00000000;
    ram[15325] = 8'b00000000;
    ram[15324] = 8'b00000000;
    ram[15323] = 8'b00000000;
    ram[15322] = 8'b00000000;
    ram[15321] = 8'b00000000;
    ram[15320] = 8'b00000000;
    ram[15319] = 8'b00000000;
    ram[15318] = 8'b00000000;
    ram[15317] = 8'b00000000;
    ram[15316] = 8'b00000000;
    ram[15315] = 8'b00000000;
    ram[15314] = 8'b00000000;
    ram[15313] = 8'b00000000;
    ram[15312] = 8'b00000000;
    ram[15311] = 8'b00000000;
    ram[15310] = 8'b00000000;
    ram[15309] = 8'b00000000;
    ram[15308] = 8'b00000000;
    ram[15307] = 8'b00000000;
    ram[15306] = 8'b00000000;
    ram[15305] = 8'b00000000;
    ram[15304] = 8'b00000000;
    ram[15303] = 8'b00000000;
    ram[15302] = 8'b00000000;
    ram[15301] = 8'b00000000;
    ram[15300] = 8'b00000000;
    ram[15299] = 8'b00000000;
    ram[15298] = 8'b00000000;
    ram[15297] = 8'b00000000;
    ram[15296] = 8'b00000000;
    ram[15295] = 8'b00000000;
    ram[15294] = 8'b00000000;
    ram[15293] = 8'b00000000;
    ram[15292] = 8'b00000000;
    ram[15291] = 8'b00000000;
    ram[15290] = 8'b00000000;
    ram[15289] = 8'b00000000;
    ram[15288] = 8'b00000000;
    ram[15287] = 8'b00000000;
    ram[15286] = 8'b00000000;
    ram[15285] = 8'b00000000;
    ram[15284] = 8'b00000000;
    ram[15283] = 8'b00000000;
    ram[15282] = 8'b00000000;
    ram[15281] = 8'b00000000;
    ram[15280] = 8'b00000000;
    ram[15279] = 8'b00000000;
    ram[15278] = 8'b00000000;
    ram[15277] = 8'b00000000;
    ram[15276] = 8'b00000000;
    ram[15275] = 8'b00000000;
    ram[15274] = 8'b00000000;
    ram[15273] = 8'b00000000;
    ram[15272] = 8'b00000000;
    ram[15271] = 8'b00000000;
    ram[15270] = 8'b00000000;
    ram[15269] = 8'b00000000;
    ram[15268] = 8'b00000000;
    ram[15267] = 8'b00000000;
    ram[15266] = 8'b00000000;
    ram[15265] = 8'b00000000;
    ram[15264] = 8'b00000000;
    ram[15263] = 8'b00000000;
    ram[15262] = 8'b00000000;
    ram[15261] = 8'b00000000;
    ram[15260] = 8'b00000000;
    ram[15259] = 8'b00000000;
    ram[15258] = 8'b00000000;
    ram[15257] = 8'b00000000;
    ram[15256] = 8'b00000000;
    ram[15255] = 8'b00000000;
    ram[15254] = 8'b00000000;
    ram[15253] = 8'b00000000;
    ram[15252] = 8'b00000000;
    ram[15251] = 8'b00000000;
    ram[15250] = 8'b00000000;
    ram[15249] = 8'b00000000;
    ram[15248] = 8'b00000000;
    ram[15247] = 8'b00000000;
    ram[15246] = 8'b00000000;
    ram[15245] = 8'b00000000;
    ram[15244] = 8'b00000000;
    ram[15243] = 8'b00000000;
    ram[15242] = 8'b00000000;
    ram[15241] = 8'b00000000;
    ram[15240] = 8'b00000000;
    ram[15239] = 8'b00000000;
    ram[15238] = 8'b00000000;
    ram[15237] = 8'b00000000;
    ram[15236] = 8'b00000000;
    ram[15235] = 8'b00000000;
    ram[15234] = 8'b00000000;
    ram[15233] = 8'b00000000;
    ram[15232] = 8'b00000000;
    ram[15231] = 8'b00000000;
    ram[15230] = 8'b00000000;
    ram[15229] = 8'b00000000;
    ram[15228] = 8'b00000000;
    ram[15227] = 8'b00000000;
    ram[15226] = 8'b00000000;
    ram[15225] = 8'b00000000;
    ram[15224] = 8'b00000000;
    ram[15223] = 8'b00000000;
    ram[15222] = 8'b00000000;
    ram[15221] = 8'b00000000;
    ram[15220] = 8'b00000000;
    ram[15219] = 8'b00000000;
    ram[15218] = 8'b00000000;
    ram[15217] = 8'b00000000;
    ram[15216] = 8'b00000000;
    ram[15215] = 8'b00000000;
    ram[15214] = 8'b00000000;
    ram[15213] = 8'b00000000;
    ram[15212] = 8'b00000000;
    ram[15211] = 8'b00000000;
    ram[15210] = 8'b00000000;
    ram[15209] = 8'b00000000;
    ram[15208] = 8'b00000000;
    ram[15207] = 8'b00000000;
    ram[15206] = 8'b00000000;
    ram[15205] = 8'b00000000;
    ram[15204] = 8'b00000000;
    ram[15203] = 8'b00000000;
    ram[15202] = 8'b00000000;
    ram[15201] = 8'b00000000;
    ram[15200] = 8'b00000000;
    ram[15199] = 8'b00000000;
    ram[15198] = 8'b00000000;
    ram[15197] = 8'b00000000;
    ram[15196] = 8'b00000000;
    ram[15195] = 8'b00000000;
    ram[15194] = 8'b00000000;
    ram[15193] = 8'b00000000;
    ram[15192] = 8'b00000000;
    ram[15191] = 8'b00000000;
    ram[15190] = 8'b00000000;
    ram[15189] = 8'b00000000;
    ram[15188] = 8'b00000000;
    ram[15187] = 8'b00000000;
    ram[15186] = 8'b00000000;
    ram[15185] = 8'b00000000;
    ram[15184] = 8'b00000000;
    ram[15183] = 8'b00000000;
    ram[15182] = 8'b00000000;
    ram[15181] = 8'b00000000;
    ram[15180] = 8'b00000000;
    ram[15179] = 8'b00000000;
    ram[15178] = 8'b00000000;
    ram[15177] = 8'b00000000;
    ram[15176] = 8'b00000000;
    ram[15175] = 8'b00000000;
    ram[15174] = 8'b00000000;
    ram[15173] = 8'b00000000;
    ram[15172] = 8'b00000000;
    ram[15171] = 8'b00000000;
    ram[15170] = 8'b00000000;
    ram[15169] = 8'b00000000;
    ram[15168] = 8'b00000000;
    ram[15167] = 8'b00000000;
    ram[15166] = 8'b00000000;
    ram[15165] = 8'b00000000;
    ram[15164] = 8'b00000000;
    ram[15163] = 8'b00000000;
    ram[15162] = 8'b00000000;
    ram[15161] = 8'b00000000;
    ram[15160] = 8'b00000000;
    ram[15159] = 8'b00000000;
    ram[15158] = 8'b00000000;
    ram[15157] = 8'b00000000;
    ram[15156] = 8'b00000000;
    ram[15155] = 8'b00000000;
    ram[15154] = 8'b00000000;
    ram[15153] = 8'b00000000;
    ram[15152] = 8'b00000000;
    ram[15151] = 8'b00000000;
    ram[15150] = 8'b00000000;
    ram[15149] = 8'b00000000;
    ram[15148] = 8'b00000000;
    ram[15147] = 8'b00000000;
    ram[15146] = 8'b00000000;
    ram[15145] = 8'b00000000;
    ram[15144] = 8'b00000000;
    ram[15143] = 8'b00000000;
    ram[15142] = 8'b00000000;
    ram[15141] = 8'b00000000;
    ram[15140] = 8'b00000000;
    ram[15139] = 8'b00000000;
    ram[15138] = 8'b00000000;
    ram[15137] = 8'b00000000;
    ram[15136] = 8'b00000000;
    ram[15135] = 8'b00000000;
    ram[15134] = 8'b00000000;
    ram[15133] = 8'b00000000;
    ram[15132] = 8'b00000000;
    ram[15131] = 8'b00000000;
    ram[15130] = 8'b00000000;
    ram[15129] = 8'b00000000;
    ram[15128] = 8'b00000000;
    ram[15127] = 8'b00000000;
    ram[15126] = 8'b00000000;
    ram[15125] = 8'b00000000;
    ram[15124] = 8'b00000000;
    ram[15123] = 8'b00000000;
    ram[15122] = 8'b00000000;
    ram[15121] = 8'b00000000;
    ram[15120] = 8'b00000000;
    ram[15119] = 8'b00000000;
    ram[15118] = 8'b00000000;
    ram[15117] = 8'b00000000;
    ram[15116] = 8'b00000000;
    ram[15115] = 8'b00000000;
    ram[15114] = 8'b00000000;
    ram[15113] = 8'b00000000;
    ram[15112] = 8'b00000000;
    ram[15111] = 8'b00000000;
    ram[15110] = 8'b00000000;
    ram[15109] = 8'b00000000;
    ram[15108] = 8'b00000000;
    ram[15107] = 8'b00000000;
    ram[15106] = 8'b00000000;
    ram[15105] = 8'b00000000;
    ram[15104] = 8'b00000000;
    ram[15103] = 8'b00000000;
    ram[15102] = 8'b00000000;
    ram[15101] = 8'b00000000;
    ram[15100] = 8'b00000000;
    ram[15099] = 8'b00000000;
    ram[15098] = 8'b00000000;
    ram[15097] = 8'b00000000;
    ram[15096] = 8'b00000000;
    ram[15095] = 8'b00000000;
    ram[15094] = 8'b00000000;
    ram[15093] = 8'b00000000;
    ram[15092] = 8'b00000000;
    ram[15091] = 8'b00000000;
    ram[15090] = 8'b00000000;
    ram[15089] = 8'b00000000;
    ram[15088] = 8'b00000000;
    ram[15087] = 8'b00000000;
    ram[15086] = 8'b00000000;
    ram[15085] = 8'b00000000;
    ram[15084] = 8'b00000000;
    ram[15083] = 8'b00000000;
    ram[15082] = 8'b00000000;
    ram[15081] = 8'b00000000;
    ram[15080] = 8'b00000000;
    ram[15079] = 8'b00000000;
    ram[15078] = 8'b00000000;
    ram[15077] = 8'b00000000;
    ram[15076] = 8'b00000000;
    ram[15075] = 8'b00000000;
    ram[15074] = 8'b00000000;
    ram[15073] = 8'b00000000;
    ram[15072] = 8'b00000000;
    ram[15071] = 8'b00000000;
    ram[15070] = 8'b00000000;
    ram[15069] = 8'b00000000;
    ram[15068] = 8'b00000000;
    ram[15067] = 8'b00000000;
    ram[15066] = 8'b00000000;
    ram[15065] = 8'b00000000;
    ram[15064] = 8'b00000000;
    ram[15063] = 8'b00000000;
    ram[15062] = 8'b00000000;
    ram[15061] = 8'b00000000;
    ram[15060] = 8'b00000000;
    ram[15059] = 8'b00000000;
    ram[15058] = 8'b00000000;
    ram[15057] = 8'b00000000;
    ram[15056] = 8'b00000000;
    ram[15055] = 8'b00000000;
    ram[15054] = 8'b00000000;
    ram[15053] = 8'b00000000;
    ram[15052] = 8'b00000000;
    ram[15051] = 8'b00000000;
    ram[15050] = 8'b00000000;
    ram[15049] = 8'b00000000;
    ram[15048] = 8'b00000000;
    ram[15047] = 8'b00000000;
    ram[15046] = 8'b00000000;
    ram[15045] = 8'b00000000;
    ram[15044] = 8'b00000000;
    ram[15043] = 8'b00000000;
    ram[15042] = 8'b00000000;
    ram[15041] = 8'b00000000;
    ram[15040] = 8'b00000000;
    ram[15039] = 8'b00000000;
    ram[15038] = 8'b00000000;
    ram[15037] = 8'b00000000;
    ram[15036] = 8'b00000000;
    ram[15035] = 8'b00000000;
    ram[15034] = 8'b00000000;
    ram[15033] = 8'b00000000;
    ram[15032] = 8'b00000000;
    ram[15031] = 8'b00000000;
    ram[15030] = 8'b00000000;
    ram[15029] = 8'b00000000;
    ram[15028] = 8'b00000000;
    ram[15027] = 8'b00000000;
    ram[15026] = 8'b00000000;
    ram[15025] = 8'b00000000;
    ram[15024] = 8'b00000000;
    ram[15023] = 8'b00000000;
    ram[15022] = 8'b00000000;
    ram[15021] = 8'b00000000;
    ram[15020] = 8'b00000000;
    ram[15019] = 8'b00000000;
    ram[15018] = 8'b00000000;
    ram[15017] = 8'b00000000;
    ram[15016] = 8'b00000000;
    ram[15015] = 8'b00000000;
    ram[15014] = 8'b00000000;
    ram[15013] = 8'b00000000;
    ram[15012] = 8'b00000000;
    ram[15011] = 8'b00000000;
    ram[15010] = 8'b00000000;
    ram[15009] = 8'b00000000;
    ram[15008] = 8'b00000000;
    ram[15007] = 8'b00000000;
    ram[15006] = 8'b00000000;
    ram[15005] = 8'b00000000;
    ram[15004] = 8'b00000000;
    ram[15003] = 8'b00000000;
    ram[15002] = 8'b00000000;
    ram[15001] = 8'b00000000;
    ram[15000] = 8'b00000000;
    ram[14999] = 8'b00000000;
    ram[14998] = 8'b00000000;
    ram[14997] = 8'b00000000;
    ram[14996] = 8'b00000000;
    ram[14995] = 8'b00000000;
    ram[14994] = 8'b00000000;
    ram[14993] = 8'b00000000;
    ram[14992] = 8'b00000000;
    ram[14991] = 8'b00000000;
    ram[14990] = 8'b00000000;
    ram[14989] = 8'b00000000;
    ram[14988] = 8'b00000000;
    ram[14987] = 8'b00000000;
    ram[14986] = 8'b00000000;
    ram[14985] = 8'b00000000;
    ram[14984] = 8'b00000000;
    ram[14983] = 8'b00000000;
    ram[14982] = 8'b00000000;
    ram[14981] = 8'b00000000;
    ram[14980] = 8'b00000000;
    ram[14979] = 8'b00000000;
    ram[14978] = 8'b00000000;
    ram[14977] = 8'b00000000;
    ram[14976] = 8'b00000000;
    ram[14975] = 8'b00000000;
    ram[14974] = 8'b00000000;
    ram[14973] = 8'b00000000;
    ram[14972] = 8'b00000000;
    ram[14971] = 8'b00000000;
    ram[14970] = 8'b00000000;
    ram[14969] = 8'b00000000;
    ram[14968] = 8'b00000000;
    ram[14967] = 8'b00000000;
    ram[14966] = 8'b00000000;
    ram[14965] = 8'b00000000;
    ram[14964] = 8'b00000000;
    ram[14963] = 8'b00000000;
    ram[14962] = 8'b00000000;
    ram[14961] = 8'b00000000;
    ram[14960] = 8'b00000000;
    ram[14959] = 8'b00000000;
    ram[14958] = 8'b00000000;
    ram[14957] = 8'b00000000;
    ram[14956] = 8'b00000000;
    ram[14955] = 8'b00000000;
    ram[14954] = 8'b00000000;
    ram[14953] = 8'b00000000;
    ram[14952] = 8'b00000000;
    ram[14951] = 8'b00000000;
    ram[14950] = 8'b00000000;
    ram[14949] = 8'b00000000;
    ram[14948] = 8'b00000000;
    ram[14947] = 8'b00000000;
    ram[14946] = 8'b00000000;
    ram[14945] = 8'b00000000;
    ram[14944] = 8'b00000000;
    ram[14943] = 8'b00000000;
    ram[14942] = 8'b00000000;
    ram[14941] = 8'b00000000;
    ram[14940] = 8'b00000000;
    ram[14939] = 8'b00000000;
    ram[14938] = 8'b00000000;
    ram[14937] = 8'b00000000;
    ram[14936] = 8'b00000000;
    ram[14935] = 8'b00000000;
    ram[14934] = 8'b00000000;
    ram[14933] = 8'b00000000;
    ram[14932] = 8'b00000000;
    ram[14931] = 8'b00000000;
    ram[14930] = 8'b00000000;
    ram[14929] = 8'b00000000;
    ram[14928] = 8'b00000000;
    ram[14927] = 8'b00000000;
    ram[14926] = 8'b00000000;
    ram[14925] = 8'b00000000;
    ram[14924] = 8'b00000000;
    ram[14923] = 8'b00000000;
    ram[14922] = 8'b00000000;
    ram[14921] = 8'b00000000;
    ram[14920] = 8'b00000000;
    ram[14919] = 8'b00000000;
    ram[14918] = 8'b00000000;
    ram[14917] = 8'b00000000;
    ram[14916] = 8'b00000000;
    ram[14915] = 8'b00000000;
    ram[14914] = 8'b00000000;
    ram[14913] = 8'b00000000;
    ram[14912] = 8'b00000000;
    ram[14911] = 8'b00000000;
    ram[14910] = 8'b00000000;
    ram[14909] = 8'b00000000;
    ram[14908] = 8'b00000000;
    ram[14907] = 8'b00000000;
    ram[14906] = 8'b00000000;
    ram[14905] = 8'b00000000;
    ram[14904] = 8'b00000000;
    ram[14903] = 8'b00000000;
    ram[14902] = 8'b00000000;
    ram[14901] = 8'b00000000;
    ram[14900] = 8'b00000000;
    ram[14899] = 8'b00000000;
    ram[14898] = 8'b00000000;
    ram[14897] = 8'b00000000;
    ram[14896] = 8'b00000000;
    ram[14895] = 8'b00000000;
    ram[14894] = 8'b00000000;
    ram[14893] = 8'b00000000;
    ram[14892] = 8'b00000000;
    ram[14891] = 8'b00000000;
    ram[14890] = 8'b00000000;
    ram[14889] = 8'b00000000;
    ram[14888] = 8'b00000000;
    ram[14887] = 8'b00000000;
    ram[14886] = 8'b00000000;
    ram[14885] = 8'b00000000;
    ram[14884] = 8'b00000000;
    ram[14883] = 8'b00000000;
    ram[14882] = 8'b00000000;
    ram[14881] = 8'b00000000;
    ram[14880] = 8'b00000000;
    ram[14879] = 8'b00000000;
    ram[14878] = 8'b00000000;
    ram[14877] = 8'b00000000;
    ram[14876] = 8'b00000000;
    ram[14875] = 8'b00000000;
    ram[14874] = 8'b00000000;
    ram[14873] = 8'b00000000;
    ram[14872] = 8'b00000000;
    ram[14871] = 8'b00000000;
    ram[14870] = 8'b00000000;
    ram[14869] = 8'b00000000;
    ram[14868] = 8'b00000000;
    ram[14867] = 8'b00000000;
    ram[14866] = 8'b00000000;
    ram[14865] = 8'b00000000;
    ram[14864] = 8'b00000000;
    ram[14863] = 8'b00000000;
    ram[14862] = 8'b00000000;
    ram[14861] = 8'b00000000;
    ram[14860] = 8'b00000000;
    ram[14859] = 8'b00000000;
    ram[14858] = 8'b00000000;
    ram[14857] = 8'b00000000;
    ram[14856] = 8'b00000000;
    ram[14855] = 8'b00000000;
    ram[14854] = 8'b00000000;
    ram[14853] = 8'b00000000;
    ram[14852] = 8'b00000000;
    ram[14851] = 8'b00000000;
    ram[14850] = 8'b00000000;
    ram[14849] = 8'b00000000;
    ram[14848] = 8'b00000000;
    ram[14847] = 8'b00000000;
    ram[14846] = 8'b00000000;
    ram[14845] = 8'b00000000;
    ram[14844] = 8'b00000000;
    ram[14843] = 8'b00000000;
    ram[14842] = 8'b00000000;
    ram[14841] = 8'b00000000;
    ram[14840] = 8'b00000000;
    ram[14839] = 8'b00000000;
    ram[14838] = 8'b00000000;
    ram[14837] = 8'b00000000;
    ram[14836] = 8'b00000000;
    ram[14835] = 8'b00000000;
    ram[14834] = 8'b00000000;
    ram[14833] = 8'b00000000;
    ram[14832] = 8'b00000000;
    ram[14831] = 8'b00000000;
    ram[14830] = 8'b00000000;
    ram[14829] = 8'b00000000;
    ram[14828] = 8'b00000000;
    ram[14827] = 8'b00000000;
    ram[14826] = 8'b00000000;
    ram[14825] = 8'b00000000;
    ram[14824] = 8'b00000000;
    ram[14823] = 8'b00000000;
    ram[14822] = 8'b00000000;
    ram[14821] = 8'b00000000;
    ram[14820] = 8'b00000000;
    ram[14819] = 8'b00000000;
    ram[14818] = 8'b00000000;
    ram[14817] = 8'b00000000;
    ram[14816] = 8'b00000000;
    ram[14815] = 8'b00000000;
    ram[14814] = 8'b00000000;
    ram[14813] = 8'b00000000;
    ram[14812] = 8'b00000000;
    ram[14811] = 8'b00000000;
    ram[14810] = 8'b00000000;
    ram[14809] = 8'b00000000;
    ram[14808] = 8'b00000000;
    ram[14807] = 8'b00000000;
    ram[14806] = 8'b00000000;
    ram[14805] = 8'b00000000;
    ram[14804] = 8'b00000000;
    ram[14803] = 8'b00000000;
    ram[14802] = 8'b00000000;
    ram[14801] = 8'b00000000;
    ram[14800] = 8'b00000000;
    ram[14799] = 8'b00000000;
    ram[14798] = 8'b00000000;
    ram[14797] = 8'b00000000;
    ram[14796] = 8'b00000000;
    ram[14795] = 8'b00000000;
    ram[14794] = 8'b00000000;
    ram[14793] = 8'b00000000;
    ram[14792] = 8'b00000000;
    ram[14791] = 8'b00000000;
    ram[14790] = 8'b00000000;
    ram[14789] = 8'b00000000;
    ram[14788] = 8'b00000000;
    ram[14787] = 8'b00000000;
    ram[14786] = 8'b00000000;
    ram[14785] = 8'b00000000;
    ram[14784] = 8'b00000000;
    ram[14783] = 8'b00000000;
    ram[14782] = 8'b00000000;
    ram[14781] = 8'b00000000;
    ram[14780] = 8'b00000000;
    ram[14779] = 8'b00000000;
    ram[14778] = 8'b00000000;
    ram[14777] = 8'b00000000;
    ram[14776] = 8'b00000000;
    ram[14775] = 8'b00000000;
    ram[14774] = 8'b00000000;
    ram[14773] = 8'b00000000;
    ram[14772] = 8'b00000000;
    ram[14771] = 8'b00000000;
    ram[14770] = 8'b00000000;
    ram[14769] = 8'b00000000;
    ram[14768] = 8'b00000000;
    ram[14767] = 8'b00000000;
    ram[14766] = 8'b00000000;
    ram[14765] = 8'b00000000;
    ram[14764] = 8'b00000000;
    ram[14763] = 8'b00000000;
    ram[14762] = 8'b00000000;
    ram[14761] = 8'b00000000;
    ram[14760] = 8'b00000000;
    ram[14759] = 8'b00000000;
    ram[14758] = 8'b00000000;
    ram[14757] = 8'b00000000;
    ram[14756] = 8'b00000000;
    ram[14755] = 8'b00000000;
    ram[14754] = 8'b00000000;
    ram[14753] = 8'b00000000;
    ram[14752] = 8'b00000000;
    ram[14751] = 8'b00000000;
    ram[14750] = 8'b00000000;
    ram[14749] = 8'b00000000;
    ram[14748] = 8'b00000000;
    ram[14747] = 8'b00000000;
    ram[14746] = 8'b00000000;
    ram[14745] = 8'b00000000;
    ram[14744] = 8'b00000000;
    ram[14743] = 8'b00000000;
    ram[14742] = 8'b00000000;
    ram[14741] = 8'b00000000;
    ram[14740] = 8'b00000000;
    ram[14739] = 8'b00000000;
    ram[14738] = 8'b00000000;
    ram[14737] = 8'b00000000;
    ram[14736] = 8'b00000000;
    ram[14735] = 8'b00000000;
    ram[14734] = 8'b00000000;
    ram[14733] = 8'b00000000;
    ram[14732] = 8'b00000000;
    ram[14731] = 8'b00000000;
    ram[14730] = 8'b00000000;
    ram[14729] = 8'b00000000;
    ram[14728] = 8'b00000000;
    ram[14727] = 8'b00000000;
    ram[14726] = 8'b00000000;
    ram[14725] = 8'b00000000;
    ram[14724] = 8'b00000000;
    ram[14723] = 8'b00000000;
    ram[14722] = 8'b00000000;
    ram[14721] = 8'b00000000;
    ram[14720] = 8'b00000000;
    ram[14719] = 8'b00000000;
    ram[14718] = 8'b00000000;
    ram[14717] = 8'b00000000;
    ram[14716] = 8'b00000000;
    ram[14715] = 8'b00000000;
    ram[14714] = 8'b00000000;
    ram[14713] = 8'b00000000;
    ram[14712] = 8'b00000000;
    ram[14711] = 8'b00000000;
    ram[14710] = 8'b00000000;
    ram[14709] = 8'b00000000;
    ram[14708] = 8'b00000000;
    ram[14707] = 8'b00000000;
    ram[14706] = 8'b00000000;
    ram[14705] = 8'b00000000;
    ram[14704] = 8'b00000000;
    ram[14703] = 8'b00000000;
    ram[14702] = 8'b00000000;
    ram[14701] = 8'b00000000;
    ram[14700] = 8'b00000000;
    ram[14699] = 8'b00000000;
    ram[14698] = 8'b00000000;
    ram[14697] = 8'b00000000;
    ram[14696] = 8'b00000000;
    ram[14695] = 8'b00000000;
    ram[14694] = 8'b00000000;
    ram[14693] = 8'b00000000;
    ram[14692] = 8'b00000000;
    ram[14691] = 8'b00000000;
    ram[14690] = 8'b00000000;
    ram[14689] = 8'b00000000;
    ram[14688] = 8'b00000000;
    ram[14687] = 8'b00000000;
    ram[14686] = 8'b00000000;
    ram[14685] = 8'b00000000;
    ram[14684] = 8'b00000000;
    ram[14683] = 8'b00000000;
    ram[14682] = 8'b00000000;
    ram[14681] = 8'b00000000;
    ram[14680] = 8'b00000000;
    ram[14679] = 8'b00000000;
    ram[14678] = 8'b00000000;
    ram[14677] = 8'b00000000;
    ram[14676] = 8'b00000000;
    ram[14675] = 8'b00000000;
    ram[14674] = 8'b00000000;
    ram[14673] = 8'b00000000;
    ram[14672] = 8'b00000000;
    ram[14671] = 8'b00000000;
    ram[14670] = 8'b00000000;
    ram[14669] = 8'b00000000;
    ram[14668] = 8'b00000000;
    ram[14667] = 8'b00000000;
    ram[14666] = 8'b00000000;
    ram[14665] = 8'b00000000;
    ram[14664] = 8'b00000000;
    ram[14663] = 8'b00000000;
    ram[14662] = 8'b00000000;
    ram[14661] = 8'b00000000;
    ram[14660] = 8'b00000000;
    ram[14659] = 8'b00000000;
    ram[14658] = 8'b00000000;
    ram[14657] = 8'b00000000;
    ram[14656] = 8'b00000000;
    ram[14655] = 8'b00000000;
    ram[14654] = 8'b00000000;
    ram[14653] = 8'b00000000;
    ram[14652] = 8'b00000000;
    ram[14651] = 8'b00000000;
    ram[14650] = 8'b00000000;
    ram[14649] = 8'b00000000;
    ram[14648] = 8'b00000000;
    ram[14647] = 8'b00000000;
    ram[14646] = 8'b00000000;
    ram[14645] = 8'b00000000;
    ram[14644] = 8'b00000000;
    ram[14643] = 8'b00000000;
    ram[14642] = 8'b00000000;
    ram[14641] = 8'b00000000;
    ram[14640] = 8'b00000000;
    ram[14639] = 8'b00000000;
    ram[14638] = 8'b00000000;
    ram[14637] = 8'b00000000;
    ram[14636] = 8'b00000000;
    ram[14635] = 8'b00000000;
    ram[14634] = 8'b00000000;
    ram[14633] = 8'b00000000;
    ram[14632] = 8'b00000000;
    ram[14631] = 8'b00000000;
    ram[14630] = 8'b00000000;
    ram[14629] = 8'b00000000;
    ram[14628] = 8'b00000000;
    ram[14627] = 8'b00000000;
    ram[14626] = 8'b00000000;
    ram[14625] = 8'b00000000;
    ram[14624] = 8'b00000000;
    ram[14623] = 8'b00000000;
    ram[14622] = 8'b00000000;
    ram[14621] = 8'b00000000;
    ram[14620] = 8'b00000000;
    ram[14619] = 8'b00000000;
    ram[14618] = 8'b00000000;
    ram[14617] = 8'b00000000;
    ram[14616] = 8'b00000000;
    ram[14615] = 8'b00000000;
    ram[14614] = 8'b00000000;
    ram[14613] = 8'b00000000;
    ram[14612] = 8'b00000000;
    ram[14611] = 8'b00000000;
    ram[14610] = 8'b00000000;
    ram[14609] = 8'b00000000;
    ram[14608] = 8'b00000000;
    ram[14607] = 8'b00000000;
    ram[14606] = 8'b00000000;
    ram[14605] = 8'b00000000;
    ram[14604] = 8'b00000000;
    ram[14603] = 8'b00000000;
    ram[14602] = 8'b00000000;
    ram[14601] = 8'b00000000;
    ram[14600] = 8'b00000000;
    ram[14599] = 8'b00000000;
    ram[14598] = 8'b00000000;
    ram[14597] = 8'b00000000;
    ram[14596] = 8'b00000000;
    ram[14595] = 8'b00000000;
    ram[14594] = 8'b00000000;
    ram[14593] = 8'b00000000;
    ram[14592] = 8'b00000000;
    ram[14591] = 8'b00000000;
    ram[14590] = 8'b00000000;
    ram[14589] = 8'b00000000;
    ram[14588] = 8'b00000000;
    ram[14587] = 8'b00000000;
    ram[14586] = 8'b00000000;
    ram[14585] = 8'b00000000;
    ram[14584] = 8'b00000000;
    ram[14583] = 8'b00000000;
    ram[14582] = 8'b00000000;
    ram[14581] = 8'b00000000;
    ram[14580] = 8'b00000000;
    ram[14579] = 8'b00000000;
    ram[14578] = 8'b00000000;
    ram[14577] = 8'b00000000;
    ram[14576] = 8'b00000000;
    ram[14575] = 8'b00000000;
    ram[14574] = 8'b00000000;
    ram[14573] = 8'b00000000;
    ram[14572] = 8'b00000000;
    ram[14571] = 8'b00000000;
    ram[14570] = 8'b00000000;
    ram[14569] = 8'b00000000;
    ram[14568] = 8'b00000000;
    ram[14567] = 8'b00000000;
    ram[14566] = 8'b00000000;
    ram[14565] = 8'b00000000;
    ram[14564] = 8'b00000000;
    ram[14563] = 8'b00000000;
    ram[14562] = 8'b00000000;
    ram[14561] = 8'b00000000;
    ram[14560] = 8'b00000000;
    ram[14559] = 8'b00000000;
    ram[14558] = 8'b00000000;
    ram[14557] = 8'b00000000;
    ram[14556] = 8'b00000000;
    ram[14555] = 8'b00000000;
    ram[14554] = 8'b00000000;
    ram[14553] = 8'b00000000;
    ram[14552] = 8'b00000000;
    ram[14551] = 8'b00000000;
    ram[14550] = 8'b00000000;
    ram[14549] = 8'b00000000;
    ram[14548] = 8'b00000000;
    ram[14547] = 8'b00000000;
    ram[14546] = 8'b00000000;
    ram[14545] = 8'b00000000;
    ram[14544] = 8'b00000000;
    ram[14543] = 8'b00000000;
    ram[14542] = 8'b00000000;
    ram[14541] = 8'b00000000;
    ram[14540] = 8'b00000000;
    ram[14539] = 8'b00000000;
    ram[14538] = 8'b00000000;
    ram[14537] = 8'b00000000;
    ram[14536] = 8'b00000000;
    ram[14535] = 8'b00000000;
    ram[14534] = 8'b00000000;
    ram[14533] = 8'b00000000;
    ram[14532] = 8'b00000000;
    ram[14531] = 8'b00000000;
    ram[14530] = 8'b00000000;
    ram[14529] = 8'b00000000;
    ram[14528] = 8'b00000000;
    ram[14527] = 8'b00000000;
    ram[14526] = 8'b00000000;
    ram[14525] = 8'b00000000;
    ram[14524] = 8'b00000000;
    ram[14523] = 8'b00000000;
    ram[14522] = 8'b00000000;
    ram[14521] = 8'b00000000;
    ram[14520] = 8'b00000000;
    ram[14519] = 8'b00000000;
    ram[14518] = 8'b00000000;
    ram[14517] = 8'b00000000;
    ram[14516] = 8'b00000000;
    ram[14515] = 8'b00000000;
    ram[14514] = 8'b00000000;
    ram[14513] = 8'b00000000;
    ram[14512] = 8'b00000000;
    ram[14511] = 8'b00000000;
    ram[14510] = 8'b00000000;
    ram[14509] = 8'b00000000;
    ram[14508] = 8'b00000000;
    ram[14507] = 8'b00000000;
    ram[14506] = 8'b00000000;
    ram[14505] = 8'b00000000;
    ram[14504] = 8'b00000000;
    ram[14503] = 8'b00000000;
    ram[14502] = 8'b00000000;
    ram[14501] = 8'b00000000;
    ram[14500] = 8'b00000000;
    ram[14499] = 8'b00000000;
    ram[14498] = 8'b00000000;
    ram[14497] = 8'b00000000;
    ram[14496] = 8'b00000000;
    ram[14495] = 8'b00000000;
    ram[14494] = 8'b00000000;
    ram[14493] = 8'b00000000;
    ram[14492] = 8'b00000000;
    ram[14491] = 8'b00000000;
    ram[14490] = 8'b00000000;
    ram[14489] = 8'b00000000;
    ram[14488] = 8'b00000000;
    ram[14487] = 8'b00000000;
    ram[14486] = 8'b00000000;
    ram[14485] = 8'b00000000;
    ram[14484] = 8'b00000000;
    ram[14483] = 8'b00000000;
    ram[14482] = 8'b00000000;
    ram[14481] = 8'b00000000;
    ram[14480] = 8'b00000000;
    ram[14479] = 8'b00000000;
    ram[14478] = 8'b00000000;
    ram[14477] = 8'b00000000;
    ram[14476] = 8'b00000000;
    ram[14475] = 8'b00000000;
    ram[14474] = 8'b00000000;
    ram[14473] = 8'b00000000;
    ram[14472] = 8'b00000000;
    ram[14471] = 8'b00000000;
    ram[14470] = 8'b00000000;
    ram[14469] = 8'b00000000;
    ram[14468] = 8'b00000000;
    ram[14467] = 8'b00000000;
    ram[14466] = 8'b00000000;
    ram[14465] = 8'b00000000;
    ram[14464] = 8'b00000000;
    ram[14463] = 8'b00000000;
    ram[14462] = 8'b00000000;
    ram[14461] = 8'b00000000;
    ram[14460] = 8'b00000000;
    ram[14459] = 8'b00000000;
    ram[14458] = 8'b00000000;
    ram[14457] = 8'b00000000;
    ram[14456] = 8'b00000000;
    ram[14455] = 8'b00000000;
    ram[14454] = 8'b00000000;
    ram[14453] = 8'b00000000;
    ram[14452] = 8'b00000000;
    ram[14451] = 8'b00000000;
    ram[14450] = 8'b00000000;
    ram[14449] = 8'b00000000;
    ram[14448] = 8'b00000000;
    ram[14447] = 8'b00000000;
    ram[14446] = 8'b00000000;
    ram[14445] = 8'b00000000;
    ram[14444] = 8'b00000000;
    ram[14443] = 8'b00000000;
    ram[14442] = 8'b00000000;
    ram[14441] = 8'b00000000;
    ram[14440] = 8'b00000000;
    ram[14439] = 8'b00000000;
    ram[14438] = 8'b00000000;
    ram[14437] = 8'b00000000;
    ram[14436] = 8'b00000000;
    ram[14435] = 8'b00000000;
    ram[14434] = 8'b00000000;
    ram[14433] = 8'b00000000;
    ram[14432] = 8'b00000000;
    ram[14431] = 8'b00000000;
    ram[14430] = 8'b00000000;
    ram[14429] = 8'b00000000;
    ram[14428] = 8'b00000000;
    ram[14427] = 8'b00000000;
    ram[14426] = 8'b00000000;
    ram[14425] = 8'b00000000;
    ram[14424] = 8'b00000000;
    ram[14423] = 8'b00000000;
    ram[14422] = 8'b00000000;
    ram[14421] = 8'b00000000;
    ram[14420] = 8'b00000000;
    ram[14419] = 8'b00000000;
    ram[14418] = 8'b00000000;
    ram[14417] = 8'b00000000;
    ram[14416] = 8'b00000000;
    ram[14415] = 8'b00000000;
    ram[14414] = 8'b00000000;
    ram[14413] = 8'b00000000;
    ram[14412] = 8'b00000000;
    ram[14411] = 8'b00000000;
    ram[14410] = 8'b00000000;
    ram[14409] = 8'b00000000;
    ram[14408] = 8'b00000000;
    ram[14407] = 8'b00000000;
    ram[14406] = 8'b00000000;
    ram[14405] = 8'b00000000;
    ram[14404] = 8'b00000000;
    ram[14403] = 8'b00000000;
    ram[14402] = 8'b00000000;
    ram[14401] = 8'b00000000;
    ram[14400] = 8'b00000000;
    ram[14399] = 8'b00000000;
    ram[14398] = 8'b00000000;
    ram[14397] = 8'b00000000;
    ram[14396] = 8'b00000000;
    ram[14395] = 8'b00000000;
    ram[14394] = 8'b00000000;
    ram[14393] = 8'b00000000;
    ram[14392] = 8'b00000000;
    ram[14391] = 8'b00000000;
    ram[14390] = 8'b00000000;
    ram[14389] = 8'b00000000;
    ram[14388] = 8'b00000000;
    ram[14387] = 8'b00000000;
    ram[14386] = 8'b00000000;
    ram[14385] = 8'b00000000;
    ram[14384] = 8'b00000000;
    ram[14383] = 8'b00000000;
    ram[14382] = 8'b00000000;
    ram[14381] = 8'b00000000;
    ram[14380] = 8'b00000000;
    ram[14379] = 8'b00000000;
    ram[14378] = 8'b00000000;
    ram[14377] = 8'b00000000;
    ram[14376] = 8'b00000000;
    ram[14375] = 8'b00000000;
    ram[14374] = 8'b00000000;
    ram[14373] = 8'b00000000;
    ram[14372] = 8'b00000000;
    ram[14371] = 8'b00000000;
    ram[14370] = 8'b00000000;
    ram[14369] = 8'b00000000;
    ram[14368] = 8'b00000000;
    ram[14367] = 8'b00000000;
    ram[14366] = 8'b00000000;
    ram[14365] = 8'b00000000;
    ram[14364] = 8'b00000000;
    ram[14363] = 8'b00000000;
    ram[14362] = 8'b00000000;
    ram[14361] = 8'b00000000;
    ram[14360] = 8'b00000000;
    ram[14359] = 8'b00000000;
    ram[14358] = 8'b00000000;
    ram[14357] = 8'b00000000;
    ram[14356] = 8'b00000000;
    ram[14355] = 8'b00000000;
    ram[14354] = 8'b00000000;
    ram[14353] = 8'b00000000;
    ram[14352] = 8'b00000000;
    ram[14351] = 8'b00000000;
    ram[14350] = 8'b00000000;
    ram[14349] = 8'b00000000;
    ram[14348] = 8'b00000000;
    ram[14347] = 8'b00000000;
    ram[14346] = 8'b00000000;
    ram[14345] = 8'b00000000;
    ram[14344] = 8'b00000000;
    ram[14343] = 8'b00000000;
    ram[14342] = 8'b00000000;
    ram[14341] = 8'b00000000;
    ram[14340] = 8'b00000000;
    ram[14339] = 8'b00000000;
    ram[14338] = 8'b00000000;
    ram[14337] = 8'b00000000;
    ram[14336] = 8'b00000000;
    ram[14335] = 8'b00000000;
    ram[14334] = 8'b00000000;
    ram[14333] = 8'b00000000;
    ram[14332] = 8'b00000000;
    ram[14331] = 8'b00000000;
    ram[14330] = 8'b00000000;
    ram[14329] = 8'b00000000;
    ram[14328] = 8'b00000000;
    ram[14327] = 8'b00000000;
    ram[14326] = 8'b00000000;
    ram[14325] = 8'b00000000;
    ram[14324] = 8'b00000000;
    ram[14323] = 8'b00000000;
    ram[14322] = 8'b00000000;
    ram[14321] = 8'b00000000;
    ram[14320] = 8'b00000000;
    ram[14319] = 8'b00000000;
    ram[14318] = 8'b00000000;
    ram[14317] = 8'b00000000;
    ram[14316] = 8'b00000000;
    ram[14315] = 8'b00000000;
    ram[14314] = 8'b00000000;
    ram[14313] = 8'b00000000;
    ram[14312] = 8'b00000000;
    ram[14311] = 8'b00000000;
    ram[14310] = 8'b00000000;
    ram[14309] = 8'b00000000;
    ram[14308] = 8'b00000000;
    ram[14307] = 8'b00000000;
    ram[14306] = 8'b00000000;
    ram[14305] = 8'b00000000;
    ram[14304] = 8'b00000000;
    ram[14303] = 8'b00000000;
    ram[14302] = 8'b00000000;
    ram[14301] = 8'b00000000;
    ram[14300] = 8'b00000000;
    ram[14299] = 8'b00000000;
    ram[14298] = 8'b00000000;
    ram[14297] = 8'b00000000;
    ram[14296] = 8'b00000000;
    ram[14295] = 8'b00000000;
    ram[14294] = 8'b00000000;
    ram[14293] = 8'b00000000;
    ram[14292] = 8'b00000000;
    ram[14291] = 8'b00000000;
    ram[14290] = 8'b00000000;
    ram[14289] = 8'b00000000;
    ram[14288] = 8'b00000000;
    ram[14287] = 8'b00000000;
    ram[14286] = 8'b00000000;
    ram[14285] = 8'b00000000;
    ram[14284] = 8'b00000000;
    ram[14283] = 8'b00000000;
    ram[14282] = 8'b00000000;
    ram[14281] = 8'b00000000;
    ram[14280] = 8'b00000000;
    ram[14279] = 8'b00000000;
    ram[14278] = 8'b00000000;
    ram[14277] = 8'b00000000;
    ram[14276] = 8'b00000000;
    ram[14275] = 8'b00000000;
    ram[14274] = 8'b00000000;
    ram[14273] = 8'b00000000;
    ram[14272] = 8'b00000000;
    ram[14271] = 8'b00000000;
    ram[14270] = 8'b00000000;
    ram[14269] = 8'b00000000;
    ram[14268] = 8'b00000000;
    ram[14267] = 8'b00000000;
    ram[14266] = 8'b00000000;
    ram[14265] = 8'b00000000;
    ram[14264] = 8'b00000000;
    ram[14263] = 8'b00000000;
    ram[14262] = 8'b00000000;
    ram[14261] = 8'b00000000;
    ram[14260] = 8'b00000000;
    ram[14259] = 8'b00000000;
    ram[14258] = 8'b00000000;
    ram[14257] = 8'b00000000;
    ram[14256] = 8'b00000000;
    ram[14255] = 8'b00000000;
    ram[14254] = 8'b00000000;
    ram[14253] = 8'b00000000;
    ram[14252] = 8'b00000000;
    ram[14251] = 8'b00000000;
    ram[14250] = 8'b00000000;
    ram[14249] = 8'b00000000;
    ram[14248] = 8'b00000000;
    ram[14247] = 8'b00000000;
    ram[14246] = 8'b00000000;
    ram[14245] = 8'b00000000;
    ram[14244] = 8'b00000000;
    ram[14243] = 8'b00000000;
    ram[14242] = 8'b00000000;
    ram[14241] = 8'b00000000;
    ram[14240] = 8'b00000000;
    ram[14239] = 8'b00000000;
    ram[14238] = 8'b00000000;
    ram[14237] = 8'b00000000;
    ram[14236] = 8'b00000000;
    ram[14235] = 8'b00000000;
    ram[14234] = 8'b00000000;
    ram[14233] = 8'b00000000;
    ram[14232] = 8'b00000000;
    ram[14231] = 8'b00000000;
    ram[14230] = 8'b00000000;
    ram[14229] = 8'b00000000;
    ram[14228] = 8'b00000000;
    ram[14227] = 8'b00000000;
    ram[14226] = 8'b00000000;
    ram[14225] = 8'b00000000;
    ram[14224] = 8'b00000000;
    ram[14223] = 8'b00000000;
    ram[14222] = 8'b00000000;
    ram[14221] = 8'b00000000;
    ram[14220] = 8'b00000000;
    ram[14219] = 8'b00000000;
    ram[14218] = 8'b00000000;
    ram[14217] = 8'b00000000;
    ram[14216] = 8'b00000000;
    ram[14215] = 8'b00000000;
    ram[14214] = 8'b00000000;
    ram[14213] = 8'b00000000;
    ram[14212] = 8'b00000000;
    ram[14211] = 8'b00000000;
    ram[14210] = 8'b00000000;
    ram[14209] = 8'b00000000;
    ram[14208] = 8'b00000000;
    ram[14207] = 8'b00000000;
    ram[14206] = 8'b00000000;
    ram[14205] = 8'b00000000;
    ram[14204] = 8'b00000000;
    ram[14203] = 8'b00000000;
    ram[14202] = 8'b00000000;
    ram[14201] = 8'b00000000;
    ram[14200] = 8'b00000000;
    ram[14199] = 8'b00000000;
    ram[14198] = 8'b00000000;
    ram[14197] = 8'b00000000;
    ram[14196] = 8'b00000000;
    ram[14195] = 8'b00000000;
    ram[14194] = 8'b00000000;
    ram[14193] = 8'b00000000;
    ram[14192] = 8'b00000000;
    ram[14191] = 8'b00000000;
    ram[14190] = 8'b00000000;
    ram[14189] = 8'b00000000;
    ram[14188] = 8'b00000000;
    ram[14187] = 8'b00000000;
    ram[14186] = 8'b00000000;
    ram[14185] = 8'b00000000;
    ram[14184] = 8'b00000000;
    ram[14183] = 8'b00000000;
    ram[14182] = 8'b00000000;
    ram[14181] = 8'b00000000;
    ram[14180] = 8'b00000000;
    ram[14179] = 8'b00000000;
    ram[14178] = 8'b00000000;
    ram[14177] = 8'b00000000;
    ram[14176] = 8'b00000000;
    ram[14175] = 8'b00000000;
    ram[14174] = 8'b00000000;
    ram[14173] = 8'b00000000;
    ram[14172] = 8'b00000000;
    ram[14171] = 8'b00000000;
    ram[14170] = 8'b00000000;
    ram[14169] = 8'b00000000;
    ram[14168] = 8'b00000000;
    ram[14167] = 8'b00000000;
    ram[14166] = 8'b00000000;
    ram[14165] = 8'b00000000;
    ram[14164] = 8'b00000000;
    ram[14163] = 8'b00000000;
    ram[14162] = 8'b00000000;
    ram[14161] = 8'b00000000;
    ram[14160] = 8'b00000000;
    ram[14159] = 8'b00000000;
    ram[14158] = 8'b00000000;
    ram[14157] = 8'b00000000;
    ram[14156] = 8'b00000000;
    ram[14155] = 8'b00000000;
    ram[14154] = 8'b00000000;
    ram[14153] = 8'b00000000;
    ram[14152] = 8'b00000000;
    ram[14151] = 8'b00000000;
    ram[14150] = 8'b00000000;
    ram[14149] = 8'b00000000;
    ram[14148] = 8'b00000000;
    ram[14147] = 8'b00000000;
    ram[14146] = 8'b00000000;
    ram[14145] = 8'b00000000;
    ram[14144] = 8'b00000000;
    ram[14143] = 8'b00000000;
    ram[14142] = 8'b00000000;
    ram[14141] = 8'b00000000;
    ram[14140] = 8'b00000000;
    ram[14139] = 8'b00000000;
    ram[14138] = 8'b00000000;
    ram[14137] = 8'b00000000;
    ram[14136] = 8'b00000000;
    ram[14135] = 8'b00000000;
    ram[14134] = 8'b00000000;
    ram[14133] = 8'b00000000;
    ram[14132] = 8'b00000000;
    ram[14131] = 8'b00000000;
    ram[14130] = 8'b00000000;
    ram[14129] = 8'b00000000;
    ram[14128] = 8'b00000000;
    ram[14127] = 8'b00000000;
    ram[14126] = 8'b00000000;
    ram[14125] = 8'b00000000;
    ram[14124] = 8'b00000000;
    ram[14123] = 8'b00000000;
    ram[14122] = 8'b00000000;
    ram[14121] = 8'b00000000;
    ram[14120] = 8'b00000000;
    ram[14119] = 8'b00000000;
    ram[14118] = 8'b00000000;
    ram[14117] = 8'b00000000;
    ram[14116] = 8'b00000000;
    ram[14115] = 8'b00000000;
    ram[14114] = 8'b00000000;
    ram[14113] = 8'b00000000;
    ram[14112] = 8'b00000000;
    ram[14111] = 8'b00000000;
    ram[14110] = 8'b00000000;
    ram[14109] = 8'b00000000;
    ram[14108] = 8'b00000000;
    ram[14107] = 8'b00000000;
    ram[14106] = 8'b00000000;
    ram[14105] = 8'b00000000;
    ram[14104] = 8'b00000000;
    ram[14103] = 8'b00000000;
    ram[14102] = 8'b00000000;
    ram[14101] = 8'b00000000;
    ram[14100] = 8'b00000000;
    ram[14099] = 8'b00000000;
    ram[14098] = 8'b00000000;
    ram[14097] = 8'b00000000;
    ram[14096] = 8'b00000000;
    ram[14095] = 8'b00000000;
    ram[14094] = 8'b00000000;
    ram[14093] = 8'b00000000;
    ram[14092] = 8'b00000000;
    ram[14091] = 8'b00000000;
    ram[14090] = 8'b00000000;
    ram[14089] = 8'b00000000;
    ram[14088] = 8'b00000000;
    ram[14087] = 8'b00000000;
    ram[14086] = 8'b00000000;
    ram[14085] = 8'b00000000;
    ram[14084] = 8'b00000000;
    ram[14083] = 8'b00000000;
    ram[14082] = 8'b00000000;
    ram[14081] = 8'b00000000;
    ram[14080] = 8'b00000000;
    ram[14079] = 8'b00000000;
    ram[14078] = 8'b00000000;
    ram[14077] = 8'b00000000;
    ram[14076] = 8'b00000000;
    ram[14075] = 8'b00000000;
    ram[14074] = 8'b00000000;
    ram[14073] = 8'b00000000;
    ram[14072] = 8'b00000000;
    ram[14071] = 8'b00000000;
    ram[14070] = 8'b00000000;
    ram[14069] = 8'b00000000;
    ram[14068] = 8'b00000000;
    ram[14067] = 8'b00000000;
    ram[14066] = 8'b00000000;
    ram[14065] = 8'b00000000;
    ram[14064] = 8'b00000000;
    ram[14063] = 8'b00000000;
    ram[14062] = 8'b00000000;
    ram[14061] = 8'b00000000;
    ram[14060] = 8'b00000000;
    ram[14059] = 8'b00000000;
    ram[14058] = 8'b00000000;
    ram[14057] = 8'b00000000;
    ram[14056] = 8'b00000000;
    ram[14055] = 8'b00000000;
    ram[14054] = 8'b00000000;
    ram[14053] = 8'b00000000;
    ram[14052] = 8'b00000000;
    ram[14051] = 8'b00000000;
    ram[14050] = 8'b00000000;
    ram[14049] = 8'b00000000;
    ram[14048] = 8'b00000000;
    ram[14047] = 8'b00000000;
    ram[14046] = 8'b00000000;
    ram[14045] = 8'b00000000;
    ram[14044] = 8'b00000000;
    ram[14043] = 8'b00000000;
    ram[14042] = 8'b00000000;
    ram[14041] = 8'b00000000;
    ram[14040] = 8'b00000000;
    ram[14039] = 8'b00000000;
    ram[14038] = 8'b00000000;
    ram[14037] = 8'b00000000;
    ram[14036] = 8'b00000000;
    ram[14035] = 8'b00000000;
    ram[14034] = 8'b00000000;
    ram[14033] = 8'b00000000;
    ram[14032] = 8'b00000000;
    ram[14031] = 8'b00000000;
    ram[14030] = 8'b00000000;
    ram[14029] = 8'b00000000;
    ram[14028] = 8'b00000000;
    ram[14027] = 8'b00000000;
    ram[14026] = 8'b00000000;
    ram[14025] = 8'b00000000;
    ram[14024] = 8'b00000000;
    ram[14023] = 8'b00000000;
    ram[14022] = 8'b00000000;
    ram[14021] = 8'b00000000;
    ram[14020] = 8'b00000000;
    ram[14019] = 8'b00000000;
    ram[14018] = 8'b00000000;
    ram[14017] = 8'b00000000;
    ram[14016] = 8'b00000000;
    ram[14015] = 8'b00000000;
    ram[14014] = 8'b00000000;
    ram[14013] = 8'b00000000;
    ram[14012] = 8'b00000000;
    ram[14011] = 8'b00000000;
    ram[14010] = 8'b00000000;
    ram[14009] = 8'b00000000;
    ram[14008] = 8'b00000000;
    ram[14007] = 8'b00000000;
    ram[14006] = 8'b00000000;
    ram[14005] = 8'b00000000;
    ram[14004] = 8'b00000000;
    ram[14003] = 8'b00000000;
    ram[14002] = 8'b00000000;
    ram[14001] = 8'b00000000;
    ram[14000] = 8'b00000000;
    ram[13999] = 8'b00000000;
    ram[13998] = 8'b00000000;
    ram[13997] = 8'b00000000;
    ram[13996] = 8'b00000000;
    ram[13995] = 8'b00000000;
    ram[13994] = 8'b00000000;
    ram[13993] = 8'b00000000;
    ram[13992] = 8'b00000000;
    ram[13991] = 8'b00000000;
    ram[13990] = 8'b00000000;
    ram[13989] = 8'b00000000;
    ram[13988] = 8'b00000000;
    ram[13987] = 8'b00000000;
    ram[13986] = 8'b00000000;
    ram[13985] = 8'b00000000;
    ram[13984] = 8'b00000000;
    ram[13983] = 8'b00000000;
    ram[13982] = 8'b00000000;
    ram[13981] = 8'b00000000;
    ram[13980] = 8'b00000000;
    ram[13979] = 8'b00000000;
    ram[13978] = 8'b00000000;
    ram[13977] = 8'b00000000;
    ram[13976] = 8'b00000000;
    ram[13975] = 8'b00000000;
    ram[13974] = 8'b00000000;
    ram[13973] = 8'b00000000;
    ram[13972] = 8'b00000000;
    ram[13971] = 8'b00000000;
    ram[13970] = 8'b00000000;
    ram[13969] = 8'b00000000;
    ram[13968] = 8'b00000000;
    ram[13967] = 8'b00000000;
    ram[13966] = 8'b00000000;
    ram[13965] = 8'b00000000;
    ram[13964] = 8'b00000000;
    ram[13963] = 8'b00000000;
    ram[13962] = 8'b00000000;
    ram[13961] = 8'b00000000;
    ram[13960] = 8'b00000000;
    ram[13959] = 8'b00000000;
    ram[13958] = 8'b00000000;
    ram[13957] = 8'b00000000;
    ram[13956] = 8'b00000000;
    ram[13955] = 8'b00000000;
    ram[13954] = 8'b00000000;
    ram[13953] = 8'b00000000;
    ram[13952] = 8'b00000000;
    ram[13951] = 8'b00000000;
    ram[13950] = 8'b00000000;
    ram[13949] = 8'b00000000;
    ram[13948] = 8'b00000000;
    ram[13947] = 8'b00000000;
    ram[13946] = 8'b00000000;
    ram[13945] = 8'b00000000;
    ram[13944] = 8'b00000000;
    ram[13943] = 8'b00000000;
    ram[13942] = 8'b00000000;
    ram[13941] = 8'b00000000;
    ram[13940] = 8'b00000000;
    ram[13939] = 8'b00000000;
    ram[13938] = 8'b00000000;
    ram[13937] = 8'b00000000;
    ram[13936] = 8'b00000000;
    ram[13935] = 8'b00000000;
    ram[13934] = 8'b00000000;
    ram[13933] = 8'b00000000;
    ram[13932] = 8'b00000000;
    ram[13931] = 8'b00000000;
    ram[13930] = 8'b00000000;
    ram[13929] = 8'b00000000;
    ram[13928] = 8'b00000000;
    ram[13927] = 8'b00000000;
    ram[13926] = 8'b00000000;
    ram[13925] = 8'b00000000;
    ram[13924] = 8'b00000000;
    ram[13923] = 8'b00000000;
    ram[13922] = 8'b00000000;
    ram[13921] = 8'b00000000;
    ram[13920] = 8'b00000000;
    ram[13919] = 8'b00000000;
    ram[13918] = 8'b00000000;
    ram[13917] = 8'b00000000;
    ram[13916] = 8'b00000000;
    ram[13915] = 8'b00000000;
    ram[13914] = 8'b00000000;
    ram[13913] = 8'b00000000;
    ram[13912] = 8'b00000000;
    ram[13911] = 8'b00000000;
    ram[13910] = 8'b00000000;
    ram[13909] = 8'b00000000;
    ram[13908] = 8'b00000000;
    ram[13907] = 8'b00000000;
    ram[13906] = 8'b00000000;
    ram[13905] = 8'b00000000;
    ram[13904] = 8'b00000000;
    ram[13903] = 8'b00000000;
    ram[13902] = 8'b00000000;
    ram[13901] = 8'b00000000;
    ram[13900] = 8'b00000000;
    ram[13899] = 8'b00000000;
    ram[13898] = 8'b00000000;
    ram[13897] = 8'b00000000;
    ram[13896] = 8'b00000000;
    ram[13895] = 8'b00000000;
    ram[13894] = 8'b00000000;
    ram[13893] = 8'b00000000;
    ram[13892] = 8'b00000000;
    ram[13891] = 8'b00000000;
    ram[13890] = 8'b00000000;
    ram[13889] = 8'b00000000;
    ram[13888] = 8'b00000000;
    ram[13887] = 8'b00000000;
    ram[13886] = 8'b00000000;
    ram[13885] = 8'b00000000;
    ram[13884] = 8'b00000000;
    ram[13883] = 8'b00000000;
    ram[13882] = 8'b00000000;
    ram[13881] = 8'b00000000;
    ram[13880] = 8'b00000000;
    ram[13879] = 8'b00000000;
    ram[13878] = 8'b00000000;
    ram[13877] = 8'b00000000;
    ram[13876] = 8'b00000000;
    ram[13875] = 8'b00000000;
    ram[13874] = 8'b00000000;
    ram[13873] = 8'b00000000;
    ram[13872] = 8'b00000000;
    ram[13871] = 8'b00000000;
    ram[13870] = 8'b00000000;
    ram[13869] = 8'b00000000;
    ram[13868] = 8'b00000000;
    ram[13867] = 8'b00000000;
    ram[13866] = 8'b00000000;
    ram[13865] = 8'b00000000;
    ram[13864] = 8'b00000000;
    ram[13863] = 8'b00000000;
    ram[13862] = 8'b00000000;
    ram[13861] = 8'b00000000;
    ram[13860] = 8'b00000000;
    ram[13859] = 8'b00000000;
    ram[13858] = 8'b00000000;
    ram[13857] = 8'b00000000;
    ram[13856] = 8'b00000000;
    ram[13855] = 8'b00000000;
    ram[13854] = 8'b00000000;
    ram[13853] = 8'b00000000;
    ram[13852] = 8'b00000000;
    ram[13851] = 8'b00000000;
    ram[13850] = 8'b00000000;
    ram[13849] = 8'b00000000;
    ram[13848] = 8'b00000000;
    ram[13847] = 8'b00000000;
    ram[13846] = 8'b00000000;
    ram[13845] = 8'b00000000;
    ram[13844] = 8'b00000000;
    ram[13843] = 8'b00000000;
    ram[13842] = 8'b00000000;
    ram[13841] = 8'b00000000;
    ram[13840] = 8'b00000000;
    ram[13839] = 8'b00000000;
    ram[13838] = 8'b00000000;
    ram[13837] = 8'b00000000;
    ram[13836] = 8'b00000000;
    ram[13835] = 8'b00000000;
    ram[13834] = 8'b00000000;
    ram[13833] = 8'b00000000;
    ram[13832] = 8'b00000000;
    ram[13831] = 8'b00000000;
    ram[13830] = 8'b00000000;
    ram[13829] = 8'b00000000;
    ram[13828] = 8'b00000000;
    ram[13827] = 8'b00000000;
    ram[13826] = 8'b00000000;
    ram[13825] = 8'b00000000;
    ram[13824] = 8'b00000000;
    ram[13823] = 8'b00000000;
    ram[13822] = 8'b00000000;
    ram[13821] = 8'b00000000;
    ram[13820] = 8'b00000000;
    ram[13819] = 8'b00000000;
    ram[13818] = 8'b00000000;
    ram[13817] = 8'b00000000;
    ram[13816] = 8'b00000000;
    ram[13815] = 8'b00000000;
    ram[13814] = 8'b00000000;
    ram[13813] = 8'b00000000;
    ram[13812] = 8'b00000000;
    ram[13811] = 8'b00000000;
    ram[13810] = 8'b00000000;
    ram[13809] = 8'b00000000;
    ram[13808] = 8'b00000000;
    ram[13807] = 8'b00000000;
    ram[13806] = 8'b00000000;
    ram[13805] = 8'b00000000;
    ram[13804] = 8'b00000000;
    ram[13803] = 8'b00000000;
    ram[13802] = 8'b00000000;
    ram[13801] = 8'b00000000;
    ram[13800] = 8'b00000000;
    ram[13799] = 8'b00000000;
    ram[13798] = 8'b00000000;
    ram[13797] = 8'b00000000;
    ram[13796] = 8'b00000000;
    ram[13795] = 8'b00000000;
    ram[13794] = 8'b00000000;
    ram[13793] = 8'b00000000;
    ram[13792] = 8'b00000000;
    ram[13791] = 8'b00000000;
    ram[13790] = 8'b00000000;
    ram[13789] = 8'b00000000;
    ram[13788] = 8'b00000000;
    ram[13787] = 8'b00000000;
    ram[13786] = 8'b00000000;
    ram[13785] = 8'b00000000;
    ram[13784] = 8'b00000000;
    ram[13783] = 8'b00000000;
    ram[13782] = 8'b00000000;
    ram[13781] = 8'b00000000;
    ram[13780] = 8'b00000000;
    ram[13779] = 8'b00000000;
    ram[13778] = 8'b00000000;
    ram[13777] = 8'b00000000;
    ram[13776] = 8'b00000000;
    ram[13775] = 8'b00000000;
    ram[13774] = 8'b00000000;
    ram[13773] = 8'b00000000;
    ram[13772] = 8'b00000000;
    ram[13771] = 8'b00000000;
    ram[13770] = 8'b00000000;
    ram[13769] = 8'b00000000;
    ram[13768] = 8'b00000000;
    ram[13767] = 8'b00000000;
    ram[13766] = 8'b00000000;
    ram[13765] = 8'b00000000;
    ram[13764] = 8'b00000000;
    ram[13763] = 8'b00000000;
    ram[13762] = 8'b00000000;
    ram[13761] = 8'b00000000;
    ram[13760] = 8'b00000000;
    ram[13759] = 8'b00000000;
    ram[13758] = 8'b00000000;
    ram[13757] = 8'b00000000;
    ram[13756] = 8'b00000000;
    ram[13755] = 8'b00000000;
    ram[13754] = 8'b00000000;
    ram[13753] = 8'b00000000;
    ram[13752] = 8'b00000000;
    ram[13751] = 8'b00000000;
    ram[13750] = 8'b00000000;
    ram[13749] = 8'b00000000;
    ram[13748] = 8'b00000000;
    ram[13747] = 8'b00000000;
    ram[13746] = 8'b00000000;
    ram[13745] = 8'b00000000;
    ram[13744] = 8'b00000000;
    ram[13743] = 8'b00000000;
    ram[13742] = 8'b00000000;
    ram[13741] = 8'b00000000;
    ram[13740] = 8'b00000000;
    ram[13739] = 8'b00000000;
    ram[13738] = 8'b00000000;
    ram[13737] = 8'b00000000;
    ram[13736] = 8'b00000000;
    ram[13735] = 8'b00000000;
    ram[13734] = 8'b00000000;
    ram[13733] = 8'b00000000;
    ram[13732] = 8'b00000000;
    ram[13731] = 8'b00000000;
    ram[13730] = 8'b00000000;
    ram[13729] = 8'b00000000;
    ram[13728] = 8'b00000000;
    ram[13727] = 8'b00000000;
    ram[13726] = 8'b00000000;
    ram[13725] = 8'b00000000;
    ram[13724] = 8'b00000000;
    ram[13723] = 8'b00000000;
    ram[13722] = 8'b00000000;
    ram[13721] = 8'b00000000;
    ram[13720] = 8'b00000000;
    ram[13719] = 8'b00000000;
    ram[13718] = 8'b00000000;
    ram[13717] = 8'b00000000;
    ram[13716] = 8'b00000000;
    ram[13715] = 8'b00000000;
    ram[13714] = 8'b00000000;
    ram[13713] = 8'b00000000;
    ram[13712] = 8'b00000000;
    ram[13711] = 8'b00000000;
    ram[13710] = 8'b00000000;
    ram[13709] = 8'b00000000;
    ram[13708] = 8'b00000000;
    ram[13707] = 8'b00000000;
    ram[13706] = 8'b00000000;
    ram[13705] = 8'b00000000;
    ram[13704] = 8'b00000000;
    ram[13703] = 8'b00000000;
    ram[13702] = 8'b00000000;
    ram[13701] = 8'b00000000;
    ram[13700] = 8'b00000000;
    ram[13699] = 8'b00000000;
    ram[13698] = 8'b00000000;
    ram[13697] = 8'b00000000;
    ram[13696] = 8'b00000000;
    ram[13695] = 8'b00000000;
    ram[13694] = 8'b00000000;
    ram[13693] = 8'b00000000;
    ram[13692] = 8'b00000000;
    ram[13691] = 8'b00000000;
    ram[13690] = 8'b00000000;
    ram[13689] = 8'b00000000;
    ram[13688] = 8'b00000000;
    ram[13687] = 8'b00000000;
    ram[13686] = 8'b00000000;
    ram[13685] = 8'b00000000;
    ram[13684] = 8'b00000000;
    ram[13683] = 8'b00000000;
    ram[13682] = 8'b00000000;
    ram[13681] = 8'b00000000;
    ram[13680] = 8'b00000000;
    ram[13679] = 8'b00000000;
    ram[13678] = 8'b00000000;
    ram[13677] = 8'b00000000;
    ram[13676] = 8'b00000000;
    ram[13675] = 8'b00000000;
    ram[13674] = 8'b00000000;
    ram[13673] = 8'b00000000;
    ram[13672] = 8'b00000000;
    ram[13671] = 8'b00000000;
    ram[13670] = 8'b00000000;
    ram[13669] = 8'b00000000;
    ram[13668] = 8'b00000000;
    ram[13667] = 8'b00000000;
    ram[13666] = 8'b00000000;
    ram[13665] = 8'b00000000;
    ram[13664] = 8'b00000000;
    ram[13663] = 8'b00000000;
    ram[13662] = 8'b00000000;
    ram[13661] = 8'b00000000;
    ram[13660] = 8'b00000000;
    ram[13659] = 8'b00000000;
    ram[13658] = 8'b00000000;
    ram[13657] = 8'b00000000;
    ram[13656] = 8'b00000000;
    ram[13655] = 8'b00000000;
    ram[13654] = 8'b00000000;
    ram[13653] = 8'b00000000;
    ram[13652] = 8'b00000000;
    ram[13651] = 8'b00000000;
    ram[13650] = 8'b00000000;
    ram[13649] = 8'b00000000;
    ram[13648] = 8'b00000000;
    ram[13647] = 8'b00000000;
    ram[13646] = 8'b00000000;
    ram[13645] = 8'b00000000;
    ram[13644] = 8'b00000000;
    ram[13643] = 8'b00000000;
    ram[13642] = 8'b00000000;
    ram[13641] = 8'b00000000;
    ram[13640] = 8'b00000000;
    ram[13639] = 8'b00000000;
    ram[13638] = 8'b00000000;
    ram[13637] = 8'b00000000;
    ram[13636] = 8'b00000000;
    ram[13635] = 8'b00000000;
    ram[13634] = 8'b00000000;
    ram[13633] = 8'b00000000;
    ram[13632] = 8'b00000000;
    ram[13631] = 8'b00000000;
    ram[13630] = 8'b00000000;
    ram[13629] = 8'b00000000;
    ram[13628] = 8'b00000000;
    ram[13627] = 8'b00000000;
    ram[13626] = 8'b00000000;
    ram[13625] = 8'b00000000;
    ram[13624] = 8'b00000000;
    ram[13623] = 8'b00000000;
    ram[13622] = 8'b00000000;
    ram[13621] = 8'b00000000;
    ram[13620] = 8'b00000000;
    ram[13619] = 8'b00000000;
    ram[13618] = 8'b00000000;
    ram[13617] = 8'b00000000;
    ram[13616] = 8'b00000000;
    ram[13615] = 8'b00000000;
    ram[13614] = 8'b00000000;
    ram[13613] = 8'b00000000;
    ram[13612] = 8'b00000000;
    ram[13611] = 8'b00000000;
    ram[13610] = 8'b00000000;
    ram[13609] = 8'b00000000;
    ram[13608] = 8'b00000000;
    ram[13607] = 8'b00000000;
    ram[13606] = 8'b00000000;
    ram[13605] = 8'b00000000;
    ram[13604] = 8'b00000000;
    ram[13603] = 8'b00000000;
    ram[13602] = 8'b00000000;
    ram[13601] = 8'b00000000;
    ram[13600] = 8'b00000000;
    ram[13599] = 8'b00000000;
    ram[13598] = 8'b00000000;
    ram[13597] = 8'b00000000;
    ram[13596] = 8'b00000000;
    ram[13595] = 8'b00000000;
    ram[13594] = 8'b00000000;
    ram[13593] = 8'b00000000;
    ram[13592] = 8'b00000000;
    ram[13591] = 8'b00000000;
    ram[13590] = 8'b00000000;
    ram[13589] = 8'b00000000;
    ram[13588] = 8'b00000000;
    ram[13587] = 8'b00000000;
    ram[13586] = 8'b00000000;
    ram[13585] = 8'b00000000;
    ram[13584] = 8'b00000000;
    ram[13583] = 8'b00000000;
    ram[13582] = 8'b00000000;
    ram[13581] = 8'b00000000;
    ram[13580] = 8'b00000000;
    ram[13579] = 8'b00000000;
    ram[13578] = 8'b00000000;
    ram[13577] = 8'b00000000;
    ram[13576] = 8'b00000000;
    ram[13575] = 8'b00000000;
    ram[13574] = 8'b00000000;
    ram[13573] = 8'b00000000;
    ram[13572] = 8'b00000000;
    ram[13571] = 8'b00000000;
    ram[13570] = 8'b00000000;
    ram[13569] = 8'b00000000;
    ram[13568] = 8'b00000000;
    ram[13567] = 8'b00000000;
    ram[13566] = 8'b00000000;
    ram[13565] = 8'b00000000;
    ram[13564] = 8'b00000000;
    ram[13563] = 8'b00000000;
    ram[13562] = 8'b00000000;
    ram[13561] = 8'b00000000;
    ram[13560] = 8'b00000000;
    ram[13559] = 8'b00000000;
    ram[13558] = 8'b00000000;
    ram[13557] = 8'b00000000;
    ram[13556] = 8'b00000000;
    ram[13555] = 8'b00000000;
    ram[13554] = 8'b00000000;
    ram[13553] = 8'b00000000;
    ram[13552] = 8'b00000000;
    ram[13551] = 8'b00000000;
    ram[13550] = 8'b00000000;
    ram[13549] = 8'b00000000;
    ram[13548] = 8'b00000000;
    ram[13547] = 8'b00000000;
    ram[13546] = 8'b00000000;
    ram[13545] = 8'b00000000;
    ram[13544] = 8'b00000000;
    ram[13543] = 8'b00000000;
    ram[13542] = 8'b00000000;
    ram[13541] = 8'b00000000;
    ram[13540] = 8'b00000000;
    ram[13539] = 8'b00000000;
    ram[13538] = 8'b00000000;
    ram[13537] = 8'b00000000;
    ram[13536] = 8'b00000000;
    ram[13535] = 8'b00000000;
    ram[13534] = 8'b00000000;
    ram[13533] = 8'b00000000;
    ram[13532] = 8'b00000000;
    ram[13531] = 8'b00000000;
    ram[13530] = 8'b00000000;
    ram[13529] = 8'b00000000;
    ram[13528] = 8'b00000000;
    ram[13527] = 8'b00000000;
    ram[13526] = 8'b00000000;
    ram[13525] = 8'b00000000;
    ram[13524] = 8'b00000000;
    ram[13523] = 8'b00000000;
    ram[13522] = 8'b00000000;
    ram[13521] = 8'b00000000;
    ram[13520] = 8'b00000000;
    ram[13519] = 8'b00000000;
    ram[13518] = 8'b00000000;
    ram[13517] = 8'b00000000;
    ram[13516] = 8'b00000000;
    ram[13515] = 8'b00000000;
    ram[13514] = 8'b00000000;
    ram[13513] = 8'b00000000;
    ram[13512] = 8'b00000000;
    ram[13511] = 8'b00000000;
    ram[13510] = 8'b00000000;
    ram[13509] = 8'b00000000;
    ram[13508] = 8'b00000000;
    ram[13507] = 8'b00000000;
    ram[13506] = 8'b00000000;
    ram[13505] = 8'b00000000;
    ram[13504] = 8'b00000000;
    ram[13503] = 8'b00000000;
    ram[13502] = 8'b00000000;
    ram[13501] = 8'b00000000;
    ram[13500] = 8'b00000000;
    ram[13499] = 8'b00000000;
    ram[13498] = 8'b00000000;
    ram[13497] = 8'b00000000;
    ram[13496] = 8'b00000000;
    ram[13495] = 8'b00000000;
    ram[13494] = 8'b00000000;
    ram[13493] = 8'b00000000;
    ram[13492] = 8'b00000000;
    ram[13491] = 8'b00000000;
    ram[13490] = 8'b00000000;
    ram[13489] = 8'b00000000;
    ram[13488] = 8'b00000000;
    ram[13487] = 8'b00000000;
    ram[13486] = 8'b00000000;
    ram[13485] = 8'b00000000;
    ram[13484] = 8'b00000000;
    ram[13483] = 8'b00000000;
    ram[13482] = 8'b00000000;
    ram[13481] = 8'b00000000;
    ram[13480] = 8'b00000000;
    ram[13479] = 8'b00000000;
    ram[13478] = 8'b00000000;
    ram[13477] = 8'b00000000;
    ram[13476] = 8'b00000000;
    ram[13475] = 8'b00000000;
    ram[13474] = 8'b00000000;
    ram[13473] = 8'b00000000;
    ram[13472] = 8'b00000000;
    ram[13471] = 8'b00000000;
    ram[13470] = 8'b00000000;
    ram[13469] = 8'b00000000;
    ram[13468] = 8'b00000000;
    ram[13467] = 8'b00000000;
    ram[13466] = 8'b00000000;
    ram[13465] = 8'b00000000;
    ram[13464] = 8'b00000000;
    ram[13463] = 8'b00000000;
    ram[13462] = 8'b00000000;
    ram[13461] = 8'b00000000;
    ram[13460] = 8'b00000000;
    ram[13459] = 8'b00000000;
    ram[13458] = 8'b00000000;
    ram[13457] = 8'b00000000;
    ram[13456] = 8'b00000000;
    ram[13455] = 8'b00000000;
    ram[13454] = 8'b00000000;
    ram[13453] = 8'b00000000;
    ram[13452] = 8'b00000000;
    ram[13451] = 8'b00000000;
    ram[13450] = 8'b00000000;
    ram[13449] = 8'b00000000;
    ram[13448] = 8'b00000000;
    ram[13447] = 8'b00000000;
    ram[13446] = 8'b00000000;
    ram[13445] = 8'b00000000;
    ram[13444] = 8'b00000000;
    ram[13443] = 8'b00000000;
    ram[13442] = 8'b00000000;
    ram[13441] = 8'b00000000;
    ram[13440] = 8'b00000000;
    ram[13439] = 8'b00000000;
    ram[13438] = 8'b00000000;
    ram[13437] = 8'b00000000;
    ram[13436] = 8'b00000000;
    ram[13435] = 8'b00000000;
    ram[13434] = 8'b00000000;
    ram[13433] = 8'b00000000;
    ram[13432] = 8'b00000000;
    ram[13431] = 8'b00000000;
    ram[13430] = 8'b00000000;
    ram[13429] = 8'b00000000;
    ram[13428] = 8'b00000000;
    ram[13427] = 8'b00000000;
    ram[13426] = 8'b00000000;
    ram[13425] = 8'b00000000;
    ram[13424] = 8'b00000000;
    ram[13423] = 8'b00000000;
    ram[13422] = 8'b00000000;
    ram[13421] = 8'b00000000;
    ram[13420] = 8'b00000000;
    ram[13419] = 8'b00000000;
    ram[13418] = 8'b00000000;
    ram[13417] = 8'b00000000;
    ram[13416] = 8'b00000000;
    ram[13415] = 8'b00000000;
    ram[13414] = 8'b00000000;
    ram[13413] = 8'b00000000;
    ram[13412] = 8'b00000000;
    ram[13411] = 8'b00000000;
    ram[13410] = 8'b00000000;
    ram[13409] = 8'b00000000;
    ram[13408] = 8'b00000000;
    ram[13407] = 8'b00000000;
    ram[13406] = 8'b00000000;
    ram[13405] = 8'b00000000;
    ram[13404] = 8'b00000000;
    ram[13403] = 8'b00000000;
    ram[13402] = 8'b00000000;
    ram[13401] = 8'b00000000;
    ram[13400] = 8'b00000000;
    ram[13399] = 8'b00000000;
    ram[13398] = 8'b00000000;
    ram[13397] = 8'b00000000;
    ram[13396] = 8'b00000000;
    ram[13395] = 8'b00000000;
    ram[13394] = 8'b00000000;
    ram[13393] = 8'b00000000;
    ram[13392] = 8'b00000000;
    ram[13391] = 8'b00000000;
    ram[13390] = 8'b00000000;
    ram[13389] = 8'b00000000;
    ram[13388] = 8'b00000000;
    ram[13387] = 8'b00000000;
    ram[13386] = 8'b00000000;
    ram[13385] = 8'b00000000;
    ram[13384] = 8'b00000000;
    ram[13383] = 8'b00000000;
    ram[13382] = 8'b00000000;
    ram[13381] = 8'b00000000;
    ram[13380] = 8'b00000000;
    ram[13379] = 8'b00000000;
    ram[13378] = 8'b00000000;
    ram[13377] = 8'b00000000;
    ram[13376] = 8'b00000000;
    ram[13375] = 8'b00000000;
    ram[13374] = 8'b00000000;
    ram[13373] = 8'b00000000;
    ram[13372] = 8'b00000000;
    ram[13371] = 8'b00000000;
    ram[13370] = 8'b00000000;
    ram[13369] = 8'b00000000;
    ram[13368] = 8'b00000000;
    ram[13367] = 8'b00000000;
    ram[13366] = 8'b00000000;
    ram[13365] = 8'b00000000;
    ram[13364] = 8'b00000000;
    ram[13363] = 8'b00000000;
    ram[13362] = 8'b00000000;
    ram[13361] = 8'b00000000;
    ram[13360] = 8'b00000000;
    ram[13359] = 8'b00000000;
    ram[13358] = 8'b00000000;
    ram[13357] = 8'b00000000;
    ram[13356] = 8'b00000000;
    ram[13355] = 8'b00000000;
    ram[13354] = 8'b00000000;
    ram[13353] = 8'b00000000;
    ram[13352] = 8'b00000000;
    ram[13351] = 8'b00000000;
    ram[13350] = 8'b00000000;
    ram[13349] = 8'b00000000;
    ram[13348] = 8'b00000000;
    ram[13347] = 8'b00000000;
    ram[13346] = 8'b00000000;
    ram[13345] = 8'b00000000;
    ram[13344] = 8'b00000000;
    ram[13343] = 8'b00000000;
    ram[13342] = 8'b00000000;
    ram[13341] = 8'b00000000;
    ram[13340] = 8'b00000000;
    ram[13339] = 8'b00000000;
    ram[13338] = 8'b00000000;
    ram[13337] = 8'b00000000;
    ram[13336] = 8'b00000000;
    ram[13335] = 8'b00000000;
    ram[13334] = 8'b00000000;
    ram[13333] = 8'b00000000;
    ram[13332] = 8'b00000000;
    ram[13331] = 8'b00000000;
    ram[13330] = 8'b00000000;
    ram[13329] = 8'b00000000;
    ram[13328] = 8'b00000000;
    ram[13327] = 8'b00000000;
    ram[13326] = 8'b00000000;
    ram[13325] = 8'b00000000;
    ram[13324] = 8'b00000000;
    ram[13323] = 8'b00000000;
    ram[13322] = 8'b00000000;
    ram[13321] = 8'b00000000;
    ram[13320] = 8'b00000000;
    ram[13319] = 8'b00000000;
    ram[13318] = 8'b00000000;
    ram[13317] = 8'b00000000;
    ram[13316] = 8'b00000000;
    ram[13315] = 8'b00000000;
    ram[13314] = 8'b00000000;
    ram[13313] = 8'b00000000;
    ram[13312] = 8'b00000000;
    ram[13311] = 8'b00000000;
    ram[13310] = 8'b00000000;
    ram[13309] = 8'b00000000;
    ram[13308] = 8'b00000000;
    ram[13307] = 8'b00000000;
    ram[13306] = 8'b00000000;
    ram[13305] = 8'b00000000;
    ram[13304] = 8'b00000000;
    ram[13303] = 8'b00000000;
    ram[13302] = 8'b00000000;
    ram[13301] = 8'b00000000;
    ram[13300] = 8'b00000000;
    ram[13299] = 8'b00000000;
    ram[13298] = 8'b00000000;
    ram[13297] = 8'b00000000;
    ram[13296] = 8'b00000000;
    ram[13295] = 8'b00000000;
    ram[13294] = 8'b00000000;
    ram[13293] = 8'b00000000;
    ram[13292] = 8'b00000000;
    ram[13291] = 8'b00000000;
    ram[13290] = 8'b00000000;
    ram[13289] = 8'b00000000;
    ram[13288] = 8'b00000000;
    ram[13287] = 8'b00000000;
    ram[13286] = 8'b00000000;
    ram[13285] = 8'b00000000;
    ram[13284] = 8'b00000000;
    ram[13283] = 8'b00000000;
    ram[13282] = 8'b00000000;
    ram[13281] = 8'b00000000;
    ram[13280] = 8'b00000000;
    ram[13279] = 8'b00000000;
    ram[13278] = 8'b00000000;
    ram[13277] = 8'b00000000;
    ram[13276] = 8'b00000000;
    ram[13275] = 8'b00000000;
    ram[13274] = 8'b00000000;
    ram[13273] = 8'b00000000;
    ram[13272] = 8'b00000000;
    ram[13271] = 8'b00000000;
    ram[13270] = 8'b00000000;
    ram[13269] = 8'b00000000;
    ram[13268] = 8'b00000000;
    ram[13267] = 8'b00000000;
    ram[13266] = 8'b00000000;
    ram[13265] = 8'b00000000;
    ram[13264] = 8'b00000000;
    ram[13263] = 8'b00000000;
    ram[13262] = 8'b00000000;
    ram[13261] = 8'b00000000;
    ram[13260] = 8'b00000000;
    ram[13259] = 8'b00000000;
    ram[13258] = 8'b00000000;
    ram[13257] = 8'b00000000;
    ram[13256] = 8'b00000000;
    ram[13255] = 8'b00000000;
    ram[13254] = 8'b00000000;
    ram[13253] = 8'b00000000;
    ram[13252] = 8'b00000000;
    ram[13251] = 8'b00000000;
    ram[13250] = 8'b00000000;
    ram[13249] = 8'b00000000;
    ram[13248] = 8'b00000000;
    ram[13247] = 8'b00000000;
    ram[13246] = 8'b00000000;
    ram[13245] = 8'b00000000;
    ram[13244] = 8'b00000000;
    ram[13243] = 8'b00000000;
    ram[13242] = 8'b00000000;
    ram[13241] = 8'b00000000;
    ram[13240] = 8'b00000000;
    ram[13239] = 8'b00000000;
    ram[13238] = 8'b00000000;
    ram[13237] = 8'b00000000;
    ram[13236] = 8'b00000000;
    ram[13235] = 8'b00000000;
    ram[13234] = 8'b00000000;
    ram[13233] = 8'b00000000;
    ram[13232] = 8'b00000000;
    ram[13231] = 8'b00000000;
    ram[13230] = 8'b00000000;
    ram[13229] = 8'b00000000;
    ram[13228] = 8'b00000000;
    ram[13227] = 8'b00000000;
    ram[13226] = 8'b00000000;
    ram[13225] = 8'b00000000;
    ram[13224] = 8'b00000000;
    ram[13223] = 8'b00000000;
    ram[13222] = 8'b00000000;
    ram[13221] = 8'b00000000;
    ram[13220] = 8'b00000000;
    ram[13219] = 8'b00000000;
    ram[13218] = 8'b00000000;
    ram[13217] = 8'b00000000;
    ram[13216] = 8'b00000000;
    ram[13215] = 8'b00000000;
    ram[13214] = 8'b00000000;
    ram[13213] = 8'b00000000;
    ram[13212] = 8'b00000000;
    ram[13211] = 8'b00000000;
    ram[13210] = 8'b00000000;
    ram[13209] = 8'b00000000;
    ram[13208] = 8'b00000000;
    ram[13207] = 8'b00000000;
    ram[13206] = 8'b00000000;
    ram[13205] = 8'b00000000;
    ram[13204] = 8'b00000000;
    ram[13203] = 8'b00000000;
    ram[13202] = 8'b00000000;
    ram[13201] = 8'b00000000;
    ram[13200] = 8'b00000000;
    ram[13199] = 8'b00000000;
    ram[13198] = 8'b00000000;
    ram[13197] = 8'b00000000;
    ram[13196] = 8'b00000000;
    ram[13195] = 8'b00000000;
    ram[13194] = 8'b00000000;
    ram[13193] = 8'b00000000;
    ram[13192] = 8'b00000000;
    ram[13191] = 8'b00000000;
    ram[13190] = 8'b00000000;
    ram[13189] = 8'b00000000;
    ram[13188] = 8'b00000000;
    ram[13187] = 8'b00000000;
    ram[13186] = 8'b00000000;
    ram[13185] = 8'b00000000;
    ram[13184] = 8'b00000000;
    ram[13183] = 8'b00000000;
    ram[13182] = 8'b00000000;
    ram[13181] = 8'b00000000;
    ram[13180] = 8'b00000000;
    ram[13179] = 8'b00000000;
    ram[13178] = 8'b00000000;
    ram[13177] = 8'b00000000;
    ram[13176] = 8'b00000000;
    ram[13175] = 8'b00000000;
    ram[13174] = 8'b00000000;
    ram[13173] = 8'b00000000;
    ram[13172] = 8'b00000000;
    ram[13171] = 8'b00000000;
    ram[13170] = 8'b00000000;
    ram[13169] = 8'b00000000;
    ram[13168] = 8'b00000000;
    ram[13167] = 8'b00000000;
    ram[13166] = 8'b00000000;
    ram[13165] = 8'b00000000;
    ram[13164] = 8'b00000000;
    ram[13163] = 8'b00000000;
    ram[13162] = 8'b00000000;
    ram[13161] = 8'b00000000;
    ram[13160] = 8'b00000000;
    ram[13159] = 8'b00000000;
    ram[13158] = 8'b00000000;
    ram[13157] = 8'b00000000;
    ram[13156] = 8'b00000000;
    ram[13155] = 8'b00000000;
    ram[13154] = 8'b00000000;
    ram[13153] = 8'b00000000;
    ram[13152] = 8'b00000000;
    ram[13151] = 8'b00000000;
    ram[13150] = 8'b00000000;
    ram[13149] = 8'b00000000;
    ram[13148] = 8'b00000000;
    ram[13147] = 8'b00000000;
    ram[13146] = 8'b00000000;
    ram[13145] = 8'b00000000;
    ram[13144] = 8'b00000000;
    ram[13143] = 8'b00000000;
    ram[13142] = 8'b00000000;
    ram[13141] = 8'b00000000;
    ram[13140] = 8'b00000000;
    ram[13139] = 8'b00000000;
    ram[13138] = 8'b00000000;
    ram[13137] = 8'b00000000;
    ram[13136] = 8'b00000000;
    ram[13135] = 8'b00000000;
    ram[13134] = 8'b00000000;
    ram[13133] = 8'b00000000;
    ram[13132] = 8'b00000000;
    ram[13131] = 8'b00000000;
    ram[13130] = 8'b00000000;
    ram[13129] = 8'b00000000;
    ram[13128] = 8'b00000000;
    ram[13127] = 8'b00000000;
    ram[13126] = 8'b00000000;
    ram[13125] = 8'b00000000;
    ram[13124] = 8'b00000000;
    ram[13123] = 8'b00000000;
    ram[13122] = 8'b00000000;
    ram[13121] = 8'b00000000;
    ram[13120] = 8'b00000000;
    ram[13119] = 8'b00000000;
    ram[13118] = 8'b00000000;
    ram[13117] = 8'b00000000;
    ram[13116] = 8'b00000000;
    ram[13115] = 8'b00000000;
    ram[13114] = 8'b00000000;
    ram[13113] = 8'b00000000;
    ram[13112] = 8'b00000000;
    ram[13111] = 8'b00000000;
    ram[13110] = 8'b00000000;
    ram[13109] = 8'b00000000;
    ram[13108] = 8'b00000000;
    ram[13107] = 8'b00000000;
    ram[13106] = 8'b00000000;
    ram[13105] = 8'b00000000;
    ram[13104] = 8'b00000000;
    ram[13103] = 8'b00000000;
    ram[13102] = 8'b00000000;
    ram[13101] = 8'b00000000;
    ram[13100] = 8'b00000000;
    ram[13099] = 8'b00000000;
    ram[13098] = 8'b00000000;
    ram[13097] = 8'b00000000;
    ram[13096] = 8'b00000000;
    ram[13095] = 8'b00000000;
    ram[13094] = 8'b00000000;
    ram[13093] = 8'b00000000;
    ram[13092] = 8'b00000000;
    ram[13091] = 8'b00000000;
    ram[13090] = 8'b00000000;
    ram[13089] = 8'b00000000;
    ram[13088] = 8'b00000000;
    ram[13087] = 8'b00000000;
    ram[13086] = 8'b00000000;
    ram[13085] = 8'b00000000;
    ram[13084] = 8'b00000000;
    ram[13083] = 8'b00000000;
    ram[13082] = 8'b00000000;
    ram[13081] = 8'b00000000;
    ram[13080] = 8'b00000000;
    ram[13079] = 8'b00000000;
    ram[13078] = 8'b00000000;
    ram[13077] = 8'b00000000;
    ram[13076] = 8'b00000000;
    ram[13075] = 8'b00000000;
    ram[13074] = 8'b00000000;
    ram[13073] = 8'b00000000;
    ram[13072] = 8'b00000000;
    ram[13071] = 8'b00000000;
    ram[13070] = 8'b00000000;
    ram[13069] = 8'b00000000;
    ram[13068] = 8'b00000000;
    ram[13067] = 8'b00000000;
    ram[13066] = 8'b00000000;
    ram[13065] = 8'b00000000;
    ram[13064] = 8'b00000000;
    ram[13063] = 8'b00000000;
    ram[13062] = 8'b00000000;
    ram[13061] = 8'b00000000;
    ram[13060] = 8'b00000000;
    ram[13059] = 8'b00000000;
    ram[13058] = 8'b00000000;
    ram[13057] = 8'b00000000;
    ram[13056] = 8'b00000000;
    ram[13055] = 8'b00000000;
    ram[13054] = 8'b00000000;
    ram[13053] = 8'b00000000;
    ram[13052] = 8'b00000000;
    ram[13051] = 8'b00000000;
    ram[13050] = 8'b00000000;
    ram[13049] = 8'b00000000;
    ram[13048] = 8'b00000000;
    ram[13047] = 8'b00000000;
    ram[13046] = 8'b00000000;
    ram[13045] = 8'b00000000;
    ram[13044] = 8'b00000000;
    ram[13043] = 8'b00000000;
    ram[13042] = 8'b00000000;
    ram[13041] = 8'b00000000;
    ram[13040] = 8'b00000000;
    ram[13039] = 8'b00000000;
    ram[13038] = 8'b00000000;
    ram[13037] = 8'b00000000;
    ram[13036] = 8'b00000000;
    ram[13035] = 8'b00000000;
    ram[13034] = 8'b00000000;
    ram[13033] = 8'b00000000;
    ram[13032] = 8'b00000000;
    ram[13031] = 8'b00000000;
    ram[13030] = 8'b00000000;
    ram[13029] = 8'b00000000;
    ram[13028] = 8'b00000000;
    ram[13027] = 8'b00000000;
    ram[13026] = 8'b00000000;
    ram[13025] = 8'b00000000;
    ram[13024] = 8'b00000000;
    ram[13023] = 8'b00000000;
    ram[13022] = 8'b00000000;
    ram[13021] = 8'b00000000;
    ram[13020] = 8'b00000000;
    ram[13019] = 8'b00000000;
    ram[13018] = 8'b00000000;
    ram[13017] = 8'b00000000;
    ram[13016] = 8'b00000000;
    ram[13015] = 8'b00000000;
    ram[13014] = 8'b00000000;
    ram[13013] = 8'b00000000;
    ram[13012] = 8'b00000000;
    ram[13011] = 8'b00000000;
    ram[13010] = 8'b00000000;
    ram[13009] = 8'b00000000;
    ram[13008] = 8'b00000000;
    ram[13007] = 8'b00000000;
    ram[13006] = 8'b00000000;
    ram[13005] = 8'b00000000;
    ram[13004] = 8'b00000000;
    ram[13003] = 8'b00000000;
    ram[13002] = 8'b00000000;
    ram[13001] = 8'b00000000;
    ram[13000] = 8'b00000000;
    ram[12999] = 8'b00000000;
    ram[12998] = 8'b00000000;
    ram[12997] = 8'b00000000;
    ram[12996] = 8'b00000000;
    ram[12995] = 8'b00000000;
    ram[12994] = 8'b00000000;
    ram[12993] = 8'b00000000;
    ram[12992] = 8'b00000000;
    ram[12991] = 8'b00000000;
    ram[12990] = 8'b00000000;
    ram[12989] = 8'b00000000;
    ram[12988] = 8'b00000000;
    ram[12987] = 8'b00000000;
    ram[12986] = 8'b00000000;
    ram[12985] = 8'b00000000;
    ram[12984] = 8'b00000000;
    ram[12983] = 8'b00000000;
    ram[12982] = 8'b00000000;
    ram[12981] = 8'b00000000;
    ram[12980] = 8'b00000000;
    ram[12979] = 8'b00000000;
    ram[12978] = 8'b00000000;
    ram[12977] = 8'b00000000;
    ram[12976] = 8'b00000000;
    ram[12975] = 8'b00000000;
    ram[12974] = 8'b00000000;
    ram[12973] = 8'b00000000;
    ram[12972] = 8'b00000000;
    ram[12971] = 8'b00000000;
    ram[12970] = 8'b00000000;
    ram[12969] = 8'b00000000;
    ram[12968] = 8'b00000000;
    ram[12967] = 8'b00000000;
    ram[12966] = 8'b00000000;
    ram[12965] = 8'b00000000;
    ram[12964] = 8'b00000000;
    ram[12963] = 8'b00000000;
    ram[12962] = 8'b00000000;
    ram[12961] = 8'b00000000;
    ram[12960] = 8'b00000000;
    ram[12959] = 8'b00000000;
    ram[12958] = 8'b00000000;
    ram[12957] = 8'b00000000;
    ram[12956] = 8'b00000000;
    ram[12955] = 8'b00000000;
    ram[12954] = 8'b00000000;
    ram[12953] = 8'b00000000;
    ram[12952] = 8'b00000000;
    ram[12951] = 8'b00000000;
    ram[12950] = 8'b00000000;
    ram[12949] = 8'b00000000;
    ram[12948] = 8'b00000000;
    ram[12947] = 8'b00000000;
    ram[12946] = 8'b00000000;
    ram[12945] = 8'b00000000;
    ram[12944] = 8'b00000000;
    ram[12943] = 8'b00000000;
    ram[12942] = 8'b00000000;
    ram[12941] = 8'b00000000;
    ram[12940] = 8'b00000000;
    ram[12939] = 8'b00000000;
    ram[12938] = 8'b00000000;
    ram[12937] = 8'b00000000;
    ram[12936] = 8'b00000000;
    ram[12935] = 8'b00000000;
    ram[12934] = 8'b00000000;
    ram[12933] = 8'b00000000;
    ram[12932] = 8'b00000000;
    ram[12931] = 8'b00000000;
    ram[12930] = 8'b00000000;
    ram[12929] = 8'b00000000;
    ram[12928] = 8'b00000000;
    ram[12927] = 8'b00000000;
    ram[12926] = 8'b00000000;
    ram[12925] = 8'b00000000;
    ram[12924] = 8'b00000000;
    ram[12923] = 8'b00000000;
    ram[12922] = 8'b00000000;
    ram[12921] = 8'b00000000;
    ram[12920] = 8'b00000000;
    ram[12919] = 8'b00000000;
    ram[12918] = 8'b00000000;
    ram[12917] = 8'b00000000;
    ram[12916] = 8'b00000000;
    ram[12915] = 8'b00000000;
    ram[12914] = 8'b00000000;
    ram[12913] = 8'b00000000;
    ram[12912] = 8'b00000000;
    ram[12911] = 8'b00000000;
    ram[12910] = 8'b00000000;
    ram[12909] = 8'b00000000;
    ram[12908] = 8'b00000000;
    ram[12907] = 8'b00000000;
    ram[12906] = 8'b00000000;
    ram[12905] = 8'b00000000;
    ram[12904] = 8'b00000000;
    ram[12903] = 8'b00000000;
    ram[12902] = 8'b00000000;
    ram[12901] = 8'b00000000;
    ram[12900] = 8'b00000000;
    ram[12899] = 8'b00000000;
    ram[12898] = 8'b00000000;
    ram[12897] = 8'b00000000;
    ram[12896] = 8'b00000000;
    ram[12895] = 8'b00000000;
    ram[12894] = 8'b00000000;
    ram[12893] = 8'b00000000;
    ram[12892] = 8'b00000000;
    ram[12891] = 8'b00000000;
    ram[12890] = 8'b00000000;
    ram[12889] = 8'b00000000;
    ram[12888] = 8'b00000000;
    ram[12887] = 8'b00000000;
    ram[12886] = 8'b00000000;
    ram[12885] = 8'b00000000;
    ram[12884] = 8'b00000000;
    ram[12883] = 8'b00000000;
    ram[12882] = 8'b00000000;
    ram[12881] = 8'b00000000;
    ram[12880] = 8'b00000000;
    ram[12879] = 8'b00000000;
    ram[12878] = 8'b00000000;
    ram[12877] = 8'b00000000;
    ram[12876] = 8'b00000000;
    ram[12875] = 8'b00000000;
    ram[12874] = 8'b00000000;
    ram[12873] = 8'b00000000;
    ram[12872] = 8'b00000000;
    ram[12871] = 8'b00000000;
    ram[12870] = 8'b00000000;
    ram[12869] = 8'b00000000;
    ram[12868] = 8'b00000000;
    ram[12867] = 8'b00000000;
    ram[12866] = 8'b00000000;
    ram[12865] = 8'b00000000;
    ram[12864] = 8'b00000000;
    ram[12863] = 8'b00000000;
    ram[12862] = 8'b00000000;
    ram[12861] = 8'b00000000;
    ram[12860] = 8'b00000000;
    ram[12859] = 8'b00000000;
    ram[12858] = 8'b00000000;
    ram[12857] = 8'b00000000;
    ram[12856] = 8'b00000000;
    ram[12855] = 8'b00000000;
    ram[12854] = 8'b00000000;
    ram[12853] = 8'b00000000;
    ram[12852] = 8'b00000000;
    ram[12851] = 8'b00000000;
    ram[12850] = 8'b00000000;
    ram[12849] = 8'b00000000;
    ram[12848] = 8'b00000000;
    ram[12847] = 8'b00000000;
    ram[12846] = 8'b00000000;
    ram[12845] = 8'b00000000;
    ram[12844] = 8'b00000000;
    ram[12843] = 8'b00000000;
    ram[12842] = 8'b00000000;
    ram[12841] = 8'b00000000;
    ram[12840] = 8'b00000000;
    ram[12839] = 8'b00000000;
    ram[12838] = 8'b00000000;
    ram[12837] = 8'b00000000;
    ram[12836] = 8'b00000000;
    ram[12835] = 8'b00000000;
    ram[12834] = 8'b00000000;
    ram[12833] = 8'b00000000;
    ram[12832] = 8'b00000000;
    ram[12831] = 8'b00000000;
    ram[12830] = 8'b00000000;
    ram[12829] = 8'b00000000;
    ram[12828] = 8'b00000000;
    ram[12827] = 8'b00000000;
    ram[12826] = 8'b00000000;
    ram[12825] = 8'b00000000;
    ram[12824] = 8'b00000000;
    ram[12823] = 8'b00000000;
    ram[12822] = 8'b00000000;
    ram[12821] = 8'b00000000;
    ram[12820] = 8'b00000000;
    ram[12819] = 8'b00000000;
    ram[12818] = 8'b00000000;
    ram[12817] = 8'b00000000;
    ram[12816] = 8'b00000000;
    ram[12815] = 8'b00000000;
    ram[12814] = 8'b00000000;
    ram[12813] = 8'b00000000;
    ram[12812] = 8'b00000000;
    ram[12811] = 8'b00000000;
    ram[12810] = 8'b00000000;
    ram[12809] = 8'b00000000;
    ram[12808] = 8'b00000000;
    ram[12807] = 8'b00000000;
    ram[12806] = 8'b00000000;
    ram[12805] = 8'b00000000;
    ram[12804] = 8'b00000000;
    ram[12803] = 8'b00000000;
    ram[12802] = 8'b00000000;
    ram[12801] = 8'b00000000;
    ram[12800] = 8'b00000000;
    ram[12799] = 8'b00000000;
    ram[12798] = 8'b00000000;
    ram[12797] = 8'b00000000;
    ram[12796] = 8'b00000000;
    ram[12795] = 8'b00000000;
    ram[12794] = 8'b00000000;
    ram[12793] = 8'b00000000;
    ram[12792] = 8'b00000000;
    ram[12791] = 8'b00000000;
    ram[12790] = 8'b00000000;
    ram[12789] = 8'b00000000;
    ram[12788] = 8'b00000000;
    ram[12787] = 8'b00000000;
    ram[12786] = 8'b00000000;
    ram[12785] = 8'b00000000;
    ram[12784] = 8'b00000000;
    ram[12783] = 8'b00000000;
    ram[12782] = 8'b00000000;
    ram[12781] = 8'b00000000;
    ram[12780] = 8'b00000000;
    ram[12779] = 8'b00000000;
    ram[12778] = 8'b00000000;
    ram[12777] = 8'b00000000;
    ram[12776] = 8'b00000000;
    ram[12775] = 8'b00000000;
    ram[12774] = 8'b00000000;
    ram[12773] = 8'b00000000;
    ram[12772] = 8'b00000000;
    ram[12771] = 8'b00000000;
    ram[12770] = 8'b00000000;
    ram[12769] = 8'b00000000;
    ram[12768] = 8'b00000000;
    ram[12767] = 8'b00000000;
    ram[12766] = 8'b00000000;
    ram[12765] = 8'b00000000;
    ram[12764] = 8'b00000000;
    ram[12763] = 8'b00000000;
    ram[12762] = 8'b00000000;
    ram[12761] = 8'b00000000;
    ram[12760] = 8'b00000000;
    ram[12759] = 8'b00000000;
    ram[12758] = 8'b00000000;
    ram[12757] = 8'b00000000;
    ram[12756] = 8'b00000000;
    ram[12755] = 8'b00000000;
    ram[12754] = 8'b00000000;
    ram[12753] = 8'b00000000;
    ram[12752] = 8'b00000000;
    ram[12751] = 8'b00000000;
    ram[12750] = 8'b00000000;
    ram[12749] = 8'b00000000;
    ram[12748] = 8'b00000000;
    ram[12747] = 8'b00000000;
    ram[12746] = 8'b00000000;
    ram[12745] = 8'b00000000;
    ram[12744] = 8'b00000000;
    ram[12743] = 8'b00000000;
    ram[12742] = 8'b00000000;
    ram[12741] = 8'b00000000;
    ram[12740] = 8'b00000000;
    ram[12739] = 8'b00000000;
    ram[12738] = 8'b00000000;
    ram[12737] = 8'b00000000;
    ram[12736] = 8'b00000000;
    ram[12735] = 8'b00000000;
    ram[12734] = 8'b00000000;
    ram[12733] = 8'b00000000;
    ram[12732] = 8'b00000000;
    ram[12731] = 8'b00000000;
    ram[12730] = 8'b00000000;
    ram[12729] = 8'b00000000;
    ram[12728] = 8'b00000000;
    ram[12727] = 8'b00000000;
    ram[12726] = 8'b00000000;
    ram[12725] = 8'b00000000;
    ram[12724] = 8'b00000000;
    ram[12723] = 8'b00000000;
    ram[12722] = 8'b00000000;
    ram[12721] = 8'b00000000;
    ram[12720] = 8'b00000000;
    ram[12719] = 8'b00000000;
    ram[12718] = 8'b00000000;
    ram[12717] = 8'b00000000;
    ram[12716] = 8'b00000000;
    ram[12715] = 8'b00000000;
    ram[12714] = 8'b00000000;
    ram[12713] = 8'b00000000;
    ram[12712] = 8'b00000000;
    ram[12711] = 8'b00000000;
    ram[12710] = 8'b00000000;
    ram[12709] = 8'b00000000;
    ram[12708] = 8'b00000000;
    ram[12707] = 8'b00000000;
    ram[12706] = 8'b00000000;
    ram[12705] = 8'b00000000;
    ram[12704] = 8'b00000000;
    ram[12703] = 8'b00000000;
    ram[12702] = 8'b00000000;
    ram[12701] = 8'b00000000;
    ram[12700] = 8'b00000000;
    ram[12699] = 8'b00000000;
    ram[12698] = 8'b00000000;
    ram[12697] = 8'b00000000;
    ram[12696] = 8'b00000000;
    ram[12695] = 8'b00000000;
    ram[12694] = 8'b00000000;
    ram[12693] = 8'b00000000;
    ram[12692] = 8'b00000000;
    ram[12691] = 8'b00000000;
    ram[12690] = 8'b00000000;
    ram[12689] = 8'b00000000;
    ram[12688] = 8'b00000000;
    ram[12687] = 8'b00000000;
    ram[12686] = 8'b00000000;
    ram[12685] = 8'b00000000;
    ram[12684] = 8'b00000000;
    ram[12683] = 8'b00000000;
    ram[12682] = 8'b00000000;
    ram[12681] = 8'b00000000;
    ram[12680] = 8'b00000000;
    ram[12679] = 8'b00000000;
    ram[12678] = 8'b00000000;
    ram[12677] = 8'b00000000;
    ram[12676] = 8'b00000000;
    ram[12675] = 8'b00000000;
    ram[12674] = 8'b00000000;
    ram[12673] = 8'b00000000;
    ram[12672] = 8'b00000000;
    ram[12671] = 8'b00000000;
    ram[12670] = 8'b00000000;
    ram[12669] = 8'b00000000;
    ram[12668] = 8'b00000000;
    ram[12667] = 8'b00000000;
    ram[12666] = 8'b00000000;
    ram[12665] = 8'b00000000;
    ram[12664] = 8'b00000000;
    ram[12663] = 8'b00000000;
    ram[12662] = 8'b00000000;
    ram[12661] = 8'b00000000;
    ram[12660] = 8'b00000000;
    ram[12659] = 8'b00000000;
    ram[12658] = 8'b00000000;
    ram[12657] = 8'b00000000;
    ram[12656] = 8'b00000000;
    ram[12655] = 8'b00000000;
    ram[12654] = 8'b00000000;
    ram[12653] = 8'b00000000;
    ram[12652] = 8'b00000000;
    ram[12651] = 8'b00000000;
    ram[12650] = 8'b00000000;
    ram[12649] = 8'b00000000;
    ram[12648] = 8'b00000000;
    ram[12647] = 8'b00000000;
    ram[12646] = 8'b00000000;
    ram[12645] = 8'b00000000;
    ram[12644] = 8'b00000000;
    ram[12643] = 8'b00000000;
    ram[12642] = 8'b00000000;
    ram[12641] = 8'b00000000;
    ram[12640] = 8'b00000000;
    ram[12639] = 8'b00000000;
    ram[12638] = 8'b00000000;
    ram[12637] = 8'b00000000;
    ram[12636] = 8'b00000000;
    ram[12635] = 8'b00000000;
    ram[12634] = 8'b00000000;
    ram[12633] = 8'b00000000;
    ram[12632] = 8'b00000000;
    ram[12631] = 8'b00000000;
    ram[12630] = 8'b00000000;
    ram[12629] = 8'b00000000;
    ram[12628] = 8'b00000000;
    ram[12627] = 8'b00000000;
    ram[12626] = 8'b00000000;
    ram[12625] = 8'b00000000;
    ram[12624] = 8'b00000000;
    ram[12623] = 8'b00000000;
    ram[12622] = 8'b00000000;
    ram[12621] = 8'b00000000;
    ram[12620] = 8'b00000000;
    ram[12619] = 8'b00000000;
    ram[12618] = 8'b00000000;
    ram[12617] = 8'b00000000;
    ram[12616] = 8'b00000000;
    ram[12615] = 8'b00000000;
    ram[12614] = 8'b00000000;
    ram[12613] = 8'b00000000;
    ram[12612] = 8'b00000000;
    ram[12611] = 8'b00000000;
    ram[12610] = 8'b00000000;
    ram[12609] = 8'b00000000;
    ram[12608] = 8'b00000000;
    ram[12607] = 8'b00000000;
    ram[12606] = 8'b00000000;
    ram[12605] = 8'b00000000;
    ram[12604] = 8'b00000000;
    ram[12603] = 8'b00000000;
    ram[12602] = 8'b00000000;
    ram[12601] = 8'b00000000;
    ram[12600] = 8'b00000000;
    ram[12599] = 8'b00000000;
    ram[12598] = 8'b00000000;
    ram[12597] = 8'b00000000;
    ram[12596] = 8'b00000000;
    ram[12595] = 8'b00000000;
    ram[12594] = 8'b00000000;
    ram[12593] = 8'b00000000;
    ram[12592] = 8'b00000000;
    ram[12591] = 8'b00000000;
    ram[12590] = 8'b00000000;
    ram[12589] = 8'b00000000;
    ram[12588] = 8'b00000000;
    ram[12587] = 8'b00000000;
    ram[12586] = 8'b00000000;
    ram[12585] = 8'b00000000;
    ram[12584] = 8'b00000000;
    ram[12583] = 8'b00000000;
    ram[12582] = 8'b00000000;
    ram[12581] = 8'b00000000;
    ram[12580] = 8'b00000000;
    ram[12579] = 8'b00000000;
    ram[12578] = 8'b00000000;
    ram[12577] = 8'b00000000;
    ram[12576] = 8'b00000000;
    ram[12575] = 8'b00000000;
    ram[12574] = 8'b00000000;
    ram[12573] = 8'b00000000;
    ram[12572] = 8'b00000000;
    ram[12571] = 8'b00000000;
    ram[12570] = 8'b00000000;
    ram[12569] = 8'b00000000;
    ram[12568] = 8'b00000000;
    ram[12567] = 8'b00000000;
    ram[12566] = 8'b00000000;
    ram[12565] = 8'b00000000;
    ram[12564] = 8'b00000000;
    ram[12563] = 8'b00000000;
    ram[12562] = 8'b00000000;
    ram[12561] = 8'b00000000;
    ram[12560] = 8'b00000000;
    ram[12559] = 8'b00000000;
    ram[12558] = 8'b00000000;
    ram[12557] = 8'b00000000;
    ram[12556] = 8'b00000000;
    ram[12555] = 8'b00000000;
    ram[12554] = 8'b00000000;
    ram[12553] = 8'b00000000;
    ram[12552] = 8'b00000000;
    ram[12551] = 8'b00000000;
    ram[12550] = 8'b00000000;
    ram[12549] = 8'b00000000;
    ram[12548] = 8'b00000000;
    ram[12547] = 8'b00000000;
    ram[12546] = 8'b00000000;
    ram[12545] = 8'b00000000;
    ram[12544] = 8'b00000000;
    ram[12543] = 8'b00000000;
    ram[12542] = 8'b00000000;
    ram[12541] = 8'b00000000;
    ram[12540] = 8'b00000000;
    ram[12539] = 8'b00000000;
    ram[12538] = 8'b00000000;
    ram[12537] = 8'b00000000;
    ram[12536] = 8'b00000000;
    ram[12535] = 8'b00000000;
    ram[12534] = 8'b00000000;
    ram[12533] = 8'b00000000;
    ram[12532] = 8'b00000000;
    ram[12531] = 8'b00000000;
    ram[12530] = 8'b00000000;
    ram[12529] = 8'b00000000;
    ram[12528] = 8'b00000000;
    ram[12527] = 8'b00000000;
    ram[12526] = 8'b00000000;
    ram[12525] = 8'b00000000;
    ram[12524] = 8'b00000000;
    ram[12523] = 8'b00000000;
    ram[12522] = 8'b00000000;
    ram[12521] = 8'b00000000;
    ram[12520] = 8'b00000000;
    ram[12519] = 8'b00000000;
    ram[12518] = 8'b00000000;
    ram[12517] = 8'b00000000;
    ram[12516] = 8'b00000000;
    ram[12515] = 8'b00000000;
    ram[12514] = 8'b00000000;
    ram[12513] = 8'b00000000;
    ram[12512] = 8'b00000000;
    ram[12511] = 8'b00000000;
    ram[12510] = 8'b00000000;
    ram[12509] = 8'b00000000;
    ram[12508] = 8'b00000000;
    ram[12507] = 8'b00000000;
    ram[12506] = 8'b00000000;
    ram[12505] = 8'b00000000;
    ram[12504] = 8'b00000000;
    ram[12503] = 8'b00000000;
    ram[12502] = 8'b00000000;
    ram[12501] = 8'b00000000;
    ram[12500] = 8'b00000000;
    ram[12499] = 8'b00000000;
    ram[12498] = 8'b00000000;
    ram[12497] = 8'b00000000;
    ram[12496] = 8'b00000000;
    ram[12495] = 8'b00000000;
    ram[12494] = 8'b00000000;
    ram[12493] = 8'b00000000;
    ram[12492] = 8'b00000000;
    ram[12491] = 8'b00000000;
    ram[12490] = 8'b00000000;
    ram[12489] = 8'b00000000;
    ram[12488] = 8'b00000000;
    ram[12487] = 8'b00000000;
    ram[12486] = 8'b00000000;
    ram[12485] = 8'b00000000;
    ram[12484] = 8'b00000000;
    ram[12483] = 8'b00000000;
    ram[12482] = 8'b00000000;
    ram[12481] = 8'b00000000;
    ram[12480] = 8'b00000000;
    ram[12479] = 8'b00000000;
    ram[12478] = 8'b00000000;
    ram[12477] = 8'b00000000;
    ram[12476] = 8'b00000000;
    ram[12475] = 8'b00000000;
    ram[12474] = 8'b00000000;
    ram[12473] = 8'b00000000;
    ram[12472] = 8'b00000000;
    ram[12471] = 8'b00000000;
    ram[12470] = 8'b00000000;
    ram[12469] = 8'b00000000;
    ram[12468] = 8'b00000000;
    ram[12467] = 8'b00000000;
    ram[12466] = 8'b00000000;
    ram[12465] = 8'b00000000;
    ram[12464] = 8'b00000000;
    ram[12463] = 8'b00000000;
    ram[12462] = 8'b00000000;
    ram[12461] = 8'b00000000;
    ram[12460] = 8'b00000000;
    ram[12459] = 8'b00000000;
    ram[12458] = 8'b00000000;
    ram[12457] = 8'b00000000;
    ram[12456] = 8'b00000000;
    ram[12455] = 8'b00000000;
    ram[12454] = 8'b00000000;
    ram[12453] = 8'b00000000;
    ram[12452] = 8'b00000000;
    ram[12451] = 8'b00000000;
    ram[12450] = 8'b00000000;
    ram[12449] = 8'b00000000;
    ram[12448] = 8'b00000000;
    ram[12447] = 8'b00000000;
    ram[12446] = 8'b00000000;
    ram[12445] = 8'b00000000;
    ram[12444] = 8'b00000000;
    ram[12443] = 8'b00000000;
    ram[12442] = 8'b00000000;
    ram[12441] = 8'b00000000;
    ram[12440] = 8'b00000000;
    ram[12439] = 8'b00000000;
    ram[12438] = 8'b00000000;
    ram[12437] = 8'b00000000;
    ram[12436] = 8'b00000000;
    ram[12435] = 8'b00000000;
    ram[12434] = 8'b00000000;
    ram[12433] = 8'b00000000;
    ram[12432] = 8'b00000000;
    ram[12431] = 8'b00000000;
    ram[12430] = 8'b00000000;
    ram[12429] = 8'b00000000;
    ram[12428] = 8'b00000000;
    ram[12427] = 8'b00000000;
    ram[12426] = 8'b00000000;
    ram[12425] = 8'b00000000;
    ram[12424] = 8'b00000000;
    ram[12423] = 8'b00000000;
    ram[12422] = 8'b00000000;
    ram[12421] = 8'b00000000;
    ram[12420] = 8'b00000000;
    ram[12419] = 8'b00000000;
    ram[12418] = 8'b00000000;
    ram[12417] = 8'b00000000;
    ram[12416] = 8'b00000000;
    ram[12415] = 8'b00000000;
    ram[12414] = 8'b00000000;
    ram[12413] = 8'b00000000;
    ram[12412] = 8'b00000000;
    ram[12411] = 8'b00000000;
    ram[12410] = 8'b00000000;
    ram[12409] = 8'b00000000;
    ram[12408] = 8'b00000000;
    ram[12407] = 8'b00000000;
    ram[12406] = 8'b00000000;
    ram[12405] = 8'b00000000;
    ram[12404] = 8'b00000000;
    ram[12403] = 8'b00000000;
    ram[12402] = 8'b00000000;
    ram[12401] = 8'b00000000;
    ram[12400] = 8'b00000000;
    ram[12399] = 8'b00000000;
    ram[12398] = 8'b00000000;
    ram[12397] = 8'b00000000;
    ram[12396] = 8'b00000000;
    ram[12395] = 8'b00000000;
    ram[12394] = 8'b00000000;
    ram[12393] = 8'b00000000;
    ram[12392] = 8'b00000000;
    ram[12391] = 8'b00000000;
    ram[12390] = 8'b00000000;
    ram[12389] = 8'b00000000;
    ram[12388] = 8'b00000000;
    ram[12387] = 8'b00000000;
    ram[12386] = 8'b00000000;
    ram[12385] = 8'b00000000;
    ram[12384] = 8'b00000000;
    ram[12383] = 8'b00000000;
    ram[12382] = 8'b00000000;
    ram[12381] = 8'b00000000;
    ram[12380] = 8'b00000000;
    ram[12379] = 8'b00000000;
    ram[12378] = 8'b00000000;
    ram[12377] = 8'b00000000;
    ram[12376] = 8'b00000000;
    ram[12375] = 8'b00000000;
    ram[12374] = 8'b00000000;
    ram[12373] = 8'b00000000;
    ram[12372] = 8'b00000000;
    ram[12371] = 8'b00000000;
    ram[12370] = 8'b00000000;
    ram[12369] = 8'b00000000;
    ram[12368] = 8'b00000000;
    ram[12367] = 8'b00000000;
    ram[12366] = 8'b00000000;
    ram[12365] = 8'b00000000;
    ram[12364] = 8'b00000000;
    ram[12363] = 8'b00000000;
    ram[12362] = 8'b00000000;
    ram[12361] = 8'b00000000;
    ram[12360] = 8'b00000000;
    ram[12359] = 8'b00000000;
    ram[12358] = 8'b00000000;
    ram[12357] = 8'b00000000;
    ram[12356] = 8'b00000000;
    ram[12355] = 8'b00000000;
    ram[12354] = 8'b00000000;
    ram[12353] = 8'b00000000;
    ram[12352] = 8'b00000000;
    ram[12351] = 8'b00000000;
    ram[12350] = 8'b00000000;
    ram[12349] = 8'b00000000;
    ram[12348] = 8'b00000000;
    ram[12347] = 8'b00000000;
    ram[12346] = 8'b00000000;
    ram[12345] = 8'b00000000;
    ram[12344] = 8'b00000000;
    ram[12343] = 8'b00000000;
    ram[12342] = 8'b00000000;
    ram[12341] = 8'b00000000;
    ram[12340] = 8'b00000000;
    ram[12339] = 8'b00000000;
    ram[12338] = 8'b00000000;
    ram[12337] = 8'b00000000;
    ram[12336] = 8'b00000000;
    ram[12335] = 8'b00000000;
    ram[12334] = 8'b00000000;
    ram[12333] = 8'b00000000;
    ram[12332] = 8'b00000000;
    ram[12331] = 8'b00000000;
    ram[12330] = 8'b00000000;
    ram[12329] = 8'b00000000;
    ram[12328] = 8'b00000000;
    ram[12327] = 8'b00000000;
    ram[12326] = 8'b00000000;
    ram[12325] = 8'b00000000;
    ram[12324] = 8'b00000000;
    ram[12323] = 8'b00000000;
    ram[12322] = 8'b00000000;
    ram[12321] = 8'b00000000;
    ram[12320] = 8'b00000000;
    ram[12319] = 8'b00000000;
    ram[12318] = 8'b00000000;
    ram[12317] = 8'b00000000;
    ram[12316] = 8'b00000000;
    ram[12315] = 8'b00000000;
    ram[12314] = 8'b00000000;
    ram[12313] = 8'b00000000;
    ram[12312] = 8'b00000000;
    ram[12311] = 8'b00000000;
    ram[12310] = 8'b00000000;
    ram[12309] = 8'b00000000;
    ram[12308] = 8'b00000000;
    ram[12307] = 8'b00000000;
    ram[12306] = 8'b00000000;
    ram[12305] = 8'b00000000;
    ram[12304] = 8'b00000000;
    ram[12303] = 8'b00000000;
    ram[12302] = 8'b00000000;
    ram[12301] = 8'b00000000;
    ram[12300] = 8'b00000000;
    ram[12299] = 8'b00000000;
    ram[12298] = 8'b00000000;
    ram[12297] = 8'b00000000;
    ram[12296] = 8'b00000000;
    ram[12295] = 8'b00000000;
    ram[12294] = 8'b00000000;
    ram[12293] = 8'b00000000;
    ram[12292] = 8'b00000000;
    ram[12291] = 8'b00000000;
    ram[12290] = 8'b00000000;
    ram[12289] = 8'b00000000;
    ram[12288] = 8'b00000000;
    ram[12287] = 8'b00000000;
    ram[12286] = 8'b00000000;
    ram[12285] = 8'b00000000;
    ram[12284] = 8'b00000000;
    ram[12283] = 8'b00000000;
    ram[12282] = 8'b00000000;
    ram[12281] = 8'b00000000;
    ram[12280] = 8'b00000000;
    ram[12279] = 8'b00000000;
    ram[12278] = 8'b00000000;
    ram[12277] = 8'b00000000;
    ram[12276] = 8'b00000000;
    ram[12275] = 8'b00000000;
    ram[12274] = 8'b00000000;
    ram[12273] = 8'b00000000;
    ram[12272] = 8'b00000000;
    ram[12271] = 8'b00000000;
    ram[12270] = 8'b00000000;
    ram[12269] = 8'b00000000;
    ram[12268] = 8'b00000000;
    ram[12267] = 8'b00000000;
    ram[12266] = 8'b00000000;
    ram[12265] = 8'b00000000;
    ram[12264] = 8'b00000000;
    ram[12263] = 8'b00000000;
    ram[12262] = 8'b00000000;
    ram[12261] = 8'b00000000;
    ram[12260] = 8'b00000000;
    ram[12259] = 8'b00000000;
    ram[12258] = 8'b00000000;
    ram[12257] = 8'b00000000;
    ram[12256] = 8'b00000000;
    ram[12255] = 8'b00000000;
    ram[12254] = 8'b00000000;
    ram[12253] = 8'b00000000;
    ram[12252] = 8'b00000000;
    ram[12251] = 8'b00000000;
    ram[12250] = 8'b00000000;
    ram[12249] = 8'b00000000;
    ram[12248] = 8'b00000000;
    ram[12247] = 8'b00000000;
    ram[12246] = 8'b00000000;
    ram[12245] = 8'b00000000;
    ram[12244] = 8'b00000000;
    ram[12243] = 8'b00000000;
    ram[12242] = 8'b00000000;
    ram[12241] = 8'b00000000;
    ram[12240] = 8'b00000000;
    ram[12239] = 8'b00000000;
    ram[12238] = 8'b00000000;
    ram[12237] = 8'b00000000;
    ram[12236] = 8'b00000000;
    ram[12235] = 8'b00000000;
    ram[12234] = 8'b00000000;
    ram[12233] = 8'b00000000;
    ram[12232] = 8'b00000000;
    ram[12231] = 8'b00000000;
    ram[12230] = 8'b00000000;
    ram[12229] = 8'b00000000;
    ram[12228] = 8'b00000000;
    ram[12227] = 8'b00000000;
    ram[12226] = 8'b00000000;
    ram[12225] = 8'b00000000;
    ram[12224] = 8'b00000000;
    ram[12223] = 8'b00000000;
    ram[12222] = 8'b00000000;
    ram[12221] = 8'b00000000;
    ram[12220] = 8'b00000000;
    ram[12219] = 8'b00000000;
    ram[12218] = 8'b00000000;
    ram[12217] = 8'b00000000;
    ram[12216] = 8'b00000000;
    ram[12215] = 8'b00000000;
    ram[12214] = 8'b00000000;
    ram[12213] = 8'b00000000;
    ram[12212] = 8'b00000000;
    ram[12211] = 8'b00000000;
    ram[12210] = 8'b00000000;
    ram[12209] = 8'b00000000;
    ram[12208] = 8'b00000000;
    ram[12207] = 8'b00000000;
    ram[12206] = 8'b00000000;
    ram[12205] = 8'b00000000;
    ram[12204] = 8'b00000000;
    ram[12203] = 8'b00000000;
    ram[12202] = 8'b00000000;
    ram[12201] = 8'b00000000;
    ram[12200] = 8'b00000000;
    ram[12199] = 8'b00000000;
    ram[12198] = 8'b00000000;
    ram[12197] = 8'b00000000;
    ram[12196] = 8'b00000000;
    ram[12195] = 8'b00000000;
    ram[12194] = 8'b00000000;
    ram[12193] = 8'b00000000;
    ram[12192] = 8'b00000000;
    ram[12191] = 8'b00000000;
    ram[12190] = 8'b00000000;
    ram[12189] = 8'b00000000;
    ram[12188] = 8'b00000000;
    ram[12187] = 8'b00000000;
    ram[12186] = 8'b00000000;
    ram[12185] = 8'b00000000;
    ram[12184] = 8'b00000000;
    ram[12183] = 8'b00000000;
    ram[12182] = 8'b00000000;
    ram[12181] = 8'b00000000;
    ram[12180] = 8'b00000000;
    ram[12179] = 8'b00000000;
    ram[12178] = 8'b00000000;
    ram[12177] = 8'b00000000;
    ram[12176] = 8'b00000000;
    ram[12175] = 8'b00000000;
    ram[12174] = 8'b00000000;
    ram[12173] = 8'b00000000;
    ram[12172] = 8'b00000000;
    ram[12171] = 8'b00000000;
    ram[12170] = 8'b00000000;
    ram[12169] = 8'b00000000;
    ram[12168] = 8'b00000000;
    ram[12167] = 8'b00000000;
    ram[12166] = 8'b00000000;
    ram[12165] = 8'b00000000;
    ram[12164] = 8'b00000000;
    ram[12163] = 8'b00000000;
    ram[12162] = 8'b00000000;
    ram[12161] = 8'b00000000;
    ram[12160] = 8'b00000000;
    ram[12159] = 8'b00000000;
    ram[12158] = 8'b00000000;
    ram[12157] = 8'b00000000;
    ram[12156] = 8'b00000000;
    ram[12155] = 8'b00000000;
    ram[12154] = 8'b00000000;
    ram[12153] = 8'b00000000;
    ram[12152] = 8'b00000000;
    ram[12151] = 8'b00000000;
    ram[12150] = 8'b00000000;
    ram[12149] = 8'b00000000;
    ram[12148] = 8'b00000000;
    ram[12147] = 8'b00000000;
    ram[12146] = 8'b00000000;
    ram[12145] = 8'b00000000;
    ram[12144] = 8'b00000000;
    ram[12143] = 8'b00000000;
    ram[12142] = 8'b00000000;
    ram[12141] = 8'b00000000;
    ram[12140] = 8'b00000000;
    ram[12139] = 8'b00000000;
    ram[12138] = 8'b00000000;
    ram[12137] = 8'b00000000;
    ram[12136] = 8'b00000000;
    ram[12135] = 8'b00000000;
    ram[12134] = 8'b00000000;
    ram[12133] = 8'b00000000;
    ram[12132] = 8'b00000000;
    ram[12131] = 8'b00000000;
    ram[12130] = 8'b00000000;
    ram[12129] = 8'b00000000;
    ram[12128] = 8'b00000000;
    ram[12127] = 8'b00000000;
    ram[12126] = 8'b00000000;
    ram[12125] = 8'b00000000;
    ram[12124] = 8'b00000000;
    ram[12123] = 8'b00000000;
    ram[12122] = 8'b00000000;
    ram[12121] = 8'b00000000;
    ram[12120] = 8'b00000000;
    ram[12119] = 8'b00000000;
    ram[12118] = 8'b00000000;
    ram[12117] = 8'b00000000;
    ram[12116] = 8'b00000000;
    ram[12115] = 8'b00000000;
    ram[12114] = 8'b00000000;
    ram[12113] = 8'b00000000;
    ram[12112] = 8'b00000000;
    ram[12111] = 8'b00000000;
    ram[12110] = 8'b00000000;
    ram[12109] = 8'b00000000;
    ram[12108] = 8'b00000000;
    ram[12107] = 8'b00000000;
    ram[12106] = 8'b00000000;
    ram[12105] = 8'b00000000;
    ram[12104] = 8'b00000000;
    ram[12103] = 8'b00000000;
    ram[12102] = 8'b00000000;
    ram[12101] = 8'b00000000;
    ram[12100] = 8'b00000000;
    ram[12099] = 8'b00000000;
    ram[12098] = 8'b00000000;
    ram[12097] = 8'b00000000;
    ram[12096] = 8'b00000000;
    ram[12095] = 8'b00000000;
    ram[12094] = 8'b00000000;
    ram[12093] = 8'b00000000;
    ram[12092] = 8'b00000000;
    ram[12091] = 8'b00000000;
    ram[12090] = 8'b00000000;
    ram[12089] = 8'b00000000;
    ram[12088] = 8'b00000000;
    ram[12087] = 8'b00000000;
    ram[12086] = 8'b00000000;
    ram[12085] = 8'b00000000;
    ram[12084] = 8'b00000000;
    ram[12083] = 8'b00000000;
    ram[12082] = 8'b00000000;
    ram[12081] = 8'b00000000;
    ram[12080] = 8'b00000000;
    ram[12079] = 8'b00000000;
    ram[12078] = 8'b00000000;
    ram[12077] = 8'b00000000;
    ram[12076] = 8'b00000000;
    ram[12075] = 8'b00000000;
    ram[12074] = 8'b00000000;
    ram[12073] = 8'b00000000;
    ram[12072] = 8'b00000000;
    ram[12071] = 8'b00000000;
    ram[12070] = 8'b00000000;
    ram[12069] = 8'b00000000;
    ram[12068] = 8'b00000000;
    ram[12067] = 8'b00000000;
    ram[12066] = 8'b00000000;
    ram[12065] = 8'b00000000;
    ram[12064] = 8'b00000000;
    ram[12063] = 8'b00000000;
    ram[12062] = 8'b00000000;
    ram[12061] = 8'b00000000;
    ram[12060] = 8'b00000000;
    ram[12059] = 8'b00000000;
    ram[12058] = 8'b00000000;
    ram[12057] = 8'b00000000;
    ram[12056] = 8'b00000000;
    ram[12055] = 8'b00000000;
    ram[12054] = 8'b00000000;
    ram[12053] = 8'b00000000;
    ram[12052] = 8'b00000000;
    ram[12051] = 8'b00000000;
    ram[12050] = 8'b00000000;
    ram[12049] = 8'b00000000;
    ram[12048] = 8'b00000000;
    ram[12047] = 8'b00000000;
    ram[12046] = 8'b00000000;
    ram[12045] = 8'b00000000;
    ram[12044] = 8'b00000000;
    ram[12043] = 8'b00000000;
    ram[12042] = 8'b00000000;
    ram[12041] = 8'b00000000;
    ram[12040] = 8'b00000000;
    ram[12039] = 8'b00000000;
    ram[12038] = 8'b00000000;
    ram[12037] = 8'b00000000;
    ram[12036] = 8'b00000000;
    ram[12035] = 8'b00000000;
    ram[12034] = 8'b00000000;
    ram[12033] = 8'b00000000;
    ram[12032] = 8'b00000000;
    ram[12031] = 8'b00000000;
    ram[12030] = 8'b00000000;
    ram[12029] = 8'b00000000;
    ram[12028] = 8'b00000000;
    ram[12027] = 8'b00000000;
    ram[12026] = 8'b00000000;
    ram[12025] = 8'b00000000;
    ram[12024] = 8'b00000000;
    ram[12023] = 8'b00000000;
    ram[12022] = 8'b00000000;
    ram[12021] = 8'b00000000;
    ram[12020] = 8'b00000000;
    ram[12019] = 8'b00000000;
    ram[12018] = 8'b00000000;
    ram[12017] = 8'b00000000;
    ram[12016] = 8'b00000000;
    ram[12015] = 8'b00000000;
    ram[12014] = 8'b00000000;
    ram[12013] = 8'b00000000;
    ram[12012] = 8'b00000000;
    ram[12011] = 8'b00000000;
    ram[12010] = 8'b00000000;
    ram[12009] = 8'b00000000;
    ram[12008] = 8'b00000000;
    ram[12007] = 8'b00000000;
    ram[12006] = 8'b00000000;
    ram[12005] = 8'b00000000;
    ram[12004] = 8'b00000000;
    ram[12003] = 8'b00000000;
    ram[12002] = 8'b00000000;
    ram[12001] = 8'b00000000;
    ram[12000] = 8'b00000000;
    ram[11999] = 8'b00000000;
    ram[11998] = 8'b00000000;
    ram[11997] = 8'b00000000;
    ram[11996] = 8'b00000000;
    ram[11995] = 8'b00000000;
    ram[11994] = 8'b00000000;
    ram[11993] = 8'b00000000;
    ram[11992] = 8'b00000000;
    ram[11991] = 8'b00000000;
    ram[11990] = 8'b00000000;
    ram[11989] = 8'b00000000;
    ram[11988] = 8'b00000000;
    ram[11987] = 8'b00000000;
    ram[11986] = 8'b00000000;
    ram[11985] = 8'b00000000;
    ram[11984] = 8'b00000000;
    ram[11983] = 8'b00000000;
    ram[11982] = 8'b00000000;
    ram[11981] = 8'b00000000;
    ram[11980] = 8'b00000000;
    ram[11979] = 8'b00000000;
    ram[11978] = 8'b00000000;
    ram[11977] = 8'b00000000;
    ram[11976] = 8'b00000000;
    ram[11975] = 8'b00000000;
    ram[11974] = 8'b00000000;
    ram[11973] = 8'b00000000;
    ram[11972] = 8'b00000000;
    ram[11971] = 8'b00000000;
    ram[11970] = 8'b00000000;
    ram[11969] = 8'b00000000;
    ram[11968] = 8'b00000000;
    ram[11967] = 8'b00000000;
    ram[11966] = 8'b00000000;
    ram[11965] = 8'b00000000;
    ram[11964] = 8'b00000000;
    ram[11963] = 8'b00000000;
    ram[11962] = 8'b00000000;
    ram[11961] = 8'b00000000;
    ram[11960] = 8'b00000000;
    ram[11959] = 8'b00000000;
    ram[11958] = 8'b00000000;
    ram[11957] = 8'b00000000;
    ram[11956] = 8'b00000000;
    ram[11955] = 8'b00000000;
    ram[11954] = 8'b00000000;
    ram[11953] = 8'b00000000;
    ram[11952] = 8'b00000000;
    ram[11951] = 8'b00000000;
    ram[11950] = 8'b00000000;
    ram[11949] = 8'b00000000;
    ram[11948] = 8'b00000000;
    ram[11947] = 8'b00000000;
    ram[11946] = 8'b00000000;
    ram[11945] = 8'b00000000;
    ram[11944] = 8'b00000000;
    ram[11943] = 8'b00000000;
    ram[11942] = 8'b00000000;
    ram[11941] = 8'b00000000;
    ram[11940] = 8'b00000000;
    ram[11939] = 8'b00000000;
    ram[11938] = 8'b00000000;
    ram[11937] = 8'b00000000;
    ram[11936] = 8'b00000000;
    ram[11935] = 8'b00000000;
    ram[11934] = 8'b00000000;
    ram[11933] = 8'b00000000;
    ram[11932] = 8'b00000000;
    ram[11931] = 8'b00000000;
    ram[11930] = 8'b00000000;
    ram[11929] = 8'b00000000;
    ram[11928] = 8'b00000000;
    ram[11927] = 8'b00000000;
    ram[11926] = 8'b00000000;
    ram[11925] = 8'b00000000;
    ram[11924] = 8'b00000000;
    ram[11923] = 8'b00000000;
    ram[11922] = 8'b00000000;
    ram[11921] = 8'b00000000;
    ram[11920] = 8'b00000000;
    ram[11919] = 8'b00000000;
    ram[11918] = 8'b00000000;
    ram[11917] = 8'b00000000;
    ram[11916] = 8'b00000000;
    ram[11915] = 8'b00000000;
    ram[11914] = 8'b00000000;
    ram[11913] = 8'b00000000;
    ram[11912] = 8'b00000000;
    ram[11911] = 8'b00000000;
    ram[11910] = 8'b00000000;
    ram[11909] = 8'b00000000;
    ram[11908] = 8'b00000000;
    ram[11907] = 8'b00000000;
    ram[11906] = 8'b00000000;
    ram[11905] = 8'b00000000;
    ram[11904] = 8'b00000000;
    ram[11903] = 8'b00000000;
    ram[11902] = 8'b00000000;
    ram[11901] = 8'b00000000;
    ram[11900] = 8'b00000000;
    ram[11899] = 8'b00000000;
    ram[11898] = 8'b00000000;
    ram[11897] = 8'b00000000;
    ram[11896] = 8'b00000000;
    ram[11895] = 8'b00000000;
    ram[11894] = 8'b00000000;
    ram[11893] = 8'b00000000;
    ram[11892] = 8'b00000000;
    ram[11891] = 8'b00000000;
    ram[11890] = 8'b00000000;
    ram[11889] = 8'b00000000;
    ram[11888] = 8'b00000000;
    ram[11887] = 8'b00000000;
    ram[11886] = 8'b00000000;
    ram[11885] = 8'b00000000;
    ram[11884] = 8'b00000000;
    ram[11883] = 8'b00000000;
    ram[11882] = 8'b00000000;
    ram[11881] = 8'b00000000;
    ram[11880] = 8'b00000000;
    ram[11879] = 8'b00000000;
    ram[11878] = 8'b00000000;
    ram[11877] = 8'b00000000;
    ram[11876] = 8'b00000000;
    ram[11875] = 8'b00000000;
    ram[11874] = 8'b00000000;
    ram[11873] = 8'b00000000;
    ram[11872] = 8'b00000000;
    ram[11871] = 8'b00000000;
    ram[11870] = 8'b00000000;
    ram[11869] = 8'b00000000;
    ram[11868] = 8'b00000000;
    ram[11867] = 8'b00000000;
    ram[11866] = 8'b00000000;
    ram[11865] = 8'b00000000;
    ram[11864] = 8'b00000000;
    ram[11863] = 8'b00000000;
    ram[11862] = 8'b00000000;
    ram[11861] = 8'b00000000;
    ram[11860] = 8'b00000000;
    ram[11859] = 8'b00000000;
    ram[11858] = 8'b00000000;
    ram[11857] = 8'b00000000;
    ram[11856] = 8'b00000000;
    ram[11855] = 8'b00000000;
    ram[11854] = 8'b00000000;
    ram[11853] = 8'b00000000;
    ram[11852] = 8'b00000000;
    ram[11851] = 8'b00000000;
    ram[11850] = 8'b00000000;
    ram[11849] = 8'b00000000;
    ram[11848] = 8'b00000000;
    ram[11847] = 8'b00000000;
    ram[11846] = 8'b00000000;
    ram[11845] = 8'b00000000;
    ram[11844] = 8'b00000000;
    ram[11843] = 8'b00000000;
    ram[11842] = 8'b00000000;
    ram[11841] = 8'b00000000;
    ram[11840] = 8'b00000000;
    ram[11839] = 8'b00000000;
    ram[11838] = 8'b00000000;
    ram[11837] = 8'b00000000;
    ram[11836] = 8'b00000000;
    ram[11835] = 8'b00000000;
    ram[11834] = 8'b00000000;
    ram[11833] = 8'b00000000;
    ram[11832] = 8'b00000000;
    ram[11831] = 8'b00000000;
    ram[11830] = 8'b00000000;
    ram[11829] = 8'b00000000;
    ram[11828] = 8'b00000000;
    ram[11827] = 8'b00000000;
    ram[11826] = 8'b00000000;
    ram[11825] = 8'b00000000;
    ram[11824] = 8'b00000000;
    ram[11823] = 8'b00000000;
    ram[11822] = 8'b00000000;
    ram[11821] = 8'b00000000;
    ram[11820] = 8'b00000000;
    ram[11819] = 8'b00000000;
    ram[11818] = 8'b00000000;
    ram[11817] = 8'b00000000;
    ram[11816] = 8'b00000000;
    ram[11815] = 8'b00000000;
    ram[11814] = 8'b00000000;
    ram[11813] = 8'b00000000;
    ram[11812] = 8'b00000000;
    ram[11811] = 8'b00000000;
    ram[11810] = 8'b00000000;
    ram[11809] = 8'b00000000;
    ram[11808] = 8'b00000000;
    ram[11807] = 8'b00000000;
    ram[11806] = 8'b00000000;
    ram[11805] = 8'b00000000;
    ram[11804] = 8'b00000000;
    ram[11803] = 8'b00000000;
    ram[11802] = 8'b00000000;
    ram[11801] = 8'b00000000;
    ram[11800] = 8'b00000000;
    ram[11799] = 8'b00000000;
    ram[11798] = 8'b00000000;
    ram[11797] = 8'b00000000;
    ram[11796] = 8'b00000000;
    ram[11795] = 8'b00000000;
    ram[11794] = 8'b00000000;
    ram[11793] = 8'b00000000;
    ram[11792] = 8'b00000000;
    ram[11791] = 8'b00000000;
    ram[11790] = 8'b00000000;
    ram[11789] = 8'b00000000;
    ram[11788] = 8'b00000000;
    ram[11787] = 8'b00000000;
    ram[11786] = 8'b00000000;
    ram[11785] = 8'b00000000;
    ram[11784] = 8'b00000000;
    ram[11783] = 8'b00000000;
    ram[11782] = 8'b00000000;
    ram[11781] = 8'b00000000;
    ram[11780] = 8'b00000000;
    ram[11779] = 8'b00000000;
    ram[11778] = 8'b00000000;
    ram[11777] = 8'b00000000;
    ram[11776] = 8'b00000000;
    ram[11775] = 8'b00000000;
    ram[11774] = 8'b00000000;
    ram[11773] = 8'b00000000;
    ram[11772] = 8'b00000000;
    ram[11771] = 8'b00000000;
    ram[11770] = 8'b00000000;
    ram[11769] = 8'b00000000;
    ram[11768] = 8'b00000000;
    ram[11767] = 8'b00000000;
    ram[11766] = 8'b00000000;
    ram[11765] = 8'b00000000;
    ram[11764] = 8'b00000000;
    ram[11763] = 8'b00000000;
    ram[11762] = 8'b00000000;
    ram[11761] = 8'b00000000;
    ram[11760] = 8'b00000000;
    ram[11759] = 8'b00000000;
    ram[11758] = 8'b00000000;
    ram[11757] = 8'b00000000;
    ram[11756] = 8'b00000000;
    ram[11755] = 8'b00000000;
    ram[11754] = 8'b00000000;
    ram[11753] = 8'b00000000;
    ram[11752] = 8'b00000000;
    ram[11751] = 8'b00000000;
    ram[11750] = 8'b00000000;
    ram[11749] = 8'b00000000;
    ram[11748] = 8'b00000000;
    ram[11747] = 8'b00000000;
    ram[11746] = 8'b00000000;
    ram[11745] = 8'b00000000;
    ram[11744] = 8'b00000000;
    ram[11743] = 8'b00000000;
    ram[11742] = 8'b00000000;
    ram[11741] = 8'b00000000;
    ram[11740] = 8'b00000000;
    ram[11739] = 8'b00000000;
    ram[11738] = 8'b00000000;
    ram[11737] = 8'b00000000;
    ram[11736] = 8'b00000000;
    ram[11735] = 8'b00000000;
    ram[11734] = 8'b00000000;
    ram[11733] = 8'b00000000;
    ram[11732] = 8'b00000000;
    ram[11731] = 8'b00000000;
    ram[11730] = 8'b00000000;
    ram[11729] = 8'b00000000;
    ram[11728] = 8'b00000000;
    ram[11727] = 8'b00000000;
    ram[11726] = 8'b00000000;
    ram[11725] = 8'b00000000;
    ram[11724] = 8'b00000000;
    ram[11723] = 8'b00000000;
    ram[11722] = 8'b00000000;
    ram[11721] = 8'b00000000;
    ram[11720] = 8'b00000000;
    ram[11719] = 8'b00000000;
    ram[11718] = 8'b00000000;
    ram[11717] = 8'b00000000;
    ram[11716] = 8'b00000000;
    ram[11715] = 8'b00000000;
    ram[11714] = 8'b00000000;
    ram[11713] = 8'b00000000;
    ram[11712] = 8'b00000000;
    ram[11711] = 8'b00000000;
    ram[11710] = 8'b00000000;
    ram[11709] = 8'b00000000;
    ram[11708] = 8'b00000000;
    ram[11707] = 8'b00000000;
    ram[11706] = 8'b00000000;
    ram[11705] = 8'b00000000;
    ram[11704] = 8'b00000000;
    ram[11703] = 8'b00000000;
    ram[11702] = 8'b00000000;
    ram[11701] = 8'b00000000;
    ram[11700] = 8'b00000000;
    ram[11699] = 8'b00000000;
    ram[11698] = 8'b00000000;
    ram[11697] = 8'b00000000;
    ram[11696] = 8'b00000000;
    ram[11695] = 8'b00000000;
    ram[11694] = 8'b00000000;
    ram[11693] = 8'b00000000;
    ram[11692] = 8'b00000000;
    ram[11691] = 8'b00000000;
    ram[11690] = 8'b00000000;
    ram[11689] = 8'b00000000;
    ram[11688] = 8'b00000000;
    ram[11687] = 8'b00000000;
    ram[11686] = 8'b00000000;
    ram[11685] = 8'b00000000;
    ram[11684] = 8'b00000000;
    ram[11683] = 8'b00000000;
    ram[11682] = 8'b00000000;
    ram[11681] = 8'b00000000;
    ram[11680] = 8'b00000000;
    ram[11679] = 8'b00000000;
    ram[11678] = 8'b00000000;
    ram[11677] = 8'b00000000;
    ram[11676] = 8'b00000000;
    ram[11675] = 8'b00000000;
    ram[11674] = 8'b00000000;
    ram[11673] = 8'b00000000;
    ram[11672] = 8'b00000000;
    ram[11671] = 8'b00000000;
    ram[11670] = 8'b00000000;
    ram[11669] = 8'b00000000;
    ram[11668] = 8'b00000000;
    ram[11667] = 8'b00000000;
    ram[11666] = 8'b00000000;
    ram[11665] = 8'b00000000;
    ram[11664] = 8'b00000000;
    ram[11663] = 8'b00000000;
    ram[11662] = 8'b00000000;
    ram[11661] = 8'b00000000;
    ram[11660] = 8'b00000000;
    ram[11659] = 8'b00000000;
    ram[11658] = 8'b00000000;
    ram[11657] = 8'b00000000;
    ram[11656] = 8'b00000000;
    ram[11655] = 8'b00000000;
    ram[11654] = 8'b00000000;
    ram[11653] = 8'b00000000;
    ram[11652] = 8'b00000000;
    ram[11651] = 8'b00000000;
    ram[11650] = 8'b00000000;
    ram[11649] = 8'b00000000;
    ram[11648] = 8'b00000000;
    ram[11647] = 8'b00000000;
    ram[11646] = 8'b00000000;
    ram[11645] = 8'b00000000;
    ram[11644] = 8'b00000000;
    ram[11643] = 8'b00000000;
    ram[11642] = 8'b00000000;
    ram[11641] = 8'b00000000;
    ram[11640] = 8'b00000000;
    ram[11639] = 8'b00000000;
    ram[11638] = 8'b00000000;
    ram[11637] = 8'b00000000;
    ram[11636] = 8'b00000000;
    ram[11635] = 8'b00000000;
    ram[11634] = 8'b00000000;
    ram[11633] = 8'b00000000;
    ram[11632] = 8'b00000000;
    ram[11631] = 8'b00000000;
    ram[11630] = 8'b00000000;
    ram[11629] = 8'b00000000;
    ram[11628] = 8'b00000000;
    ram[11627] = 8'b00000000;
    ram[11626] = 8'b00000000;
    ram[11625] = 8'b00000000;
    ram[11624] = 8'b00000000;
    ram[11623] = 8'b00000000;
    ram[11622] = 8'b00000000;
    ram[11621] = 8'b00000000;
    ram[11620] = 8'b00000000;
    ram[11619] = 8'b00000000;
    ram[11618] = 8'b00000000;
    ram[11617] = 8'b00000000;
    ram[11616] = 8'b00000000;
    ram[11615] = 8'b00000000;
    ram[11614] = 8'b00000000;
    ram[11613] = 8'b00000000;
    ram[11612] = 8'b00000000;
    ram[11611] = 8'b00000000;
    ram[11610] = 8'b00000000;
    ram[11609] = 8'b00000000;
    ram[11608] = 8'b00000000;
    ram[11607] = 8'b00000000;
    ram[11606] = 8'b00000000;
    ram[11605] = 8'b00000000;
    ram[11604] = 8'b00000000;
    ram[11603] = 8'b00000000;
    ram[11602] = 8'b00000000;
    ram[11601] = 8'b00000000;
    ram[11600] = 8'b00000000;
    ram[11599] = 8'b00000000;
    ram[11598] = 8'b00000000;
    ram[11597] = 8'b00000000;
    ram[11596] = 8'b00000000;
    ram[11595] = 8'b00000000;
    ram[11594] = 8'b00000000;
    ram[11593] = 8'b00000000;
    ram[11592] = 8'b00000000;
    ram[11591] = 8'b00000000;
    ram[11590] = 8'b00000000;
    ram[11589] = 8'b00000000;
    ram[11588] = 8'b00000000;
    ram[11587] = 8'b00000000;
    ram[11586] = 8'b00000000;
    ram[11585] = 8'b00000000;
    ram[11584] = 8'b00000000;
    ram[11583] = 8'b00000000;
    ram[11582] = 8'b00000000;
    ram[11581] = 8'b00000000;
    ram[11580] = 8'b00000000;
    ram[11579] = 8'b00000000;
    ram[11578] = 8'b00000000;
    ram[11577] = 8'b00000000;
    ram[11576] = 8'b00000000;
    ram[11575] = 8'b00000000;
    ram[11574] = 8'b00000000;
    ram[11573] = 8'b00000000;
    ram[11572] = 8'b00000000;
    ram[11571] = 8'b00000000;
    ram[11570] = 8'b00000000;
    ram[11569] = 8'b00000000;
    ram[11568] = 8'b00000000;
    ram[11567] = 8'b00000000;
    ram[11566] = 8'b00000000;
    ram[11565] = 8'b00000000;
    ram[11564] = 8'b00000000;
    ram[11563] = 8'b00000000;
    ram[11562] = 8'b00000000;
    ram[11561] = 8'b00000000;
    ram[11560] = 8'b00000000;
    ram[11559] = 8'b00000000;
    ram[11558] = 8'b00000000;
    ram[11557] = 8'b00000000;
    ram[11556] = 8'b00000000;
    ram[11555] = 8'b00000000;
    ram[11554] = 8'b00000000;
    ram[11553] = 8'b00000000;
    ram[11552] = 8'b00000000;
    ram[11551] = 8'b00000000;
    ram[11550] = 8'b00000000;
    ram[11549] = 8'b00000000;
    ram[11548] = 8'b00000000;
    ram[11547] = 8'b00000000;
    ram[11546] = 8'b00000000;
    ram[11545] = 8'b00000000;
    ram[11544] = 8'b00000000;
    ram[11543] = 8'b00000000;
    ram[11542] = 8'b00000000;
    ram[11541] = 8'b00000000;
    ram[11540] = 8'b00000000;
    ram[11539] = 8'b00000000;
    ram[11538] = 8'b00000000;
    ram[11537] = 8'b00000000;
    ram[11536] = 8'b00000000;
    ram[11535] = 8'b00000000;
    ram[11534] = 8'b00000000;
    ram[11533] = 8'b00000000;
    ram[11532] = 8'b00000000;
    ram[11531] = 8'b00000000;
    ram[11530] = 8'b00000000;
    ram[11529] = 8'b00000000;
    ram[11528] = 8'b00000000;
    ram[11527] = 8'b00000000;
    ram[11526] = 8'b00000000;
    ram[11525] = 8'b00000000;
    ram[11524] = 8'b00000000;
    ram[11523] = 8'b00000000;
    ram[11522] = 8'b00000000;
    ram[11521] = 8'b00000000;
    ram[11520] = 8'b00000000;
    ram[11519] = 8'b00000000;
    ram[11518] = 8'b00000000;
    ram[11517] = 8'b00000000;
    ram[11516] = 8'b00000000;
    ram[11515] = 8'b00000000;
    ram[11514] = 8'b00000000;
    ram[11513] = 8'b00000000;
    ram[11512] = 8'b00000000;
    ram[11511] = 8'b00000000;
    ram[11510] = 8'b00000000;
    ram[11509] = 8'b00000000;
    ram[11508] = 8'b00000000;
    ram[11507] = 8'b00000000;
    ram[11506] = 8'b00000000;
    ram[11505] = 8'b00000000;
    ram[11504] = 8'b00000000;
    ram[11503] = 8'b00000000;
    ram[11502] = 8'b00000000;
    ram[11501] = 8'b00000000;
    ram[11500] = 8'b00000000;
    ram[11499] = 8'b00000000;
    ram[11498] = 8'b00000000;
    ram[11497] = 8'b00000000;
    ram[11496] = 8'b00000000;
    ram[11495] = 8'b00000000;
    ram[11494] = 8'b00000000;
    ram[11493] = 8'b00000000;
    ram[11492] = 8'b00000000;
    ram[11491] = 8'b00000000;
    ram[11490] = 8'b00000000;
    ram[11489] = 8'b00000000;
    ram[11488] = 8'b00000000;
    ram[11487] = 8'b00000000;
    ram[11486] = 8'b00000000;
    ram[11485] = 8'b00000000;
    ram[11484] = 8'b00000000;
    ram[11483] = 8'b00000000;
    ram[11482] = 8'b00000000;
    ram[11481] = 8'b00000000;
    ram[11480] = 8'b00000000;
    ram[11479] = 8'b00000000;
    ram[11478] = 8'b00000000;
    ram[11477] = 8'b00000000;
    ram[11476] = 8'b00000000;
    ram[11475] = 8'b00000000;
    ram[11474] = 8'b00000000;
    ram[11473] = 8'b00000000;
    ram[11472] = 8'b00000000;
    ram[11471] = 8'b00000000;
    ram[11470] = 8'b00000000;
    ram[11469] = 8'b00000000;
    ram[11468] = 8'b00000000;
    ram[11467] = 8'b00000000;
    ram[11466] = 8'b00000000;
    ram[11465] = 8'b00000000;
    ram[11464] = 8'b00000000;
    ram[11463] = 8'b00000000;
    ram[11462] = 8'b00000000;
    ram[11461] = 8'b00000000;
    ram[11460] = 8'b00000000;
    ram[11459] = 8'b00000000;
    ram[11458] = 8'b00000000;
    ram[11457] = 8'b00000000;
    ram[11456] = 8'b00000000;
    ram[11455] = 8'b00000000;
    ram[11454] = 8'b00000000;
    ram[11453] = 8'b00000000;
    ram[11452] = 8'b00000000;
    ram[11451] = 8'b00000000;
    ram[11450] = 8'b00000000;
    ram[11449] = 8'b00000000;
    ram[11448] = 8'b00000000;
    ram[11447] = 8'b00000000;
    ram[11446] = 8'b00000000;
    ram[11445] = 8'b00000000;
    ram[11444] = 8'b00000000;
    ram[11443] = 8'b00000000;
    ram[11442] = 8'b00000000;
    ram[11441] = 8'b00000000;
    ram[11440] = 8'b00000000;
    ram[11439] = 8'b00000000;
    ram[11438] = 8'b00000000;
    ram[11437] = 8'b00000000;
    ram[11436] = 8'b00000000;
    ram[11435] = 8'b00000000;
    ram[11434] = 8'b00000000;
    ram[11433] = 8'b00000000;
    ram[11432] = 8'b00000000;
    ram[11431] = 8'b00000000;
    ram[11430] = 8'b00000000;
    ram[11429] = 8'b00000000;
    ram[11428] = 8'b00000000;
    ram[11427] = 8'b00000000;
    ram[11426] = 8'b00000000;
    ram[11425] = 8'b00000000;
    ram[11424] = 8'b00000000;
    ram[11423] = 8'b00000000;
    ram[11422] = 8'b00000000;
    ram[11421] = 8'b00000000;
    ram[11420] = 8'b00000000;
    ram[11419] = 8'b00000000;
    ram[11418] = 8'b00000000;
    ram[11417] = 8'b00000000;
    ram[11416] = 8'b00000000;
    ram[11415] = 8'b00000000;
    ram[11414] = 8'b00000000;
    ram[11413] = 8'b00000000;
    ram[11412] = 8'b00000000;
    ram[11411] = 8'b00000000;
    ram[11410] = 8'b00000000;
    ram[11409] = 8'b00000000;
    ram[11408] = 8'b00000000;
    ram[11407] = 8'b00000000;
    ram[11406] = 8'b00000000;
    ram[11405] = 8'b00000000;
    ram[11404] = 8'b00000000;
    ram[11403] = 8'b00000000;
    ram[11402] = 8'b00000000;
    ram[11401] = 8'b00000000;
    ram[11400] = 8'b00000000;
    ram[11399] = 8'b00000000;
    ram[11398] = 8'b00000000;
    ram[11397] = 8'b00000000;
    ram[11396] = 8'b00000000;
    ram[11395] = 8'b00000000;
    ram[11394] = 8'b00000000;
    ram[11393] = 8'b00000000;
    ram[11392] = 8'b00000000;
    ram[11391] = 8'b00000000;
    ram[11390] = 8'b00000000;
    ram[11389] = 8'b00000000;
    ram[11388] = 8'b00000000;
    ram[11387] = 8'b00000000;
    ram[11386] = 8'b00000000;
    ram[11385] = 8'b00000000;
    ram[11384] = 8'b00000000;
    ram[11383] = 8'b00000000;
    ram[11382] = 8'b00000000;
    ram[11381] = 8'b00000000;
    ram[11380] = 8'b00000000;
    ram[11379] = 8'b00000000;
    ram[11378] = 8'b00000000;
    ram[11377] = 8'b00000000;
    ram[11376] = 8'b00000000;
    ram[11375] = 8'b00000000;
    ram[11374] = 8'b00000000;
    ram[11373] = 8'b00000000;
    ram[11372] = 8'b00000000;
    ram[11371] = 8'b00000000;
    ram[11370] = 8'b00000000;
    ram[11369] = 8'b00000000;
    ram[11368] = 8'b00000000;
    ram[11367] = 8'b00000000;
    ram[11366] = 8'b00000000;
    ram[11365] = 8'b00000000;
    ram[11364] = 8'b00000000;
    ram[11363] = 8'b00000000;
    ram[11362] = 8'b00000000;
    ram[11361] = 8'b00000000;
    ram[11360] = 8'b00000000;
    ram[11359] = 8'b00000000;
    ram[11358] = 8'b00000000;
    ram[11357] = 8'b00000000;
    ram[11356] = 8'b00000000;
    ram[11355] = 8'b00000000;
    ram[11354] = 8'b00000000;
    ram[11353] = 8'b00000000;
    ram[11352] = 8'b00000000;
    ram[11351] = 8'b00000000;
    ram[11350] = 8'b00000000;
    ram[11349] = 8'b00000000;
    ram[11348] = 8'b00000000;
    ram[11347] = 8'b00000000;
    ram[11346] = 8'b00000000;
    ram[11345] = 8'b00000000;
    ram[11344] = 8'b00000000;
    ram[11343] = 8'b00000000;
    ram[11342] = 8'b00000000;
    ram[11341] = 8'b00000000;
    ram[11340] = 8'b00000000;
    ram[11339] = 8'b00000000;
    ram[11338] = 8'b00000000;
    ram[11337] = 8'b00000000;
    ram[11336] = 8'b00000000;
    ram[11335] = 8'b00000000;
    ram[11334] = 8'b00000000;
    ram[11333] = 8'b00000000;
    ram[11332] = 8'b00000000;
    ram[11331] = 8'b00000000;
    ram[11330] = 8'b00000000;
    ram[11329] = 8'b00000000;
    ram[11328] = 8'b00000000;
    ram[11327] = 8'b00000000;
    ram[11326] = 8'b00000000;
    ram[11325] = 8'b00000000;
    ram[11324] = 8'b00000000;
    ram[11323] = 8'b00000000;
    ram[11322] = 8'b00000000;
    ram[11321] = 8'b00000000;
    ram[11320] = 8'b00000000;
    ram[11319] = 8'b00000000;
    ram[11318] = 8'b00000000;
    ram[11317] = 8'b00000000;
    ram[11316] = 8'b00000000;
    ram[11315] = 8'b00000000;
    ram[11314] = 8'b00000000;
    ram[11313] = 8'b00000000;
    ram[11312] = 8'b00000000;
    ram[11311] = 8'b00000000;
    ram[11310] = 8'b00000000;
    ram[11309] = 8'b00000000;
    ram[11308] = 8'b00000000;
    ram[11307] = 8'b00000000;
    ram[11306] = 8'b00000000;
    ram[11305] = 8'b00000000;
    ram[11304] = 8'b00000000;
    ram[11303] = 8'b00000000;
    ram[11302] = 8'b00000000;
    ram[11301] = 8'b00000000;
    ram[11300] = 8'b00000000;
    ram[11299] = 8'b00000000;
    ram[11298] = 8'b00000000;
    ram[11297] = 8'b00000000;
    ram[11296] = 8'b00000000;
    ram[11295] = 8'b00000000;
    ram[11294] = 8'b00000000;
    ram[11293] = 8'b00000000;
    ram[11292] = 8'b00000000;
    ram[11291] = 8'b00000000;
    ram[11290] = 8'b00000000;
    ram[11289] = 8'b00000000;
    ram[11288] = 8'b00000000;
    ram[11287] = 8'b00000000;
    ram[11286] = 8'b00000000;
    ram[11285] = 8'b00000000;
    ram[11284] = 8'b00000000;
    ram[11283] = 8'b00000000;
    ram[11282] = 8'b00000000;
    ram[11281] = 8'b00000000;
    ram[11280] = 8'b00000000;
    ram[11279] = 8'b00000000;
    ram[11278] = 8'b00000000;
    ram[11277] = 8'b00000000;
    ram[11276] = 8'b00000000;
    ram[11275] = 8'b00000000;
    ram[11274] = 8'b00000000;
    ram[11273] = 8'b00000000;
    ram[11272] = 8'b00000000;
    ram[11271] = 8'b00000000;
    ram[11270] = 8'b00000000;
    ram[11269] = 8'b00000000;
    ram[11268] = 8'b00000000;
    ram[11267] = 8'b00000000;
    ram[11266] = 8'b00000000;
    ram[11265] = 8'b00000000;
    ram[11264] = 8'b00000000;
    ram[11263] = 8'b00000000;
    ram[11262] = 8'b00000000;
    ram[11261] = 8'b00000000;
    ram[11260] = 8'b00000000;
    ram[11259] = 8'b00000000;
    ram[11258] = 8'b00000000;
    ram[11257] = 8'b00000000;
    ram[11256] = 8'b00000000;
    ram[11255] = 8'b00000000;
    ram[11254] = 8'b00000000;
    ram[11253] = 8'b00000000;
    ram[11252] = 8'b00000000;
    ram[11251] = 8'b00000000;
    ram[11250] = 8'b00000000;
    ram[11249] = 8'b00000000;
    ram[11248] = 8'b00000000;
    ram[11247] = 8'b00000000;
    ram[11246] = 8'b00000000;
    ram[11245] = 8'b00000000;
    ram[11244] = 8'b00000000;
    ram[11243] = 8'b00000000;
    ram[11242] = 8'b00000000;
    ram[11241] = 8'b00000000;
    ram[11240] = 8'b00000000;
    ram[11239] = 8'b00000000;
    ram[11238] = 8'b00000000;
    ram[11237] = 8'b00000000;
    ram[11236] = 8'b00000000;
    ram[11235] = 8'b00000000;
    ram[11234] = 8'b00000000;
    ram[11233] = 8'b00000000;
    ram[11232] = 8'b00000000;
    ram[11231] = 8'b00000000;
    ram[11230] = 8'b00000000;
    ram[11229] = 8'b00000000;
    ram[11228] = 8'b00000000;
    ram[11227] = 8'b00000000;
    ram[11226] = 8'b00000000;
    ram[11225] = 8'b00000000;
    ram[11224] = 8'b00000000;
    ram[11223] = 8'b00000000;
    ram[11222] = 8'b00000000;
    ram[11221] = 8'b00000000;
    ram[11220] = 8'b00000000;
    ram[11219] = 8'b00000000;
    ram[11218] = 8'b00000000;
    ram[11217] = 8'b00000000;
    ram[11216] = 8'b00000000;
    ram[11215] = 8'b00000000;
    ram[11214] = 8'b00000000;
    ram[11213] = 8'b00000000;
    ram[11212] = 8'b00000000;
    ram[11211] = 8'b00000000;
    ram[11210] = 8'b00000000;
    ram[11209] = 8'b00000000;
    ram[11208] = 8'b00000000;
    ram[11207] = 8'b00000000;
    ram[11206] = 8'b00000000;
    ram[11205] = 8'b00000000;
    ram[11204] = 8'b00000000;
    ram[11203] = 8'b00000000;
    ram[11202] = 8'b00000000;
    ram[11201] = 8'b00000000;
    ram[11200] = 8'b00000000;
    ram[11199] = 8'b00000000;
    ram[11198] = 8'b00000000;
    ram[11197] = 8'b00000000;
    ram[11196] = 8'b00000000;
    ram[11195] = 8'b00000000;
    ram[11194] = 8'b00000000;
    ram[11193] = 8'b00000000;
    ram[11192] = 8'b00000000;
    ram[11191] = 8'b00000000;
    ram[11190] = 8'b00000000;
    ram[11189] = 8'b00000000;
    ram[11188] = 8'b00000000;
    ram[11187] = 8'b00000000;
    ram[11186] = 8'b00000000;
    ram[11185] = 8'b00000000;
    ram[11184] = 8'b00000000;
    ram[11183] = 8'b00000000;
    ram[11182] = 8'b00000000;
    ram[11181] = 8'b00000000;
    ram[11180] = 8'b00000000;
    ram[11179] = 8'b00000000;
    ram[11178] = 8'b00000000;
    ram[11177] = 8'b00000000;
    ram[11176] = 8'b00000000;
    ram[11175] = 8'b00000000;
    ram[11174] = 8'b00000000;
    ram[11173] = 8'b00000000;
    ram[11172] = 8'b00000000;
    ram[11171] = 8'b00000000;
    ram[11170] = 8'b00000000;
    ram[11169] = 8'b00000000;
    ram[11168] = 8'b00000000;
    ram[11167] = 8'b00000000;
    ram[11166] = 8'b00000000;
    ram[11165] = 8'b00000000;
    ram[11164] = 8'b00000000;
    ram[11163] = 8'b00000000;
    ram[11162] = 8'b00000000;
    ram[11161] = 8'b00000000;
    ram[11160] = 8'b00000000;
    ram[11159] = 8'b00000000;
    ram[11158] = 8'b00000000;
    ram[11157] = 8'b00000000;
    ram[11156] = 8'b00000000;
    ram[11155] = 8'b00000000;
    ram[11154] = 8'b00000000;
    ram[11153] = 8'b00000000;
    ram[11152] = 8'b00000000;
    ram[11151] = 8'b00000000;
    ram[11150] = 8'b00000000;
    ram[11149] = 8'b00000000;
    ram[11148] = 8'b00000000;
    ram[11147] = 8'b00000000;
    ram[11146] = 8'b00000000;
    ram[11145] = 8'b00000000;
    ram[11144] = 8'b00000000;
    ram[11143] = 8'b00000000;
    ram[11142] = 8'b00000000;
    ram[11141] = 8'b00000000;
    ram[11140] = 8'b00000000;
    ram[11139] = 8'b00000000;
    ram[11138] = 8'b00000000;
    ram[11137] = 8'b00000000;
    ram[11136] = 8'b00000000;
    ram[11135] = 8'b00000000;
    ram[11134] = 8'b00000000;
    ram[11133] = 8'b00000000;
    ram[11132] = 8'b00000000;
    ram[11131] = 8'b00000000;
    ram[11130] = 8'b00000000;
    ram[11129] = 8'b00000000;
    ram[11128] = 8'b00000000;
    ram[11127] = 8'b00000000;
    ram[11126] = 8'b00000000;
    ram[11125] = 8'b00000000;
    ram[11124] = 8'b00000000;
    ram[11123] = 8'b00000000;
    ram[11122] = 8'b00000000;
    ram[11121] = 8'b00000000;
    ram[11120] = 8'b00000000;
    ram[11119] = 8'b00000000;
    ram[11118] = 8'b00000000;
    ram[11117] = 8'b00000000;
    ram[11116] = 8'b00000000;
    ram[11115] = 8'b00000000;
    ram[11114] = 8'b00000000;
    ram[11113] = 8'b00000000;
    ram[11112] = 8'b00000000;
    ram[11111] = 8'b00000000;
    ram[11110] = 8'b00000000;
    ram[11109] = 8'b00000000;
    ram[11108] = 8'b00000000;
    ram[11107] = 8'b00000000;
    ram[11106] = 8'b00000000;
    ram[11105] = 8'b00000000;
    ram[11104] = 8'b00000000;
    ram[11103] = 8'b00000000;
    ram[11102] = 8'b00000000;
    ram[11101] = 8'b00000000;
    ram[11100] = 8'b00000000;
    ram[11099] = 8'b00000000;
    ram[11098] = 8'b00000000;
    ram[11097] = 8'b00000000;
    ram[11096] = 8'b00000000;
    ram[11095] = 8'b00000000;
    ram[11094] = 8'b00000000;
    ram[11093] = 8'b00000000;
    ram[11092] = 8'b00000000;
    ram[11091] = 8'b00000000;
    ram[11090] = 8'b00000000;
    ram[11089] = 8'b00000000;
    ram[11088] = 8'b00000000;
    ram[11087] = 8'b00000000;
    ram[11086] = 8'b00000000;
    ram[11085] = 8'b00000000;
    ram[11084] = 8'b00000000;
    ram[11083] = 8'b00000000;
    ram[11082] = 8'b00000000;
    ram[11081] = 8'b00000000;
    ram[11080] = 8'b00000000;
    ram[11079] = 8'b00000000;
    ram[11078] = 8'b00000000;
    ram[11077] = 8'b00000000;
    ram[11076] = 8'b00000000;
    ram[11075] = 8'b00000000;
    ram[11074] = 8'b00000000;
    ram[11073] = 8'b00000000;
    ram[11072] = 8'b00000000;
    ram[11071] = 8'b00000000;
    ram[11070] = 8'b00000000;
    ram[11069] = 8'b00000000;
    ram[11068] = 8'b00000000;
    ram[11067] = 8'b00000000;
    ram[11066] = 8'b00000000;
    ram[11065] = 8'b00000000;
    ram[11064] = 8'b00000000;
    ram[11063] = 8'b00000000;
    ram[11062] = 8'b00000000;
    ram[11061] = 8'b00000000;
    ram[11060] = 8'b00000000;
    ram[11059] = 8'b00000000;
    ram[11058] = 8'b00000000;
    ram[11057] = 8'b00000000;
    ram[11056] = 8'b00000000;
    ram[11055] = 8'b00000000;
    ram[11054] = 8'b00000000;
    ram[11053] = 8'b00000000;
    ram[11052] = 8'b00000000;
    ram[11051] = 8'b00000000;
    ram[11050] = 8'b00000000;
    ram[11049] = 8'b00000000;
    ram[11048] = 8'b00000000;
    ram[11047] = 8'b00000000;
    ram[11046] = 8'b00000000;
    ram[11045] = 8'b00000000;
    ram[11044] = 8'b00000000;
    ram[11043] = 8'b00000000;
    ram[11042] = 8'b00000000;
    ram[11041] = 8'b00000000;
    ram[11040] = 8'b00000000;
    ram[11039] = 8'b00000000;
    ram[11038] = 8'b00000000;
    ram[11037] = 8'b00000000;
    ram[11036] = 8'b00000000;
    ram[11035] = 8'b00000000;
    ram[11034] = 8'b00000000;
    ram[11033] = 8'b00000000;
    ram[11032] = 8'b00000000;
    ram[11031] = 8'b00000000;
    ram[11030] = 8'b00000000;
    ram[11029] = 8'b00000000;
    ram[11028] = 8'b00000000;
    ram[11027] = 8'b00000000;
    ram[11026] = 8'b00000000;
    ram[11025] = 8'b00000000;
    ram[11024] = 8'b00000000;
    ram[11023] = 8'b00000000;
    ram[11022] = 8'b00000000;
    ram[11021] = 8'b00000000;
    ram[11020] = 8'b00000000;
    ram[11019] = 8'b00000000;
    ram[11018] = 8'b00000000;
    ram[11017] = 8'b00000000;
    ram[11016] = 8'b00000000;
    ram[11015] = 8'b00000000;
    ram[11014] = 8'b00000000;
    ram[11013] = 8'b00000000;
    ram[11012] = 8'b00000000;
    ram[11011] = 8'b00000000;
    ram[11010] = 8'b00000000;
    ram[11009] = 8'b00000000;
    ram[11008] = 8'b00000000;
    ram[11007] = 8'b00000000;
    ram[11006] = 8'b00000000;
    ram[11005] = 8'b00000000;
    ram[11004] = 8'b00000000;
    ram[11003] = 8'b00000000;
    ram[11002] = 8'b00000000;
    ram[11001] = 8'b00000000;
    ram[11000] = 8'b00000000;
    ram[10999] = 8'b00000000;
    ram[10998] = 8'b00000000;
    ram[10997] = 8'b00000000;
    ram[10996] = 8'b00000000;
    ram[10995] = 8'b00000000;
    ram[10994] = 8'b00000000;
    ram[10993] = 8'b00000000;
    ram[10992] = 8'b00000000;
    ram[10991] = 8'b00000000;
    ram[10990] = 8'b00000000;
    ram[10989] = 8'b00000000;
    ram[10988] = 8'b00000000;
    ram[10987] = 8'b00000000;
    ram[10986] = 8'b00000000;
    ram[10985] = 8'b00000000;
    ram[10984] = 8'b00000000;
    ram[10983] = 8'b00000000;
    ram[10982] = 8'b00000000;
    ram[10981] = 8'b00000000;
    ram[10980] = 8'b00000000;
    ram[10979] = 8'b00000000;
    ram[10978] = 8'b00000000;
    ram[10977] = 8'b00000000;
    ram[10976] = 8'b00000000;
    ram[10975] = 8'b00000000;
    ram[10974] = 8'b00000000;
    ram[10973] = 8'b00000000;
    ram[10972] = 8'b00000000;
    ram[10971] = 8'b00000000;
    ram[10970] = 8'b00000000;
    ram[10969] = 8'b00000000;
    ram[10968] = 8'b00000000;
    ram[10967] = 8'b00000000;
    ram[10966] = 8'b00000000;
    ram[10965] = 8'b00000000;
    ram[10964] = 8'b00000000;
    ram[10963] = 8'b00000000;
    ram[10962] = 8'b00000000;
    ram[10961] = 8'b00000000;
    ram[10960] = 8'b00000000;
    ram[10959] = 8'b00000000;
    ram[10958] = 8'b00000000;
    ram[10957] = 8'b00000000;
    ram[10956] = 8'b00000000;
    ram[10955] = 8'b00000000;
    ram[10954] = 8'b00000000;
    ram[10953] = 8'b00000000;
    ram[10952] = 8'b00000000;
    ram[10951] = 8'b00000000;
    ram[10950] = 8'b00000000;
    ram[10949] = 8'b00000000;
    ram[10948] = 8'b00000000;
    ram[10947] = 8'b00000000;
    ram[10946] = 8'b00000000;
    ram[10945] = 8'b00000000;
    ram[10944] = 8'b00000000;
    ram[10943] = 8'b00000000;
    ram[10942] = 8'b00000000;
    ram[10941] = 8'b00000000;
    ram[10940] = 8'b00000000;
    ram[10939] = 8'b00000000;
    ram[10938] = 8'b00000000;
    ram[10937] = 8'b00000000;
    ram[10936] = 8'b00000000;
    ram[10935] = 8'b00000000;
    ram[10934] = 8'b00000000;
    ram[10933] = 8'b00000000;
    ram[10932] = 8'b00000000;
    ram[10931] = 8'b00000000;
    ram[10930] = 8'b00000000;
    ram[10929] = 8'b00000000;
    ram[10928] = 8'b00000000;
    ram[10927] = 8'b00000000;
    ram[10926] = 8'b00000000;
    ram[10925] = 8'b00000000;
    ram[10924] = 8'b00000000;
    ram[10923] = 8'b00000000;
    ram[10922] = 8'b00000000;
    ram[10921] = 8'b00000000;
    ram[10920] = 8'b00000000;
    ram[10919] = 8'b00000000;
    ram[10918] = 8'b00000000;
    ram[10917] = 8'b00000000;
    ram[10916] = 8'b00000000;
    ram[10915] = 8'b00000000;
    ram[10914] = 8'b00000000;
    ram[10913] = 8'b00000000;
    ram[10912] = 8'b00000000;
    ram[10911] = 8'b00000000;
    ram[10910] = 8'b00000000;
    ram[10909] = 8'b00000000;
    ram[10908] = 8'b00000000;
    ram[10907] = 8'b00000000;
    ram[10906] = 8'b00000000;
    ram[10905] = 8'b00000000;
    ram[10904] = 8'b00000000;
    ram[10903] = 8'b00000000;
    ram[10902] = 8'b00000000;
    ram[10901] = 8'b00000000;
    ram[10900] = 8'b00000000;
    ram[10899] = 8'b00000000;
    ram[10898] = 8'b00000000;
    ram[10897] = 8'b00000000;
    ram[10896] = 8'b00000000;
    ram[10895] = 8'b00000000;
    ram[10894] = 8'b00000000;
    ram[10893] = 8'b00000000;
    ram[10892] = 8'b00000000;
    ram[10891] = 8'b00000000;
    ram[10890] = 8'b00000000;
    ram[10889] = 8'b00000000;
    ram[10888] = 8'b00000000;
    ram[10887] = 8'b00000000;
    ram[10886] = 8'b00000000;
    ram[10885] = 8'b00000000;
    ram[10884] = 8'b00000000;
    ram[10883] = 8'b00000000;
    ram[10882] = 8'b00000000;
    ram[10881] = 8'b00000000;
    ram[10880] = 8'b00000000;
    ram[10879] = 8'b00000000;
    ram[10878] = 8'b00000000;
    ram[10877] = 8'b00000000;
    ram[10876] = 8'b00000000;
    ram[10875] = 8'b00000000;
    ram[10874] = 8'b00000000;
    ram[10873] = 8'b00000000;
    ram[10872] = 8'b00000000;
    ram[10871] = 8'b00000000;
    ram[10870] = 8'b00000000;
    ram[10869] = 8'b00000000;
    ram[10868] = 8'b00000000;
    ram[10867] = 8'b00000000;
    ram[10866] = 8'b00000000;
    ram[10865] = 8'b00000000;
    ram[10864] = 8'b00000000;
    ram[10863] = 8'b00000000;
    ram[10862] = 8'b00000000;
    ram[10861] = 8'b00000000;
    ram[10860] = 8'b00000000;
    ram[10859] = 8'b00000000;
    ram[10858] = 8'b00000000;
    ram[10857] = 8'b00000000;
    ram[10856] = 8'b00000000;
    ram[10855] = 8'b00000000;
    ram[10854] = 8'b00000000;
    ram[10853] = 8'b00000000;
    ram[10852] = 8'b00000000;
    ram[10851] = 8'b00000000;
    ram[10850] = 8'b00000000;
    ram[10849] = 8'b00000000;
    ram[10848] = 8'b00000000;
    ram[10847] = 8'b00000000;
    ram[10846] = 8'b00000000;
    ram[10845] = 8'b00000000;
    ram[10844] = 8'b00000000;
    ram[10843] = 8'b00000000;
    ram[10842] = 8'b00000000;
    ram[10841] = 8'b00000000;
    ram[10840] = 8'b00000000;
    ram[10839] = 8'b00000000;
    ram[10838] = 8'b00000000;
    ram[10837] = 8'b00000000;
    ram[10836] = 8'b00000000;
    ram[10835] = 8'b00000000;
    ram[10834] = 8'b00000000;
    ram[10833] = 8'b00000000;
    ram[10832] = 8'b00000000;
    ram[10831] = 8'b00000000;
    ram[10830] = 8'b00000000;
    ram[10829] = 8'b00000000;
    ram[10828] = 8'b00000000;
    ram[10827] = 8'b00000000;
    ram[10826] = 8'b00000000;
    ram[10825] = 8'b00000000;
    ram[10824] = 8'b00000000;
    ram[10823] = 8'b00000000;
    ram[10822] = 8'b00000000;
    ram[10821] = 8'b00000000;
    ram[10820] = 8'b00000000;
    ram[10819] = 8'b00000000;
    ram[10818] = 8'b00000000;
    ram[10817] = 8'b00000000;
    ram[10816] = 8'b00000000;
    ram[10815] = 8'b00000000;
    ram[10814] = 8'b00000000;
    ram[10813] = 8'b00000000;
    ram[10812] = 8'b00000000;
    ram[10811] = 8'b00000000;
    ram[10810] = 8'b00000000;
    ram[10809] = 8'b00000000;
    ram[10808] = 8'b00000000;
    ram[10807] = 8'b00000000;
    ram[10806] = 8'b00000000;
    ram[10805] = 8'b00000000;
    ram[10804] = 8'b00000000;
    ram[10803] = 8'b00000000;
    ram[10802] = 8'b00000000;
    ram[10801] = 8'b00000000;
    ram[10800] = 8'b00000000;
    ram[10799] = 8'b00000000;
    ram[10798] = 8'b00000000;
    ram[10797] = 8'b00000000;
    ram[10796] = 8'b00000000;
    ram[10795] = 8'b00000000;
    ram[10794] = 8'b00000000;
    ram[10793] = 8'b00000000;
    ram[10792] = 8'b00000000;
    ram[10791] = 8'b00000000;
    ram[10790] = 8'b00000000;
    ram[10789] = 8'b00000000;
    ram[10788] = 8'b00000000;
    ram[10787] = 8'b00000000;
    ram[10786] = 8'b00000000;
    ram[10785] = 8'b00000000;
    ram[10784] = 8'b00000000;
    ram[10783] = 8'b00000000;
    ram[10782] = 8'b00000000;
    ram[10781] = 8'b00000000;
    ram[10780] = 8'b00000000;
    ram[10779] = 8'b00000000;
    ram[10778] = 8'b00000000;
    ram[10777] = 8'b00000000;
    ram[10776] = 8'b00000000;
    ram[10775] = 8'b00000000;
    ram[10774] = 8'b00000000;
    ram[10773] = 8'b00000000;
    ram[10772] = 8'b00000000;
    ram[10771] = 8'b00000000;
    ram[10770] = 8'b00000000;
    ram[10769] = 8'b00000000;
    ram[10768] = 8'b00000000;
    ram[10767] = 8'b00000000;
    ram[10766] = 8'b00000000;
    ram[10765] = 8'b00000000;
    ram[10764] = 8'b00000000;
    ram[10763] = 8'b00000000;
    ram[10762] = 8'b00000000;
    ram[10761] = 8'b00000000;
    ram[10760] = 8'b00000000;
    ram[10759] = 8'b00000000;
    ram[10758] = 8'b00000000;
    ram[10757] = 8'b00000000;
    ram[10756] = 8'b00000000;
    ram[10755] = 8'b00000000;
    ram[10754] = 8'b00000000;
    ram[10753] = 8'b00000000;
    ram[10752] = 8'b00000000;
    ram[10751] = 8'b00000000;
    ram[10750] = 8'b00000000;
    ram[10749] = 8'b00000000;
    ram[10748] = 8'b00000000;
    ram[10747] = 8'b00000000;
    ram[10746] = 8'b00000000;
    ram[10745] = 8'b00000000;
    ram[10744] = 8'b00000000;
    ram[10743] = 8'b00000000;
    ram[10742] = 8'b00000000;
    ram[10741] = 8'b00000000;
    ram[10740] = 8'b00000000;
    ram[10739] = 8'b00000000;
    ram[10738] = 8'b00000000;
    ram[10737] = 8'b00000000;
    ram[10736] = 8'b00000000;
    ram[10735] = 8'b00000000;
    ram[10734] = 8'b00000000;
    ram[10733] = 8'b00000000;
    ram[10732] = 8'b00000000;
    ram[10731] = 8'b00000000;
    ram[10730] = 8'b00000000;
    ram[10729] = 8'b00000000;
    ram[10728] = 8'b00000000;
    ram[10727] = 8'b00000000;
    ram[10726] = 8'b00000000;
    ram[10725] = 8'b00000000;
    ram[10724] = 8'b00000000;
    ram[10723] = 8'b00000000;
    ram[10722] = 8'b00000000;
    ram[10721] = 8'b00000000;
    ram[10720] = 8'b00000000;
    ram[10719] = 8'b00000000;
    ram[10718] = 8'b00000000;
    ram[10717] = 8'b00000000;
    ram[10716] = 8'b00000000;
    ram[10715] = 8'b00000000;
    ram[10714] = 8'b00000000;
    ram[10713] = 8'b00000000;
    ram[10712] = 8'b00000000;
    ram[10711] = 8'b00000000;
    ram[10710] = 8'b00000000;
    ram[10709] = 8'b00000000;
    ram[10708] = 8'b00000000;
    ram[10707] = 8'b00000000;
    ram[10706] = 8'b00000000;
    ram[10705] = 8'b00000000;
    ram[10704] = 8'b00000000;
    ram[10703] = 8'b00000000;
    ram[10702] = 8'b00000000;
    ram[10701] = 8'b00000000;
    ram[10700] = 8'b00000000;
    ram[10699] = 8'b00000000;
    ram[10698] = 8'b00000000;
    ram[10697] = 8'b00000000;
    ram[10696] = 8'b00000000;
    ram[10695] = 8'b00000000;
    ram[10694] = 8'b00000000;
    ram[10693] = 8'b00000000;
    ram[10692] = 8'b00000000;
    ram[10691] = 8'b00000000;
    ram[10690] = 8'b00000000;
    ram[10689] = 8'b00000000;
    ram[10688] = 8'b00000000;
    ram[10687] = 8'b00000000;
    ram[10686] = 8'b00000000;
    ram[10685] = 8'b00000000;
    ram[10684] = 8'b00000000;
    ram[10683] = 8'b00000000;
    ram[10682] = 8'b00000000;
    ram[10681] = 8'b00000000;
    ram[10680] = 8'b00000000;
    ram[10679] = 8'b00000000;
    ram[10678] = 8'b00000000;
    ram[10677] = 8'b00000000;
    ram[10676] = 8'b00000000;
    ram[10675] = 8'b00000000;
    ram[10674] = 8'b00000000;
    ram[10673] = 8'b00000000;
    ram[10672] = 8'b00000000;
    ram[10671] = 8'b00000000;
    ram[10670] = 8'b00000000;
    ram[10669] = 8'b00000000;
    ram[10668] = 8'b00000000;
    ram[10667] = 8'b00000000;
    ram[10666] = 8'b00000000;
    ram[10665] = 8'b00000000;
    ram[10664] = 8'b00000000;
    ram[10663] = 8'b00000000;
    ram[10662] = 8'b00000000;
    ram[10661] = 8'b00000000;
    ram[10660] = 8'b00000000;
    ram[10659] = 8'b00000000;
    ram[10658] = 8'b00000000;
    ram[10657] = 8'b00000000;
    ram[10656] = 8'b00000000;
    ram[10655] = 8'b00000000;
    ram[10654] = 8'b00000000;
    ram[10653] = 8'b00000000;
    ram[10652] = 8'b00000000;
    ram[10651] = 8'b00000000;
    ram[10650] = 8'b00000000;
    ram[10649] = 8'b00000000;
    ram[10648] = 8'b00000000;
    ram[10647] = 8'b00000000;
    ram[10646] = 8'b00000000;
    ram[10645] = 8'b00000000;
    ram[10644] = 8'b00000000;
    ram[10643] = 8'b00000000;
    ram[10642] = 8'b00000000;
    ram[10641] = 8'b00000000;
    ram[10640] = 8'b00000000;
    ram[10639] = 8'b00000000;
    ram[10638] = 8'b00000000;
    ram[10637] = 8'b00000000;
    ram[10636] = 8'b00000000;
    ram[10635] = 8'b00000000;
    ram[10634] = 8'b00000000;
    ram[10633] = 8'b00000000;
    ram[10632] = 8'b00000000;
    ram[10631] = 8'b00000000;
    ram[10630] = 8'b00000000;
    ram[10629] = 8'b00000000;
    ram[10628] = 8'b00000000;
    ram[10627] = 8'b00000000;
    ram[10626] = 8'b00000000;
    ram[10625] = 8'b00000000;
    ram[10624] = 8'b00000000;
    ram[10623] = 8'b00000000;
    ram[10622] = 8'b00000000;
    ram[10621] = 8'b00000000;
    ram[10620] = 8'b00000000;
    ram[10619] = 8'b00000000;
    ram[10618] = 8'b00000000;
    ram[10617] = 8'b00000000;
    ram[10616] = 8'b00000000;
    ram[10615] = 8'b00000000;
    ram[10614] = 8'b00000000;
    ram[10613] = 8'b00000000;
    ram[10612] = 8'b00000000;
    ram[10611] = 8'b00000000;
    ram[10610] = 8'b00000000;
    ram[10609] = 8'b00000000;
    ram[10608] = 8'b00000000;
    ram[10607] = 8'b00000000;
    ram[10606] = 8'b00000000;
    ram[10605] = 8'b00000000;
    ram[10604] = 8'b00000000;
    ram[10603] = 8'b00000000;
    ram[10602] = 8'b00000000;
    ram[10601] = 8'b00000000;
    ram[10600] = 8'b00000000;
    ram[10599] = 8'b00000000;
    ram[10598] = 8'b00000000;
    ram[10597] = 8'b00000000;
    ram[10596] = 8'b00000000;
    ram[10595] = 8'b00000000;
    ram[10594] = 8'b00000000;
    ram[10593] = 8'b00000000;
    ram[10592] = 8'b00000000;
    ram[10591] = 8'b00000000;
    ram[10590] = 8'b00000000;
    ram[10589] = 8'b00000000;
    ram[10588] = 8'b00000000;
    ram[10587] = 8'b00000000;
    ram[10586] = 8'b00000000;
    ram[10585] = 8'b00000000;
    ram[10584] = 8'b00000000;
    ram[10583] = 8'b00000000;
    ram[10582] = 8'b00000000;
    ram[10581] = 8'b00000000;
    ram[10580] = 8'b00000000;
    ram[10579] = 8'b00000000;
    ram[10578] = 8'b00000000;
    ram[10577] = 8'b00000000;
    ram[10576] = 8'b00000000;
    ram[10575] = 8'b00000000;
    ram[10574] = 8'b00000000;
    ram[10573] = 8'b00000000;
    ram[10572] = 8'b00000000;
    ram[10571] = 8'b00000000;
    ram[10570] = 8'b00000000;
    ram[10569] = 8'b00000000;
    ram[10568] = 8'b00000000;
    ram[10567] = 8'b00000000;
    ram[10566] = 8'b00000000;
    ram[10565] = 8'b00000000;
    ram[10564] = 8'b00000000;
    ram[10563] = 8'b00000000;
    ram[10562] = 8'b00000000;
    ram[10561] = 8'b00000000;
    ram[10560] = 8'b00000000;
    ram[10559] = 8'b00000000;
    ram[10558] = 8'b00000000;
    ram[10557] = 8'b00000000;
    ram[10556] = 8'b00000000;
    ram[10555] = 8'b00000000;
    ram[10554] = 8'b00000000;
    ram[10553] = 8'b00000000;
    ram[10552] = 8'b00000000;
    ram[10551] = 8'b00000000;
    ram[10550] = 8'b00000000;
    ram[10549] = 8'b00000000;
    ram[10548] = 8'b00000000;
    ram[10547] = 8'b00000000;
    ram[10546] = 8'b00000000;
    ram[10545] = 8'b00000000;
    ram[10544] = 8'b00000000;
    ram[10543] = 8'b00000000;
    ram[10542] = 8'b00000000;
    ram[10541] = 8'b00000000;
    ram[10540] = 8'b00000000;
    ram[10539] = 8'b00000000;
    ram[10538] = 8'b00000000;
    ram[10537] = 8'b00000000;
    ram[10536] = 8'b00000000;
    ram[10535] = 8'b00000000;
    ram[10534] = 8'b00000000;
    ram[10533] = 8'b00000000;
    ram[10532] = 8'b00000000;
    ram[10531] = 8'b00000000;
    ram[10530] = 8'b00000000;
    ram[10529] = 8'b00000000;
    ram[10528] = 8'b00000000;
    ram[10527] = 8'b00000000;
    ram[10526] = 8'b00000000;
    ram[10525] = 8'b00000000;
    ram[10524] = 8'b00000000;
    ram[10523] = 8'b00000000;
    ram[10522] = 8'b00000000;
    ram[10521] = 8'b00000000;
    ram[10520] = 8'b00000000;
    ram[10519] = 8'b00000000;
    ram[10518] = 8'b00000000;
    ram[10517] = 8'b00000000;
    ram[10516] = 8'b00000000;
    ram[10515] = 8'b00000000;
    ram[10514] = 8'b00000000;
    ram[10513] = 8'b00000000;
    ram[10512] = 8'b00000000;
    ram[10511] = 8'b00000000;
    ram[10510] = 8'b00000000;
    ram[10509] = 8'b00000000;
    ram[10508] = 8'b00000000;
    ram[10507] = 8'b00000000;
    ram[10506] = 8'b00000000;
    ram[10505] = 8'b00000000;
    ram[10504] = 8'b00000000;
    ram[10503] = 8'b00000000;
    ram[10502] = 8'b00000000;
    ram[10501] = 8'b00000000;
    ram[10500] = 8'b00000000;
    ram[10499] = 8'b00000000;
    ram[10498] = 8'b00000000;
    ram[10497] = 8'b00000000;
    ram[10496] = 8'b00000000;
    ram[10495] = 8'b00000000;
    ram[10494] = 8'b00000000;
    ram[10493] = 8'b00000000;
    ram[10492] = 8'b00000000;
    ram[10491] = 8'b00000000;
    ram[10490] = 8'b00000000;
    ram[10489] = 8'b00000000;
    ram[10488] = 8'b00000000;
    ram[10487] = 8'b00000000;
    ram[10486] = 8'b00000000;
    ram[10485] = 8'b00000000;
    ram[10484] = 8'b00000000;
    ram[10483] = 8'b00000000;
    ram[10482] = 8'b00000000;
    ram[10481] = 8'b00000000;
    ram[10480] = 8'b00000000;
    ram[10479] = 8'b00000000;
    ram[10478] = 8'b00000000;
    ram[10477] = 8'b00000000;
    ram[10476] = 8'b00000000;
    ram[10475] = 8'b00000000;
    ram[10474] = 8'b00000000;
    ram[10473] = 8'b00000000;
    ram[10472] = 8'b00000000;
    ram[10471] = 8'b00000000;
    ram[10470] = 8'b00000000;
    ram[10469] = 8'b00000000;
    ram[10468] = 8'b00000000;
    ram[10467] = 8'b00000000;
    ram[10466] = 8'b00000000;
    ram[10465] = 8'b00000000;
    ram[10464] = 8'b00000000;
    ram[10463] = 8'b00000000;
    ram[10462] = 8'b00000000;
    ram[10461] = 8'b00000000;
    ram[10460] = 8'b00000000;
    ram[10459] = 8'b00000000;
    ram[10458] = 8'b00000000;
    ram[10457] = 8'b00000000;
    ram[10456] = 8'b00000000;
    ram[10455] = 8'b00000000;
    ram[10454] = 8'b00000000;
    ram[10453] = 8'b00000000;
    ram[10452] = 8'b00000000;
    ram[10451] = 8'b00000000;
    ram[10450] = 8'b00000000;
    ram[10449] = 8'b00000000;
    ram[10448] = 8'b00000000;
    ram[10447] = 8'b00000000;
    ram[10446] = 8'b00000000;
    ram[10445] = 8'b00000000;
    ram[10444] = 8'b00000000;
    ram[10443] = 8'b00000000;
    ram[10442] = 8'b00000000;
    ram[10441] = 8'b00000000;
    ram[10440] = 8'b00000000;
    ram[10439] = 8'b00000000;
    ram[10438] = 8'b00000000;
    ram[10437] = 8'b00000000;
    ram[10436] = 8'b00000000;
    ram[10435] = 8'b00000000;
    ram[10434] = 8'b00000000;
    ram[10433] = 8'b00000000;
    ram[10432] = 8'b00000000;
    ram[10431] = 8'b00000000;
    ram[10430] = 8'b00000000;
    ram[10429] = 8'b00000000;
    ram[10428] = 8'b00000000;
    ram[10427] = 8'b00000000;
    ram[10426] = 8'b00000000;
    ram[10425] = 8'b00000000;
    ram[10424] = 8'b00000000;
    ram[10423] = 8'b00000000;
    ram[10422] = 8'b00000000;
    ram[10421] = 8'b00000000;
    ram[10420] = 8'b00000000;
    ram[10419] = 8'b00000000;
    ram[10418] = 8'b00000000;
    ram[10417] = 8'b00000000;
    ram[10416] = 8'b00000000;
    ram[10415] = 8'b00000000;
    ram[10414] = 8'b00000000;
    ram[10413] = 8'b00000000;
    ram[10412] = 8'b00000000;
    ram[10411] = 8'b00000000;
    ram[10410] = 8'b00000000;
    ram[10409] = 8'b00000000;
    ram[10408] = 8'b00000000;
    ram[10407] = 8'b00000000;
    ram[10406] = 8'b00000000;
    ram[10405] = 8'b00000000;
    ram[10404] = 8'b00000000;
    ram[10403] = 8'b00000000;
    ram[10402] = 8'b00000000;
    ram[10401] = 8'b00000000;
    ram[10400] = 8'b00000000;
    ram[10399] = 8'b00000000;
    ram[10398] = 8'b00000000;
    ram[10397] = 8'b00000000;
    ram[10396] = 8'b00000000;
    ram[10395] = 8'b00000000;
    ram[10394] = 8'b00000000;
    ram[10393] = 8'b00000000;
    ram[10392] = 8'b00000000;
    ram[10391] = 8'b00000000;
    ram[10390] = 8'b00000000;
    ram[10389] = 8'b00000000;
    ram[10388] = 8'b00000000;
    ram[10387] = 8'b00000000;
    ram[10386] = 8'b00000000;
    ram[10385] = 8'b00000000;
    ram[10384] = 8'b00000000;
    ram[10383] = 8'b00000000;
    ram[10382] = 8'b00000000;
    ram[10381] = 8'b00000000;
    ram[10380] = 8'b00000000;
    ram[10379] = 8'b00000000;
    ram[10378] = 8'b00000000;
    ram[10377] = 8'b00000000;
    ram[10376] = 8'b00000000;
    ram[10375] = 8'b00000000;
    ram[10374] = 8'b00000000;
    ram[10373] = 8'b00000000;
    ram[10372] = 8'b00000000;
    ram[10371] = 8'b00000000;
    ram[10370] = 8'b00000000;
    ram[10369] = 8'b00000000;
    ram[10368] = 8'b00000000;
    ram[10367] = 8'b00000000;
    ram[10366] = 8'b00000000;
    ram[10365] = 8'b00000000;
    ram[10364] = 8'b00000000;
    ram[10363] = 8'b00000000;
    ram[10362] = 8'b00000000;
    ram[10361] = 8'b00000000;
    ram[10360] = 8'b00000000;
    ram[10359] = 8'b00000000;
    ram[10358] = 8'b00000000;
    ram[10357] = 8'b00000000;
    ram[10356] = 8'b00000000;
    ram[10355] = 8'b00000000;
    ram[10354] = 8'b00000000;
    ram[10353] = 8'b00000000;
    ram[10352] = 8'b00000000;
    ram[10351] = 8'b00000000;
    ram[10350] = 8'b00000000;
    ram[10349] = 8'b00000000;
    ram[10348] = 8'b00000000;
    ram[10347] = 8'b00000000;
    ram[10346] = 8'b00000000;
    ram[10345] = 8'b00000000;
    ram[10344] = 8'b00000000;
    ram[10343] = 8'b00000000;
    ram[10342] = 8'b00000000;
    ram[10341] = 8'b00000000;
    ram[10340] = 8'b00000000;
    ram[10339] = 8'b00000000;
    ram[10338] = 8'b00000000;
    ram[10337] = 8'b00000000;
    ram[10336] = 8'b00000000;
    ram[10335] = 8'b00000000;
    ram[10334] = 8'b00000000;
    ram[10333] = 8'b00000000;
    ram[10332] = 8'b00000000;
    ram[10331] = 8'b00000000;
    ram[10330] = 8'b00000000;
    ram[10329] = 8'b00000000;
    ram[10328] = 8'b00000000;
    ram[10327] = 8'b00000000;
    ram[10326] = 8'b00000000;
    ram[10325] = 8'b00000000;
    ram[10324] = 8'b00000000;
    ram[10323] = 8'b00000000;
    ram[10322] = 8'b00000000;
    ram[10321] = 8'b00000000;
    ram[10320] = 8'b00000000;
    ram[10319] = 8'b00000000;
    ram[10318] = 8'b00000000;
    ram[10317] = 8'b00000000;
    ram[10316] = 8'b00000000;
    ram[10315] = 8'b00000000;
    ram[10314] = 8'b00000000;
    ram[10313] = 8'b00000000;
    ram[10312] = 8'b00000000;
    ram[10311] = 8'b00000000;
    ram[10310] = 8'b00000000;
    ram[10309] = 8'b00000000;
    ram[10308] = 8'b00000000;
    ram[10307] = 8'b00000000;
    ram[10306] = 8'b00000000;
    ram[10305] = 8'b00000000;
    ram[10304] = 8'b00000000;
    ram[10303] = 8'b00000000;
    ram[10302] = 8'b00000000;
    ram[10301] = 8'b00000000;
    ram[10300] = 8'b00000000;
    ram[10299] = 8'b00000000;
    ram[10298] = 8'b00000000;
    ram[10297] = 8'b00000000;
    ram[10296] = 8'b00000000;
    ram[10295] = 8'b00000000;
    ram[10294] = 8'b00000000;
    ram[10293] = 8'b00000000;
    ram[10292] = 8'b00000000;
    ram[10291] = 8'b00000000;
    ram[10290] = 8'b00000000;
    ram[10289] = 8'b00000000;
    ram[10288] = 8'b00000000;
    ram[10287] = 8'b00000000;
    ram[10286] = 8'b00000000;
    ram[10285] = 8'b00000000;
    ram[10284] = 8'b00000000;
    ram[10283] = 8'b00000000;
    ram[10282] = 8'b00000000;
    ram[10281] = 8'b00000000;
    ram[10280] = 8'b00000000;
    ram[10279] = 8'b00000000;
    ram[10278] = 8'b00000000;
    ram[10277] = 8'b00000000;
    ram[10276] = 8'b00000000;
    ram[10275] = 8'b00000000;
    ram[10274] = 8'b00000000;
    ram[10273] = 8'b00000000;
    ram[10272] = 8'b00000000;
    ram[10271] = 8'b00000000;
    ram[10270] = 8'b00000000;
    ram[10269] = 8'b00000000;
    ram[10268] = 8'b00000000;
    ram[10267] = 8'b00000000;
    ram[10266] = 8'b00000000;
    ram[10265] = 8'b00000000;
    ram[10264] = 8'b00000000;
    ram[10263] = 8'b00000000;
    ram[10262] = 8'b00000000;
    ram[10261] = 8'b00000000;
    ram[10260] = 8'b00000000;
    ram[10259] = 8'b00000000;
    ram[10258] = 8'b00000000;
    ram[10257] = 8'b00000000;
    ram[10256] = 8'b00000000;
    ram[10255] = 8'b00000000;
    ram[10254] = 8'b00000000;
    ram[10253] = 8'b00000000;
    ram[10252] = 8'b00000000;
    ram[10251] = 8'b00000000;
    ram[10250] = 8'b00000000;
    ram[10249] = 8'b00000000;
    ram[10248] = 8'b00000000;
    ram[10247] = 8'b00000000;
    ram[10246] = 8'b00000000;
    ram[10245] = 8'b00000000;
    ram[10244] = 8'b00000000;
    ram[10243] = 8'b00000000;
    ram[10242] = 8'b00000000;
    ram[10241] = 8'b00000000;
    ram[10240] = 8'b00000000;
    ram[10239] = 8'b00000000;
    ram[10238] = 8'b00000000;
    ram[10237] = 8'b00000000;
    ram[10236] = 8'b00000000;
    ram[10235] = 8'b00000000;
    ram[10234] = 8'b00000000;
    ram[10233] = 8'b00000000;
    ram[10232] = 8'b00000000;
    ram[10231] = 8'b00000000;
    ram[10230] = 8'b00000000;
    ram[10229] = 8'b00000000;
    ram[10228] = 8'b00000000;
    ram[10227] = 8'b00000000;
    ram[10226] = 8'b00000000;
    ram[10225] = 8'b00000000;
    ram[10224] = 8'b00000000;
    ram[10223] = 8'b00000000;
    ram[10222] = 8'b00000000;
    ram[10221] = 8'b00000000;
    ram[10220] = 8'b00000000;
    ram[10219] = 8'b00000000;
    ram[10218] = 8'b00000000;
    ram[10217] = 8'b00000000;
    ram[10216] = 8'b00000000;
    ram[10215] = 8'b00000000;
    ram[10214] = 8'b00000000;
    ram[10213] = 8'b00000000;
    ram[10212] = 8'b00000000;
    ram[10211] = 8'b00000000;
    ram[10210] = 8'b00000000;
    ram[10209] = 8'b00000000;
    ram[10208] = 8'b00000000;
    ram[10207] = 8'b00000000;
    ram[10206] = 8'b00000000;
    ram[10205] = 8'b00000000;
    ram[10204] = 8'b00000000;
    ram[10203] = 8'b00000000;
    ram[10202] = 8'b00000000;
    ram[10201] = 8'b00000000;
    ram[10200] = 8'b00000000;
    ram[10199] = 8'b00000000;
    ram[10198] = 8'b00000000;
    ram[10197] = 8'b00000000;
    ram[10196] = 8'b00000000;
    ram[10195] = 8'b00000000;
    ram[10194] = 8'b00000000;
    ram[10193] = 8'b00000000;
    ram[10192] = 8'b00000000;
    ram[10191] = 8'b00000000;
    ram[10190] = 8'b00000000;
    ram[10189] = 8'b00000000;
    ram[10188] = 8'b00000000;
    ram[10187] = 8'b00000000;
    ram[10186] = 8'b00000000;
    ram[10185] = 8'b00000000;
    ram[10184] = 8'b00000000;
    ram[10183] = 8'b00000000;
    ram[10182] = 8'b00000000;
    ram[10181] = 8'b00000000;
    ram[10180] = 8'b00000000;
    ram[10179] = 8'b00000000;
    ram[10178] = 8'b00000000;
    ram[10177] = 8'b00000000;
    ram[10176] = 8'b00000000;
    ram[10175] = 8'b00000000;
    ram[10174] = 8'b00000000;
    ram[10173] = 8'b00000000;
    ram[10172] = 8'b00000000;
    ram[10171] = 8'b00000000;
    ram[10170] = 8'b00000000;
    ram[10169] = 8'b00000000;
    ram[10168] = 8'b00000000;
    ram[10167] = 8'b00000000;
    ram[10166] = 8'b00000000;
    ram[10165] = 8'b00000000;
    ram[10164] = 8'b00000000;
    ram[10163] = 8'b00000000;
    ram[10162] = 8'b00000000;
    ram[10161] = 8'b00000000;
    ram[10160] = 8'b00000000;
    ram[10159] = 8'b00000000;
    ram[10158] = 8'b00000000;
    ram[10157] = 8'b00000000;
    ram[10156] = 8'b00000000;
    ram[10155] = 8'b00000000;
    ram[10154] = 8'b00000000;
    ram[10153] = 8'b00000000;
    ram[10152] = 8'b00000000;
    ram[10151] = 8'b00000000;
    ram[10150] = 8'b00000000;
    ram[10149] = 8'b00000000;
    ram[10148] = 8'b00000000;
    ram[10147] = 8'b00000000;
    ram[10146] = 8'b00000000;
    ram[10145] = 8'b00000000;
    ram[10144] = 8'b00000000;
    ram[10143] = 8'b00000000;
    ram[10142] = 8'b00000000;
    ram[10141] = 8'b00000000;
    ram[10140] = 8'b00000000;
    ram[10139] = 8'b00000000;
    ram[10138] = 8'b00000000;
    ram[10137] = 8'b00000000;
    ram[10136] = 8'b00000000;
    ram[10135] = 8'b00000000;
    ram[10134] = 8'b00000000;
    ram[10133] = 8'b00000000;
    ram[10132] = 8'b00000000;
    ram[10131] = 8'b00000000;
    ram[10130] = 8'b00000000;
    ram[10129] = 8'b00000000;
    ram[10128] = 8'b00000000;
    ram[10127] = 8'b00000000;
    ram[10126] = 8'b00000000;
    ram[10125] = 8'b00000000;
    ram[10124] = 8'b00000000;
    ram[10123] = 8'b00000000;
    ram[10122] = 8'b00000000;
    ram[10121] = 8'b00000000;
    ram[10120] = 8'b00000000;
    ram[10119] = 8'b00000000;
    ram[10118] = 8'b00000000;
    ram[10117] = 8'b00000000;
    ram[10116] = 8'b00000000;
    ram[10115] = 8'b00000000;
    ram[10114] = 8'b00000000;
    ram[10113] = 8'b00000000;
    ram[10112] = 8'b00000000;
    ram[10111] = 8'b00000000;
    ram[10110] = 8'b00000000;
    ram[10109] = 8'b00000000;
    ram[10108] = 8'b00000000;
    ram[10107] = 8'b00000000;
    ram[10106] = 8'b00000000;
    ram[10105] = 8'b00000000;
    ram[10104] = 8'b00000000;
    ram[10103] = 8'b00000000;
    ram[10102] = 8'b00000000;
    ram[10101] = 8'b00000000;
    ram[10100] = 8'b00000000;
    ram[10099] = 8'b00000000;
    ram[10098] = 8'b00000000;
    ram[10097] = 8'b00000000;
    ram[10096] = 8'b00000000;
    ram[10095] = 8'b00000000;
    ram[10094] = 8'b00000000;
    ram[10093] = 8'b00000000;
    ram[10092] = 8'b00000000;
    ram[10091] = 8'b00000000;
    ram[10090] = 8'b00000000;
    ram[10089] = 8'b00000000;
    ram[10088] = 8'b00000000;
    ram[10087] = 8'b00000000;
    ram[10086] = 8'b00000000;
    ram[10085] = 8'b00000000;
    ram[10084] = 8'b00000000;
    ram[10083] = 8'b00000000;
    ram[10082] = 8'b00000000;
    ram[10081] = 8'b00000000;
    ram[10080] = 8'b00000000;
    ram[10079] = 8'b00000000;
    ram[10078] = 8'b00000000;
    ram[10077] = 8'b00000000;
    ram[10076] = 8'b00000000;
    ram[10075] = 8'b00000000;
    ram[10074] = 8'b00000000;
    ram[10073] = 8'b00000000;
    ram[10072] = 8'b00000000;
    ram[10071] = 8'b00000000;
    ram[10070] = 8'b00000000;
    ram[10069] = 8'b00000000;
    ram[10068] = 8'b00000000;
    ram[10067] = 8'b00000000;
    ram[10066] = 8'b00000000;
    ram[10065] = 8'b00000000;
    ram[10064] = 8'b00000000;
    ram[10063] = 8'b00000000;
    ram[10062] = 8'b00000000;
    ram[10061] = 8'b00000000;
    ram[10060] = 8'b00000000;
    ram[10059] = 8'b00000000;
    ram[10058] = 8'b00000000;
    ram[10057] = 8'b00000000;
    ram[10056] = 8'b00000000;
    ram[10055] = 8'b00000000;
    ram[10054] = 8'b00000000;
    ram[10053] = 8'b00000000;
    ram[10052] = 8'b00000000;
    ram[10051] = 8'b00000000;
    ram[10050] = 8'b00000000;
    ram[10049] = 8'b00000000;
    ram[10048] = 8'b00000000;
    ram[10047] = 8'b00000000;
    ram[10046] = 8'b00000000;
    ram[10045] = 8'b00000000;
    ram[10044] = 8'b00000000;
    ram[10043] = 8'b00000000;
    ram[10042] = 8'b00000000;
    ram[10041] = 8'b00000000;
    ram[10040] = 8'b00000000;
    ram[10039] = 8'b00000000;
    ram[10038] = 8'b00000000;
    ram[10037] = 8'b00000000;
    ram[10036] = 8'b00000000;
    ram[10035] = 8'b00000000;
    ram[10034] = 8'b00000000;
    ram[10033] = 8'b00000000;
    ram[10032] = 8'b00000000;
    ram[10031] = 8'b00000000;
    ram[10030] = 8'b00000000;
    ram[10029] = 8'b00000000;
    ram[10028] = 8'b00000000;
    ram[10027] = 8'b00000000;
    ram[10026] = 8'b00000000;
    ram[10025] = 8'b00000000;
    ram[10024] = 8'b00000000;
    ram[10023] = 8'b00000000;
    ram[10022] = 8'b00000000;
    ram[10021] = 8'b00000000;
    ram[10020] = 8'b00000000;
    ram[10019] = 8'b00000000;
    ram[10018] = 8'b00000000;
    ram[10017] = 8'b00000000;
    ram[10016] = 8'b00000000;
    ram[10015] = 8'b00000000;
    ram[10014] = 8'b00000000;
    ram[10013] = 8'b00000000;
    ram[10012] = 8'b00000000;
    ram[10011] = 8'b00000000;
    ram[10010] = 8'b00000000;
    ram[10009] = 8'b00000000;
    ram[10008] = 8'b00000000;
    ram[10007] = 8'b00000000;
    ram[10006] = 8'b00000000;
    ram[10005] = 8'b00000000;
    ram[10004] = 8'b00000000;
    ram[10003] = 8'b00000000;
    ram[10002] = 8'b00000000;
    ram[10001] = 8'b00000000;
    ram[10000] = 8'b00000000;
    ram[9999] = 8'b00000000;
    ram[9998] = 8'b00000000;
    ram[9997] = 8'b00000000;
    ram[9996] = 8'b00000000;
    ram[9995] = 8'b00000000;
    ram[9994] = 8'b00000000;
    ram[9993] = 8'b00000000;
    ram[9992] = 8'b00000000;
    ram[9991] = 8'b00000000;
    ram[9990] = 8'b00000000;
    ram[9989] = 8'b00000000;
    ram[9988] = 8'b00000000;
    ram[9987] = 8'b00000000;
    ram[9986] = 8'b00000000;
    ram[9985] = 8'b00000000;
    ram[9984] = 8'b00000000;
    ram[9983] = 8'b00000000;
    ram[9982] = 8'b00000000;
    ram[9981] = 8'b00000000;
    ram[9980] = 8'b00000000;
    ram[9979] = 8'b00000000;
    ram[9978] = 8'b00000000;
    ram[9977] = 8'b00000000;
    ram[9976] = 8'b00000000;
    ram[9975] = 8'b00000000;
    ram[9974] = 8'b00000000;
    ram[9973] = 8'b00000000;
    ram[9972] = 8'b00000000;
    ram[9971] = 8'b00000000;
    ram[9970] = 8'b00000000;
    ram[9969] = 8'b00000000;
    ram[9968] = 8'b00000000;
    ram[9967] = 8'b00000000;
    ram[9966] = 8'b00000000;
    ram[9965] = 8'b00000000;
    ram[9964] = 8'b00000000;
    ram[9963] = 8'b00000000;
    ram[9962] = 8'b00000000;
    ram[9961] = 8'b00000000;
    ram[9960] = 8'b00000000;
    ram[9959] = 8'b00000000;
    ram[9958] = 8'b00000000;
    ram[9957] = 8'b00000000;
    ram[9956] = 8'b00000000;
    ram[9955] = 8'b00000000;
    ram[9954] = 8'b00000000;
    ram[9953] = 8'b00000000;
    ram[9952] = 8'b00000000;
    ram[9951] = 8'b00000000;
    ram[9950] = 8'b00000000;
    ram[9949] = 8'b00000000;
    ram[9948] = 8'b00000000;
    ram[9947] = 8'b00000000;
    ram[9946] = 8'b00000000;
    ram[9945] = 8'b00000000;
    ram[9944] = 8'b00000000;
    ram[9943] = 8'b00000000;
    ram[9942] = 8'b00000000;
    ram[9941] = 8'b00000000;
    ram[9940] = 8'b00000000;
    ram[9939] = 8'b00000000;
    ram[9938] = 8'b00000000;
    ram[9937] = 8'b00000000;
    ram[9936] = 8'b00000000;
    ram[9935] = 8'b00000000;
    ram[9934] = 8'b00000000;
    ram[9933] = 8'b00000000;
    ram[9932] = 8'b00000000;
    ram[9931] = 8'b00000000;
    ram[9930] = 8'b00000000;
    ram[9929] = 8'b00000000;
    ram[9928] = 8'b00000000;
    ram[9927] = 8'b00000000;
    ram[9926] = 8'b00000000;
    ram[9925] = 8'b00000000;
    ram[9924] = 8'b00000000;
    ram[9923] = 8'b00000000;
    ram[9922] = 8'b00000000;
    ram[9921] = 8'b00000000;
    ram[9920] = 8'b00000000;
    ram[9919] = 8'b00000000;
    ram[9918] = 8'b00000000;
    ram[9917] = 8'b00000000;
    ram[9916] = 8'b00000000;
    ram[9915] = 8'b00000000;
    ram[9914] = 8'b00000000;
    ram[9913] = 8'b00000000;
    ram[9912] = 8'b00000000;
    ram[9911] = 8'b00000000;
    ram[9910] = 8'b00000000;
    ram[9909] = 8'b00000000;
    ram[9908] = 8'b00000000;
    ram[9907] = 8'b00000000;
    ram[9906] = 8'b00000000;
    ram[9905] = 8'b00000000;
    ram[9904] = 8'b00000000;
    ram[9903] = 8'b00000000;
    ram[9902] = 8'b00000000;
    ram[9901] = 8'b00000000;
    ram[9900] = 8'b00000000;
    ram[9899] = 8'b00000000;
    ram[9898] = 8'b00000000;
    ram[9897] = 8'b00000000;
    ram[9896] = 8'b00000000;
    ram[9895] = 8'b00000000;
    ram[9894] = 8'b00000000;
    ram[9893] = 8'b00000000;
    ram[9892] = 8'b00000000;
    ram[9891] = 8'b00000000;
    ram[9890] = 8'b00000000;
    ram[9889] = 8'b00000000;
    ram[9888] = 8'b00000000;
    ram[9887] = 8'b00000000;
    ram[9886] = 8'b00000000;
    ram[9885] = 8'b00000000;
    ram[9884] = 8'b00000000;
    ram[9883] = 8'b00000000;
    ram[9882] = 8'b00000000;
    ram[9881] = 8'b00000000;
    ram[9880] = 8'b00000000;
    ram[9879] = 8'b00000000;
    ram[9878] = 8'b00000000;
    ram[9877] = 8'b00000000;
    ram[9876] = 8'b00000000;
    ram[9875] = 8'b00000000;
    ram[9874] = 8'b00000000;
    ram[9873] = 8'b00000000;
    ram[9872] = 8'b00000000;
    ram[9871] = 8'b00000000;
    ram[9870] = 8'b00000000;
    ram[9869] = 8'b00000000;
    ram[9868] = 8'b00000000;
    ram[9867] = 8'b00000000;
    ram[9866] = 8'b00000000;
    ram[9865] = 8'b00000000;
    ram[9864] = 8'b00000000;
    ram[9863] = 8'b00000000;
    ram[9862] = 8'b00000000;
    ram[9861] = 8'b00000000;
    ram[9860] = 8'b00000000;
    ram[9859] = 8'b00000000;
    ram[9858] = 8'b00000000;
    ram[9857] = 8'b00000000;
    ram[9856] = 8'b00000000;
    ram[9855] = 8'b00000000;
    ram[9854] = 8'b00000000;
    ram[9853] = 8'b00000000;
    ram[9852] = 8'b00000000;
    ram[9851] = 8'b00000000;
    ram[9850] = 8'b00000000;
    ram[9849] = 8'b00000000;
    ram[9848] = 8'b00000000;
    ram[9847] = 8'b00000000;
    ram[9846] = 8'b00000000;
    ram[9845] = 8'b00000000;
    ram[9844] = 8'b00000000;
    ram[9843] = 8'b00000000;
    ram[9842] = 8'b00000000;
    ram[9841] = 8'b00000000;
    ram[9840] = 8'b00000000;
    ram[9839] = 8'b00000000;
    ram[9838] = 8'b00000000;
    ram[9837] = 8'b00000000;
    ram[9836] = 8'b00000000;
    ram[9835] = 8'b00000000;
    ram[9834] = 8'b00000000;
    ram[9833] = 8'b00000000;
    ram[9832] = 8'b00000000;
    ram[9831] = 8'b00000000;
    ram[9830] = 8'b00000000;
    ram[9829] = 8'b00000000;
    ram[9828] = 8'b00000000;
    ram[9827] = 8'b00000000;
    ram[9826] = 8'b00000000;
    ram[9825] = 8'b00000000;
    ram[9824] = 8'b00000000;
    ram[9823] = 8'b00000000;
    ram[9822] = 8'b00000000;
    ram[9821] = 8'b00000000;
    ram[9820] = 8'b00000000;
    ram[9819] = 8'b00000000;
    ram[9818] = 8'b00000000;
    ram[9817] = 8'b00000000;
    ram[9816] = 8'b00000000;
    ram[9815] = 8'b00000000;
    ram[9814] = 8'b00000000;
    ram[9813] = 8'b00000000;
    ram[9812] = 8'b00000000;
    ram[9811] = 8'b00000000;
    ram[9810] = 8'b00000000;
    ram[9809] = 8'b00000000;
    ram[9808] = 8'b00000000;
    ram[9807] = 8'b00000000;
    ram[9806] = 8'b00000000;
    ram[9805] = 8'b00000000;
    ram[9804] = 8'b00000000;
    ram[9803] = 8'b00000000;
    ram[9802] = 8'b00000000;
    ram[9801] = 8'b00000000;
    ram[9800] = 8'b00000000;
    ram[9799] = 8'b00000000;
    ram[9798] = 8'b00000000;
    ram[9797] = 8'b00000000;
    ram[9796] = 8'b00000000;
    ram[9795] = 8'b00000000;
    ram[9794] = 8'b00000000;
    ram[9793] = 8'b00000000;
    ram[9792] = 8'b00000000;
    ram[9791] = 8'b00000000;
    ram[9790] = 8'b00000000;
    ram[9789] = 8'b00000000;
    ram[9788] = 8'b00000000;
    ram[9787] = 8'b00000000;
    ram[9786] = 8'b00000000;
    ram[9785] = 8'b00000000;
    ram[9784] = 8'b00000000;
    ram[9783] = 8'b00000000;
    ram[9782] = 8'b00000000;
    ram[9781] = 8'b00000000;
    ram[9780] = 8'b00000000;
    ram[9779] = 8'b00000000;
    ram[9778] = 8'b00000000;
    ram[9777] = 8'b00000000;
    ram[9776] = 8'b00000000;
    ram[9775] = 8'b00000000;
    ram[9774] = 8'b00000000;
    ram[9773] = 8'b00000000;
    ram[9772] = 8'b00000000;
    ram[9771] = 8'b00000000;
    ram[9770] = 8'b00000000;
    ram[9769] = 8'b00000000;
    ram[9768] = 8'b00000000;
    ram[9767] = 8'b00000000;
    ram[9766] = 8'b00000000;
    ram[9765] = 8'b00000000;
    ram[9764] = 8'b00000000;
    ram[9763] = 8'b00000000;
    ram[9762] = 8'b00000000;
    ram[9761] = 8'b00000000;
    ram[9760] = 8'b00000000;
    ram[9759] = 8'b00000000;
    ram[9758] = 8'b00000000;
    ram[9757] = 8'b00000000;
    ram[9756] = 8'b00000000;
    ram[9755] = 8'b00000000;
    ram[9754] = 8'b00000000;
    ram[9753] = 8'b00000000;
    ram[9752] = 8'b00000000;
    ram[9751] = 8'b00000000;
    ram[9750] = 8'b00000000;
    ram[9749] = 8'b00000000;
    ram[9748] = 8'b00000000;
    ram[9747] = 8'b00000000;
    ram[9746] = 8'b00000000;
    ram[9745] = 8'b00000000;
    ram[9744] = 8'b00000000;
    ram[9743] = 8'b00000000;
    ram[9742] = 8'b00000000;
    ram[9741] = 8'b00000000;
    ram[9740] = 8'b00000000;
    ram[9739] = 8'b00000000;
    ram[9738] = 8'b00000000;
    ram[9737] = 8'b00000000;
    ram[9736] = 8'b00000000;
    ram[9735] = 8'b00000000;
    ram[9734] = 8'b00000000;
    ram[9733] = 8'b00000000;
    ram[9732] = 8'b00000000;
    ram[9731] = 8'b00000000;
    ram[9730] = 8'b00000000;
    ram[9729] = 8'b00000000;
    ram[9728] = 8'b00000000;
    ram[9727] = 8'b00000000;
    ram[9726] = 8'b00000000;
    ram[9725] = 8'b00000000;
    ram[9724] = 8'b00000000;
    ram[9723] = 8'b00000000;
    ram[9722] = 8'b00000000;
    ram[9721] = 8'b00000000;
    ram[9720] = 8'b00000000;
    ram[9719] = 8'b00000000;
    ram[9718] = 8'b00000000;
    ram[9717] = 8'b00000000;
    ram[9716] = 8'b00000000;
    ram[9715] = 8'b00000000;
    ram[9714] = 8'b00000000;
    ram[9713] = 8'b00000000;
    ram[9712] = 8'b00000000;
    ram[9711] = 8'b00000000;
    ram[9710] = 8'b00000000;
    ram[9709] = 8'b00000000;
    ram[9708] = 8'b00000000;
    ram[9707] = 8'b00000000;
    ram[9706] = 8'b00000000;
    ram[9705] = 8'b00000000;
    ram[9704] = 8'b00000000;
    ram[9703] = 8'b00000000;
    ram[9702] = 8'b00000000;
    ram[9701] = 8'b00000000;
    ram[9700] = 8'b00000000;
    ram[9699] = 8'b00000000;
    ram[9698] = 8'b00000000;
    ram[9697] = 8'b00000000;
    ram[9696] = 8'b00000000;
    ram[9695] = 8'b00000000;
    ram[9694] = 8'b00000000;
    ram[9693] = 8'b00000000;
    ram[9692] = 8'b00000000;
    ram[9691] = 8'b00000000;
    ram[9690] = 8'b00000000;
    ram[9689] = 8'b00000000;
    ram[9688] = 8'b00000000;
    ram[9687] = 8'b00000000;
    ram[9686] = 8'b00000000;
    ram[9685] = 8'b00000000;
    ram[9684] = 8'b00000000;
    ram[9683] = 8'b00000000;
    ram[9682] = 8'b00000000;
    ram[9681] = 8'b00000000;
    ram[9680] = 8'b00000000;
    ram[9679] = 8'b00000000;
    ram[9678] = 8'b00000000;
    ram[9677] = 8'b00000000;
    ram[9676] = 8'b00000000;
    ram[9675] = 8'b00000000;
    ram[9674] = 8'b00000000;
    ram[9673] = 8'b00000000;
    ram[9672] = 8'b00000000;
    ram[9671] = 8'b00000000;
    ram[9670] = 8'b00000000;
    ram[9669] = 8'b00000000;
    ram[9668] = 8'b00000000;
    ram[9667] = 8'b00000000;
    ram[9666] = 8'b00000000;
    ram[9665] = 8'b00000000;
    ram[9664] = 8'b00000000;
    ram[9663] = 8'b00000000;
    ram[9662] = 8'b00000000;
    ram[9661] = 8'b00000000;
    ram[9660] = 8'b00000000;
    ram[9659] = 8'b00000000;
    ram[9658] = 8'b00000000;
    ram[9657] = 8'b00000000;
    ram[9656] = 8'b00000000;
    ram[9655] = 8'b00000000;
    ram[9654] = 8'b00000000;
    ram[9653] = 8'b00000000;
    ram[9652] = 8'b00000000;
    ram[9651] = 8'b00000000;
    ram[9650] = 8'b00000000;
    ram[9649] = 8'b00000000;
    ram[9648] = 8'b00000000;
    ram[9647] = 8'b00000000;
    ram[9646] = 8'b00000000;
    ram[9645] = 8'b00000000;
    ram[9644] = 8'b00000000;
    ram[9643] = 8'b00000000;
    ram[9642] = 8'b00000000;
    ram[9641] = 8'b00000000;
    ram[9640] = 8'b00000000;
    ram[9639] = 8'b00000000;
    ram[9638] = 8'b00000000;
    ram[9637] = 8'b00000000;
    ram[9636] = 8'b00000000;
    ram[9635] = 8'b00000000;
    ram[9634] = 8'b00000000;
    ram[9633] = 8'b00000000;
    ram[9632] = 8'b00000000;
    ram[9631] = 8'b00000000;
    ram[9630] = 8'b00000000;
    ram[9629] = 8'b00000000;
    ram[9628] = 8'b00000000;
    ram[9627] = 8'b00000000;
    ram[9626] = 8'b00000000;
    ram[9625] = 8'b00000000;
    ram[9624] = 8'b00000000;
    ram[9623] = 8'b00000000;
    ram[9622] = 8'b00000000;
    ram[9621] = 8'b00000000;
    ram[9620] = 8'b00000000;
    ram[9619] = 8'b00000000;
    ram[9618] = 8'b00000000;
    ram[9617] = 8'b00000000;
    ram[9616] = 8'b00000000;
    ram[9615] = 8'b00000000;
    ram[9614] = 8'b00000000;
    ram[9613] = 8'b00000000;
    ram[9612] = 8'b00000000;
    ram[9611] = 8'b00000000;
    ram[9610] = 8'b00000000;
    ram[9609] = 8'b00000000;
    ram[9608] = 8'b00000000;
    ram[9607] = 8'b00000000;
    ram[9606] = 8'b00000000;
    ram[9605] = 8'b00000000;
    ram[9604] = 8'b00000000;
    ram[9603] = 8'b00000000;
    ram[9602] = 8'b00000000;
    ram[9601] = 8'b00000000;
    ram[9600] = 8'b00000000;
    ram[9599] = 8'b00000000;
    ram[9598] = 8'b00000000;
    ram[9597] = 8'b00000000;
    ram[9596] = 8'b00000000;
    ram[9595] = 8'b00000000;
    ram[9594] = 8'b00000000;
    ram[9593] = 8'b00000000;
    ram[9592] = 8'b00000000;
    ram[9591] = 8'b00000000;
    ram[9590] = 8'b00000000;
    ram[9589] = 8'b00000000;
    ram[9588] = 8'b00000000;
    ram[9587] = 8'b00000000;
    ram[9586] = 8'b00000000;
    ram[9585] = 8'b00000000;
    ram[9584] = 8'b00000000;
    ram[9583] = 8'b00000000;
    ram[9582] = 8'b00000000;
    ram[9581] = 8'b00000000;
    ram[9580] = 8'b00000000;
    ram[9579] = 8'b00000000;
    ram[9578] = 8'b00000000;
    ram[9577] = 8'b00000000;
    ram[9576] = 8'b00000000;
    ram[9575] = 8'b00000000;
    ram[9574] = 8'b00000000;
    ram[9573] = 8'b00000000;
    ram[9572] = 8'b00000000;
    ram[9571] = 8'b00000000;
    ram[9570] = 8'b00000000;
    ram[9569] = 8'b00000000;
    ram[9568] = 8'b00000000;
    ram[9567] = 8'b00000000;
    ram[9566] = 8'b00000000;
    ram[9565] = 8'b00000000;
    ram[9564] = 8'b00000000;
    ram[9563] = 8'b00000000;
    ram[9562] = 8'b00000000;
    ram[9561] = 8'b00000000;
    ram[9560] = 8'b00000000;
    ram[9559] = 8'b00000000;
    ram[9558] = 8'b00000000;
    ram[9557] = 8'b00000000;
    ram[9556] = 8'b00000000;
    ram[9555] = 8'b00000000;
    ram[9554] = 8'b00000000;
    ram[9553] = 8'b00000000;
    ram[9552] = 8'b00000000;
    ram[9551] = 8'b00000000;
    ram[9550] = 8'b00000000;
    ram[9549] = 8'b00000000;
    ram[9548] = 8'b00000000;
    ram[9547] = 8'b00000000;
    ram[9546] = 8'b00000000;
    ram[9545] = 8'b00000000;
    ram[9544] = 8'b00000000;
    ram[9543] = 8'b00000000;
    ram[9542] = 8'b00000000;
    ram[9541] = 8'b00000000;
    ram[9540] = 8'b00000000;
    ram[9539] = 8'b00000000;
    ram[9538] = 8'b00000000;
    ram[9537] = 8'b00000000;
    ram[9536] = 8'b00000000;
    ram[9535] = 8'b00000000;
    ram[9534] = 8'b00000000;
    ram[9533] = 8'b00000000;
    ram[9532] = 8'b00000000;
    ram[9531] = 8'b00000000;
    ram[9530] = 8'b00000000;
    ram[9529] = 8'b00000000;
    ram[9528] = 8'b00000000;
    ram[9527] = 8'b00000000;
    ram[9526] = 8'b00000000;
    ram[9525] = 8'b00000000;
    ram[9524] = 8'b00000000;
    ram[9523] = 8'b00000000;
    ram[9522] = 8'b00000000;
    ram[9521] = 8'b00000000;
    ram[9520] = 8'b00000000;
    ram[9519] = 8'b00000000;
    ram[9518] = 8'b00000000;
    ram[9517] = 8'b00000000;
    ram[9516] = 8'b00000000;
    ram[9515] = 8'b00000000;
    ram[9514] = 8'b00000000;
    ram[9513] = 8'b00000000;
    ram[9512] = 8'b00000000;
    ram[9511] = 8'b00000000;
    ram[9510] = 8'b00000000;
    ram[9509] = 8'b00000000;
    ram[9508] = 8'b00000000;
    ram[9507] = 8'b00000000;
    ram[9506] = 8'b00000000;
    ram[9505] = 8'b00000000;
    ram[9504] = 8'b00000000;
    ram[9503] = 8'b00000000;
    ram[9502] = 8'b00000000;
    ram[9501] = 8'b00000000;
    ram[9500] = 8'b00000000;
    ram[9499] = 8'b00000000;
    ram[9498] = 8'b00000000;
    ram[9497] = 8'b00000000;
    ram[9496] = 8'b00000000;
    ram[9495] = 8'b00000000;
    ram[9494] = 8'b00000000;
    ram[9493] = 8'b00000000;
    ram[9492] = 8'b00000000;
    ram[9491] = 8'b00000000;
    ram[9490] = 8'b00000000;
    ram[9489] = 8'b00000000;
    ram[9488] = 8'b00000000;
    ram[9487] = 8'b00000000;
    ram[9486] = 8'b00000000;
    ram[9485] = 8'b00000000;
    ram[9484] = 8'b00000000;
    ram[9483] = 8'b00000000;
    ram[9482] = 8'b00000000;
    ram[9481] = 8'b00000000;
    ram[9480] = 8'b00000000;
    ram[9479] = 8'b00000000;
    ram[9478] = 8'b00000000;
    ram[9477] = 8'b00000000;
    ram[9476] = 8'b00000000;
    ram[9475] = 8'b00000000;
    ram[9474] = 8'b00000000;
    ram[9473] = 8'b00000000;
    ram[9472] = 8'b00000000;
    ram[9471] = 8'b00000000;
    ram[9470] = 8'b00000000;
    ram[9469] = 8'b00000000;
    ram[9468] = 8'b00000000;
    ram[9467] = 8'b00000000;
    ram[9466] = 8'b00000000;
    ram[9465] = 8'b00000000;
    ram[9464] = 8'b00000000;
    ram[9463] = 8'b00000000;
    ram[9462] = 8'b00000000;
    ram[9461] = 8'b00000000;
    ram[9460] = 8'b00000000;
    ram[9459] = 8'b00000000;
    ram[9458] = 8'b00000000;
    ram[9457] = 8'b00000000;
    ram[9456] = 8'b00000000;
    ram[9455] = 8'b00000000;
    ram[9454] = 8'b00000000;
    ram[9453] = 8'b00000000;
    ram[9452] = 8'b00000000;
    ram[9451] = 8'b00000000;
    ram[9450] = 8'b00000000;
    ram[9449] = 8'b00000000;
    ram[9448] = 8'b00000000;
    ram[9447] = 8'b00000000;
    ram[9446] = 8'b00000000;
    ram[9445] = 8'b00000000;
    ram[9444] = 8'b00000000;
    ram[9443] = 8'b00000000;
    ram[9442] = 8'b00000000;
    ram[9441] = 8'b00000000;
    ram[9440] = 8'b00000000;
    ram[9439] = 8'b00000000;
    ram[9438] = 8'b00000000;
    ram[9437] = 8'b00000000;
    ram[9436] = 8'b00000000;
    ram[9435] = 8'b00000000;
    ram[9434] = 8'b00000000;
    ram[9433] = 8'b00000000;
    ram[9432] = 8'b00000000;
    ram[9431] = 8'b00000000;
    ram[9430] = 8'b00000000;
    ram[9429] = 8'b00000000;
    ram[9428] = 8'b00000000;
    ram[9427] = 8'b00000000;
    ram[9426] = 8'b00000000;
    ram[9425] = 8'b00000000;
    ram[9424] = 8'b00000000;
    ram[9423] = 8'b00000000;
    ram[9422] = 8'b00000000;
    ram[9421] = 8'b00000000;
    ram[9420] = 8'b00000000;
    ram[9419] = 8'b00000000;
    ram[9418] = 8'b00000000;
    ram[9417] = 8'b00000000;
    ram[9416] = 8'b00000000;
    ram[9415] = 8'b00000000;
    ram[9414] = 8'b00000000;
    ram[9413] = 8'b00000000;
    ram[9412] = 8'b00000000;
    ram[9411] = 8'b00000000;
    ram[9410] = 8'b00000000;
    ram[9409] = 8'b00000000;
    ram[9408] = 8'b00000000;
    ram[9407] = 8'b00000000;
    ram[9406] = 8'b00000000;
    ram[9405] = 8'b00000000;
    ram[9404] = 8'b00000000;
    ram[9403] = 8'b00000000;
    ram[9402] = 8'b00000000;
    ram[9401] = 8'b00000000;
    ram[9400] = 8'b00000000;
    ram[9399] = 8'b00000000;
    ram[9398] = 8'b00000000;
    ram[9397] = 8'b00000000;
    ram[9396] = 8'b00000000;
    ram[9395] = 8'b00000000;
    ram[9394] = 8'b00000000;
    ram[9393] = 8'b00000000;
    ram[9392] = 8'b00000000;
    ram[9391] = 8'b00000000;
    ram[9390] = 8'b00000000;
    ram[9389] = 8'b00000000;
    ram[9388] = 8'b00000000;
    ram[9387] = 8'b00000000;
    ram[9386] = 8'b00000000;
    ram[9385] = 8'b00000000;
    ram[9384] = 8'b00000000;
    ram[9383] = 8'b00000000;
    ram[9382] = 8'b00000000;
    ram[9381] = 8'b00000000;
    ram[9380] = 8'b00000000;
    ram[9379] = 8'b00000000;
    ram[9378] = 8'b00000000;
    ram[9377] = 8'b00000000;
    ram[9376] = 8'b00000000;
    ram[9375] = 8'b00000000;
    ram[9374] = 8'b00000000;
    ram[9373] = 8'b00000000;
    ram[9372] = 8'b00000000;
    ram[9371] = 8'b00000000;
    ram[9370] = 8'b00000000;
    ram[9369] = 8'b00000000;
    ram[9368] = 8'b00000000;
    ram[9367] = 8'b00000000;
    ram[9366] = 8'b00000000;
    ram[9365] = 8'b00000000;
    ram[9364] = 8'b00000000;
    ram[9363] = 8'b00000000;
    ram[9362] = 8'b00000000;
    ram[9361] = 8'b00000000;
    ram[9360] = 8'b00000000;
    ram[9359] = 8'b00000000;
    ram[9358] = 8'b00000000;
    ram[9357] = 8'b00000000;
    ram[9356] = 8'b00000000;
    ram[9355] = 8'b00000000;
    ram[9354] = 8'b00000000;
    ram[9353] = 8'b00000000;
    ram[9352] = 8'b00000000;
    ram[9351] = 8'b00000000;
    ram[9350] = 8'b00000000;
    ram[9349] = 8'b00000000;
    ram[9348] = 8'b00000000;
    ram[9347] = 8'b00000000;
    ram[9346] = 8'b00000000;
    ram[9345] = 8'b00000000;
    ram[9344] = 8'b00000000;
    ram[9343] = 8'b00000000;
    ram[9342] = 8'b00000000;
    ram[9341] = 8'b00000000;
    ram[9340] = 8'b00000000;
    ram[9339] = 8'b00000000;
    ram[9338] = 8'b00000000;
    ram[9337] = 8'b00000000;
    ram[9336] = 8'b00000000;
    ram[9335] = 8'b00000000;
    ram[9334] = 8'b00000000;
    ram[9333] = 8'b00000000;
    ram[9332] = 8'b00000000;
    ram[9331] = 8'b00000000;
    ram[9330] = 8'b00000000;
    ram[9329] = 8'b00000000;
    ram[9328] = 8'b00000000;
    ram[9327] = 8'b00000000;
    ram[9326] = 8'b00000000;
    ram[9325] = 8'b00000000;
    ram[9324] = 8'b00000000;
    ram[9323] = 8'b00000000;
    ram[9322] = 8'b00000000;
    ram[9321] = 8'b00000000;
    ram[9320] = 8'b00000000;
    ram[9319] = 8'b00000000;
    ram[9318] = 8'b00000000;
    ram[9317] = 8'b00000000;
    ram[9316] = 8'b00000000;
    ram[9315] = 8'b00000000;
    ram[9314] = 8'b00000000;
    ram[9313] = 8'b00000000;
    ram[9312] = 8'b00000000;
    ram[9311] = 8'b00000000;
    ram[9310] = 8'b00000000;
    ram[9309] = 8'b00000000;
    ram[9308] = 8'b00000000;
    ram[9307] = 8'b00000000;
    ram[9306] = 8'b00000000;
    ram[9305] = 8'b00000000;
    ram[9304] = 8'b00000000;
    ram[9303] = 8'b00000000;
    ram[9302] = 8'b00000000;
    ram[9301] = 8'b00000000;
    ram[9300] = 8'b00000000;
    ram[9299] = 8'b00000000;
    ram[9298] = 8'b00000000;
    ram[9297] = 8'b00000000;
    ram[9296] = 8'b00000000;
    ram[9295] = 8'b00000000;
    ram[9294] = 8'b00000000;
    ram[9293] = 8'b00000000;
    ram[9292] = 8'b00000000;
    ram[9291] = 8'b00000000;
    ram[9290] = 8'b00000000;
    ram[9289] = 8'b00000000;
    ram[9288] = 8'b00000000;
    ram[9287] = 8'b00000000;
    ram[9286] = 8'b00000000;
    ram[9285] = 8'b00000000;
    ram[9284] = 8'b00000000;
    ram[9283] = 8'b00000000;
    ram[9282] = 8'b00000000;
    ram[9281] = 8'b00000000;
    ram[9280] = 8'b00000000;
    ram[9279] = 8'b00000000;
    ram[9278] = 8'b00000000;
    ram[9277] = 8'b00000000;
    ram[9276] = 8'b00000000;
    ram[9275] = 8'b00000000;
    ram[9274] = 8'b00000000;
    ram[9273] = 8'b00000000;
    ram[9272] = 8'b00000000;
    ram[9271] = 8'b00000000;
    ram[9270] = 8'b00000000;
    ram[9269] = 8'b00000000;
    ram[9268] = 8'b00000000;
    ram[9267] = 8'b00000000;
    ram[9266] = 8'b00000000;
    ram[9265] = 8'b00000000;
    ram[9264] = 8'b00000000;
    ram[9263] = 8'b00000000;
    ram[9262] = 8'b00000000;
    ram[9261] = 8'b00000000;
    ram[9260] = 8'b00000000;
    ram[9259] = 8'b00000000;
    ram[9258] = 8'b00000000;
    ram[9257] = 8'b00000000;
    ram[9256] = 8'b00000000;
    ram[9255] = 8'b00000000;
    ram[9254] = 8'b00000000;
    ram[9253] = 8'b00000000;
    ram[9252] = 8'b00000000;
    ram[9251] = 8'b00000000;
    ram[9250] = 8'b00000000;
    ram[9249] = 8'b00000000;
    ram[9248] = 8'b00000000;
    ram[9247] = 8'b00000000;
    ram[9246] = 8'b00000000;
    ram[9245] = 8'b00000000;
    ram[9244] = 8'b00000000;
    ram[9243] = 8'b00000000;
    ram[9242] = 8'b00000000;
    ram[9241] = 8'b00000000;
    ram[9240] = 8'b00000000;
    ram[9239] = 8'b00000000;
    ram[9238] = 8'b00000000;
    ram[9237] = 8'b00000000;
    ram[9236] = 8'b00000000;
    ram[9235] = 8'b00000000;
    ram[9234] = 8'b00000000;
    ram[9233] = 8'b00000000;
    ram[9232] = 8'b00000000;
    ram[9231] = 8'b00000000;
    ram[9230] = 8'b00000000;
    ram[9229] = 8'b00000000;
    ram[9228] = 8'b00000000;
    ram[9227] = 8'b00000000;
    ram[9226] = 8'b00000000;
    ram[9225] = 8'b00000000;
    ram[9224] = 8'b00000000;
    ram[9223] = 8'b00000000;
    ram[9222] = 8'b00000000;
    ram[9221] = 8'b00000000;
    ram[9220] = 8'b00000000;
    ram[9219] = 8'b00000000;
    ram[9218] = 8'b00000000;
    ram[9217] = 8'b00000000;
    ram[9216] = 8'b00000000;
    ram[9215] = 8'b00000000;
    ram[9214] = 8'b00000000;
    ram[9213] = 8'b00000000;
    ram[9212] = 8'b00000000;
    ram[9211] = 8'b00000000;
    ram[9210] = 8'b00000000;
    ram[9209] = 8'b00000000;
    ram[9208] = 8'b00000000;
    ram[9207] = 8'b00000000;
    ram[9206] = 8'b00000000;
    ram[9205] = 8'b00000000;
    ram[9204] = 8'b00000000;
    ram[9203] = 8'b00000000;
    ram[9202] = 8'b00000000;
    ram[9201] = 8'b00000000;
    ram[9200] = 8'b00000000;
    ram[9199] = 8'b00000000;
    ram[9198] = 8'b00000000;
    ram[9197] = 8'b00000000;
    ram[9196] = 8'b00000000;
    ram[9195] = 8'b00000000;
    ram[9194] = 8'b00000000;
    ram[9193] = 8'b00000000;
    ram[9192] = 8'b00000000;
    ram[9191] = 8'b00000000;
    ram[9190] = 8'b00000000;
    ram[9189] = 8'b00000000;
    ram[9188] = 8'b00000000;
    ram[9187] = 8'b00000000;
    ram[9186] = 8'b00000000;
    ram[9185] = 8'b00000000;
    ram[9184] = 8'b00000000;
    ram[9183] = 8'b00000000;
    ram[9182] = 8'b00000000;
    ram[9181] = 8'b00000000;
    ram[9180] = 8'b00000000;
    ram[9179] = 8'b00000000;
    ram[9178] = 8'b00000000;
    ram[9177] = 8'b00000000;
    ram[9176] = 8'b00000000;
    ram[9175] = 8'b00000000;
    ram[9174] = 8'b00000000;
    ram[9173] = 8'b00000000;
    ram[9172] = 8'b00000000;
    ram[9171] = 8'b00000000;
    ram[9170] = 8'b00000000;
    ram[9169] = 8'b00000000;
    ram[9168] = 8'b00000000;
    ram[9167] = 8'b00000000;
    ram[9166] = 8'b00000000;
    ram[9165] = 8'b00000000;
    ram[9164] = 8'b00000000;
    ram[9163] = 8'b00000000;
    ram[9162] = 8'b00000000;
    ram[9161] = 8'b00000000;
    ram[9160] = 8'b00000000;
    ram[9159] = 8'b00000000;
    ram[9158] = 8'b00000000;
    ram[9157] = 8'b00000000;
    ram[9156] = 8'b00000000;
    ram[9155] = 8'b00000000;
    ram[9154] = 8'b00000000;
    ram[9153] = 8'b00000000;
    ram[9152] = 8'b00000000;
    ram[9151] = 8'b00000000;
    ram[9150] = 8'b00000000;
    ram[9149] = 8'b00000000;
    ram[9148] = 8'b00000000;
    ram[9147] = 8'b00000000;
    ram[9146] = 8'b00000000;
    ram[9145] = 8'b00000000;
    ram[9144] = 8'b00000000;
    ram[9143] = 8'b00000000;
    ram[9142] = 8'b00000000;
    ram[9141] = 8'b00000000;
    ram[9140] = 8'b00000000;
    ram[9139] = 8'b00000000;
    ram[9138] = 8'b00000000;
    ram[9137] = 8'b00000000;
    ram[9136] = 8'b00000000;
    ram[9135] = 8'b00000000;
    ram[9134] = 8'b00000000;
    ram[9133] = 8'b00000000;
    ram[9132] = 8'b00000000;
    ram[9131] = 8'b00000000;
    ram[9130] = 8'b00000000;
    ram[9129] = 8'b00000000;
    ram[9128] = 8'b00000000;
    ram[9127] = 8'b00000000;
    ram[9126] = 8'b00000000;
    ram[9125] = 8'b00000000;
    ram[9124] = 8'b00000000;
    ram[9123] = 8'b00000000;
    ram[9122] = 8'b00000000;
    ram[9121] = 8'b00000000;
    ram[9120] = 8'b00000000;
    ram[9119] = 8'b00000000;
    ram[9118] = 8'b00000000;
    ram[9117] = 8'b00000000;
    ram[9116] = 8'b00000000;
    ram[9115] = 8'b00000000;
    ram[9114] = 8'b00000000;
    ram[9113] = 8'b00000000;
    ram[9112] = 8'b00000000;
    ram[9111] = 8'b00000000;
    ram[9110] = 8'b00000000;
    ram[9109] = 8'b00000000;
    ram[9108] = 8'b00000000;
    ram[9107] = 8'b00000000;
    ram[9106] = 8'b00000000;
    ram[9105] = 8'b00000000;
    ram[9104] = 8'b00000000;
    ram[9103] = 8'b00000000;
    ram[9102] = 8'b00000000;
    ram[9101] = 8'b00000000;
    ram[9100] = 8'b00000000;
    ram[9099] = 8'b00000000;
    ram[9098] = 8'b00000000;
    ram[9097] = 8'b00000000;
    ram[9096] = 8'b00000000;
    ram[9095] = 8'b00000000;
    ram[9094] = 8'b00000000;
    ram[9093] = 8'b00000000;
    ram[9092] = 8'b00000000;
    ram[9091] = 8'b00000000;
    ram[9090] = 8'b00000000;
    ram[9089] = 8'b00000000;
    ram[9088] = 8'b00000000;
    ram[9087] = 8'b00000000;
    ram[9086] = 8'b00000000;
    ram[9085] = 8'b00000000;
    ram[9084] = 8'b00000000;
    ram[9083] = 8'b00000000;
    ram[9082] = 8'b00000000;
    ram[9081] = 8'b00000000;
    ram[9080] = 8'b00000000;
    ram[9079] = 8'b00000000;
    ram[9078] = 8'b00000000;
    ram[9077] = 8'b00000000;
    ram[9076] = 8'b00000000;
    ram[9075] = 8'b00000000;
    ram[9074] = 8'b00000000;
    ram[9073] = 8'b00000000;
    ram[9072] = 8'b00000000;
    ram[9071] = 8'b00000000;
    ram[9070] = 8'b00000000;
    ram[9069] = 8'b00000000;
    ram[9068] = 8'b00000000;
    ram[9067] = 8'b00000000;
    ram[9066] = 8'b00000000;
    ram[9065] = 8'b00000000;
    ram[9064] = 8'b00000000;
    ram[9063] = 8'b00000000;
    ram[9062] = 8'b00000000;
    ram[9061] = 8'b00000000;
    ram[9060] = 8'b00000000;
    ram[9059] = 8'b00000000;
    ram[9058] = 8'b00000000;
    ram[9057] = 8'b00000000;
    ram[9056] = 8'b00000000;
    ram[9055] = 8'b00000000;
    ram[9054] = 8'b00000000;
    ram[9053] = 8'b00000000;
    ram[9052] = 8'b00000000;
    ram[9051] = 8'b00000000;
    ram[9050] = 8'b00000000;
    ram[9049] = 8'b00000000;
    ram[9048] = 8'b00000000;
    ram[9047] = 8'b00000000;
    ram[9046] = 8'b00000000;
    ram[9045] = 8'b00000000;
    ram[9044] = 8'b00000000;
    ram[9043] = 8'b00000000;
    ram[9042] = 8'b00000000;
    ram[9041] = 8'b00000000;
    ram[9040] = 8'b00000000;
    ram[9039] = 8'b00000000;
    ram[9038] = 8'b00000000;
    ram[9037] = 8'b00000000;
    ram[9036] = 8'b00000000;
    ram[9035] = 8'b00000000;
    ram[9034] = 8'b00000000;
    ram[9033] = 8'b00000000;
    ram[9032] = 8'b00000000;
    ram[9031] = 8'b00000000;
    ram[9030] = 8'b00000000;
    ram[9029] = 8'b00000000;
    ram[9028] = 8'b00000000;
    ram[9027] = 8'b00000000;
    ram[9026] = 8'b00000000;
    ram[9025] = 8'b00000000;
    ram[9024] = 8'b00000000;
    ram[9023] = 8'b00000000;
    ram[9022] = 8'b00000000;
    ram[9021] = 8'b00000000;
    ram[9020] = 8'b00000000;
    ram[9019] = 8'b00000000;
    ram[9018] = 8'b00000000;
    ram[9017] = 8'b00000000;
    ram[9016] = 8'b00000000;
    ram[9015] = 8'b00000000;
    ram[9014] = 8'b00000000;
    ram[9013] = 8'b00000000;
    ram[9012] = 8'b00000000;
    ram[9011] = 8'b00000000;
    ram[9010] = 8'b00000000;
    ram[9009] = 8'b00000000;
    ram[9008] = 8'b00000000;
    ram[9007] = 8'b00000000;
    ram[9006] = 8'b00000000;
    ram[9005] = 8'b00000000;
    ram[9004] = 8'b00000000;
    ram[9003] = 8'b00000000;
    ram[9002] = 8'b00000000;
    ram[9001] = 8'b00000000;
    ram[9000] = 8'b00000000;
    ram[8999] = 8'b00000000;
    ram[8998] = 8'b00000000;
    ram[8997] = 8'b00000000;
    ram[8996] = 8'b00000000;
    ram[8995] = 8'b00000000;
    ram[8994] = 8'b00000000;
    ram[8993] = 8'b00000000;
    ram[8992] = 8'b00000000;
    ram[8991] = 8'b00000000;
    ram[8990] = 8'b00000000;
    ram[8989] = 8'b00000000;
    ram[8988] = 8'b00000000;
    ram[8987] = 8'b00000000;
    ram[8986] = 8'b00000000;
    ram[8985] = 8'b00000000;
    ram[8984] = 8'b00000000;
    ram[8983] = 8'b00000000;
    ram[8982] = 8'b00000000;
    ram[8981] = 8'b00000000;
    ram[8980] = 8'b00000000;
    ram[8979] = 8'b00000000;
    ram[8978] = 8'b00000000;
    ram[8977] = 8'b00000000;
    ram[8976] = 8'b00000000;
    ram[8975] = 8'b00000000;
    ram[8974] = 8'b00000000;
    ram[8973] = 8'b00000000;
    ram[8972] = 8'b00000000;
    ram[8971] = 8'b00000000;
    ram[8970] = 8'b00000000;
    ram[8969] = 8'b00000000;
    ram[8968] = 8'b00000000;
    ram[8967] = 8'b00000000;
    ram[8966] = 8'b00000000;
    ram[8965] = 8'b00000000;
    ram[8964] = 8'b00000000;
    ram[8963] = 8'b00000000;
    ram[8962] = 8'b00000000;
    ram[8961] = 8'b00000000;
    ram[8960] = 8'b00000000;
    ram[8959] = 8'b00000000;
    ram[8958] = 8'b00000000;
    ram[8957] = 8'b00000000;
    ram[8956] = 8'b00000000;
    ram[8955] = 8'b00000000;
    ram[8954] = 8'b00000000;
    ram[8953] = 8'b00000000;
    ram[8952] = 8'b00000000;
    ram[8951] = 8'b00000000;
    ram[8950] = 8'b00000000;
    ram[8949] = 8'b00000000;
    ram[8948] = 8'b00000000;
    ram[8947] = 8'b00000000;
    ram[8946] = 8'b00000000;
    ram[8945] = 8'b00000000;
    ram[8944] = 8'b00000000;
    ram[8943] = 8'b00000000;
    ram[8942] = 8'b00000000;
    ram[8941] = 8'b00000000;
    ram[8940] = 8'b00000000;
    ram[8939] = 8'b00000000;
    ram[8938] = 8'b00000000;
    ram[8937] = 8'b00000000;
    ram[8936] = 8'b00000000;
    ram[8935] = 8'b00000000;
    ram[8934] = 8'b00000000;
    ram[8933] = 8'b00000000;
    ram[8932] = 8'b00000000;
    ram[8931] = 8'b00000000;
    ram[8930] = 8'b00000000;
    ram[8929] = 8'b00000000;
    ram[8928] = 8'b00000000;
    ram[8927] = 8'b00000000;
    ram[8926] = 8'b00000000;
    ram[8925] = 8'b00000000;
    ram[8924] = 8'b00000000;
    ram[8923] = 8'b00000000;
    ram[8922] = 8'b00000000;
    ram[8921] = 8'b00000000;
    ram[8920] = 8'b00000000;
    ram[8919] = 8'b00000000;
    ram[8918] = 8'b00000000;
    ram[8917] = 8'b00000000;
    ram[8916] = 8'b00000000;
    ram[8915] = 8'b00000000;
    ram[8914] = 8'b00000000;
    ram[8913] = 8'b00000000;
    ram[8912] = 8'b00000000;
    ram[8911] = 8'b00000000;
    ram[8910] = 8'b00000000;
    ram[8909] = 8'b00000000;
    ram[8908] = 8'b00000000;
    ram[8907] = 8'b00000000;
    ram[8906] = 8'b00000000;
    ram[8905] = 8'b00000000;
    ram[8904] = 8'b00000000;
    ram[8903] = 8'b00000000;
    ram[8902] = 8'b00000000;
    ram[8901] = 8'b00000000;
    ram[8900] = 8'b00000000;
    ram[8899] = 8'b00000000;
    ram[8898] = 8'b00000000;
    ram[8897] = 8'b00000000;
    ram[8896] = 8'b00000000;
    ram[8895] = 8'b00000000;
    ram[8894] = 8'b00000000;
    ram[8893] = 8'b00000000;
    ram[8892] = 8'b00000000;
    ram[8891] = 8'b00000000;
    ram[8890] = 8'b00000000;
    ram[8889] = 8'b00000000;
    ram[8888] = 8'b00000000;
    ram[8887] = 8'b00000000;
    ram[8886] = 8'b00000000;
    ram[8885] = 8'b00000000;
    ram[8884] = 8'b00000000;
    ram[8883] = 8'b00000000;
    ram[8882] = 8'b00000000;
    ram[8881] = 8'b00000000;
    ram[8880] = 8'b00000000;
    ram[8879] = 8'b00000000;
    ram[8878] = 8'b00000000;
    ram[8877] = 8'b00000000;
    ram[8876] = 8'b00000000;
    ram[8875] = 8'b00000000;
    ram[8874] = 8'b00000000;
    ram[8873] = 8'b00000000;
    ram[8872] = 8'b00000000;
    ram[8871] = 8'b00000000;
    ram[8870] = 8'b00000000;
    ram[8869] = 8'b00000000;
    ram[8868] = 8'b00000000;
    ram[8867] = 8'b00000000;
    ram[8866] = 8'b00000000;
    ram[8865] = 8'b00000000;
    ram[8864] = 8'b00000000;
    ram[8863] = 8'b00000000;
    ram[8862] = 8'b00000000;
    ram[8861] = 8'b00000000;
    ram[8860] = 8'b00000000;
    ram[8859] = 8'b00000000;
    ram[8858] = 8'b00000000;
    ram[8857] = 8'b00000000;
    ram[8856] = 8'b00000000;
    ram[8855] = 8'b00000000;
    ram[8854] = 8'b00000000;
    ram[8853] = 8'b00000000;
    ram[8852] = 8'b00000000;
    ram[8851] = 8'b00000000;
    ram[8850] = 8'b00000000;
    ram[8849] = 8'b00000000;
    ram[8848] = 8'b00000000;
    ram[8847] = 8'b00000000;
    ram[8846] = 8'b00000000;
    ram[8845] = 8'b00000000;
    ram[8844] = 8'b00000000;
    ram[8843] = 8'b00000000;
    ram[8842] = 8'b00000000;
    ram[8841] = 8'b00000000;
    ram[8840] = 8'b00000000;
    ram[8839] = 8'b00000000;
    ram[8838] = 8'b00000000;
    ram[8837] = 8'b00000000;
    ram[8836] = 8'b00000000;
    ram[8835] = 8'b00000000;
    ram[8834] = 8'b00000000;
    ram[8833] = 8'b00000000;
    ram[8832] = 8'b00000000;
    ram[8831] = 8'b00000000;
    ram[8830] = 8'b00000000;
    ram[8829] = 8'b00000000;
    ram[8828] = 8'b00000000;
    ram[8827] = 8'b00000000;
    ram[8826] = 8'b00000000;
    ram[8825] = 8'b00000000;
    ram[8824] = 8'b00000000;
    ram[8823] = 8'b00000000;
    ram[8822] = 8'b00000000;
    ram[8821] = 8'b00000000;
    ram[8820] = 8'b00000000;
    ram[8819] = 8'b00000000;
    ram[8818] = 8'b00000000;
    ram[8817] = 8'b00000000;
    ram[8816] = 8'b00000000;
    ram[8815] = 8'b00000000;
    ram[8814] = 8'b00000000;
    ram[8813] = 8'b00000000;
    ram[8812] = 8'b00000000;
    ram[8811] = 8'b00000000;
    ram[8810] = 8'b00000000;
    ram[8809] = 8'b00000000;
    ram[8808] = 8'b00000000;
    ram[8807] = 8'b00000000;
    ram[8806] = 8'b00000000;
    ram[8805] = 8'b00000000;
    ram[8804] = 8'b00000000;
    ram[8803] = 8'b00000000;
    ram[8802] = 8'b00000000;
    ram[8801] = 8'b00000000;
    ram[8800] = 8'b00000000;
    ram[8799] = 8'b00000000;
    ram[8798] = 8'b00000000;
    ram[8797] = 8'b00000000;
    ram[8796] = 8'b00000000;
    ram[8795] = 8'b00000000;
    ram[8794] = 8'b00000000;
    ram[8793] = 8'b00000000;
    ram[8792] = 8'b00000000;
    ram[8791] = 8'b00000000;
    ram[8790] = 8'b00000000;
    ram[8789] = 8'b00000000;
    ram[8788] = 8'b00000000;
    ram[8787] = 8'b00000000;
    ram[8786] = 8'b00000000;
    ram[8785] = 8'b00000000;
    ram[8784] = 8'b00000000;
    ram[8783] = 8'b00000000;
    ram[8782] = 8'b00000000;
    ram[8781] = 8'b00000000;
    ram[8780] = 8'b00000000;
    ram[8779] = 8'b00000000;
    ram[8778] = 8'b00000000;
    ram[8777] = 8'b00000000;
    ram[8776] = 8'b00000000;
    ram[8775] = 8'b00000000;
    ram[8774] = 8'b00000000;
    ram[8773] = 8'b00000000;
    ram[8772] = 8'b00000000;
    ram[8771] = 8'b00000000;
    ram[8770] = 8'b00000000;
    ram[8769] = 8'b00000000;
    ram[8768] = 8'b00000000;
    ram[8767] = 8'b00000000;
    ram[8766] = 8'b00000000;
    ram[8765] = 8'b00000000;
    ram[8764] = 8'b00000000;
    ram[8763] = 8'b00000000;
    ram[8762] = 8'b00000000;
    ram[8761] = 8'b00000000;
    ram[8760] = 8'b00000000;
    ram[8759] = 8'b00000000;
    ram[8758] = 8'b00000000;
    ram[8757] = 8'b00000000;
    ram[8756] = 8'b00000000;
    ram[8755] = 8'b00000000;
    ram[8754] = 8'b00000000;
    ram[8753] = 8'b00000000;
    ram[8752] = 8'b00000000;
    ram[8751] = 8'b00000000;
    ram[8750] = 8'b00000000;
    ram[8749] = 8'b00000000;
    ram[8748] = 8'b00000000;
    ram[8747] = 8'b00000000;
    ram[8746] = 8'b00000000;
    ram[8745] = 8'b00000000;
    ram[8744] = 8'b00000000;
    ram[8743] = 8'b00000000;
    ram[8742] = 8'b00000000;
    ram[8741] = 8'b00000000;
    ram[8740] = 8'b00000000;
    ram[8739] = 8'b00000000;
    ram[8738] = 8'b00000000;
    ram[8737] = 8'b00000000;
    ram[8736] = 8'b00000000;
    ram[8735] = 8'b00000000;
    ram[8734] = 8'b00000000;
    ram[8733] = 8'b00000000;
    ram[8732] = 8'b00000000;
    ram[8731] = 8'b00000000;
    ram[8730] = 8'b00000000;
    ram[8729] = 8'b00000000;
    ram[8728] = 8'b00000000;
    ram[8727] = 8'b00000000;
    ram[8726] = 8'b00000000;
    ram[8725] = 8'b00000000;
    ram[8724] = 8'b00000000;
    ram[8723] = 8'b00000000;
    ram[8722] = 8'b00000000;
    ram[8721] = 8'b00000000;
    ram[8720] = 8'b00000000;
    ram[8719] = 8'b00000000;
    ram[8718] = 8'b00000000;
    ram[8717] = 8'b00000000;
    ram[8716] = 8'b00000000;
    ram[8715] = 8'b00000000;
    ram[8714] = 8'b00000000;
    ram[8713] = 8'b00000000;
    ram[8712] = 8'b00000000;
    ram[8711] = 8'b00000000;
    ram[8710] = 8'b00000000;
    ram[8709] = 8'b00000000;
    ram[8708] = 8'b00000000;
    ram[8707] = 8'b00000000;
    ram[8706] = 8'b00000000;
    ram[8705] = 8'b00000000;
    ram[8704] = 8'b00000000;
    ram[8703] = 8'b00000000;
    ram[8702] = 8'b00000000;
    ram[8701] = 8'b00000000;
    ram[8700] = 8'b00000000;
    ram[8699] = 8'b00000000;
    ram[8698] = 8'b00000000;
    ram[8697] = 8'b00000000;
    ram[8696] = 8'b00000000;
    ram[8695] = 8'b00000000;
    ram[8694] = 8'b00000000;
    ram[8693] = 8'b00000000;
    ram[8692] = 8'b00000000;
    ram[8691] = 8'b00000000;
    ram[8690] = 8'b00000000;
    ram[8689] = 8'b00000000;
    ram[8688] = 8'b00000000;
    ram[8687] = 8'b00000000;
    ram[8686] = 8'b00000000;
    ram[8685] = 8'b00000000;
    ram[8684] = 8'b00000000;
    ram[8683] = 8'b00000000;
    ram[8682] = 8'b00000000;
    ram[8681] = 8'b00000000;
    ram[8680] = 8'b00000000;
    ram[8679] = 8'b00000000;
    ram[8678] = 8'b00000000;
    ram[8677] = 8'b00000000;
    ram[8676] = 8'b00000000;
    ram[8675] = 8'b00000000;
    ram[8674] = 8'b00000000;
    ram[8673] = 8'b00000000;
    ram[8672] = 8'b00000000;
    ram[8671] = 8'b00000000;
    ram[8670] = 8'b00000000;
    ram[8669] = 8'b00000000;
    ram[8668] = 8'b00000000;
    ram[8667] = 8'b00000000;
    ram[8666] = 8'b00000000;
    ram[8665] = 8'b00000000;
    ram[8664] = 8'b00000000;
    ram[8663] = 8'b00000000;
    ram[8662] = 8'b00000000;
    ram[8661] = 8'b00000000;
    ram[8660] = 8'b00000000;
    ram[8659] = 8'b00000000;
    ram[8658] = 8'b00000000;
    ram[8657] = 8'b00000000;
    ram[8656] = 8'b00000000;
    ram[8655] = 8'b00000000;
    ram[8654] = 8'b00000000;
    ram[8653] = 8'b00000000;
    ram[8652] = 8'b00000000;
    ram[8651] = 8'b00000000;
    ram[8650] = 8'b00000000;
    ram[8649] = 8'b00000000;
    ram[8648] = 8'b00000000;
    ram[8647] = 8'b00000000;
    ram[8646] = 8'b00000000;
    ram[8645] = 8'b00000000;
    ram[8644] = 8'b00000000;
    ram[8643] = 8'b00000000;
    ram[8642] = 8'b00000000;
    ram[8641] = 8'b00000000;
    ram[8640] = 8'b00000000;
    ram[8639] = 8'b00000000;
    ram[8638] = 8'b00000000;
    ram[8637] = 8'b00000000;
    ram[8636] = 8'b00000000;
    ram[8635] = 8'b00000000;
    ram[8634] = 8'b00000000;
    ram[8633] = 8'b00000000;
    ram[8632] = 8'b00000000;
    ram[8631] = 8'b00000000;
    ram[8630] = 8'b00000000;
    ram[8629] = 8'b00000000;
    ram[8628] = 8'b00000000;
    ram[8627] = 8'b00000000;
    ram[8626] = 8'b00000000;
    ram[8625] = 8'b00000000;
    ram[8624] = 8'b00000000;
    ram[8623] = 8'b00000000;
    ram[8622] = 8'b00000000;
    ram[8621] = 8'b00000000;
    ram[8620] = 8'b00000000;
    ram[8619] = 8'b00000000;
    ram[8618] = 8'b00000000;
    ram[8617] = 8'b00000000;
    ram[8616] = 8'b00000000;
    ram[8615] = 8'b00000000;
    ram[8614] = 8'b00000000;
    ram[8613] = 8'b00000000;
    ram[8612] = 8'b00000000;
    ram[8611] = 8'b00000000;
    ram[8610] = 8'b00000000;
    ram[8609] = 8'b00000000;
    ram[8608] = 8'b00000000;
    ram[8607] = 8'b00000000;
    ram[8606] = 8'b00000000;
    ram[8605] = 8'b00000000;
    ram[8604] = 8'b00000000;
    ram[8603] = 8'b00000000;
    ram[8602] = 8'b00000000;
    ram[8601] = 8'b00000000;
    ram[8600] = 8'b00000000;
    ram[8599] = 8'b00000000;
    ram[8598] = 8'b00000000;
    ram[8597] = 8'b00000000;
    ram[8596] = 8'b00000000;
    ram[8595] = 8'b00000000;
    ram[8594] = 8'b00000000;
    ram[8593] = 8'b00000000;
    ram[8592] = 8'b00000000;
    ram[8591] = 8'b00000000;
    ram[8590] = 8'b00000000;
    ram[8589] = 8'b00000000;
    ram[8588] = 8'b00000000;
    ram[8587] = 8'b00000000;
    ram[8586] = 8'b00000000;
    ram[8585] = 8'b00000000;
    ram[8584] = 8'b00000000;
    ram[8583] = 8'b00000000;
    ram[8582] = 8'b00000000;
    ram[8581] = 8'b00000000;
    ram[8580] = 8'b00000000;
    ram[8579] = 8'b00000000;
    ram[8578] = 8'b00000000;
    ram[8577] = 8'b00000000;
    ram[8576] = 8'b00000000;
    ram[8575] = 8'b00000000;
    ram[8574] = 8'b00000000;
    ram[8573] = 8'b00000000;
    ram[8572] = 8'b00000000;
    ram[8571] = 8'b00000000;
    ram[8570] = 8'b00000000;
    ram[8569] = 8'b00000000;
    ram[8568] = 8'b00000000;
    ram[8567] = 8'b00000000;
    ram[8566] = 8'b00000000;
    ram[8565] = 8'b00000000;
    ram[8564] = 8'b00000000;
    ram[8563] = 8'b00000000;
    ram[8562] = 8'b00000000;
    ram[8561] = 8'b00000000;
    ram[8560] = 8'b00000000;
    ram[8559] = 8'b00000000;
    ram[8558] = 8'b00000000;
    ram[8557] = 8'b00000000;
    ram[8556] = 8'b00000000;
    ram[8555] = 8'b00000000;
    ram[8554] = 8'b00000000;
    ram[8553] = 8'b00000000;
    ram[8552] = 8'b00000000;
    ram[8551] = 8'b00000000;
    ram[8550] = 8'b00000000;
    ram[8549] = 8'b00000000;
    ram[8548] = 8'b00000000;
    ram[8547] = 8'b00000000;
    ram[8546] = 8'b00000000;
    ram[8545] = 8'b00000000;
    ram[8544] = 8'b00000000;
    ram[8543] = 8'b00000000;
    ram[8542] = 8'b00000000;
    ram[8541] = 8'b00000000;
    ram[8540] = 8'b00000000;
    ram[8539] = 8'b00000000;
    ram[8538] = 8'b00000000;
    ram[8537] = 8'b00000000;
    ram[8536] = 8'b00000000;
    ram[8535] = 8'b00000000;
    ram[8534] = 8'b00000000;
    ram[8533] = 8'b00000000;
    ram[8532] = 8'b00000000;
    ram[8531] = 8'b00000000;
    ram[8530] = 8'b00000000;
    ram[8529] = 8'b00000000;
    ram[8528] = 8'b00000000;
    ram[8527] = 8'b00000000;
    ram[8526] = 8'b00000000;
    ram[8525] = 8'b00000000;
    ram[8524] = 8'b00000000;
    ram[8523] = 8'b00000000;
    ram[8522] = 8'b00000000;
    ram[8521] = 8'b00000000;
    ram[8520] = 8'b00000000;
    ram[8519] = 8'b00000000;
    ram[8518] = 8'b00000000;
    ram[8517] = 8'b00000000;
    ram[8516] = 8'b00000000;
    ram[8515] = 8'b00000000;
    ram[8514] = 8'b00000000;
    ram[8513] = 8'b00000000;
    ram[8512] = 8'b00000000;
    ram[8511] = 8'b00000000;
    ram[8510] = 8'b00000000;
    ram[8509] = 8'b00000000;
    ram[8508] = 8'b00000000;
    ram[8507] = 8'b00000000;
    ram[8506] = 8'b00000000;
    ram[8505] = 8'b00000000;
    ram[8504] = 8'b00000000;
    ram[8503] = 8'b00000000;
    ram[8502] = 8'b00000000;
    ram[8501] = 8'b00000000;
    ram[8500] = 8'b00000000;
    ram[8499] = 8'b00000000;
    ram[8498] = 8'b00000000;
    ram[8497] = 8'b00000000;
    ram[8496] = 8'b00000000;
    ram[8495] = 8'b00000000;
    ram[8494] = 8'b00000000;
    ram[8493] = 8'b00000000;
    ram[8492] = 8'b00000000;
    ram[8491] = 8'b00000000;
    ram[8490] = 8'b00000000;
    ram[8489] = 8'b00000000;
    ram[8488] = 8'b00000000;
    ram[8487] = 8'b00000000;
    ram[8486] = 8'b00000000;
    ram[8485] = 8'b00000000;
    ram[8484] = 8'b00000000;
    ram[8483] = 8'b00000000;
    ram[8482] = 8'b00000000;
    ram[8481] = 8'b00000000;
    ram[8480] = 8'b00000000;
    ram[8479] = 8'b00000000;
    ram[8478] = 8'b00000000;
    ram[8477] = 8'b00000000;
    ram[8476] = 8'b00000000;
    ram[8475] = 8'b00000000;
    ram[8474] = 8'b00000000;
    ram[8473] = 8'b00000000;
    ram[8472] = 8'b00000000;
    ram[8471] = 8'b00000000;
    ram[8470] = 8'b00000000;
    ram[8469] = 8'b00000000;
    ram[8468] = 8'b00000000;
    ram[8467] = 8'b00000000;
    ram[8466] = 8'b00000000;
    ram[8465] = 8'b00000000;
    ram[8464] = 8'b00000000;
    ram[8463] = 8'b00000000;
    ram[8462] = 8'b00000000;
    ram[8461] = 8'b00000000;
    ram[8460] = 8'b00000000;
    ram[8459] = 8'b00000000;
    ram[8458] = 8'b00000000;
    ram[8457] = 8'b00000000;
    ram[8456] = 8'b00000000;
    ram[8455] = 8'b00000000;
    ram[8454] = 8'b00000000;
    ram[8453] = 8'b00000000;
    ram[8452] = 8'b00000000;
    ram[8451] = 8'b00000000;
    ram[8450] = 8'b00000000;
    ram[8449] = 8'b00000000;
    ram[8448] = 8'b00000000;
    ram[8447] = 8'b00000000;
    ram[8446] = 8'b00000000;
    ram[8445] = 8'b00000000;
    ram[8444] = 8'b00000000;
    ram[8443] = 8'b00000000;
    ram[8442] = 8'b00000000;
    ram[8441] = 8'b00000000;
    ram[8440] = 8'b00000000;
    ram[8439] = 8'b00000000;
    ram[8438] = 8'b00000000;
    ram[8437] = 8'b00000000;
    ram[8436] = 8'b00000000;
    ram[8435] = 8'b00000000;
    ram[8434] = 8'b00000000;
    ram[8433] = 8'b00000000;
    ram[8432] = 8'b00000000;
    ram[8431] = 8'b00000000;
    ram[8430] = 8'b00000000;
    ram[8429] = 8'b00000000;
    ram[8428] = 8'b00000000;
    ram[8427] = 8'b00000000;
    ram[8426] = 8'b00000000;
    ram[8425] = 8'b00000000;
    ram[8424] = 8'b00000000;
    ram[8423] = 8'b00000000;
    ram[8422] = 8'b00000000;
    ram[8421] = 8'b00000000;
    ram[8420] = 8'b00000000;
    ram[8419] = 8'b00000000;
    ram[8418] = 8'b00000000;
    ram[8417] = 8'b00000000;
    ram[8416] = 8'b00000000;
    ram[8415] = 8'b00000000;
    ram[8414] = 8'b00000000;
    ram[8413] = 8'b00000000;
    ram[8412] = 8'b00000000;
    ram[8411] = 8'b00000000;
    ram[8410] = 8'b00000000;
    ram[8409] = 8'b00000000;
    ram[8408] = 8'b00000000;
    ram[8407] = 8'b00000000;
    ram[8406] = 8'b00000000;
    ram[8405] = 8'b00000000;
    ram[8404] = 8'b00000000;
    ram[8403] = 8'b00000000;
    ram[8402] = 8'b00000000;
    ram[8401] = 8'b00000000;
    ram[8400] = 8'b00000000;
    ram[8399] = 8'b00000000;
    ram[8398] = 8'b00000000;
    ram[8397] = 8'b00000000;
    ram[8396] = 8'b00000000;
    ram[8395] = 8'b00000000;
    ram[8394] = 8'b00000000;
    ram[8393] = 8'b00000000;
    ram[8392] = 8'b00000000;
    ram[8391] = 8'b00000000;
    ram[8390] = 8'b00000000;
    ram[8389] = 8'b00000000;
    ram[8388] = 8'b00000000;
    ram[8387] = 8'b00000000;
    ram[8386] = 8'b00000000;
    ram[8385] = 8'b00000000;
    ram[8384] = 8'b00000000;
    ram[8383] = 8'b00000000;
    ram[8382] = 8'b00000000;
    ram[8381] = 8'b00000000;
    ram[8380] = 8'b00000000;
    ram[8379] = 8'b00000000;
    ram[8378] = 8'b00000000;
    ram[8377] = 8'b00000000;
    ram[8376] = 8'b00000000;
    ram[8375] = 8'b00000000;
    ram[8374] = 8'b00000000;
    ram[8373] = 8'b00000000;
    ram[8372] = 8'b00000000;
    ram[8371] = 8'b00000000;
    ram[8370] = 8'b00000000;
    ram[8369] = 8'b00000000;
    ram[8368] = 8'b00000000;
    ram[8367] = 8'b00000000;
    ram[8366] = 8'b00000000;
    ram[8365] = 8'b00000000;
    ram[8364] = 8'b00000000;
    ram[8363] = 8'b00000000;
    ram[8362] = 8'b00000000;
    ram[8361] = 8'b00000000;
    ram[8360] = 8'b00000000;
    ram[8359] = 8'b00000000;
    ram[8358] = 8'b00000000;
    ram[8357] = 8'b00000000;
    ram[8356] = 8'b00000000;
    ram[8355] = 8'b00000000;
    ram[8354] = 8'b00000000;
    ram[8353] = 8'b00000000;
    ram[8352] = 8'b00000000;
    ram[8351] = 8'b00000000;
    ram[8350] = 8'b00000000;
    ram[8349] = 8'b00000000;
    ram[8348] = 8'b00000000;
    ram[8347] = 8'b00000000;
    ram[8346] = 8'b00000000;
    ram[8345] = 8'b00000000;
    ram[8344] = 8'b00000000;
    ram[8343] = 8'b00000000;
    ram[8342] = 8'b00000000;
    ram[8341] = 8'b00000000;
    ram[8340] = 8'b00000000;
    ram[8339] = 8'b00000000;
    ram[8338] = 8'b00000000;
    ram[8337] = 8'b00000000;
    ram[8336] = 8'b00000000;
    ram[8335] = 8'b00000000;
    ram[8334] = 8'b00000000;
    ram[8333] = 8'b00000000;
    ram[8332] = 8'b00000000;
    ram[8331] = 8'b00000000;
    ram[8330] = 8'b00000000;
    ram[8329] = 8'b00000000;
    ram[8328] = 8'b00000000;
    ram[8327] = 8'b00000000;
    ram[8326] = 8'b00000000;
    ram[8325] = 8'b00000000;
    ram[8324] = 8'b00000000;
    ram[8323] = 8'b00000000;
    ram[8322] = 8'b00000000;
    ram[8321] = 8'b00000000;
    ram[8320] = 8'b00000000;
    ram[8319] = 8'b00000000;
    ram[8318] = 8'b00000000;
    ram[8317] = 8'b00000000;
    ram[8316] = 8'b00000000;
    ram[8315] = 8'b00000000;
    ram[8314] = 8'b00000000;
    ram[8313] = 8'b00000000;
    ram[8312] = 8'b00000000;
    ram[8311] = 8'b00000000;
    ram[8310] = 8'b00000000;
    ram[8309] = 8'b00000000;
    ram[8308] = 8'b00000000;
    ram[8307] = 8'b00000000;
    ram[8306] = 8'b00000000;
    ram[8305] = 8'b00000000;
    ram[8304] = 8'b00000000;
    ram[8303] = 8'b00000000;
    ram[8302] = 8'b00000000;
    ram[8301] = 8'b00000000;
    ram[8300] = 8'b00000000;
    ram[8299] = 8'b00000000;
    ram[8298] = 8'b00000000;
    ram[8297] = 8'b00000000;
    ram[8296] = 8'b00000000;
    ram[8295] = 8'b00000000;
    ram[8294] = 8'b00000000;
    ram[8293] = 8'b00000000;
    ram[8292] = 8'b00000000;
    ram[8291] = 8'b00000000;
    ram[8290] = 8'b00000000;
    ram[8289] = 8'b00000000;
    ram[8288] = 8'b00000000;
    ram[8287] = 8'b00000000;
    ram[8286] = 8'b00000000;
    ram[8285] = 8'b00000000;
    ram[8284] = 8'b00000000;
    ram[8283] = 8'b00000000;
    ram[8282] = 8'b00000000;
    ram[8281] = 8'b00000000;
    ram[8280] = 8'b00000000;
    ram[8279] = 8'b00000000;
    ram[8278] = 8'b00000000;
    ram[8277] = 8'b00000000;
    ram[8276] = 8'b00000000;
    ram[8275] = 8'b00000000;
    ram[8274] = 8'b00000000;
    ram[8273] = 8'b00000000;
    ram[8272] = 8'b00000000;
    ram[8271] = 8'b00000000;
    ram[8270] = 8'b00000000;
    ram[8269] = 8'b00000000;
    ram[8268] = 8'b00000000;
    ram[8267] = 8'b00000000;
    ram[8266] = 8'b00000000;
    ram[8265] = 8'b00000000;
    ram[8264] = 8'b00000000;
    ram[8263] = 8'b00000000;
    ram[8262] = 8'b00000000;
    ram[8261] = 8'b00000000;
    ram[8260] = 8'b00000000;
    ram[8259] = 8'b00000000;
    ram[8258] = 8'b00000000;
    ram[8257] = 8'b00000000;
    ram[8256] = 8'b00000000;
    ram[8255] = 8'b00000000;
    ram[8254] = 8'b00000000;
    ram[8253] = 8'b00000000;
    ram[8252] = 8'b00000000;
    ram[8251] = 8'b00000000;
    ram[8250] = 8'b00000000;
    ram[8249] = 8'b00000000;
    ram[8248] = 8'b00000000;
    ram[8247] = 8'b00000000;
    ram[8246] = 8'b00000000;
    ram[8245] = 8'b00000000;
    ram[8244] = 8'b00000000;
    ram[8243] = 8'b00000000;
    ram[8242] = 8'b00000000;
    ram[8241] = 8'b00000000;
    ram[8240] = 8'b00000000;
    ram[8239] = 8'b00000000;
    ram[8238] = 8'b00000000;
    ram[8237] = 8'b00000000;
    ram[8236] = 8'b00000000;
    ram[8235] = 8'b00000000;
    ram[8234] = 8'b00000000;
    ram[8233] = 8'b00000000;
    ram[8232] = 8'b00000000;
    ram[8231] = 8'b00000000;
    ram[8230] = 8'b00000000;
    ram[8229] = 8'b00000000;
    ram[8228] = 8'b00000000;
    ram[8227] = 8'b00000000;
    ram[8226] = 8'b00000000;
    ram[8225] = 8'b00000000;
    ram[8224] = 8'b00000000;
    ram[8223] = 8'b00000000;
    ram[8222] = 8'b00000000;
    ram[8221] = 8'b00000000;
    ram[8220] = 8'b00000000;
    ram[8219] = 8'b00000000;
    ram[8218] = 8'b00000000;
    ram[8217] = 8'b00000000;
    ram[8216] = 8'b00000000;
    ram[8215] = 8'b00000000;
    ram[8214] = 8'b00000000;
    ram[8213] = 8'b00000000;
    ram[8212] = 8'b00000000;
    ram[8211] = 8'b00000000;
    ram[8210] = 8'b00000000;
    ram[8209] = 8'b00000000;
    ram[8208] = 8'b00000000;
    ram[8207] = 8'b00000000;
    ram[8206] = 8'b00000000;
    ram[8205] = 8'b00000000;
    ram[8204] = 8'b00000000;
    ram[8203] = 8'b00000000;
    ram[8202] = 8'b00000000;
    ram[8201] = 8'b00000000;
    ram[8200] = 8'b00000000;
    ram[8199] = 8'b00000000;
    ram[8198] = 8'b00000000;
    ram[8197] = 8'b00000000;
    ram[8196] = 8'b00000000;
    ram[8195] = 8'b00000000;
    ram[8194] = 8'b00000000;
    ram[8193] = 8'b00000000;
    ram[8192] = 8'b00000000;
    ram[8191] = 8'b00000000;
    ram[8190] = 8'b00000000;
    ram[8189] = 8'b00000000;
    ram[8188] = 8'b00000000;
    ram[8187] = 8'b00000000;
    ram[8186] = 8'b00000000;
    ram[8185] = 8'b00000000;
    ram[8184] = 8'b00000000;
    ram[8183] = 8'b00000000;
    ram[8182] = 8'b00000000;
    ram[8181] = 8'b00000000;
    ram[8180] = 8'b00000000;
    ram[8179] = 8'b00000000;
    ram[8178] = 8'b00000000;
    ram[8177] = 8'b00000000;
    ram[8176] = 8'b00000000;
    ram[8175] = 8'b00000000;
    ram[8174] = 8'b00000000;
    ram[8173] = 8'b00000000;
    ram[8172] = 8'b00000000;
    ram[8171] = 8'b00000000;
    ram[8170] = 8'b00000000;
    ram[8169] = 8'b00000000;
    ram[8168] = 8'b00000000;
    ram[8167] = 8'b00000000;
    ram[8166] = 8'b00000000;
    ram[8165] = 8'b00000000;
    ram[8164] = 8'b00000000;
    ram[8163] = 8'b00000000;
    ram[8162] = 8'b00000000;
    ram[8161] = 8'b00000000;
    ram[8160] = 8'b00000000;
    ram[8159] = 8'b00000000;
    ram[8158] = 8'b00000000;
    ram[8157] = 8'b00000000;
    ram[8156] = 8'b00000000;
    ram[8155] = 8'b00000000;
    ram[8154] = 8'b00000000;
    ram[8153] = 8'b00000000;
    ram[8152] = 8'b00000000;
    ram[8151] = 8'b00000000;
    ram[8150] = 8'b00000000;
    ram[8149] = 8'b00000000;
    ram[8148] = 8'b00000000;
    ram[8147] = 8'b00000000;
    ram[8146] = 8'b00000000;
    ram[8145] = 8'b00000000;
    ram[8144] = 8'b00000000;
    ram[8143] = 8'b00000000;
    ram[8142] = 8'b00000000;
    ram[8141] = 8'b00000000;
    ram[8140] = 8'b00000000;
    ram[8139] = 8'b00000000;
    ram[8138] = 8'b00000000;
    ram[8137] = 8'b00000000;
    ram[8136] = 8'b00000000;
    ram[8135] = 8'b00000000;
    ram[8134] = 8'b00000000;
    ram[8133] = 8'b00000000;
    ram[8132] = 8'b00000000;
    ram[8131] = 8'b00000000;
    ram[8130] = 8'b00000000;
    ram[8129] = 8'b00000000;
    ram[8128] = 8'b00000000;
    ram[8127] = 8'b00000000;
    ram[8126] = 8'b00000000;
    ram[8125] = 8'b00000000;
    ram[8124] = 8'b00000000;
    ram[8123] = 8'b00000000;
    ram[8122] = 8'b00000000;
    ram[8121] = 8'b00000000;
    ram[8120] = 8'b00000000;
    ram[8119] = 8'b00000000;
    ram[8118] = 8'b00000000;
    ram[8117] = 8'b00000000;
    ram[8116] = 8'b00000000;
    ram[8115] = 8'b00000000;
    ram[8114] = 8'b00000000;
    ram[8113] = 8'b00000000;
    ram[8112] = 8'b00000000;
    ram[8111] = 8'b00000000;
    ram[8110] = 8'b00000000;
    ram[8109] = 8'b00000000;
    ram[8108] = 8'b00000000;
    ram[8107] = 8'b00000000;
    ram[8106] = 8'b00000000;
    ram[8105] = 8'b00000000;
    ram[8104] = 8'b00000000;
    ram[8103] = 8'b00000000;
    ram[8102] = 8'b00000000;
    ram[8101] = 8'b00000000;
    ram[8100] = 8'b00000000;
    ram[8099] = 8'b00000000;
    ram[8098] = 8'b00000000;
    ram[8097] = 8'b00000000;
    ram[8096] = 8'b00000000;
    ram[8095] = 8'b00000000;
    ram[8094] = 8'b00000000;
    ram[8093] = 8'b00000000;
    ram[8092] = 8'b00000000;
    ram[8091] = 8'b00000000;
    ram[8090] = 8'b00000000;
    ram[8089] = 8'b00000000;
    ram[8088] = 8'b00000000;
    ram[8087] = 8'b00000000;
    ram[8086] = 8'b00000000;
    ram[8085] = 8'b00000000;
    ram[8084] = 8'b00000000;
    ram[8083] = 8'b00000000;
    ram[8082] = 8'b00000000;
    ram[8081] = 8'b00000000;
    ram[8080] = 8'b00000000;
    ram[8079] = 8'b00000000;
    ram[8078] = 8'b00000000;
    ram[8077] = 8'b00000000;
    ram[8076] = 8'b00000000;
    ram[8075] = 8'b00000000;
    ram[8074] = 8'b00000000;
    ram[8073] = 8'b00000000;
    ram[8072] = 8'b00000000;
    ram[8071] = 8'b00000000;
    ram[8070] = 8'b00000000;
    ram[8069] = 8'b00000000;
    ram[8068] = 8'b00000000;
    ram[8067] = 8'b00000000;
    ram[8066] = 8'b00000000;
    ram[8065] = 8'b00000000;
    ram[8064] = 8'b00000000;
    ram[8063] = 8'b00000000;
    ram[8062] = 8'b00000000;
    ram[8061] = 8'b00000000;
    ram[8060] = 8'b00000000;
    ram[8059] = 8'b00000000;
    ram[8058] = 8'b00000000;
    ram[8057] = 8'b00000000;
    ram[8056] = 8'b00000000;
    ram[8055] = 8'b00000000;
    ram[8054] = 8'b00000000;
    ram[8053] = 8'b00000000;
    ram[8052] = 8'b00000000;
    ram[8051] = 8'b00000000;
    ram[8050] = 8'b00000000;
    ram[8049] = 8'b00000000;
    ram[8048] = 8'b00000000;
    ram[8047] = 8'b00000000;
    ram[8046] = 8'b00000000;
    ram[8045] = 8'b00000000;
    ram[8044] = 8'b00000000;
    ram[8043] = 8'b00000000;
    ram[8042] = 8'b00000000;
    ram[8041] = 8'b00000000;
    ram[8040] = 8'b00000000;
    ram[8039] = 8'b00000000;
    ram[8038] = 8'b00000000;
    ram[8037] = 8'b00000000;
    ram[8036] = 8'b00000000;
    ram[8035] = 8'b00000000;
    ram[8034] = 8'b00000000;
    ram[8033] = 8'b00000000;
    ram[8032] = 8'b00000000;
    ram[8031] = 8'b00000000;
    ram[8030] = 8'b00000000;
    ram[8029] = 8'b00000000;
    ram[8028] = 8'b00000000;
    ram[8027] = 8'b00000000;
    ram[8026] = 8'b00000000;
    ram[8025] = 8'b00000000;
    ram[8024] = 8'b00000000;
    ram[8023] = 8'b00000000;
    ram[8022] = 8'b00000000;
    ram[8021] = 8'b00000000;
    ram[8020] = 8'b00000000;
    ram[8019] = 8'b00000000;
    ram[8018] = 8'b00000000;
    ram[8017] = 8'b00000000;
    ram[8016] = 8'b00000000;
    ram[8015] = 8'b00000000;
    ram[8014] = 8'b00000000;
    ram[8013] = 8'b00000000;
    ram[8012] = 8'b00000000;
    ram[8011] = 8'b00000000;
    ram[8010] = 8'b00000000;
    ram[8009] = 8'b00000000;
    ram[8008] = 8'b00000000;
    ram[8007] = 8'b00000000;
    ram[8006] = 8'b00000000;
    ram[8005] = 8'b00000000;
    ram[8004] = 8'b00000000;
    ram[8003] = 8'b00000000;
    ram[8002] = 8'b00000000;
    ram[8001] = 8'b00000000;
    ram[8000] = 8'b00000000;
    ram[7999] = 8'b00000000;
    ram[7998] = 8'b00000000;
    ram[7997] = 8'b00000000;
    ram[7996] = 8'b00000000;
    ram[7995] = 8'b00000000;
    ram[7994] = 8'b00000000;
    ram[7993] = 8'b00000000;
    ram[7992] = 8'b00000000;
    ram[7991] = 8'b00000000;
    ram[7990] = 8'b00000000;
    ram[7989] = 8'b00000000;
    ram[7988] = 8'b00000000;
    ram[7987] = 8'b00000000;
    ram[7986] = 8'b00000000;
    ram[7985] = 8'b00000000;
    ram[7984] = 8'b00000000;
    ram[7983] = 8'b00000000;
    ram[7982] = 8'b00000000;
    ram[7981] = 8'b00000000;
    ram[7980] = 8'b00000000;
    ram[7979] = 8'b00000000;
    ram[7978] = 8'b00000000;
    ram[7977] = 8'b00000000;
    ram[7976] = 8'b00000000;
    ram[7975] = 8'b00000000;
    ram[7974] = 8'b00000000;
    ram[7973] = 8'b00000000;
    ram[7972] = 8'b00000000;
    ram[7971] = 8'b00000000;
    ram[7970] = 8'b00000000;
    ram[7969] = 8'b00000000;
    ram[7968] = 8'b00000000;
    ram[7967] = 8'b00000000;
    ram[7966] = 8'b00000000;
    ram[7965] = 8'b00000000;
    ram[7964] = 8'b00000000;
    ram[7963] = 8'b00000000;
    ram[7962] = 8'b00000000;
    ram[7961] = 8'b00000000;
    ram[7960] = 8'b00000000;
    ram[7959] = 8'b00000000;
    ram[7958] = 8'b00000000;
    ram[7957] = 8'b00000000;
    ram[7956] = 8'b00000000;
    ram[7955] = 8'b00000000;
    ram[7954] = 8'b00000000;
    ram[7953] = 8'b00000000;
    ram[7952] = 8'b00000000;
    ram[7951] = 8'b00000000;
    ram[7950] = 8'b00000000;
    ram[7949] = 8'b00000000;
    ram[7948] = 8'b00000000;
    ram[7947] = 8'b00000000;
    ram[7946] = 8'b00000000;
    ram[7945] = 8'b00000000;
    ram[7944] = 8'b00000000;
    ram[7943] = 8'b00000000;
    ram[7942] = 8'b00000000;
    ram[7941] = 8'b00000000;
    ram[7940] = 8'b00000000;
    ram[7939] = 8'b00000000;
    ram[7938] = 8'b00000000;
    ram[7937] = 8'b00000000;
    ram[7936] = 8'b00000000;
    ram[7935] = 8'b00000000;
    ram[7934] = 8'b00000000;
    ram[7933] = 8'b00000000;
    ram[7932] = 8'b00000000;
    ram[7931] = 8'b00000000;
    ram[7930] = 8'b00000000;
    ram[7929] = 8'b00000000;
    ram[7928] = 8'b00000000;
    ram[7927] = 8'b00000000;
    ram[7926] = 8'b00000000;
    ram[7925] = 8'b00000000;
    ram[7924] = 8'b00000000;
    ram[7923] = 8'b00000000;
    ram[7922] = 8'b00000000;
    ram[7921] = 8'b00000000;
    ram[7920] = 8'b00000000;
    ram[7919] = 8'b00000000;
    ram[7918] = 8'b00000000;
    ram[7917] = 8'b00000000;
    ram[7916] = 8'b00000000;
    ram[7915] = 8'b00000000;
    ram[7914] = 8'b00000000;
    ram[7913] = 8'b00000000;
    ram[7912] = 8'b00000000;
    ram[7911] = 8'b00000000;
    ram[7910] = 8'b00000000;
    ram[7909] = 8'b00000000;
    ram[7908] = 8'b00000000;
    ram[7907] = 8'b00000000;
    ram[7906] = 8'b00000000;
    ram[7905] = 8'b00000000;
    ram[7904] = 8'b00000000;
    ram[7903] = 8'b00000000;
    ram[7902] = 8'b00000000;
    ram[7901] = 8'b00000000;
    ram[7900] = 8'b00000000;
    ram[7899] = 8'b00000000;
    ram[7898] = 8'b00000000;
    ram[7897] = 8'b00000000;
    ram[7896] = 8'b00000000;
    ram[7895] = 8'b00000000;
    ram[7894] = 8'b00000000;
    ram[7893] = 8'b00000000;
    ram[7892] = 8'b00000000;
    ram[7891] = 8'b00000000;
    ram[7890] = 8'b00000000;
    ram[7889] = 8'b00000000;
    ram[7888] = 8'b00000000;
    ram[7887] = 8'b00000000;
    ram[7886] = 8'b00000000;
    ram[7885] = 8'b00000000;
    ram[7884] = 8'b00000000;
    ram[7883] = 8'b00000000;
    ram[7882] = 8'b00000000;
    ram[7881] = 8'b00000000;
    ram[7880] = 8'b00000000;
    ram[7879] = 8'b00000000;
    ram[7878] = 8'b00000000;
    ram[7877] = 8'b00000000;
    ram[7876] = 8'b00000000;
    ram[7875] = 8'b00000000;
    ram[7874] = 8'b00000000;
    ram[7873] = 8'b00000000;
    ram[7872] = 8'b00000000;
    ram[7871] = 8'b00000000;
    ram[7870] = 8'b00000000;
    ram[7869] = 8'b00000000;
    ram[7868] = 8'b00000000;
    ram[7867] = 8'b00000000;
    ram[7866] = 8'b00000000;
    ram[7865] = 8'b00000000;
    ram[7864] = 8'b00000000;
    ram[7863] = 8'b00000000;
    ram[7862] = 8'b00000000;
    ram[7861] = 8'b00000000;
    ram[7860] = 8'b00000000;
    ram[7859] = 8'b00000000;
    ram[7858] = 8'b00000000;
    ram[7857] = 8'b00000000;
    ram[7856] = 8'b00000000;
    ram[7855] = 8'b00000000;
    ram[7854] = 8'b00000000;
    ram[7853] = 8'b00000000;
    ram[7852] = 8'b00000000;
    ram[7851] = 8'b00000000;
    ram[7850] = 8'b00000000;
    ram[7849] = 8'b00000000;
    ram[7848] = 8'b00000000;
    ram[7847] = 8'b00000000;
    ram[7846] = 8'b00000000;
    ram[7845] = 8'b00000000;
    ram[7844] = 8'b00000000;
    ram[7843] = 8'b00000000;
    ram[7842] = 8'b00000000;
    ram[7841] = 8'b00000000;
    ram[7840] = 8'b00000000;
    ram[7839] = 8'b00000000;
    ram[7838] = 8'b00000000;
    ram[7837] = 8'b00000000;
    ram[7836] = 8'b00000000;
    ram[7835] = 8'b00000000;
    ram[7834] = 8'b00000000;
    ram[7833] = 8'b00000000;
    ram[7832] = 8'b00000000;
    ram[7831] = 8'b00000000;
    ram[7830] = 8'b00000000;
    ram[7829] = 8'b00000000;
    ram[7828] = 8'b00000000;
    ram[7827] = 8'b00000000;
    ram[7826] = 8'b00000000;
    ram[7825] = 8'b00000000;
    ram[7824] = 8'b00000000;
    ram[7823] = 8'b00000000;
    ram[7822] = 8'b00000000;
    ram[7821] = 8'b00000000;
    ram[7820] = 8'b00000000;
    ram[7819] = 8'b00000000;
    ram[7818] = 8'b00000000;
    ram[7817] = 8'b00000000;
    ram[7816] = 8'b00000000;
    ram[7815] = 8'b00000000;
    ram[7814] = 8'b00000000;
    ram[7813] = 8'b00000000;
    ram[7812] = 8'b00000000;
    ram[7811] = 8'b00000000;
    ram[7810] = 8'b00000000;
    ram[7809] = 8'b00000000;
    ram[7808] = 8'b00000000;
    ram[7807] = 8'b00000000;
    ram[7806] = 8'b00000000;
    ram[7805] = 8'b00000000;
    ram[7804] = 8'b00000000;
    ram[7803] = 8'b00000000;
    ram[7802] = 8'b00000000;
    ram[7801] = 8'b00000000;
    ram[7800] = 8'b00000000;
    ram[7799] = 8'b00000000;
    ram[7798] = 8'b00000000;
    ram[7797] = 8'b00000000;
    ram[7796] = 8'b00000000;
    ram[7795] = 8'b00000000;
    ram[7794] = 8'b00000000;
    ram[7793] = 8'b00000000;
    ram[7792] = 8'b00000000;
    ram[7791] = 8'b00000000;
    ram[7790] = 8'b00000000;
    ram[7789] = 8'b00000000;
    ram[7788] = 8'b00000000;
    ram[7787] = 8'b00000000;
    ram[7786] = 8'b00000000;
    ram[7785] = 8'b00000000;
    ram[7784] = 8'b00000000;
    ram[7783] = 8'b00000000;
    ram[7782] = 8'b00000000;
    ram[7781] = 8'b00000000;
    ram[7780] = 8'b00000000;
    ram[7779] = 8'b00000000;
    ram[7778] = 8'b00000000;
    ram[7777] = 8'b00000000;
    ram[7776] = 8'b00000000;
    ram[7775] = 8'b00000000;
    ram[7774] = 8'b00000000;
    ram[7773] = 8'b00000000;
    ram[7772] = 8'b00000000;
    ram[7771] = 8'b00000000;
    ram[7770] = 8'b00000000;
    ram[7769] = 8'b00000000;
    ram[7768] = 8'b00000000;
    ram[7767] = 8'b00000000;
    ram[7766] = 8'b00000000;
    ram[7765] = 8'b00000000;
    ram[7764] = 8'b00000000;
    ram[7763] = 8'b00000000;
    ram[7762] = 8'b00000000;
    ram[7761] = 8'b00000000;
    ram[7760] = 8'b00000000;
    ram[7759] = 8'b00000000;
    ram[7758] = 8'b00000000;
    ram[7757] = 8'b00000000;
    ram[7756] = 8'b00000000;
    ram[7755] = 8'b00000000;
    ram[7754] = 8'b00000000;
    ram[7753] = 8'b00000000;
    ram[7752] = 8'b00000000;
    ram[7751] = 8'b00000000;
    ram[7750] = 8'b00000000;
    ram[7749] = 8'b00000000;
    ram[7748] = 8'b00000000;
    ram[7747] = 8'b00000000;
    ram[7746] = 8'b00000000;
    ram[7745] = 8'b00000000;
    ram[7744] = 8'b00000000;
    ram[7743] = 8'b00000000;
    ram[7742] = 8'b00000000;
    ram[7741] = 8'b00000000;
    ram[7740] = 8'b00000000;
    ram[7739] = 8'b00000000;
    ram[7738] = 8'b00000000;
    ram[7737] = 8'b00000000;
    ram[7736] = 8'b00000000;
    ram[7735] = 8'b00000000;
    ram[7734] = 8'b00000000;
    ram[7733] = 8'b00000000;
    ram[7732] = 8'b00000000;
    ram[7731] = 8'b00000000;
    ram[7730] = 8'b00000000;
    ram[7729] = 8'b00000000;
    ram[7728] = 8'b00000000;
    ram[7727] = 8'b00000000;
    ram[7726] = 8'b00000000;
    ram[7725] = 8'b00000000;
    ram[7724] = 8'b00000000;
    ram[7723] = 8'b00000000;
    ram[7722] = 8'b00000000;
    ram[7721] = 8'b00000000;
    ram[7720] = 8'b00000000;
    ram[7719] = 8'b00000000;
    ram[7718] = 8'b00000000;
    ram[7717] = 8'b00000000;
    ram[7716] = 8'b00000000;
    ram[7715] = 8'b00000000;
    ram[7714] = 8'b00000000;
    ram[7713] = 8'b00000000;
    ram[7712] = 8'b00000000;
    ram[7711] = 8'b00000000;
    ram[7710] = 8'b00000000;
    ram[7709] = 8'b00000000;
    ram[7708] = 8'b00000000;
    ram[7707] = 8'b00000000;
    ram[7706] = 8'b00000000;
    ram[7705] = 8'b00000000;
    ram[7704] = 8'b00000000;
    ram[7703] = 8'b00000000;
    ram[7702] = 8'b00000000;
    ram[7701] = 8'b00000000;
    ram[7700] = 8'b00000000;
    ram[7699] = 8'b00000000;
    ram[7698] = 8'b00000000;
    ram[7697] = 8'b00000000;
    ram[7696] = 8'b00000000;
    ram[7695] = 8'b00000000;
    ram[7694] = 8'b00000000;
    ram[7693] = 8'b00000000;
    ram[7692] = 8'b00000000;
    ram[7691] = 8'b00000000;
    ram[7690] = 8'b00000000;
    ram[7689] = 8'b00000000;
    ram[7688] = 8'b00000000;
    ram[7687] = 8'b00000000;
    ram[7686] = 8'b00000000;
    ram[7685] = 8'b00000000;
    ram[7684] = 8'b00000000;
    ram[7683] = 8'b00000000;
    ram[7682] = 8'b00000000;
    ram[7681] = 8'b00000000;
    ram[7680] = 8'b00000000;
    ram[7679] = 8'b00000000;
    ram[7678] = 8'b00000000;
    ram[7677] = 8'b00000000;
    ram[7676] = 8'b00000000;
    ram[7675] = 8'b00000000;
    ram[7674] = 8'b00000000;
    ram[7673] = 8'b00000000;
    ram[7672] = 8'b00000000;
    ram[7671] = 8'b00000000;
    ram[7670] = 8'b00000000;
    ram[7669] = 8'b00000000;
    ram[7668] = 8'b00000000;
    ram[7667] = 8'b00000000;
    ram[7666] = 8'b00000000;
    ram[7665] = 8'b00000000;
    ram[7664] = 8'b00000000;
    ram[7663] = 8'b00000000;
    ram[7662] = 8'b00000000;
    ram[7661] = 8'b00000000;
    ram[7660] = 8'b00000000;
    ram[7659] = 8'b00000000;
    ram[7658] = 8'b00000000;
    ram[7657] = 8'b00000000;
    ram[7656] = 8'b00000000;
    ram[7655] = 8'b00000000;
    ram[7654] = 8'b00000000;
    ram[7653] = 8'b00000000;
    ram[7652] = 8'b00000000;
    ram[7651] = 8'b00000000;
    ram[7650] = 8'b00000000;
    ram[7649] = 8'b00000000;
    ram[7648] = 8'b00000000;
    ram[7647] = 8'b00000000;
    ram[7646] = 8'b00000000;
    ram[7645] = 8'b00000000;
    ram[7644] = 8'b00000000;
    ram[7643] = 8'b00000000;
    ram[7642] = 8'b00000000;
    ram[7641] = 8'b00000000;
    ram[7640] = 8'b00000000;
    ram[7639] = 8'b00000000;
    ram[7638] = 8'b00000000;
    ram[7637] = 8'b00000000;
    ram[7636] = 8'b00000000;
    ram[7635] = 8'b00000000;
    ram[7634] = 8'b00000000;
    ram[7633] = 8'b00000000;
    ram[7632] = 8'b00000000;
    ram[7631] = 8'b00000000;
    ram[7630] = 8'b00000000;
    ram[7629] = 8'b00000000;
    ram[7628] = 8'b00000000;
    ram[7627] = 8'b00000000;
    ram[7626] = 8'b00000000;
    ram[7625] = 8'b00000000;
    ram[7624] = 8'b00000000;
    ram[7623] = 8'b00000000;
    ram[7622] = 8'b00000000;
    ram[7621] = 8'b00000000;
    ram[7620] = 8'b00000000;
    ram[7619] = 8'b00000000;
    ram[7618] = 8'b00000000;
    ram[7617] = 8'b00000000;
    ram[7616] = 8'b00000000;
    ram[7615] = 8'b00000000;
    ram[7614] = 8'b00000000;
    ram[7613] = 8'b00000000;
    ram[7612] = 8'b00000000;
    ram[7611] = 8'b00000000;
    ram[7610] = 8'b00000000;
    ram[7609] = 8'b00000000;
    ram[7608] = 8'b00000000;
    ram[7607] = 8'b00000000;
    ram[7606] = 8'b00000000;
    ram[7605] = 8'b00000000;
    ram[7604] = 8'b00000000;
    ram[7603] = 8'b00000000;
    ram[7602] = 8'b00000000;
    ram[7601] = 8'b00000000;
    ram[7600] = 8'b00000000;
    ram[7599] = 8'b00000000;
    ram[7598] = 8'b00000000;
    ram[7597] = 8'b00000000;
    ram[7596] = 8'b00000000;
    ram[7595] = 8'b00000000;
    ram[7594] = 8'b00000000;
    ram[7593] = 8'b00000000;
    ram[7592] = 8'b00000000;
    ram[7591] = 8'b00000000;
    ram[7590] = 8'b00000000;
    ram[7589] = 8'b00000000;
    ram[7588] = 8'b00000000;
    ram[7587] = 8'b00000000;
    ram[7586] = 8'b00000000;
    ram[7585] = 8'b00000000;
    ram[7584] = 8'b00000000;
    ram[7583] = 8'b00000000;
    ram[7582] = 8'b00000000;
    ram[7581] = 8'b00000000;
    ram[7580] = 8'b00000000;
    ram[7579] = 8'b00000000;
    ram[7578] = 8'b00000000;
    ram[7577] = 8'b00000000;
    ram[7576] = 8'b00000000;
    ram[7575] = 8'b00000000;
    ram[7574] = 8'b00000000;
    ram[7573] = 8'b00000000;
    ram[7572] = 8'b00000000;
    ram[7571] = 8'b00000000;
    ram[7570] = 8'b00000000;
    ram[7569] = 8'b00000000;
    ram[7568] = 8'b00000000;
    ram[7567] = 8'b00000000;
    ram[7566] = 8'b00000000;
    ram[7565] = 8'b00000000;
    ram[7564] = 8'b00000000;
    ram[7563] = 8'b00000000;
    ram[7562] = 8'b00000000;
    ram[7561] = 8'b00000000;
    ram[7560] = 8'b00000000;
    ram[7559] = 8'b00000000;
    ram[7558] = 8'b00000000;
    ram[7557] = 8'b00000000;
    ram[7556] = 8'b00000000;
    ram[7555] = 8'b00000000;
    ram[7554] = 8'b00000000;
    ram[7553] = 8'b00000000;
    ram[7552] = 8'b00000000;
    ram[7551] = 8'b00000000;
    ram[7550] = 8'b00000000;
    ram[7549] = 8'b00000000;
    ram[7548] = 8'b00000000;
    ram[7547] = 8'b00000000;
    ram[7546] = 8'b00000000;
    ram[7545] = 8'b00000000;
    ram[7544] = 8'b00000000;
    ram[7543] = 8'b00000000;
    ram[7542] = 8'b00000000;
    ram[7541] = 8'b00000000;
    ram[7540] = 8'b00000000;
    ram[7539] = 8'b00000000;
    ram[7538] = 8'b00000000;
    ram[7537] = 8'b00000000;
    ram[7536] = 8'b00000000;
    ram[7535] = 8'b00000000;
    ram[7534] = 8'b00000000;
    ram[7533] = 8'b00000000;
    ram[7532] = 8'b00000000;
    ram[7531] = 8'b00000000;
    ram[7530] = 8'b00000000;
    ram[7529] = 8'b00000000;
    ram[7528] = 8'b00000000;
    ram[7527] = 8'b00000000;
    ram[7526] = 8'b00000000;
    ram[7525] = 8'b00000000;
    ram[7524] = 8'b00000000;
    ram[7523] = 8'b00000000;
    ram[7522] = 8'b00000000;
    ram[7521] = 8'b00000000;
    ram[7520] = 8'b00000000;
    ram[7519] = 8'b00000000;
    ram[7518] = 8'b00000000;
    ram[7517] = 8'b00000000;
    ram[7516] = 8'b00000000;
    ram[7515] = 8'b00000000;
    ram[7514] = 8'b00000000;
    ram[7513] = 8'b00000000;
    ram[7512] = 8'b00000000;
    ram[7511] = 8'b00000000;
    ram[7510] = 8'b00000000;
    ram[7509] = 8'b00000000;
    ram[7508] = 8'b00000000;
    ram[7507] = 8'b00000000;
    ram[7506] = 8'b00000000;
    ram[7505] = 8'b00000000;
    ram[7504] = 8'b00000000;
    ram[7503] = 8'b00000000;
    ram[7502] = 8'b00000000;
    ram[7501] = 8'b00000000;
    ram[7500] = 8'b00000000;
    ram[7499] = 8'b00000000;
    ram[7498] = 8'b00000000;
    ram[7497] = 8'b00000000;
    ram[7496] = 8'b00000000;
    ram[7495] = 8'b00000000;
    ram[7494] = 8'b00000000;
    ram[7493] = 8'b00000000;
    ram[7492] = 8'b00000000;
    ram[7491] = 8'b00000000;
    ram[7490] = 8'b00000000;
    ram[7489] = 8'b00000000;
    ram[7488] = 8'b00000000;
    ram[7487] = 8'b00000000;
    ram[7486] = 8'b00000000;
    ram[7485] = 8'b00000000;
    ram[7484] = 8'b00000000;
    ram[7483] = 8'b00000000;
    ram[7482] = 8'b00000000;
    ram[7481] = 8'b00000000;
    ram[7480] = 8'b00000000;
    ram[7479] = 8'b00000000;
    ram[7478] = 8'b00000000;
    ram[7477] = 8'b00000000;
    ram[7476] = 8'b00000000;
    ram[7475] = 8'b00000000;
    ram[7474] = 8'b00000000;
    ram[7473] = 8'b00000000;
    ram[7472] = 8'b00000000;
    ram[7471] = 8'b00000000;
    ram[7470] = 8'b00000000;
    ram[7469] = 8'b00000000;
    ram[7468] = 8'b00000000;
    ram[7467] = 8'b00000000;
    ram[7466] = 8'b00000000;
    ram[7465] = 8'b00000000;
    ram[7464] = 8'b00000000;
    ram[7463] = 8'b00000000;
    ram[7462] = 8'b00000000;
    ram[7461] = 8'b00000000;
    ram[7460] = 8'b00000000;
    ram[7459] = 8'b00000000;
    ram[7458] = 8'b00000000;
    ram[7457] = 8'b00000000;
    ram[7456] = 8'b00000000;
    ram[7455] = 8'b00000000;
    ram[7454] = 8'b00000000;
    ram[7453] = 8'b00000000;
    ram[7452] = 8'b00000000;
    ram[7451] = 8'b00000000;
    ram[7450] = 8'b00000000;
    ram[7449] = 8'b00000000;
    ram[7448] = 8'b00000000;
    ram[7447] = 8'b00000000;
    ram[7446] = 8'b00000000;
    ram[7445] = 8'b00000000;
    ram[7444] = 8'b00000000;
    ram[7443] = 8'b00000000;
    ram[7442] = 8'b00000000;
    ram[7441] = 8'b00000000;
    ram[7440] = 8'b00000000;
    ram[7439] = 8'b00000000;
    ram[7438] = 8'b00000000;
    ram[7437] = 8'b00000000;
    ram[7436] = 8'b00000000;
    ram[7435] = 8'b00000000;
    ram[7434] = 8'b00000000;
    ram[7433] = 8'b00000000;
    ram[7432] = 8'b00000000;
    ram[7431] = 8'b00000000;
    ram[7430] = 8'b00000000;
    ram[7429] = 8'b00000000;
    ram[7428] = 8'b00000000;
    ram[7427] = 8'b00000000;
    ram[7426] = 8'b00000000;
    ram[7425] = 8'b00000000;
    ram[7424] = 8'b00000000;
    ram[7423] = 8'b00000000;
    ram[7422] = 8'b00000000;
    ram[7421] = 8'b00000000;
    ram[7420] = 8'b00000000;
    ram[7419] = 8'b00000000;
    ram[7418] = 8'b00000000;
    ram[7417] = 8'b00000000;
    ram[7416] = 8'b00000000;
    ram[7415] = 8'b00000000;
    ram[7414] = 8'b00000000;
    ram[7413] = 8'b00000000;
    ram[7412] = 8'b00000000;
    ram[7411] = 8'b00000000;
    ram[7410] = 8'b00000000;
    ram[7409] = 8'b00000000;
    ram[7408] = 8'b00000000;
    ram[7407] = 8'b00000000;
    ram[7406] = 8'b00000000;
    ram[7405] = 8'b00000000;
    ram[7404] = 8'b00000000;
    ram[7403] = 8'b00000000;
    ram[7402] = 8'b00000000;
    ram[7401] = 8'b00000000;
    ram[7400] = 8'b00000000;
    ram[7399] = 8'b00000000;
    ram[7398] = 8'b00000000;
    ram[7397] = 8'b00000000;
    ram[7396] = 8'b00000000;
    ram[7395] = 8'b00000000;
    ram[7394] = 8'b00000000;
    ram[7393] = 8'b00000000;
    ram[7392] = 8'b00000000;
    ram[7391] = 8'b00000000;
    ram[7390] = 8'b00000000;
    ram[7389] = 8'b00000000;
    ram[7388] = 8'b00000000;
    ram[7387] = 8'b00000000;
    ram[7386] = 8'b00000000;
    ram[7385] = 8'b00000000;
    ram[7384] = 8'b00000000;
    ram[7383] = 8'b00000000;
    ram[7382] = 8'b00000000;
    ram[7381] = 8'b00000000;
    ram[7380] = 8'b00000000;
    ram[7379] = 8'b00000000;
    ram[7378] = 8'b00000000;
    ram[7377] = 8'b00000000;
    ram[7376] = 8'b00000000;
    ram[7375] = 8'b00000000;
    ram[7374] = 8'b00000000;
    ram[7373] = 8'b00000000;
    ram[7372] = 8'b00000000;
    ram[7371] = 8'b00000000;
    ram[7370] = 8'b00000000;
    ram[7369] = 8'b00000000;
    ram[7368] = 8'b00000000;
    ram[7367] = 8'b00000000;
    ram[7366] = 8'b00000000;
    ram[7365] = 8'b00000000;
    ram[7364] = 8'b00000000;
    ram[7363] = 8'b00000000;
    ram[7362] = 8'b00000000;
    ram[7361] = 8'b00000000;
    ram[7360] = 8'b00000000;
    ram[7359] = 8'b00000000;
    ram[7358] = 8'b00000000;
    ram[7357] = 8'b00000000;
    ram[7356] = 8'b00000000;
    ram[7355] = 8'b00000000;
    ram[7354] = 8'b00000000;
    ram[7353] = 8'b00000000;
    ram[7352] = 8'b00000000;
    ram[7351] = 8'b00000000;
    ram[7350] = 8'b00000000;
    ram[7349] = 8'b00000000;
    ram[7348] = 8'b00000000;
    ram[7347] = 8'b00000000;
    ram[7346] = 8'b00000000;
    ram[7345] = 8'b00000000;
    ram[7344] = 8'b00000000;
    ram[7343] = 8'b00000000;
    ram[7342] = 8'b00000000;
    ram[7341] = 8'b00000000;
    ram[7340] = 8'b00000000;
    ram[7339] = 8'b00000000;
    ram[7338] = 8'b00000000;
    ram[7337] = 8'b00000000;
    ram[7336] = 8'b00000000;
    ram[7335] = 8'b00000000;
    ram[7334] = 8'b00000000;
    ram[7333] = 8'b00000000;
    ram[7332] = 8'b00000000;
    ram[7331] = 8'b00000000;
    ram[7330] = 8'b00000000;
    ram[7329] = 8'b00000000;
    ram[7328] = 8'b00000000;
    ram[7327] = 8'b00000000;
    ram[7326] = 8'b00000000;
    ram[7325] = 8'b00000000;
    ram[7324] = 8'b00000000;
    ram[7323] = 8'b00000000;
    ram[7322] = 8'b00000000;
    ram[7321] = 8'b00000000;
    ram[7320] = 8'b00000000;
    ram[7319] = 8'b00000000;
    ram[7318] = 8'b00000000;
    ram[7317] = 8'b00000000;
    ram[7316] = 8'b00000000;
    ram[7315] = 8'b00000000;
    ram[7314] = 8'b00000000;
    ram[7313] = 8'b00000000;
    ram[7312] = 8'b00000000;
    ram[7311] = 8'b00000000;
    ram[7310] = 8'b00000000;
    ram[7309] = 8'b00000000;
    ram[7308] = 8'b00000000;
    ram[7307] = 8'b00000000;
    ram[7306] = 8'b00000000;
    ram[7305] = 8'b00000000;
    ram[7304] = 8'b00000000;
    ram[7303] = 8'b00000000;
    ram[7302] = 8'b00000000;
    ram[7301] = 8'b00000000;
    ram[7300] = 8'b00000000;
    ram[7299] = 8'b00000000;
    ram[7298] = 8'b00000000;
    ram[7297] = 8'b00000000;
    ram[7296] = 8'b00000000;
    ram[7295] = 8'b00000000;
    ram[7294] = 8'b00000000;
    ram[7293] = 8'b00000000;
    ram[7292] = 8'b00000000;
    ram[7291] = 8'b00000000;
    ram[7290] = 8'b00000000;
    ram[7289] = 8'b00000000;
    ram[7288] = 8'b00000000;
    ram[7287] = 8'b00000000;
    ram[7286] = 8'b00000000;
    ram[7285] = 8'b00000000;
    ram[7284] = 8'b00000000;
    ram[7283] = 8'b00000000;
    ram[7282] = 8'b00000000;
    ram[7281] = 8'b00000000;
    ram[7280] = 8'b00000000;
    ram[7279] = 8'b00000000;
    ram[7278] = 8'b00000000;
    ram[7277] = 8'b00000000;
    ram[7276] = 8'b00000000;
    ram[7275] = 8'b00000000;
    ram[7274] = 8'b00000000;
    ram[7273] = 8'b00000000;
    ram[7272] = 8'b00000000;
    ram[7271] = 8'b00000000;
    ram[7270] = 8'b00000000;
    ram[7269] = 8'b00000000;
    ram[7268] = 8'b00000000;
    ram[7267] = 8'b00000000;
    ram[7266] = 8'b00000000;
    ram[7265] = 8'b00000000;
    ram[7264] = 8'b00000000;
    ram[7263] = 8'b00000000;
    ram[7262] = 8'b00000000;
    ram[7261] = 8'b00000000;
    ram[7260] = 8'b00000000;
    ram[7259] = 8'b00000000;
    ram[7258] = 8'b00000000;
    ram[7257] = 8'b00000000;
    ram[7256] = 8'b00000000;
    ram[7255] = 8'b00000000;
    ram[7254] = 8'b00000000;
    ram[7253] = 8'b00000000;
    ram[7252] = 8'b00000000;
    ram[7251] = 8'b00000000;
    ram[7250] = 8'b00000000;
    ram[7249] = 8'b00000000;
    ram[7248] = 8'b00000000;
    ram[7247] = 8'b00000000;
    ram[7246] = 8'b00000000;
    ram[7245] = 8'b00000000;
    ram[7244] = 8'b00000000;
    ram[7243] = 8'b00000000;
    ram[7242] = 8'b00000000;
    ram[7241] = 8'b00000000;
    ram[7240] = 8'b00000000;
    ram[7239] = 8'b00000000;
    ram[7238] = 8'b00000000;
    ram[7237] = 8'b00000000;
    ram[7236] = 8'b00000000;
    ram[7235] = 8'b00000000;
    ram[7234] = 8'b00000000;
    ram[7233] = 8'b00000000;
    ram[7232] = 8'b00000000;
    ram[7231] = 8'b00000000;
    ram[7230] = 8'b00000000;
    ram[7229] = 8'b00000000;
    ram[7228] = 8'b00000000;
    ram[7227] = 8'b00000000;
    ram[7226] = 8'b00000000;
    ram[7225] = 8'b00000000;
    ram[7224] = 8'b00000000;
    ram[7223] = 8'b00000000;
    ram[7222] = 8'b00000000;
    ram[7221] = 8'b00000000;
    ram[7220] = 8'b00000000;
    ram[7219] = 8'b00000000;
    ram[7218] = 8'b00000000;
    ram[7217] = 8'b00000000;
    ram[7216] = 8'b00000000;
    ram[7215] = 8'b00000000;
    ram[7214] = 8'b00000000;
    ram[7213] = 8'b00000000;
    ram[7212] = 8'b00000000;
    ram[7211] = 8'b00000000;
    ram[7210] = 8'b00000000;
    ram[7209] = 8'b00000000;
    ram[7208] = 8'b00000000;
    ram[7207] = 8'b00000000;
    ram[7206] = 8'b00000000;
    ram[7205] = 8'b00000000;
    ram[7204] = 8'b00000000;
    ram[7203] = 8'b00000000;
    ram[7202] = 8'b00000000;
    ram[7201] = 8'b00000000;
    ram[7200] = 8'b00000000;
    ram[7199] = 8'b00000000;
    ram[7198] = 8'b00000000;
    ram[7197] = 8'b00000000;
    ram[7196] = 8'b00000000;
    ram[7195] = 8'b00000000;
    ram[7194] = 8'b00000000;
    ram[7193] = 8'b00000000;
    ram[7192] = 8'b00000000;
    ram[7191] = 8'b00000000;
    ram[7190] = 8'b00000000;
    ram[7189] = 8'b00000000;
    ram[7188] = 8'b00000000;
    ram[7187] = 8'b00000000;
    ram[7186] = 8'b00000000;
    ram[7185] = 8'b00000000;
    ram[7184] = 8'b00000000;
    ram[7183] = 8'b00000000;
    ram[7182] = 8'b00000000;
    ram[7181] = 8'b00000000;
    ram[7180] = 8'b00000000;
    ram[7179] = 8'b00000000;
    ram[7178] = 8'b00000000;
    ram[7177] = 8'b00000000;
    ram[7176] = 8'b00000000;
    ram[7175] = 8'b00000000;
    ram[7174] = 8'b00000000;
    ram[7173] = 8'b00000000;
    ram[7172] = 8'b00000000;
    ram[7171] = 8'b00000000;
    ram[7170] = 8'b00000000;
    ram[7169] = 8'b00000000;
    ram[7168] = 8'b00000000;
    ram[7167] = 8'b00000000;
    ram[7166] = 8'b00000000;
    ram[7165] = 8'b00000000;
    ram[7164] = 8'b00000000;
    ram[7163] = 8'b00000000;
    ram[7162] = 8'b00000000;
    ram[7161] = 8'b00000000;
    ram[7160] = 8'b00000000;
    ram[7159] = 8'b00000000;
    ram[7158] = 8'b00000000;
    ram[7157] = 8'b00000000;
    ram[7156] = 8'b00000000;
    ram[7155] = 8'b00000000;
    ram[7154] = 8'b00000000;
    ram[7153] = 8'b00000000;
    ram[7152] = 8'b00000000;
    ram[7151] = 8'b00000000;
    ram[7150] = 8'b00000000;
    ram[7149] = 8'b00000000;
    ram[7148] = 8'b00000000;
    ram[7147] = 8'b00000000;
    ram[7146] = 8'b00000000;
    ram[7145] = 8'b00000000;
    ram[7144] = 8'b00000000;
    ram[7143] = 8'b00000000;
    ram[7142] = 8'b00000000;
    ram[7141] = 8'b00000000;
    ram[7140] = 8'b00000000;
    ram[7139] = 8'b00000000;
    ram[7138] = 8'b00000000;
    ram[7137] = 8'b00000000;
    ram[7136] = 8'b00000000;
    ram[7135] = 8'b00000000;
    ram[7134] = 8'b00000000;
    ram[7133] = 8'b00000000;
    ram[7132] = 8'b00000000;
    ram[7131] = 8'b00000000;
    ram[7130] = 8'b00000000;
    ram[7129] = 8'b00000000;
    ram[7128] = 8'b00000000;
    ram[7127] = 8'b00000000;
    ram[7126] = 8'b00000000;
    ram[7125] = 8'b00000000;
    ram[7124] = 8'b00000000;
    ram[7123] = 8'b00000000;
    ram[7122] = 8'b00000000;
    ram[7121] = 8'b00000000;
    ram[7120] = 8'b00000000;
    ram[7119] = 8'b00000000;
    ram[7118] = 8'b00000000;
    ram[7117] = 8'b00000000;
    ram[7116] = 8'b00000000;
    ram[7115] = 8'b00000000;
    ram[7114] = 8'b00000000;
    ram[7113] = 8'b00000000;
    ram[7112] = 8'b00000000;
    ram[7111] = 8'b00000000;
    ram[7110] = 8'b00000000;
    ram[7109] = 8'b00000000;
    ram[7108] = 8'b00000000;
    ram[7107] = 8'b00000000;
    ram[7106] = 8'b00000000;
    ram[7105] = 8'b00000000;
    ram[7104] = 8'b00000000;
    ram[7103] = 8'b00000000;
    ram[7102] = 8'b00000000;
    ram[7101] = 8'b00000000;
    ram[7100] = 8'b00000000;
    ram[7099] = 8'b00000000;
    ram[7098] = 8'b00000000;
    ram[7097] = 8'b00000000;
    ram[7096] = 8'b00000000;
    ram[7095] = 8'b00000000;
    ram[7094] = 8'b00000000;
    ram[7093] = 8'b00000000;
    ram[7092] = 8'b00000000;
    ram[7091] = 8'b00000000;
    ram[7090] = 8'b00000000;
    ram[7089] = 8'b00000000;
    ram[7088] = 8'b00000000;
    ram[7087] = 8'b00000000;
    ram[7086] = 8'b00000000;
    ram[7085] = 8'b00000000;
    ram[7084] = 8'b00000000;
    ram[7083] = 8'b00000000;
    ram[7082] = 8'b00000000;
    ram[7081] = 8'b00000000;
    ram[7080] = 8'b00000000;
    ram[7079] = 8'b00000000;
    ram[7078] = 8'b00000000;
    ram[7077] = 8'b00000000;
    ram[7076] = 8'b00000000;
    ram[7075] = 8'b00000000;
    ram[7074] = 8'b00000000;
    ram[7073] = 8'b00000000;
    ram[7072] = 8'b00000000;
    ram[7071] = 8'b00000000;
    ram[7070] = 8'b00000000;
    ram[7069] = 8'b00000000;
    ram[7068] = 8'b00000000;
    ram[7067] = 8'b00000000;
    ram[7066] = 8'b00000000;
    ram[7065] = 8'b00000000;
    ram[7064] = 8'b00000000;
    ram[7063] = 8'b00000000;
    ram[7062] = 8'b00000000;
    ram[7061] = 8'b00000000;
    ram[7060] = 8'b00000000;
    ram[7059] = 8'b00000000;
    ram[7058] = 8'b00000000;
    ram[7057] = 8'b00000000;
    ram[7056] = 8'b00000000;
    ram[7055] = 8'b00000000;
    ram[7054] = 8'b00000000;
    ram[7053] = 8'b00000000;
    ram[7052] = 8'b00000000;
    ram[7051] = 8'b00000000;
    ram[7050] = 8'b00000000;
    ram[7049] = 8'b00000000;
    ram[7048] = 8'b00000000;
    ram[7047] = 8'b00000000;
    ram[7046] = 8'b00000000;
    ram[7045] = 8'b00000000;
    ram[7044] = 8'b00000000;
    ram[7043] = 8'b00000000;
    ram[7042] = 8'b00000000;
    ram[7041] = 8'b00000000;
    ram[7040] = 8'b00000000;
    ram[7039] = 8'b00000000;
    ram[7038] = 8'b00000000;
    ram[7037] = 8'b00000000;
    ram[7036] = 8'b00000000;
    ram[7035] = 8'b00000000;
    ram[7034] = 8'b00000000;
    ram[7033] = 8'b00000000;
    ram[7032] = 8'b00000000;
    ram[7031] = 8'b00000000;
    ram[7030] = 8'b00000000;
    ram[7029] = 8'b00000000;
    ram[7028] = 8'b00000000;
    ram[7027] = 8'b00000000;
    ram[7026] = 8'b00000000;
    ram[7025] = 8'b00000000;
    ram[7024] = 8'b00000000;
    ram[7023] = 8'b00000000;
    ram[7022] = 8'b00000000;
    ram[7021] = 8'b00000000;
    ram[7020] = 8'b00000000;
    ram[7019] = 8'b00000000;
    ram[7018] = 8'b00000000;
    ram[7017] = 8'b00000000;
    ram[7016] = 8'b00000000;
    ram[7015] = 8'b00000000;
    ram[7014] = 8'b00000000;
    ram[7013] = 8'b00000000;
    ram[7012] = 8'b00000000;
    ram[7011] = 8'b00000000;
    ram[7010] = 8'b00000000;
    ram[7009] = 8'b00000000;
    ram[7008] = 8'b00000000;
    ram[7007] = 8'b00000000;
    ram[7006] = 8'b00000000;
    ram[7005] = 8'b00000000;
    ram[7004] = 8'b00000000;
    ram[7003] = 8'b00000000;
    ram[7002] = 8'b00000000;
    ram[7001] = 8'b00000000;
    ram[7000] = 8'b00000000;
    ram[6999] = 8'b00000000;
    ram[6998] = 8'b00000000;
    ram[6997] = 8'b00000000;
    ram[6996] = 8'b00000000;
    ram[6995] = 8'b00000000;
    ram[6994] = 8'b00000000;
    ram[6993] = 8'b00000000;
    ram[6992] = 8'b00000000;
    ram[6991] = 8'b00000000;
    ram[6990] = 8'b00000000;
    ram[6989] = 8'b00000000;
    ram[6988] = 8'b00000000;
    ram[6987] = 8'b00000000;
    ram[6986] = 8'b00000000;
    ram[6985] = 8'b00000000;
    ram[6984] = 8'b00000000;
    ram[6983] = 8'b00000000;
    ram[6982] = 8'b00000000;
    ram[6981] = 8'b00000000;
    ram[6980] = 8'b00000000;
    ram[6979] = 8'b00000000;
    ram[6978] = 8'b00000000;
    ram[6977] = 8'b00000000;
    ram[6976] = 8'b00000000;
    ram[6975] = 8'b00000000;
    ram[6974] = 8'b00000000;
    ram[6973] = 8'b00000000;
    ram[6972] = 8'b00000000;
    ram[6971] = 8'b00000000;
    ram[6970] = 8'b00000000;
    ram[6969] = 8'b00000000;
    ram[6968] = 8'b00000000;
    ram[6967] = 8'b00000000;
    ram[6966] = 8'b00000000;
    ram[6965] = 8'b00000000;
    ram[6964] = 8'b00000000;
    ram[6963] = 8'b00000000;
    ram[6962] = 8'b00000000;
    ram[6961] = 8'b00000000;
    ram[6960] = 8'b00000000;
    ram[6959] = 8'b00000000;
    ram[6958] = 8'b00000000;
    ram[6957] = 8'b00000000;
    ram[6956] = 8'b00000000;
    ram[6955] = 8'b00000000;
    ram[6954] = 8'b00000000;
    ram[6953] = 8'b00000000;
    ram[6952] = 8'b00000000;
    ram[6951] = 8'b00000000;
    ram[6950] = 8'b00000000;
    ram[6949] = 8'b00000000;
    ram[6948] = 8'b00000000;
    ram[6947] = 8'b00000000;
    ram[6946] = 8'b00000000;
    ram[6945] = 8'b00000000;
    ram[6944] = 8'b00000000;
    ram[6943] = 8'b00000000;
    ram[6942] = 8'b00000000;
    ram[6941] = 8'b00000000;
    ram[6940] = 8'b00000000;
    ram[6939] = 8'b00000000;
    ram[6938] = 8'b00000000;
    ram[6937] = 8'b00000000;
    ram[6936] = 8'b00000000;
    ram[6935] = 8'b00000000;
    ram[6934] = 8'b00000000;
    ram[6933] = 8'b00000000;
    ram[6932] = 8'b00000000;
    ram[6931] = 8'b00000000;
    ram[6930] = 8'b00000000;
    ram[6929] = 8'b00000000;
    ram[6928] = 8'b00000000;
    ram[6927] = 8'b00000000;
    ram[6926] = 8'b00000000;
    ram[6925] = 8'b00000000;
    ram[6924] = 8'b00000000;
    ram[6923] = 8'b00000000;
    ram[6922] = 8'b00000000;
    ram[6921] = 8'b00000000;
    ram[6920] = 8'b00000000;
    ram[6919] = 8'b00000000;
    ram[6918] = 8'b00000000;
    ram[6917] = 8'b00000000;
    ram[6916] = 8'b00000000;
    ram[6915] = 8'b00000000;
    ram[6914] = 8'b00000000;
    ram[6913] = 8'b00000000;
    ram[6912] = 8'b00000000;
    ram[6911] = 8'b00000000;
    ram[6910] = 8'b00000000;
    ram[6909] = 8'b00000000;
    ram[6908] = 8'b00000000;
    ram[6907] = 8'b00000000;
    ram[6906] = 8'b00000000;
    ram[6905] = 8'b00000000;
    ram[6904] = 8'b00000000;
    ram[6903] = 8'b00000000;
    ram[6902] = 8'b00000000;
    ram[6901] = 8'b00000000;
    ram[6900] = 8'b00000000;
    ram[6899] = 8'b00000000;
    ram[6898] = 8'b00000000;
    ram[6897] = 8'b00000000;
    ram[6896] = 8'b00000000;
    ram[6895] = 8'b00000000;
    ram[6894] = 8'b00000000;
    ram[6893] = 8'b00000000;
    ram[6892] = 8'b00000000;
    ram[6891] = 8'b00000000;
    ram[6890] = 8'b00000000;
    ram[6889] = 8'b00000000;
    ram[6888] = 8'b00000000;
    ram[6887] = 8'b00000000;
    ram[6886] = 8'b00000000;
    ram[6885] = 8'b00000000;
    ram[6884] = 8'b00000000;
    ram[6883] = 8'b00000000;
    ram[6882] = 8'b00000000;
    ram[6881] = 8'b00000000;
    ram[6880] = 8'b00000000;
    ram[6879] = 8'b00000000;
    ram[6878] = 8'b00000000;
    ram[6877] = 8'b00000000;
    ram[6876] = 8'b00000000;
    ram[6875] = 8'b00000000;
    ram[6874] = 8'b00000000;
    ram[6873] = 8'b00000000;
    ram[6872] = 8'b00000000;
    ram[6871] = 8'b00000000;
    ram[6870] = 8'b00000000;
    ram[6869] = 8'b00000000;
    ram[6868] = 8'b00000000;
    ram[6867] = 8'b00000000;
    ram[6866] = 8'b00000000;
    ram[6865] = 8'b00000000;
    ram[6864] = 8'b00000000;
    ram[6863] = 8'b00000000;
    ram[6862] = 8'b00000000;
    ram[6861] = 8'b00000000;
    ram[6860] = 8'b00000000;
    ram[6859] = 8'b00000000;
    ram[6858] = 8'b00000000;
    ram[6857] = 8'b00000000;
    ram[6856] = 8'b00000000;
    ram[6855] = 8'b00000000;
    ram[6854] = 8'b00000000;
    ram[6853] = 8'b00000000;
    ram[6852] = 8'b00000000;
    ram[6851] = 8'b00000000;
    ram[6850] = 8'b00000000;
    ram[6849] = 8'b00000000;
    ram[6848] = 8'b00000000;
    ram[6847] = 8'b00000000;
    ram[6846] = 8'b00000000;
    ram[6845] = 8'b00000000;
    ram[6844] = 8'b00000000;
    ram[6843] = 8'b00000000;
    ram[6842] = 8'b00000000;
    ram[6841] = 8'b00000000;
    ram[6840] = 8'b00000000;
    ram[6839] = 8'b00000000;
    ram[6838] = 8'b00000000;
    ram[6837] = 8'b00000000;
    ram[6836] = 8'b00000000;
    ram[6835] = 8'b00000000;
    ram[6834] = 8'b00000000;
    ram[6833] = 8'b00000000;
    ram[6832] = 8'b00000000;
    ram[6831] = 8'b00000000;
    ram[6830] = 8'b00000000;
    ram[6829] = 8'b00000000;
    ram[6828] = 8'b00000000;
    ram[6827] = 8'b00000000;
    ram[6826] = 8'b00000000;
    ram[6825] = 8'b00000000;
    ram[6824] = 8'b00000000;
    ram[6823] = 8'b00000000;
    ram[6822] = 8'b00000000;
    ram[6821] = 8'b00000000;
    ram[6820] = 8'b00000000;
    ram[6819] = 8'b00000000;
    ram[6818] = 8'b00000000;
    ram[6817] = 8'b00000000;
    ram[6816] = 8'b00000000;
    ram[6815] = 8'b00000000;
    ram[6814] = 8'b00000000;
    ram[6813] = 8'b00000000;
    ram[6812] = 8'b00000000;
    ram[6811] = 8'b00000000;
    ram[6810] = 8'b00000000;
    ram[6809] = 8'b00000000;
    ram[6808] = 8'b00000000;
    ram[6807] = 8'b00000000;
    ram[6806] = 8'b00000000;
    ram[6805] = 8'b00000000;
    ram[6804] = 8'b00000000;
    ram[6803] = 8'b00000000;
    ram[6802] = 8'b00000000;
    ram[6801] = 8'b00000000;
    ram[6800] = 8'b00000000;
    ram[6799] = 8'b00000000;
    ram[6798] = 8'b00000000;
    ram[6797] = 8'b00000000;
    ram[6796] = 8'b00000000;
    ram[6795] = 8'b00000000;
    ram[6794] = 8'b00000000;
    ram[6793] = 8'b00000000;
    ram[6792] = 8'b00000000;
    ram[6791] = 8'b00000000;
    ram[6790] = 8'b00000000;
    ram[6789] = 8'b00000000;
    ram[6788] = 8'b00000000;
    ram[6787] = 8'b00000000;
    ram[6786] = 8'b00000000;
    ram[6785] = 8'b00000000;
    ram[6784] = 8'b00000000;
    ram[6783] = 8'b00000000;
    ram[6782] = 8'b00000000;
    ram[6781] = 8'b00000000;
    ram[6780] = 8'b00000000;
    ram[6779] = 8'b00000000;
    ram[6778] = 8'b00000000;
    ram[6777] = 8'b00000000;
    ram[6776] = 8'b00000000;
    ram[6775] = 8'b00000000;
    ram[6774] = 8'b00000000;
    ram[6773] = 8'b00000000;
    ram[6772] = 8'b00000000;
    ram[6771] = 8'b00000000;
    ram[6770] = 8'b00000000;
    ram[6769] = 8'b00000000;
    ram[6768] = 8'b00000000;
    ram[6767] = 8'b00000000;
    ram[6766] = 8'b00000000;
    ram[6765] = 8'b00000000;
    ram[6764] = 8'b00000000;
    ram[6763] = 8'b00000000;
    ram[6762] = 8'b00000000;
    ram[6761] = 8'b00000000;
    ram[6760] = 8'b00000000;
    ram[6759] = 8'b00000000;
    ram[6758] = 8'b00000000;
    ram[6757] = 8'b00000000;
    ram[6756] = 8'b00000000;
    ram[6755] = 8'b00000000;
    ram[6754] = 8'b00000000;
    ram[6753] = 8'b00000000;
    ram[6752] = 8'b00000000;
    ram[6751] = 8'b00000000;
    ram[6750] = 8'b00000000;
    ram[6749] = 8'b00000000;
    ram[6748] = 8'b00000000;
    ram[6747] = 8'b00000000;
    ram[6746] = 8'b00000000;
    ram[6745] = 8'b00000000;
    ram[6744] = 8'b00000000;
    ram[6743] = 8'b00000000;
    ram[6742] = 8'b00000000;
    ram[6741] = 8'b00000000;
    ram[6740] = 8'b00000000;
    ram[6739] = 8'b00000000;
    ram[6738] = 8'b00000000;
    ram[6737] = 8'b00000000;
    ram[6736] = 8'b00000000;
    ram[6735] = 8'b00000000;
    ram[6734] = 8'b00000000;
    ram[6733] = 8'b00000000;
    ram[6732] = 8'b00000000;
    ram[6731] = 8'b00000000;
    ram[6730] = 8'b00000000;
    ram[6729] = 8'b00000000;
    ram[6728] = 8'b00000000;
    ram[6727] = 8'b00000000;
    ram[6726] = 8'b00000000;
    ram[6725] = 8'b00000000;
    ram[6724] = 8'b00000000;
    ram[6723] = 8'b00000000;
    ram[6722] = 8'b00000000;
    ram[6721] = 8'b00000000;
    ram[6720] = 8'b00000000;
    ram[6719] = 8'b00000000;
    ram[6718] = 8'b00000000;
    ram[6717] = 8'b00000000;
    ram[6716] = 8'b00000000;
    ram[6715] = 8'b00000000;
    ram[6714] = 8'b00000000;
    ram[6713] = 8'b00000000;
    ram[6712] = 8'b00000000;
    ram[6711] = 8'b00000000;
    ram[6710] = 8'b00000000;
    ram[6709] = 8'b00000000;
    ram[6708] = 8'b00000000;
    ram[6707] = 8'b00000000;
    ram[6706] = 8'b00000000;
    ram[6705] = 8'b00000000;
    ram[6704] = 8'b00000000;
    ram[6703] = 8'b00000000;
    ram[6702] = 8'b00000000;
    ram[6701] = 8'b00000000;
    ram[6700] = 8'b00000000;
    ram[6699] = 8'b00000000;
    ram[6698] = 8'b00000000;
    ram[6697] = 8'b00000000;
    ram[6696] = 8'b00000000;
    ram[6695] = 8'b00000000;
    ram[6694] = 8'b00000000;
    ram[6693] = 8'b00000000;
    ram[6692] = 8'b00000000;
    ram[6691] = 8'b00000000;
    ram[6690] = 8'b00000000;
    ram[6689] = 8'b00000000;
    ram[6688] = 8'b00000000;
    ram[6687] = 8'b00000000;
    ram[6686] = 8'b00000000;
    ram[6685] = 8'b00000000;
    ram[6684] = 8'b00000000;
    ram[6683] = 8'b00000000;
    ram[6682] = 8'b00000000;
    ram[6681] = 8'b00000000;
    ram[6680] = 8'b00000000;
    ram[6679] = 8'b00000000;
    ram[6678] = 8'b00000000;
    ram[6677] = 8'b00000000;
    ram[6676] = 8'b00000000;
    ram[6675] = 8'b00000000;
    ram[6674] = 8'b00000000;
    ram[6673] = 8'b00000000;
    ram[6672] = 8'b00000000;
    ram[6671] = 8'b00000000;
    ram[6670] = 8'b00000000;
    ram[6669] = 8'b00000000;
    ram[6668] = 8'b00000000;
    ram[6667] = 8'b00000000;
    ram[6666] = 8'b00000000;
    ram[6665] = 8'b00000000;
    ram[6664] = 8'b00000000;
    ram[6663] = 8'b00000000;
    ram[6662] = 8'b00000000;
    ram[6661] = 8'b00000000;
    ram[6660] = 8'b00000000;
    ram[6659] = 8'b00000000;
    ram[6658] = 8'b00000000;
    ram[6657] = 8'b00000000;
    ram[6656] = 8'b00000000;
    ram[6655] = 8'b00000000;
    ram[6654] = 8'b00000000;
    ram[6653] = 8'b00000000;
    ram[6652] = 8'b00000000;
    ram[6651] = 8'b00000000;
    ram[6650] = 8'b00000000;
    ram[6649] = 8'b00000000;
    ram[6648] = 8'b00000000;
    ram[6647] = 8'b00000000;
    ram[6646] = 8'b00000000;
    ram[6645] = 8'b00000000;
    ram[6644] = 8'b00000000;
    ram[6643] = 8'b00000000;
    ram[6642] = 8'b00000000;
    ram[6641] = 8'b00000000;
    ram[6640] = 8'b00000000;
    ram[6639] = 8'b00000000;
    ram[6638] = 8'b00000000;
    ram[6637] = 8'b00000000;
    ram[6636] = 8'b00000000;
    ram[6635] = 8'b00000000;
    ram[6634] = 8'b00000000;
    ram[6633] = 8'b00000000;
    ram[6632] = 8'b00000000;
    ram[6631] = 8'b00000000;
    ram[6630] = 8'b00000000;
    ram[6629] = 8'b00000000;
    ram[6628] = 8'b00000000;
    ram[6627] = 8'b00000000;
    ram[6626] = 8'b00000000;
    ram[6625] = 8'b00000000;
    ram[6624] = 8'b00000000;
    ram[6623] = 8'b00000000;
    ram[6622] = 8'b00000000;
    ram[6621] = 8'b00000000;
    ram[6620] = 8'b00000000;
    ram[6619] = 8'b00000000;
    ram[6618] = 8'b00000000;
    ram[6617] = 8'b00000000;
    ram[6616] = 8'b00000000;
    ram[6615] = 8'b00000000;
    ram[6614] = 8'b00000000;
    ram[6613] = 8'b00000000;
    ram[6612] = 8'b00000000;
    ram[6611] = 8'b00000000;
    ram[6610] = 8'b00000000;
    ram[6609] = 8'b00000000;
    ram[6608] = 8'b00000000;
    ram[6607] = 8'b00000000;
    ram[6606] = 8'b00000000;
    ram[6605] = 8'b00000000;
    ram[6604] = 8'b00000000;
    ram[6603] = 8'b00000000;
    ram[6602] = 8'b00000000;
    ram[6601] = 8'b00000000;
    ram[6600] = 8'b00000000;
    ram[6599] = 8'b00000000;
    ram[6598] = 8'b00000000;
    ram[6597] = 8'b00000000;
    ram[6596] = 8'b00000000;
    ram[6595] = 8'b00000000;
    ram[6594] = 8'b00000000;
    ram[6593] = 8'b00000000;
    ram[6592] = 8'b00000000;
    ram[6591] = 8'b00000000;
    ram[6590] = 8'b00000000;
    ram[6589] = 8'b00000000;
    ram[6588] = 8'b00000000;
    ram[6587] = 8'b00000000;
    ram[6586] = 8'b00000000;
    ram[6585] = 8'b00000000;
    ram[6584] = 8'b00000000;
    ram[6583] = 8'b00000000;
    ram[6582] = 8'b00000000;
    ram[6581] = 8'b00000000;
    ram[6580] = 8'b00000000;
    ram[6579] = 8'b00000000;
    ram[6578] = 8'b00000000;
    ram[6577] = 8'b00000000;
    ram[6576] = 8'b00000000;
    ram[6575] = 8'b00000000;
    ram[6574] = 8'b00000000;
    ram[6573] = 8'b00000000;
    ram[6572] = 8'b00000000;
    ram[6571] = 8'b00000000;
    ram[6570] = 8'b00000000;
    ram[6569] = 8'b00000000;
    ram[6568] = 8'b00000000;
    ram[6567] = 8'b00000000;
    ram[6566] = 8'b00000000;
    ram[6565] = 8'b00000000;
    ram[6564] = 8'b00000000;
    ram[6563] = 8'b00000000;
    ram[6562] = 8'b00000000;
    ram[6561] = 8'b00000000;
    ram[6560] = 8'b00000000;
    ram[6559] = 8'b00000000;
    ram[6558] = 8'b00000000;
    ram[6557] = 8'b00000000;
    ram[6556] = 8'b00000000;
    ram[6555] = 8'b00000000;
    ram[6554] = 8'b00000000;
    ram[6553] = 8'b00000000;
    ram[6552] = 8'b00000000;
    ram[6551] = 8'b00000000;
    ram[6550] = 8'b00000000;
    ram[6549] = 8'b00000000;
    ram[6548] = 8'b00000000;
    ram[6547] = 8'b00000000;
    ram[6546] = 8'b00000000;
    ram[6545] = 8'b00000000;
    ram[6544] = 8'b00000000;
    ram[6543] = 8'b00000000;
    ram[6542] = 8'b00000000;
    ram[6541] = 8'b00000000;
    ram[6540] = 8'b00000000;
    ram[6539] = 8'b00000000;
    ram[6538] = 8'b00000000;
    ram[6537] = 8'b00000000;
    ram[6536] = 8'b00000000;
    ram[6535] = 8'b00000000;
    ram[6534] = 8'b00000000;
    ram[6533] = 8'b00000000;
    ram[6532] = 8'b00000000;
    ram[6531] = 8'b00000000;
    ram[6530] = 8'b00000000;
    ram[6529] = 8'b00000000;
    ram[6528] = 8'b00000000;
    ram[6527] = 8'b00000000;
    ram[6526] = 8'b00000000;
    ram[6525] = 8'b00000000;
    ram[6524] = 8'b00000000;
    ram[6523] = 8'b00000000;
    ram[6522] = 8'b00000000;
    ram[6521] = 8'b00000000;
    ram[6520] = 8'b00000000;
    ram[6519] = 8'b00000000;
    ram[6518] = 8'b00000000;
    ram[6517] = 8'b00000000;
    ram[6516] = 8'b00000000;
    ram[6515] = 8'b00000000;
    ram[6514] = 8'b00000000;
    ram[6513] = 8'b00000000;
    ram[6512] = 8'b00000000;
    ram[6511] = 8'b00000000;
    ram[6510] = 8'b00000000;
    ram[6509] = 8'b00000000;
    ram[6508] = 8'b00000000;
    ram[6507] = 8'b00000000;
    ram[6506] = 8'b00000000;
    ram[6505] = 8'b00000000;
    ram[6504] = 8'b00000000;
    ram[6503] = 8'b00000000;
    ram[6502] = 8'b00000000;
    ram[6501] = 8'b00000000;
    ram[6500] = 8'b00000000;
    ram[6499] = 8'b00000000;
    ram[6498] = 8'b00000000;
    ram[6497] = 8'b00000000;
    ram[6496] = 8'b00000000;
    ram[6495] = 8'b00000000;
    ram[6494] = 8'b00000000;
    ram[6493] = 8'b00000000;
    ram[6492] = 8'b00000000;
    ram[6491] = 8'b00000000;
    ram[6490] = 8'b00000000;
    ram[6489] = 8'b00000000;
    ram[6488] = 8'b00000000;
    ram[6487] = 8'b00000000;
    ram[6486] = 8'b00000000;
    ram[6485] = 8'b00000000;
    ram[6484] = 8'b00000000;
    ram[6483] = 8'b00000000;
    ram[6482] = 8'b00000000;
    ram[6481] = 8'b00000000;
    ram[6480] = 8'b00000000;
    ram[6479] = 8'b00000000;
    ram[6478] = 8'b00000000;
    ram[6477] = 8'b00000000;
    ram[6476] = 8'b00000000;
    ram[6475] = 8'b00000000;
    ram[6474] = 8'b00000000;
    ram[6473] = 8'b00000000;
    ram[6472] = 8'b00000000;
    ram[6471] = 8'b00000000;
    ram[6470] = 8'b00000000;
    ram[6469] = 8'b00000000;
    ram[6468] = 8'b00000000;
    ram[6467] = 8'b00000000;
    ram[6466] = 8'b00000000;
    ram[6465] = 8'b00000000;
    ram[6464] = 8'b00000000;
    ram[6463] = 8'b00000000;
    ram[6462] = 8'b00000000;
    ram[6461] = 8'b00000000;
    ram[6460] = 8'b00000000;
    ram[6459] = 8'b00000000;
    ram[6458] = 8'b00000000;
    ram[6457] = 8'b00000000;
    ram[6456] = 8'b00000000;
    ram[6455] = 8'b00000000;
    ram[6454] = 8'b00000000;
    ram[6453] = 8'b00000000;
    ram[6452] = 8'b00000000;
    ram[6451] = 8'b00000000;
    ram[6450] = 8'b00000000;
    ram[6449] = 8'b00000000;
    ram[6448] = 8'b00000000;
    ram[6447] = 8'b00000000;
    ram[6446] = 8'b00000000;
    ram[6445] = 8'b00000000;
    ram[6444] = 8'b00000000;
    ram[6443] = 8'b00000000;
    ram[6442] = 8'b00000000;
    ram[6441] = 8'b00000000;
    ram[6440] = 8'b00000000;
    ram[6439] = 8'b00000000;
    ram[6438] = 8'b00000000;
    ram[6437] = 8'b00000000;
    ram[6436] = 8'b00000000;
    ram[6435] = 8'b00000000;
    ram[6434] = 8'b00000000;
    ram[6433] = 8'b00000000;
    ram[6432] = 8'b00000000;
    ram[6431] = 8'b00000000;
    ram[6430] = 8'b00000000;
    ram[6429] = 8'b00000000;
    ram[6428] = 8'b00000000;
    ram[6427] = 8'b00000000;
    ram[6426] = 8'b00000000;
    ram[6425] = 8'b00000000;
    ram[6424] = 8'b00000000;
    ram[6423] = 8'b00000000;
    ram[6422] = 8'b00000000;
    ram[6421] = 8'b00000000;
    ram[6420] = 8'b00000000;
    ram[6419] = 8'b00000000;
    ram[6418] = 8'b00000000;
    ram[6417] = 8'b00000000;
    ram[6416] = 8'b00000000;
    ram[6415] = 8'b00000000;
    ram[6414] = 8'b00000000;
    ram[6413] = 8'b00000000;
    ram[6412] = 8'b00000000;
    ram[6411] = 8'b00000000;
    ram[6410] = 8'b00000000;
    ram[6409] = 8'b00000000;
    ram[6408] = 8'b00000000;
    ram[6407] = 8'b00000000;
    ram[6406] = 8'b00000000;
    ram[6405] = 8'b00000000;
    ram[6404] = 8'b00000000;
    ram[6403] = 8'b00000000;
    ram[6402] = 8'b00000000;
    ram[6401] = 8'b00000000;
    ram[6400] = 8'b00000000;
    ram[6399] = 8'b00000000;
    ram[6398] = 8'b00000000;
    ram[6397] = 8'b00000000;
    ram[6396] = 8'b00000000;
    ram[6395] = 8'b00000000;
    ram[6394] = 8'b00000000;
    ram[6393] = 8'b00000000;
    ram[6392] = 8'b00000000;
    ram[6391] = 8'b00000000;
    ram[6390] = 8'b00000000;
    ram[6389] = 8'b00000000;
    ram[6388] = 8'b00000000;
    ram[6387] = 8'b00000000;
    ram[6386] = 8'b00000000;
    ram[6385] = 8'b00000000;
    ram[6384] = 8'b00000000;
    ram[6383] = 8'b00000000;
    ram[6382] = 8'b00000000;
    ram[6381] = 8'b00000000;
    ram[6380] = 8'b00000000;
    ram[6379] = 8'b00000000;
    ram[6378] = 8'b00000000;
    ram[6377] = 8'b00000000;
    ram[6376] = 8'b00000000;
    ram[6375] = 8'b00000000;
    ram[6374] = 8'b00000000;
    ram[6373] = 8'b00000000;
    ram[6372] = 8'b00000000;
    ram[6371] = 8'b00000000;
    ram[6370] = 8'b00000000;
    ram[6369] = 8'b00000000;
    ram[6368] = 8'b00000000;
    ram[6367] = 8'b00000000;
    ram[6366] = 8'b00000000;
    ram[6365] = 8'b00000000;
    ram[6364] = 8'b00000000;
    ram[6363] = 8'b00000000;
    ram[6362] = 8'b00000000;
    ram[6361] = 8'b00000000;
    ram[6360] = 8'b00000000;
    ram[6359] = 8'b00000000;
    ram[6358] = 8'b00000000;
    ram[6357] = 8'b00000000;
    ram[6356] = 8'b00000000;
    ram[6355] = 8'b00000000;
    ram[6354] = 8'b00000000;
    ram[6353] = 8'b00000000;
    ram[6352] = 8'b00000000;
    ram[6351] = 8'b00000000;
    ram[6350] = 8'b00000000;
    ram[6349] = 8'b00000000;
    ram[6348] = 8'b00000000;
    ram[6347] = 8'b00000000;
    ram[6346] = 8'b00000000;
    ram[6345] = 8'b00000000;
    ram[6344] = 8'b00000000;
    ram[6343] = 8'b00000000;
    ram[6342] = 8'b00000000;
    ram[6341] = 8'b00000000;
    ram[6340] = 8'b00000000;
    ram[6339] = 8'b00000000;
    ram[6338] = 8'b00000000;
    ram[6337] = 8'b00000000;
    ram[6336] = 8'b00000000;
    ram[6335] = 8'b00000000;
    ram[6334] = 8'b00000000;
    ram[6333] = 8'b00000000;
    ram[6332] = 8'b00000000;
    ram[6331] = 8'b00000000;
    ram[6330] = 8'b00000000;
    ram[6329] = 8'b00000000;
    ram[6328] = 8'b00000000;
    ram[6327] = 8'b00000000;
    ram[6326] = 8'b00000000;
    ram[6325] = 8'b00000000;
    ram[6324] = 8'b00000000;
    ram[6323] = 8'b00000000;
    ram[6322] = 8'b00000000;
    ram[6321] = 8'b00000000;
    ram[6320] = 8'b00000000;
    ram[6319] = 8'b00000000;
    ram[6318] = 8'b00000000;
    ram[6317] = 8'b00000000;
    ram[6316] = 8'b00000000;
    ram[6315] = 8'b00000000;
    ram[6314] = 8'b00000000;
    ram[6313] = 8'b00000000;
    ram[6312] = 8'b00000000;
    ram[6311] = 8'b00000000;
    ram[6310] = 8'b00000000;
    ram[6309] = 8'b00000000;
    ram[6308] = 8'b00000000;
    ram[6307] = 8'b00000000;
    ram[6306] = 8'b00000000;
    ram[6305] = 8'b00000000;
    ram[6304] = 8'b00000000;
    ram[6303] = 8'b00000000;
    ram[6302] = 8'b00000000;
    ram[6301] = 8'b00000000;
    ram[6300] = 8'b00000000;
    ram[6299] = 8'b00000000;
    ram[6298] = 8'b00000000;
    ram[6297] = 8'b00000000;
    ram[6296] = 8'b00000000;
    ram[6295] = 8'b00000000;
    ram[6294] = 8'b00000000;
    ram[6293] = 8'b00000000;
    ram[6292] = 8'b00000000;
    ram[6291] = 8'b00000000;
    ram[6290] = 8'b00000000;
    ram[6289] = 8'b00000000;
    ram[6288] = 8'b00000000;
    ram[6287] = 8'b00000000;
    ram[6286] = 8'b00000000;
    ram[6285] = 8'b00000000;
    ram[6284] = 8'b00000000;
    ram[6283] = 8'b00000000;
    ram[6282] = 8'b00000000;
    ram[6281] = 8'b00000000;
    ram[6280] = 8'b00000000;
    ram[6279] = 8'b00000000;
    ram[6278] = 8'b00000000;
    ram[6277] = 8'b00000000;
    ram[6276] = 8'b00000000;
    ram[6275] = 8'b00000000;
    ram[6274] = 8'b00000000;
    ram[6273] = 8'b00000000;
    ram[6272] = 8'b00000000;
    ram[6271] = 8'b00000000;
    ram[6270] = 8'b00000000;
    ram[6269] = 8'b00000000;
    ram[6268] = 8'b00000000;
    ram[6267] = 8'b00000000;
    ram[6266] = 8'b00000000;
    ram[6265] = 8'b00000000;
    ram[6264] = 8'b00000000;
    ram[6263] = 8'b00000000;
    ram[6262] = 8'b00000000;
    ram[6261] = 8'b00000000;
    ram[6260] = 8'b00000000;
    ram[6259] = 8'b00000000;
    ram[6258] = 8'b00000000;
    ram[6257] = 8'b00000000;
    ram[6256] = 8'b00000000;
    ram[6255] = 8'b00000000;
    ram[6254] = 8'b00000000;
    ram[6253] = 8'b00000000;
    ram[6252] = 8'b00000000;
    ram[6251] = 8'b00000000;
    ram[6250] = 8'b00000000;
    ram[6249] = 8'b00000000;
    ram[6248] = 8'b00000000;
    ram[6247] = 8'b00000000;
    ram[6246] = 8'b00000000;
    ram[6245] = 8'b00000000;
    ram[6244] = 8'b00000000;
    ram[6243] = 8'b00000000;
    ram[6242] = 8'b00000000;
    ram[6241] = 8'b00000000;
    ram[6240] = 8'b00000000;
    ram[6239] = 8'b00000000;
    ram[6238] = 8'b00000000;
    ram[6237] = 8'b00000000;
    ram[6236] = 8'b00000000;
    ram[6235] = 8'b00000000;
    ram[6234] = 8'b00000000;
    ram[6233] = 8'b00000000;
    ram[6232] = 8'b00000000;
    ram[6231] = 8'b00000000;
    ram[6230] = 8'b00000000;
    ram[6229] = 8'b00000000;
    ram[6228] = 8'b00000000;
    ram[6227] = 8'b00000000;
    ram[6226] = 8'b00000000;
    ram[6225] = 8'b00000000;
    ram[6224] = 8'b00000000;
    ram[6223] = 8'b00000000;
    ram[6222] = 8'b00000000;
    ram[6221] = 8'b00000000;
    ram[6220] = 8'b00000000;
    ram[6219] = 8'b00000000;
    ram[6218] = 8'b00000000;
    ram[6217] = 8'b00000000;
    ram[6216] = 8'b00000000;
    ram[6215] = 8'b00000000;
    ram[6214] = 8'b00000000;
    ram[6213] = 8'b00000000;
    ram[6212] = 8'b00000000;
    ram[6211] = 8'b00000000;
    ram[6210] = 8'b00000000;
    ram[6209] = 8'b00000000;
    ram[6208] = 8'b00000000;
    ram[6207] = 8'b00000000;
    ram[6206] = 8'b00000000;
    ram[6205] = 8'b00000000;
    ram[6204] = 8'b00000000;
    ram[6203] = 8'b00000000;
    ram[6202] = 8'b00000000;
    ram[6201] = 8'b00000000;
    ram[6200] = 8'b00000000;
    ram[6199] = 8'b00000000;
    ram[6198] = 8'b00000000;
    ram[6197] = 8'b00000000;
    ram[6196] = 8'b00000000;
    ram[6195] = 8'b00000000;
    ram[6194] = 8'b00000000;
    ram[6193] = 8'b00000000;
    ram[6192] = 8'b00000000;
    ram[6191] = 8'b00000000;
    ram[6190] = 8'b00000000;
    ram[6189] = 8'b00000000;
    ram[6188] = 8'b00000000;
    ram[6187] = 8'b00000000;
    ram[6186] = 8'b00000000;
    ram[6185] = 8'b00000000;
    ram[6184] = 8'b00000000;
    ram[6183] = 8'b00000000;
    ram[6182] = 8'b00000000;
    ram[6181] = 8'b00000000;
    ram[6180] = 8'b00000000;
    ram[6179] = 8'b00000000;
    ram[6178] = 8'b00000000;
    ram[6177] = 8'b00000000;
    ram[6176] = 8'b00000000;
    ram[6175] = 8'b00000000;
    ram[6174] = 8'b00000000;
    ram[6173] = 8'b00000000;
    ram[6172] = 8'b00000000;
    ram[6171] = 8'b00000000;
    ram[6170] = 8'b00000000;
    ram[6169] = 8'b00000000;
    ram[6168] = 8'b00000000;
    ram[6167] = 8'b00000000;
    ram[6166] = 8'b00000000;
    ram[6165] = 8'b00000000;
    ram[6164] = 8'b00000000;
    ram[6163] = 8'b00000000;
    ram[6162] = 8'b00000000;
    ram[6161] = 8'b00000000;
    ram[6160] = 8'b00000000;
    ram[6159] = 8'b00000000;
    ram[6158] = 8'b00000000;
    ram[6157] = 8'b00000000;
    ram[6156] = 8'b00000000;
    ram[6155] = 8'b00000000;
    ram[6154] = 8'b00000000;
    ram[6153] = 8'b00000000;
    ram[6152] = 8'b00000000;
    ram[6151] = 8'b00000000;
    ram[6150] = 8'b00000000;
    ram[6149] = 8'b00000000;
    ram[6148] = 8'b00000000;
    ram[6147] = 8'b00000000;
    ram[6146] = 8'b00000000;
    ram[6145] = 8'b00000000;
    ram[6144] = 8'b00000000;
    ram[6143] = 8'b00000000;
    ram[6142] = 8'b00000000;
    ram[6141] = 8'b00000000;
    ram[6140] = 8'b00000000;
    ram[6139] = 8'b00000000;
    ram[6138] = 8'b00000000;
    ram[6137] = 8'b00000000;
    ram[6136] = 8'b00000000;
    ram[6135] = 8'b00000000;
    ram[6134] = 8'b00000000;
    ram[6133] = 8'b00000000;
    ram[6132] = 8'b00000000;
    ram[6131] = 8'b00000000;
    ram[6130] = 8'b00000000;
    ram[6129] = 8'b00000000;
    ram[6128] = 8'b00000000;
    ram[6127] = 8'b00000000;
    ram[6126] = 8'b00000000;
    ram[6125] = 8'b00000000;
    ram[6124] = 8'b00000000;
    ram[6123] = 8'b00000000;
    ram[6122] = 8'b00000000;
    ram[6121] = 8'b00000000;
    ram[6120] = 8'b00000000;
    ram[6119] = 8'b00000000;
    ram[6118] = 8'b00000000;
    ram[6117] = 8'b00000000;
    ram[6116] = 8'b00000000;
    ram[6115] = 8'b00000000;
    ram[6114] = 8'b00000000;
    ram[6113] = 8'b00000000;
    ram[6112] = 8'b00000000;
    ram[6111] = 8'b00000000;
    ram[6110] = 8'b00000000;
    ram[6109] = 8'b00000000;
    ram[6108] = 8'b00000000;
    ram[6107] = 8'b00000000;
    ram[6106] = 8'b00000000;
    ram[6105] = 8'b00000000;
    ram[6104] = 8'b00000000;
    ram[6103] = 8'b00000000;
    ram[6102] = 8'b00000000;
    ram[6101] = 8'b00000000;
    ram[6100] = 8'b00000000;
    ram[6099] = 8'b00000000;
    ram[6098] = 8'b00000000;
    ram[6097] = 8'b00000000;
    ram[6096] = 8'b00000000;
    ram[6095] = 8'b00000000;
    ram[6094] = 8'b00000000;
    ram[6093] = 8'b00000000;
    ram[6092] = 8'b00000000;
    ram[6091] = 8'b00000000;
    ram[6090] = 8'b00000000;
    ram[6089] = 8'b00000000;
    ram[6088] = 8'b00000000;
    ram[6087] = 8'b00000000;
    ram[6086] = 8'b00000000;
    ram[6085] = 8'b00000000;
    ram[6084] = 8'b00000000;
    ram[6083] = 8'b00000000;
    ram[6082] = 8'b00000000;
    ram[6081] = 8'b00000000;
    ram[6080] = 8'b00000000;
    ram[6079] = 8'b00000000;
    ram[6078] = 8'b00000000;
    ram[6077] = 8'b00000000;
    ram[6076] = 8'b00000000;
    ram[6075] = 8'b00000000;
    ram[6074] = 8'b00000000;
    ram[6073] = 8'b00000000;
    ram[6072] = 8'b00000000;
    ram[6071] = 8'b00000000;
    ram[6070] = 8'b00000000;
    ram[6069] = 8'b00000000;
    ram[6068] = 8'b00000000;
    ram[6067] = 8'b00000000;
    ram[6066] = 8'b00000000;
    ram[6065] = 8'b00000000;
    ram[6064] = 8'b00000000;
    ram[6063] = 8'b00000000;
    ram[6062] = 8'b00000000;
    ram[6061] = 8'b00000000;
    ram[6060] = 8'b00000000;
    ram[6059] = 8'b00000000;
    ram[6058] = 8'b00000000;
    ram[6057] = 8'b00000000;
    ram[6056] = 8'b00000000;
    ram[6055] = 8'b00000000;
    ram[6054] = 8'b00000000;
    ram[6053] = 8'b00000000;
    ram[6052] = 8'b00000000;
    ram[6051] = 8'b00000000;
    ram[6050] = 8'b00000000;
    ram[6049] = 8'b00000000;
    ram[6048] = 8'b00000000;
    ram[6047] = 8'b00000000;
    ram[6046] = 8'b00000000;
    ram[6045] = 8'b00000000;
    ram[6044] = 8'b00000000;
    ram[6043] = 8'b00000000;
    ram[6042] = 8'b00000000;
    ram[6041] = 8'b00000000;
    ram[6040] = 8'b00000000;
    ram[6039] = 8'b00000000;
    ram[6038] = 8'b00000000;
    ram[6037] = 8'b00000000;
    ram[6036] = 8'b00000000;
    ram[6035] = 8'b00000000;
    ram[6034] = 8'b00000000;
    ram[6033] = 8'b00000000;
    ram[6032] = 8'b00000000;
    ram[6031] = 8'b00000000;
    ram[6030] = 8'b00000000;
    ram[6029] = 8'b00000000;
    ram[6028] = 8'b00000000;
    ram[6027] = 8'b00000000;
    ram[6026] = 8'b00000000;
    ram[6025] = 8'b00000000;
    ram[6024] = 8'b00000000;
    ram[6023] = 8'b00000000;
    ram[6022] = 8'b00000000;
    ram[6021] = 8'b00000000;
    ram[6020] = 8'b00000000;
    ram[6019] = 8'b00000000;
    ram[6018] = 8'b00000000;
    ram[6017] = 8'b00000000;
    ram[6016] = 8'b00000000;
    ram[6015] = 8'b00000000;
    ram[6014] = 8'b00000000;
    ram[6013] = 8'b00000000;
    ram[6012] = 8'b00000000;
    ram[6011] = 8'b00000000;
    ram[6010] = 8'b00000000;
    ram[6009] = 8'b00000000;
    ram[6008] = 8'b00000000;
    ram[6007] = 8'b00000000;
    ram[6006] = 8'b00000000;
    ram[6005] = 8'b00000000;
    ram[6004] = 8'b00000000;
    ram[6003] = 8'b00000000;
    ram[6002] = 8'b00000000;
    ram[6001] = 8'b00000000;
    ram[6000] = 8'b00000000;
    ram[5999] = 8'b00000000;
    ram[5998] = 8'b00000000;
    ram[5997] = 8'b00000000;
    ram[5996] = 8'b00000000;
    ram[5995] = 8'b00000000;
    ram[5994] = 8'b00000000;
    ram[5993] = 8'b00000000;
    ram[5992] = 8'b00000000;
    ram[5991] = 8'b00000000;
    ram[5990] = 8'b00000000;
    ram[5989] = 8'b00000000;
    ram[5988] = 8'b00000000;
    ram[5987] = 8'b00000000;
    ram[5986] = 8'b00000000;
    ram[5985] = 8'b00000000;
    ram[5984] = 8'b00000000;
    ram[5983] = 8'b00000000;
    ram[5982] = 8'b00000000;
    ram[5981] = 8'b00000000;
    ram[5980] = 8'b00000000;
    ram[5979] = 8'b00000000;
    ram[5978] = 8'b00000000;
    ram[5977] = 8'b00000000;
    ram[5976] = 8'b00000000;
    ram[5975] = 8'b00000000;
    ram[5974] = 8'b00000000;
    ram[5973] = 8'b00000000;
    ram[5972] = 8'b00000000;
    ram[5971] = 8'b00000000;
    ram[5970] = 8'b00000000;
    ram[5969] = 8'b00000000;
    ram[5968] = 8'b00000000;
    ram[5967] = 8'b00000000;
    ram[5966] = 8'b00000000;
    ram[5965] = 8'b00000000;
    ram[5964] = 8'b00000000;
    ram[5963] = 8'b00000000;
    ram[5962] = 8'b00000000;
    ram[5961] = 8'b00000000;
    ram[5960] = 8'b00000000;
    ram[5959] = 8'b00000000;
    ram[5958] = 8'b00000000;
    ram[5957] = 8'b00000000;
    ram[5956] = 8'b00000000;
    ram[5955] = 8'b00000000;
    ram[5954] = 8'b00000000;
    ram[5953] = 8'b00000000;
    ram[5952] = 8'b00000000;
    ram[5951] = 8'b00000000;
    ram[5950] = 8'b00000000;
    ram[5949] = 8'b00000000;
    ram[5948] = 8'b00000000;
    ram[5947] = 8'b00000000;
    ram[5946] = 8'b00000000;
    ram[5945] = 8'b00000000;
    ram[5944] = 8'b00000000;
    ram[5943] = 8'b00000000;
    ram[5942] = 8'b00000000;
    ram[5941] = 8'b00000000;
    ram[5940] = 8'b00000000;
    ram[5939] = 8'b00000000;
    ram[5938] = 8'b00000000;
    ram[5937] = 8'b00000000;
    ram[5936] = 8'b00000000;
    ram[5935] = 8'b00000000;
    ram[5934] = 8'b00000000;
    ram[5933] = 8'b00000000;
    ram[5932] = 8'b00000000;
    ram[5931] = 8'b00000000;
    ram[5930] = 8'b00000000;
    ram[5929] = 8'b00000000;
    ram[5928] = 8'b00000000;
    ram[5927] = 8'b00000000;
    ram[5926] = 8'b00000000;
    ram[5925] = 8'b00000000;
    ram[5924] = 8'b00000000;
    ram[5923] = 8'b00000000;
    ram[5922] = 8'b00000000;
    ram[5921] = 8'b00000000;
    ram[5920] = 8'b00000000;
    ram[5919] = 8'b00000000;
    ram[5918] = 8'b00000000;
    ram[5917] = 8'b00000000;
    ram[5916] = 8'b00000000;
    ram[5915] = 8'b00000000;
    ram[5914] = 8'b00000000;
    ram[5913] = 8'b00000000;
    ram[5912] = 8'b00000000;
    ram[5911] = 8'b00000000;
    ram[5910] = 8'b00000000;
    ram[5909] = 8'b00000000;
    ram[5908] = 8'b00000000;
    ram[5907] = 8'b00000000;
    ram[5906] = 8'b00000000;
    ram[5905] = 8'b00000000;
    ram[5904] = 8'b00000000;
    ram[5903] = 8'b00000000;
    ram[5902] = 8'b00000000;
    ram[5901] = 8'b00000000;
    ram[5900] = 8'b00000000;
    ram[5899] = 8'b00000000;
    ram[5898] = 8'b00000000;
    ram[5897] = 8'b00000000;
    ram[5896] = 8'b00000000;
    ram[5895] = 8'b00000000;
    ram[5894] = 8'b00000000;
    ram[5893] = 8'b00000000;
    ram[5892] = 8'b00000000;
    ram[5891] = 8'b00000000;
    ram[5890] = 8'b00000000;
    ram[5889] = 8'b00000000;
    ram[5888] = 8'b00000000;
    ram[5887] = 8'b00000000;
    ram[5886] = 8'b00000000;
    ram[5885] = 8'b00000000;
    ram[5884] = 8'b00000000;
    ram[5883] = 8'b00000000;
    ram[5882] = 8'b00000000;
    ram[5881] = 8'b00000000;
    ram[5880] = 8'b00000000;
    ram[5879] = 8'b00000000;
    ram[5878] = 8'b00000000;
    ram[5877] = 8'b00000000;
    ram[5876] = 8'b00000000;
    ram[5875] = 8'b00000000;
    ram[5874] = 8'b00000000;
    ram[5873] = 8'b00000000;
    ram[5872] = 8'b00000000;
    ram[5871] = 8'b00000000;
    ram[5870] = 8'b00000000;
    ram[5869] = 8'b00000000;
    ram[5868] = 8'b00000000;
    ram[5867] = 8'b00000000;
    ram[5866] = 8'b00000000;
    ram[5865] = 8'b00000000;
    ram[5864] = 8'b00000000;
    ram[5863] = 8'b00000000;
    ram[5862] = 8'b00000000;
    ram[5861] = 8'b00000000;
    ram[5860] = 8'b00000000;
    ram[5859] = 8'b00000000;
    ram[5858] = 8'b00000000;
    ram[5857] = 8'b00000000;
    ram[5856] = 8'b00000000;
    ram[5855] = 8'b00000000;
    ram[5854] = 8'b00000000;
    ram[5853] = 8'b00000000;
    ram[5852] = 8'b00000000;
    ram[5851] = 8'b00000000;
    ram[5850] = 8'b00000000;
    ram[5849] = 8'b00000000;
    ram[5848] = 8'b00000000;
    ram[5847] = 8'b00000000;
    ram[5846] = 8'b00000000;
    ram[5845] = 8'b00000000;
    ram[5844] = 8'b00000000;
    ram[5843] = 8'b00000000;
    ram[5842] = 8'b00000000;
    ram[5841] = 8'b00000000;
    ram[5840] = 8'b00000000;
    ram[5839] = 8'b00000000;
    ram[5838] = 8'b00000000;
    ram[5837] = 8'b00000000;
    ram[5836] = 8'b00000000;
    ram[5835] = 8'b00000000;
    ram[5834] = 8'b00000000;
    ram[5833] = 8'b00000000;
    ram[5832] = 8'b00000000;
    ram[5831] = 8'b00000000;
    ram[5830] = 8'b00000000;
    ram[5829] = 8'b00000000;
    ram[5828] = 8'b00000000;
    ram[5827] = 8'b00000000;
    ram[5826] = 8'b00000000;
    ram[5825] = 8'b00000000;
    ram[5824] = 8'b00000000;
    ram[5823] = 8'b00000000;
    ram[5822] = 8'b00000000;
    ram[5821] = 8'b00000000;
    ram[5820] = 8'b00000000;
    ram[5819] = 8'b00000000;
    ram[5818] = 8'b00000000;
    ram[5817] = 8'b00000000;
    ram[5816] = 8'b00000000;
    ram[5815] = 8'b00000000;
    ram[5814] = 8'b00000000;
    ram[5813] = 8'b00000000;
    ram[5812] = 8'b00000000;
    ram[5811] = 8'b00000000;
    ram[5810] = 8'b00000000;
    ram[5809] = 8'b00000000;
    ram[5808] = 8'b00000000;
    ram[5807] = 8'b00000000;
    ram[5806] = 8'b00000000;
    ram[5805] = 8'b00000000;
    ram[5804] = 8'b00000000;
    ram[5803] = 8'b00000000;
    ram[5802] = 8'b00000000;
    ram[5801] = 8'b00000000;
    ram[5800] = 8'b00000000;
    ram[5799] = 8'b00000000;
    ram[5798] = 8'b00000000;
    ram[5797] = 8'b00000000;
    ram[5796] = 8'b00000000;
    ram[5795] = 8'b00000000;
    ram[5794] = 8'b00000000;
    ram[5793] = 8'b00000000;
    ram[5792] = 8'b00000000;
    ram[5791] = 8'b00000000;
    ram[5790] = 8'b00000000;
    ram[5789] = 8'b00000000;
    ram[5788] = 8'b00000000;
    ram[5787] = 8'b00000000;
    ram[5786] = 8'b00000000;
    ram[5785] = 8'b00000000;
    ram[5784] = 8'b00000000;
    ram[5783] = 8'b00000000;
    ram[5782] = 8'b00000000;
    ram[5781] = 8'b00000000;
    ram[5780] = 8'b00000000;
    ram[5779] = 8'b00000000;
    ram[5778] = 8'b00000000;
    ram[5777] = 8'b00000000;
    ram[5776] = 8'b00000000;
    ram[5775] = 8'b00000000;
    ram[5774] = 8'b00000000;
    ram[5773] = 8'b00000000;
    ram[5772] = 8'b00000000;
    ram[5771] = 8'b00000000;
    ram[5770] = 8'b00000000;
    ram[5769] = 8'b00000000;
    ram[5768] = 8'b00000000;
    ram[5767] = 8'b00000000;
    ram[5766] = 8'b00000000;
    ram[5765] = 8'b00000000;
    ram[5764] = 8'b00000000;
    ram[5763] = 8'b00000000;
    ram[5762] = 8'b00000000;
    ram[5761] = 8'b00000000;
    ram[5760] = 8'b00000000;
    ram[5759] = 8'b00000000;
    ram[5758] = 8'b00000000;
    ram[5757] = 8'b00000000;
    ram[5756] = 8'b00000000;
    ram[5755] = 8'b00000000;
    ram[5754] = 8'b00000000;
    ram[5753] = 8'b00000000;
    ram[5752] = 8'b00000000;
    ram[5751] = 8'b00000000;
    ram[5750] = 8'b00000000;
    ram[5749] = 8'b00000000;
    ram[5748] = 8'b00000000;
    ram[5747] = 8'b00000000;
    ram[5746] = 8'b00000000;
    ram[5745] = 8'b00000000;
    ram[5744] = 8'b00000000;
    ram[5743] = 8'b00000000;
    ram[5742] = 8'b00000000;
    ram[5741] = 8'b00000000;
    ram[5740] = 8'b00000000;
    ram[5739] = 8'b00000000;
    ram[5738] = 8'b00000000;
    ram[5737] = 8'b00000000;
    ram[5736] = 8'b00000000;
    ram[5735] = 8'b00000000;
    ram[5734] = 8'b00000000;
    ram[5733] = 8'b00000000;
    ram[5732] = 8'b00000000;
    ram[5731] = 8'b00000000;
    ram[5730] = 8'b00000000;
    ram[5729] = 8'b00000000;
    ram[5728] = 8'b00000000;
    ram[5727] = 8'b00000000;
    ram[5726] = 8'b00000000;
    ram[5725] = 8'b00000000;
    ram[5724] = 8'b00000000;
    ram[5723] = 8'b00000000;
    ram[5722] = 8'b00000000;
    ram[5721] = 8'b00000000;
    ram[5720] = 8'b00000000;
    ram[5719] = 8'b00000000;
    ram[5718] = 8'b00000000;
    ram[5717] = 8'b00000000;
    ram[5716] = 8'b00000000;
    ram[5715] = 8'b00000000;
    ram[5714] = 8'b00000000;
    ram[5713] = 8'b00000000;
    ram[5712] = 8'b00000000;
    ram[5711] = 8'b00000000;
    ram[5710] = 8'b00000000;
    ram[5709] = 8'b00000000;
    ram[5708] = 8'b00000000;
    ram[5707] = 8'b00000000;
    ram[5706] = 8'b00000000;
    ram[5705] = 8'b00000000;
    ram[5704] = 8'b00000000;
    ram[5703] = 8'b00000000;
    ram[5702] = 8'b00000000;
    ram[5701] = 8'b00000000;
    ram[5700] = 8'b00000000;
    ram[5699] = 8'b00000000;
    ram[5698] = 8'b00000000;
    ram[5697] = 8'b00000000;
    ram[5696] = 8'b00000000;
    ram[5695] = 8'b00000000;
    ram[5694] = 8'b00000000;
    ram[5693] = 8'b00000000;
    ram[5692] = 8'b00000000;
    ram[5691] = 8'b00000000;
    ram[5690] = 8'b00000000;
    ram[5689] = 8'b00000000;
    ram[5688] = 8'b00000000;
    ram[5687] = 8'b00000000;
    ram[5686] = 8'b00000000;
    ram[5685] = 8'b00000000;
    ram[5684] = 8'b00000000;
    ram[5683] = 8'b00000000;
    ram[5682] = 8'b00000000;
    ram[5681] = 8'b00000000;
    ram[5680] = 8'b00000000;
    ram[5679] = 8'b00000000;
    ram[5678] = 8'b00000000;
    ram[5677] = 8'b00000000;
    ram[5676] = 8'b00000000;
    ram[5675] = 8'b00000000;
    ram[5674] = 8'b00000000;
    ram[5673] = 8'b00000000;
    ram[5672] = 8'b00000000;
    ram[5671] = 8'b00000000;
    ram[5670] = 8'b00000000;
    ram[5669] = 8'b00000000;
    ram[5668] = 8'b00000000;
    ram[5667] = 8'b00000000;
    ram[5666] = 8'b00000000;
    ram[5665] = 8'b00000000;
    ram[5664] = 8'b00000000;
    ram[5663] = 8'b00000000;
    ram[5662] = 8'b00000000;
    ram[5661] = 8'b00000000;
    ram[5660] = 8'b00000000;
    ram[5659] = 8'b00000000;
    ram[5658] = 8'b00000000;
    ram[5657] = 8'b00000000;
    ram[5656] = 8'b00000000;
    ram[5655] = 8'b00000000;
    ram[5654] = 8'b00000000;
    ram[5653] = 8'b00000000;
    ram[5652] = 8'b00000000;
    ram[5651] = 8'b00000000;
    ram[5650] = 8'b00000000;
    ram[5649] = 8'b00000000;
    ram[5648] = 8'b00000000;
    ram[5647] = 8'b00000000;
    ram[5646] = 8'b00000000;
    ram[5645] = 8'b00000000;
    ram[5644] = 8'b00000000;
    ram[5643] = 8'b00000000;
    ram[5642] = 8'b00000000;
    ram[5641] = 8'b00000000;
    ram[5640] = 8'b00000000;
    ram[5639] = 8'b00000000;
    ram[5638] = 8'b00000000;
    ram[5637] = 8'b00000000;
    ram[5636] = 8'b00000000;
    ram[5635] = 8'b00000000;
    ram[5634] = 8'b00000000;
    ram[5633] = 8'b00000000;
    ram[5632] = 8'b00000000;
    ram[5631] = 8'b00000000;
    ram[5630] = 8'b00000000;
    ram[5629] = 8'b00000000;
    ram[5628] = 8'b00000000;
    ram[5627] = 8'b00000000;
    ram[5626] = 8'b00000000;
    ram[5625] = 8'b00000000;
    ram[5624] = 8'b00000000;
    ram[5623] = 8'b00000000;
    ram[5622] = 8'b00000000;
    ram[5621] = 8'b00000000;
    ram[5620] = 8'b00000000;
    ram[5619] = 8'b00000000;
    ram[5618] = 8'b00000000;
    ram[5617] = 8'b00000000;
    ram[5616] = 8'b00000000;
    ram[5615] = 8'b00000000;
    ram[5614] = 8'b00000000;
    ram[5613] = 8'b00000000;
    ram[5612] = 8'b00000000;
    ram[5611] = 8'b00000000;
    ram[5610] = 8'b00000000;
    ram[5609] = 8'b00000000;
    ram[5608] = 8'b00000000;
    ram[5607] = 8'b00000000;
    ram[5606] = 8'b00000000;
    ram[5605] = 8'b00000000;
    ram[5604] = 8'b00000000;
    ram[5603] = 8'b00000000;
    ram[5602] = 8'b00000000;
    ram[5601] = 8'b00000000;
    ram[5600] = 8'b00000000;
    ram[5599] = 8'b00000000;
    ram[5598] = 8'b00000000;
    ram[5597] = 8'b00000000;
    ram[5596] = 8'b00000000;
    ram[5595] = 8'b00000000;
    ram[5594] = 8'b00000000;
    ram[5593] = 8'b00000000;
    ram[5592] = 8'b00000000;
    ram[5591] = 8'b00000000;
    ram[5590] = 8'b00000000;
    ram[5589] = 8'b00000000;
    ram[5588] = 8'b00000000;
    ram[5587] = 8'b00000000;
    ram[5586] = 8'b00000000;
    ram[5585] = 8'b00000000;
    ram[5584] = 8'b00000000;
    ram[5583] = 8'b00000000;
    ram[5582] = 8'b00000000;
    ram[5581] = 8'b00000000;
    ram[5580] = 8'b00000000;
    ram[5579] = 8'b00000000;
    ram[5578] = 8'b00000000;
    ram[5577] = 8'b00000000;
    ram[5576] = 8'b00000000;
    ram[5575] = 8'b00000000;
    ram[5574] = 8'b00000000;
    ram[5573] = 8'b00000000;
    ram[5572] = 8'b00000000;
    ram[5571] = 8'b00000000;
    ram[5570] = 8'b00000000;
    ram[5569] = 8'b00000000;
    ram[5568] = 8'b00000000;
    ram[5567] = 8'b00000000;
    ram[5566] = 8'b00000000;
    ram[5565] = 8'b00000000;
    ram[5564] = 8'b00000000;
    ram[5563] = 8'b00000000;
    ram[5562] = 8'b00000000;
    ram[5561] = 8'b00000000;
    ram[5560] = 8'b00000000;
    ram[5559] = 8'b00000000;
    ram[5558] = 8'b00000000;
    ram[5557] = 8'b00000000;
    ram[5556] = 8'b00000000;
    ram[5555] = 8'b00000000;
    ram[5554] = 8'b00000000;
    ram[5553] = 8'b00000000;
    ram[5552] = 8'b00000000;
    ram[5551] = 8'b00000000;
    ram[5550] = 8'b00000000;
    ram[5549] = 8'b00000000;
    ram[5548] = 8'b00000000;
    ram[5547] = 8'b00000000;
    ram[5546] = 8'b00000000;
    ram[5545] = 8'b00000000;
    ram[5544] = 8'b00000000;
    ram[5543] = 8'b00000000;
    ram[5542] = 8'b00000000;
    ram[5541] = 8'b00000000;
    ram[5540] = 8'b00000000;
    ram[5539] = 8'b00000000;
    ram[5538] = 8'b00000000;
    ram[5537] = 8'b00000000;
    ram[5536] = 8'b00000000;
    ram[5535] = 8'b00000000;
    ram[5534] = 8'b00000000;
    ram[5533] = 8'b00000000;
    ram[5532] = 8'b00000000;
    ram[5531] = 8'b00000000;
    ram[5530] = 8'b00000000;
    ram[5529] = 8'b00000000;
    ram[5528] = 8'b00000000;
    ram[5527] = 8'b00000000;
    ram[5526] = 8'b00000000;
    ram[5525] = 8'b00000000;
    ram[5524] = 8'b00000000;
    ram[5523] = 8'b00000000;
    ram[5522] = 8'b00000000;
    ram[5521] = 8'b00000000;
    ram[5520] = 8'b00000000;
    ram[5519] = 8'b00000000;
    ram[5518] = 8'b00000000;
    ram[5517] = 8'b00000000;
    ram[5516] = 8'b00000000;
    ram[5515] = 8'b00000000;
    ram[5514] = 8'b00000000;
    ram[5513] = 8'b00000000;
    ram[5512] = 8'b00000000;
    ram[5511] = 8'b00000000;
    ram[5510] = 8'b00000000;
    ram[5509] = 8'b00000000;
    ram[5508] = 8'b00000000;
    ram[5507] = 8'b00000000;
    ram[5506] = 8'b00000000;
    ram[5505] = 8'b00000000;
    ram[5504] = 8'b00000000;
    ram[5503] = 8'b00000000;
    ram[5502] = 8'b00000000;
    ram[5501] = 8'b00000000;
    ram[5500] = 8'b00000000;
    ram[5499] = 8'b00000000;
    ram[5498] = 8'b00000000;
    ram[5497] = 8'b00000000;
    ram[5496] = 8'b00000000;
    ram[5495] = 8'b00000000;
    ram[5494] = 8'b00000000;
    ram[5493] = 8'b00000000;
    ram[5492] = 8'b00000000;
    ram[5491] = 8'b00000000;
    ram[5490] = 8'b00000000;
    ram[5489] = 8'b00000000;
    ram[5488] = 8'b00000000;
    ram[5487] = 8'b00000000;
    ram[5486] = 8'b00000000;
    ram[5485] = 8'b00000000;
    ram[5484] = 8'b00000000;
    ram[5483] = 8'b00000000;
    ram[5482] = 8'b00000000;
    ram[5481] = 8'b00000000;
    ram[5480] = 8'b00000000;
    ram[5479] = 8'b00000000;
    ram[5478] = 8'b00000000;
    ram[5477] = 8'b00000000;
    ram[5476] = 8'b00000000;
    ram[5475] = 8'b00000000;
    ram[5474] = 8'b00000000;
    ram[5473] = 8'b00000000;
    ram[5472] = 8'b00000000;
    ram[5471] = 8'b00000000;
    ram[5470] = 8'b00000000;
    ram[5469] = 8'b00000000;
    ram[5468] = 8'b00000000;
    ram[5467] = 8'b00000000;
    ram[5466] = 8'b00000000;
    ram[5465] = 8'b00000000;
    ram[5464] = 8'b00000000;
    ram[5463] = 8'b00000000;
    ram[5462] = 8'b00000000;
    ram[5461] = 8'b00000000;
    ram[5460] = 8'b00000000;
    ram[5459] = 8'b00000000;
    ram[5458] = 8'b00000000;
    ram[5457] = 8'b00000000;
    ram[5456] = 8'b00000000;
    ram[5455] = 8'b00000000;
    ram[5454] = 8'b00000000;
    ram[5453] = 8'b00000000;
    ram[5452] = 8'b00000000;
    ram[5451] = 8'b00000000;
    ram[5450] = 8'b00000000;
    ram[5449] = 8'b00000000;
    ram[5448] = 8'b00000000;
    ram[5447] = 8'b00000000;
    ram[5446] = 8'b00000000;
    ram[5445] = 8'b00000000;
    ram[5444] = 8'b00000000;
    ram[5443] = 8'b00000000;
    ram[5442] = 8'b00000000;
    ram[5441] = 8'b00000000;
    ram[5440] = 8'b00000000;
    ram[5439] = 8'b00000000;
    ram[5438] = 8'b00000000;
    ram[5437] = 8'b00000000;
    ram[5436] = 8'b00000000;
    ram[5435] = 8'b00000000;
    ram[5434] = 8'b00000000;
    ram[5433] = 8'b00000000;
    ram[5432] = 8'b00000000;
    ram[5431] = 8'b00000000;
    ram[5430] = 8'b00000000;
    ram[5429] = 8'b00000000;
    ram[5428] = 8'b00000000;
    ram[5427] = 8'b00000000;
    ram[5426] = 8'b00000000;
    ram[5425] = 8'b00000000;
    ram[5424] = 8'b00000000;
    ram[5423] = 8'b00000000;
    ram[5422] = 8'b00000000;
    ram[5421] = 8'b00000000;
    ram[5420] = 8'b00000000;
    ram[5419] = 8'b00000000;
    ram[5418] = 8'b00000000;
    ram[5417] = 8'b00000000;
    ram[5416] = 8'b00000000;
    ram[5415] = 8'b00000000;
    ram[5414] = 8'b00000000;
    ram[5413] = 8'b00000000;
    ram[5412] = 8'b00000000;
    ram[5411] = 8'b00000000;
    ram[5410] = 8'b00000000;
    ram[5409] = 8'b00000000;
    ram[5408] = 8'b00000000;
    ram[5407] = 8'b00000000;
    ram[5406] = 8'b00000000;
    ram[5405] = 8'b00000000;
    ram[5404] = 8'b00000000;
    ram[5403] = 8'b00000000;
    ram[5402] = 8'b00000000;
    ram[5401] = 8'b00000000;
    ram[5400] = 8'b00000000;
    ram[5399] = 8'b00000000;
    ram[5398] = 8'b00000000;
    ram[5397] = 8'b00000000;
    ram[5396] = 8'b00000000;
    ram[5395] = 8'b00000000;
    ram[5394] = 8'b00000000;
    ram[5393] = 8'b00000000;
    ram[5392] = 8'b00000000;
    ram[5391] = 8'b00000000;
    ram[5390] = 8'b00000000;
    ram[5389] = 8'b00000000;
    ram[5388] = 8'b00000000;
    ram[5387] = 8'b00000000;
    ram[5386] = 8'b00000000;
    ram[5385] = 8'b00000000;
    ram[5384] = 8'b00000000;
    ram[5383] = 8'b00000000;
    ram[5382] = 8'b00000000;
    ram[5381] = 8'b00000000;
    ram[5380] = 8'b00000000;
    ram[5379] = 8'b00000000;
    ram[5378] = 8'b00000000;
    ram[5377] = 8'b00000000;
    ram[5376] = 8'b00000000;
    ram[5375] = 8'b00000000;
    ram[5374] = 8'b00000000;
    ram[5373] = 8'b00000000;
    ram[5372] = 8'b00000000;
    ram[5371] = 8'b00000000;
    ram[5370] = 8'b00000000;
    ram[5369] = 8'b00000000;
    ram[5368] = 8'b00000000;
    ram[5367] = 8'b00000000;
    ram[5366] = 8'b00000000;
    ram[5365] = 8'b00000000;
    ram[5364] = 8'b00000000;
    ram[5363] = 8'b00000000;
    ram[5362] = 8'b00000000;
    ram[5361] = 8'b00000000;
    ram[5360] = 8'b00000000;
    ram[5359] = 8'b00000000;
    ram[5358] = 8'b00000000;
    ram[5357] = 8'b00000000;
    ram[5356] = 8'b00000000;
    ram[5355] = 8'b00000000;
    ram[5354] = 8'b00000000;
    ram[5353] = 8'b00000000;
    ram[5352] = 8'b00000000;
    ram[5351] = 8'b00000000;
    ram[5350] = 8'b00000000;
    ram[5349] = 8'b00000000;
    ram[5348] = 8'b00000000;
    ram[5347] = 8'b00000000;
    ram[5346] = 8'b00000000;
    ram[5345] = 8'b00000000;
    ram[5344] = 8'b00000000;
    ram[5343] = 8'b00000000;
    ram[5342] = 8'b00000000;
    ram[5341] = 8'b00000000;
    ram[5340] = 8'b00000000;
    ram[5339] = 8'b00000000;
    ram[5338] = 8'b00000000;
    ram[5337] = 8'b00000000;
    ram[5336] = 8'b00000000;
    ram[5335] = 8'b00000000;
    ram[5334] = 8'b00000000;
    ram[5333] = 8'b00000000;
    ram[5332] = 8'b00000000;
    ram[5331] = 8'b00000000;
    ram[5330] = 8'b00000000;
    ram[5329] = 8'b00000000;
    ram[5328] = 8'b00000000;
    ram[5327] = 8'b00000000;
    ram[5326] = 8'b00000000;
    ram[5325] = 8'b00000000;
    ram[5324] = 8'b00000000;
    ram[5323] = 8'b00000000;
    ram[5322] = 8'b00000000;
    ram[5321] = 8'b00000000;
    ram[5320] = 8'b00000000;
    ram[5319] = 8'b00000000;
    ram[5318] = 8'b00000000;
    ram[5317] = 8'b00000000;
    ram[5316] = 8'b00000000;
    ram[5315] = 8'b00000000;
    ram[5314] = 8'b00000000;
    ram[5313] = 8'b00000000;
    ram[5312] = 8'b00000000;
    ram[5311] = 8'b00000000;
    ram[5310] = 8'b00000000;
    ram[5309] = 8'b00000000;
    ram[5308] = 8'b00000000;
    ram[5307] = 8'b00000000;
    ram[5306] = 8'b00000000;
    ram[5305] = 8'b00000000;
    ram[5304] = 8'b00000000;
    ram[5303] = 8'b00000000;
    ram[5302] = 8'b00000000;
    ram[5301] = 8'b00000000;
    ram[5300] = 8'b00000000;
    ram[5299] = 8'b00000000;
    ram[5298] = 8'b00000000;
    ram[5297] = 8'b00000000;
    ram[5296] = 8'b00000000;
    ram[5295] = 8'b00000000;
    ram[5294] = 8'b00000000;
    ram[5293] = 8'b00000000;
    ram[5292] = 8'b00000000;
    ram[5291] = 8'b00000000;
    ram[5290] = 8'b00000000;
    ram[5289] = 8'b00000000;
    ram[5288] = 8'b00000000;
    ram[5287] = 8'b00000000;
    ram[5286] = 8'b00000000;
    ram[5285] = 8'b00000000;
    ram[5284] = 8'b00000000;
    ram[5283] = 8'b00000000;
    ram[5282] = 8'b00000000;
    ram[5281] = 8'b00000000;
    ram[5280] = 8'b00000000;
    ram[5279] = 8'b00000000;
    ram[5278] = 8'b00000000;
    ram[5277] = 8'b00000000;
    ram[5276] = 8'b00000000;
    ram[5275] = 8'b00000000;
    ram[5274] = 8'b00000000;
    ram[5273] = 8'b00000000;
    ram[5272] = 8'b00000000;
    ram[5271] = 8'b00000000;
    ram[5270] = 8'b00000000;
    ram[5269] = 8'b00000000;
    ram[5268] = 8'b00000000;
    ram[5267] = 8'b00000000;
    ram[5266] = 8'b00000000;
    ram[5265] = 8'b00000000;
    ram[5264] = 8'b00000000;
    ram[5263] = 8'b00000000;
    ram[5262] = 8'b00000000;
    ram[5261] = 8'b00000000;
    ram[5260] = 8'b00000000;
    ram[5259] = 8'b00000000;
    ram[5258] = 8'b00000000;
    ram[5257] = 8'b00000000;
    ram[5256] = 8'b00000000;
    ram[5255] = 8'b00000000;
    ram[5254] = 8'b00000000;
    ram[5253] = 8'b00000000;
    ram[5252] = 8'b00000000;
    ram[5251] = 8'b00000000;
    ram[5250] = 8'b00000000;
    ram[5249] = 8'b00000000;
    ram[5248] = 8'b00000000;
    ram[5247] = 8'b00000000;
    ram[5246] = 8'b00000000;
    ram[5245] = 8'b00000000;
    ram[5244] = 8'b00000000;
    ram[5243] = 8'b00000000;
    ram[5242] = 8'b00000000;
    ram[5241] = 8'b00000000;
    ram[5240] = 8'b00000000;
    ram[5239] = 8'b00000000;
    ram[5238] = 8'b00000000;
    ram[5237] = 8'b00000000;
    ram[5236] = 8'b00000000;
    ram[5235] = 8'b00000000;
    ram[5234] = 8'b00000000;
    ram[5233] = 8'b00000000;
    ram[5232] = 8'b00000000;
    ram[5231] = 8'b00000000;
    ram[5230] = 8'b00000000;
    ram[5229] = 8'b00000000;
    ram[5228] = 8'b00000000;
    ram[5227] = 8'b00000000;
    ram[5226] = 8'b00000000;
    ram[5225] = 8'b00000000;
    ram[5224] = 8'b00000000;
    ram[5223] = 8'b00000000;
    ram[5222] = 8'b00000000;
    ram[5221] = 8'b00000000;
    ram[5220] = 8'b00000000;
    ram[5219] = 8'b00000000;
    ram[5218] = 8'b00000000;
    ram[5217] = 8'b00000000;
    ram[5216] = 8'b00000000;
    ram[5215] = 8'b00000000;
    ram[5214] = 8'b00000000;
    ram[5213] = 8'b00000000;
    ram[5212] = 8'b00000000;
    ram[5211] = 8'b00000000;
    ram[5210] = 8'b00000000;
    ram[5209] = 8'b00000000;
    ram[5208] = 8'b00000000;
    ram[5207] = 8'b00000000;
    ram[5206] = 8'b00000000;
    ram[5205] = 8'b00000000;
    ram[5204] = 8'b00000000;
    ram[5203] = 8'b00000000;
    ram[5202] = 8'b00000000;
    ram[5201] = 8'b00000000;
    ram[5200] = 8'b00000000;
    ram[5199] = 8'b00000000;
    ram[5198] = 8'b00000000;
    ram[5197] = 8'b00000000;
    ram[5196] = 8'b00000000;
    ram[5195] = 8'b00000000;
    ram[5194] = 8'b00000000;
    ram[5193] = 8'b00000000;
    ram[5192] = 8'b00000000;
    ram[5191] = 8'b00000000;
    ram[5190] = 8'b00000000;
    ram[5189] = 8'b00000000;
    ram[5188] = 8'b00000000;
    ram[5187] = 8'b00000000;
    ram[5186] = 8'b00000000;
    ram[5185] = 8'b00000000;
    ram[5184] = 8'b00000000;
    ram[5183] = 8'b00000000;
    ram[5182] = 8'b00000000;
    ram[5181] = 8'b00000000;
    ram[5180] = 8'b00000000;
    ram[5179] = 8'b00000000;
    ram[5178] = 8'b00000000;
    ram[5177] = 8'b00000000;
    ram[5176] = 8'b00000000;
    ram[5175] = 8'b00000000;
    ram[5174] = 8'b00000000;
    ram[5173] = 8'b00000000;
    ram[5172] = 8'b00000000;
    ram[5171] = 8'b00000000;
    ram[5170] = 8'b00000000;
    ram[5169] = 8'b00000000;
    ram[5168] = 8'b00000000;
    ram[5167] = 8'b00000000;
    ram[5166] = 8'b00000000;
    ram[5165] = 8'b00000000;
    ram[5164] = 8'b00000000;
    ram[5163] = 8'b00000000;
    ram[5162] = 8'b00000000;
    ram[5161] = 8'b00000000;
    ram[5160] = 8'b00000000;
    ram[5159] = 8'b00000000;
    ram[5158] = 8'b00000000;
    ram[5157] = 8'b00000000;
    ram[5156] = 8'b00000000;
    ram[5155] = 8'b00000000;
    ram[5154] = 8'b00000000;
    ram[5153] = 8'b00000000;
    ram[5152] = 8'b00000000;
    ram[5151] = 8'b00000000;
    ram[5150] = 8'b00000000;
    ram[5149] = 8'b00000000;
    ram[5148] = 8'b00000000;
    ram[5147] = 8'b00000000;
    ram[5146] = 8'b00000000;
    ram[5145] = 8'b00000000;
    ram[5144] = 8'b00000000;
    ram[5143] = 8'b00000000;
    ram[5142] = 8'b00000000;
    ram[5141] = 8'b00000000;
    ram[5140] = 8'b00000000;
    ram[5139] = 8'b00000000;
    ram[5138] = 8'b00000000;
    ram[5137] = 8'b00000000;
    ram[5136] = 8'b00000000;
    ram[5135] = 8'b00000000;
    ram[5134] = 8'b00000000;
    ram[5133] = 8'b00000000;
    ram[5132] = 8'b00000000;
    ram[5131] = 8'b00000000;
    ram[5130] = 8'b00000000;
    ram[5129] = 8'b00000000;
    ram[5128] = 8'b00000000;
    ram[5127] = 8'b00000000;
    ram[5126] = 8'b00000000;
    ram[5125] = 8'b00000000;
    ram[5124] = 8'b00000000;
    ram[5123] = 8'b00000000;
    ram[5122] = 8'b00000000;
    ram[5121] = 8'b00000000;
    ram[5120] = 8'b00000000;
    ram[5119] = 8'b00000000;
    ram[5118] = 8'b00000000;
    ram[5117] = 8'b00000000;
    ram[5116] = 8'b00000000;
    ram[5115] = 8'b00000000;
    ram[5114] = 8'b00000000;
    ram[5113] = 8'b00000000;
    ram[5112] = 8'b00000000;
    ram[5111] = 8'b00000000;
    ram[5110] = 8'b00000000;
    ram[5109] = 8'b00000000;
    ram[5108] = 8'b00000000;
    ram[5107] = 8'b00000000;
    ram[5106] = 8'b00000000;
    ram[5105] = 8'b00000000;
    ram[5104] = 8'b00000000;
    ram[5103] = 8'b00000000;
    ram[5102] = 8'b00000000;
    ram[5101] = 8'b00000000;
    ram[5100] = 8'b00000000;
    ram[5099] = 8'b00000000;
    ram[5098] = 8'b00000000;
    ram[5097] = 8'b00000000;
    ram[5096] = 8'b00000000;
    ram[5095] = 8'b00000000;
    ram[5094] = 8'b00000000;
    ram[5093] = 8'b00000000;
    ram[5092] = 8'b00000000;
    ram[5091] = 8'b00000000;
    ram[5090] = 8'b00000000;
    ram[5089] = 8'b00000000;
    ram[5088] = 8'b00000000;
    ram[5087] = 8'b00000000;
    ram[5086] = 8'b00000000;
    ram[5085] = 8'b00000000;
    ram[5084] = 8'b00000000;
    ram[5083] = 8'b00000000;
    ram[5082] = 8'b00000000;
    ram[5081] = 8'b00000000;
    ram[5080] = 8'b00000000;
    ram[5079] = 8'b00000000;
    ram[5078] = 8'b00000000;
    ram[5077] = 8'b00000000;
    ram[5076] = 8'b00000000;
    ram[5075] = 8'b00000000;
    ram[5074] = 8'b00000000;
    ram[5073] = 8'b00000000;
    ram[5072] = 8'b00000000;
    ram[5071] = 8'b00000000;
    ram[5070] = 8'b00000000;
    ram[5069] = 8'b00000000;
    ram[5068] = 8'b00000000;
    ram[5067] = 8'b00000000;
    ram[5066] = 8'b00000000;
    ram[5065] = 8'b00000000;
    ram[5064] = 8'b00000000;
    ram[5063] = 8'b00000000;
    ram[5062] = 8'b00000000;
    ram[5061] = 8'b00000000;
    ram[5060] = 8'b00000000;
    ram[5059] = 8'b00000000;
    ram[5058] = 8'b00000000;
    ram[5057] = 8'b00000000;
    ram[5056] = 8'b00000000;
    ram[5055] = 8'b00000000;
    ram[5054] = 8'b00000000;
    ram[5053] = 8'b00000000;
    ram[5052] = 8'b00000000;
    ram[5051] = 8'b00000000;
    ram[5050] = 8'b00000000;
    ram[5049] = 8'b00000000;
    ram[5048] = 8'b00000000;
    ram[5047] = 8'b00000000;
    ram[5046] = 8'b00000000;
    ram[5045] = 8'b00000000;
    ram[5044] = 8'b00000000;
    ram[5043] = 8'b00000000;
    ram[5042] = 8'b00000000;
    ram[5041] = 8'b00000000;
    ram[5040] = 8'b00000000;
    ram[5039] = 8'b00000000;
    ram[5038] = 8'b00000000;
    ram[5037] = 8'b00000000;
    ram[5036] = 8'b00000000;
    ram[5035] = 8'b00000000;
    ram[5034] = 8'b00000000;
    ram[5033] = 8'b00000000;
    ram[5032] = 8'b00000000;
    ram[5031] = 8'b00000000;
    ram[5030] = 8'b00000000;
    ram[5029] = 8'b00000000;
    ram[5028] = 8'b00000000;
    ram[5027] = 8'b00000000;
    ram[5026] = 8'b00000000;
    ram[5025] = 8'b00000000;
    ram[5024] = 8'b00000000;
    ram[5023] = 8'b00000000;
    ram[5022] = 8'b00000000;
    ram[5021] = 8'b00000000;
    ram[5020] = 8'b00000000;
    ram[5019] = 8'b00000000;
    ram[5018] = 8'b00000000;
    ram[5017] = 8'b00000000;
    ram[5016] = 8'b00000000;
    ram[5015] = 8'b00000000;
    ram[5014] = 8'b00000000;
    ram[5013] = 8'b00000000;
    ram[5012] = 8'b00000000;
    ram[5011] = 8'b00000000;
    ram[5010] = 8'b00000000;
    ram[5009] = 8'b00000000;
    ram[5008] = 8'b00000000;
    ram[5007] = 8'b00000000;
    ram[5006] = 8'b00000000;
    ram[5005] = 8'b00000000;
    ram[5004] = 8'b00000000;
    ram[5003] = 8'b00000000;
    ram[5002] = 8'b00000000;
    ram[5001] = 8'b00000000;
    ram[5000] = 8'b00000000;
    ram[4999] = 8'b00000000;
    ram[4998] = 8'b00000000;
    ram[4997] = 8'b00000000;
    ram[4996] = 8'b00000000;
    ram[4995] = 8'b00000000;
    ram[4994] = 8'b00000000;
    ram[4993] = 8'b00000000;
    ram[4992] = 8'b00000000;
    ram[4991] = 8'b00000000;
    ram[4990] = 8'b00000000;
    ram[4989] = 8'b00000000;
    ram[4988] = 8'b00000000;
    ram[4987] = 8'b00000000;
    ram[4986] = 8'b00000000;
    ram[4985] = 8'b00000000;
    ram[4984] = 8'b00000000;
    ram[4983] = 8'b00000000;
    ram[4982] = 8'b00000000;
    ram[4981] = 8'b00000000;
    ram[4980] = 8'b00000000;
    ram[4979] = 8'b00000000;
    ram[4978] = 8'b00000000;
    ram[4977] = 8'b00000000;
    ram[4976] = 8'b00000000;
    ram[4975] = 8'b00000000;
    ram[4974] = 8'b00000000;
    ram[4973] = 8'b00000000;
    ram[4972] = 8'b00000000;
    ram[4971] = 8'b00000000;
    ram[4970] = 8'b00000000;
    ram[4969] = 8'b00000000;
    ram[4968] = 8'b00000000;
    ram[4967] = 8'b00000000;
    ram[4966] = 8'b00000000;
    ram[4965] = 8'b00000000;
    ram[4964] = 8'b00000000;
    ram[4963] = 8'b00000000;
    ram[4962] = 8'b00000000;
    ram[4961] = 8'b00000000;
    ram[4960] = 8'b00000000;
    ram[4959] = 8'b00000000;
    ram[4958] = 8'b00000000;
    ram[4957] = 8'b00000000;
    ram[4956] = 8'b00000000;
    ram[4955] = 8'b00000000;
    ram[4954] = 8'b00000000;
    ram[4953] = 8'b00000000;
    ram[4952] = 8'b00000000;
    ram[4951] = 8'b00000000;
    ram[4950] = 8'b00000000;
    ram[4949] = 8'b00000000;
    ram[4948] = 8'b00000000;
    ram[4947] = 8'b00000000;
    ram[4946] = 8'b00000000;
    ram[4945] = 8'b00000000;
    ram[4944] = 8'b00000000;
    ram[4943] = 8'b00000000;
    ram[4942] = 8'b00000000;
    ram[4941] = 8'b00000000;
    ram[4940] = 8'b00000000;
    ram[4939] = 8'b00000000;
    ram[4938] = 8'b00000000;
    ram[4937] = 8'b00000000;
    ram[4936] = 8'b00000000;
    ram[4935] = 8'b00000000;
    ram[4934] = 8'b00000000;
    ram[4933] = 8'b00000000;
    ram[4932] = 8'b00000000;
    ram[4931] = 8'b00000000;
    ram[4930] = 8'b00000000;
    ram[4929] = 8'b00000000;
    ram[4928] = 8'b00000000;
    ram[4927] = 8'b00000000;
    ram[4926] = 8'b00000000;
    ram[4925] = 8'b00000000;
    ram[4924] = 8'b00000000;
    ram[4923] = 8'b00000000;
    ram[4922] = 8'b00000000;
    ram[4921] = 8'b00000000;
    ram[4920] = 8'b00000000;
    ram[4919] = 8'b00000000;
    ram[4918] = 8'b00000000;
    ram[4917] = 8'b00000000;
    ram[4916] = 8'b00000000;
    ram[4915] = 8'b00000000;
    ram[4914] = 8'b00000000;
    ram[4913] = 8'b00000000;
    ram[4912] = 8'b00000000;
    ram[4911] = 8'b00000000;
    ram[4910] = 8'b00000000;
    ram[4909] = 8'b00000000;
    ram[4908] = 8'b00000000;
    ram[4907] = 8'b00000000;
    ram[4906] = 8'b00000000;
    ram[4905] = 8'b00000000;
    ram[4904] = 8'b00000000;
    ram[4903] = 8'b00000000;
    ram[4902] = 8'b00000000;
    ram[4901] = 8'b00000000;
    ram[4900] = 8'b00000000;
    ram[4899] = 8'b00000000;
    ram[4898] = 8'b00000000;
    ram[4897] = 8'b00000000;
    ram[4896] = 8'b00000000;
    ram[4895] = 8'b00000000;
    ram[4894] = 8'b00000000;
    ram[4893] = 8'b00000000;
    ram[4892] = 8'b00000000;
    ram[4891] = 8'b00000000;
    ram[4890] = 8'b00000000;
    ram[4889] = 8'b00000000;
    ram[4888] = 8'b00000000;
    ram[4887] = 8'b00000000;
    ram[4886] = 8'b00000000;
    ram[4885] = 8'b00000000;
    ram[4884] = 8'b00000000;
    ram[4883] = 8'b00000000;
    ram[4882] = 8'b00000000;
    ram[4881] = 8'b00000000;
    ram[4880] = 8'b00000000;
    ram[4879] = 8'b00000000;
    ram[4878] = 8'b00000000;
    ram[4877] = 8'b00000000;
    ram[4876] = 8'b00000000;
    ram[4875] = 8'b00000000;
    ram[4874] = 8'b00000000;
    ram[4873] = 8'b00000000;
    ram[4872] = 8'b00000000;
    ram[4871] = 8'b00000000;
    ram[4870] = 8'b00000000;
    ram[4869] = 8'b00000000;
    ram[4868] = 8'b00000000;
    ram[4867] = 8'b00000000;
    ram[4866] = 8'b00000000;
    ram[4865] = 8'b00000000;
    ram[4864] = 8'b00000000;
    ram[4863] = 8'b00000000;
    ram[4862] = 8'b00000000;
    ram[4861] = 8'b00000000;
    ram[4860] = 8'b00000000;
    ram[4859] = 8'b00000000;
    ram[4858] = 8'b00000000;
    ram[4857] = 8'b00000000;
    ram[4856] = 8'b00000000;
    ram[4855] = 8'b00000000;
    ram[4854] = 8'b00000000;
    ram[4853] = 8'b00000000;
    ram[4852] = 8'b00000000;
    ram[4851] = 8'b00000000;
    ram[4850] = 8'b00000000;
    ram[4849] = 8'b00000000;
    ram[4848] = 8'b00000000;
    ram[4847] = 8'b00000000;
    ram[4846] = 8'b00000000;
    ram[4845] = 8'b00000000;
    ram[4844] = 8'b00000000;
    ram[4843] = 8'b00000000;
    ram[4842] = 8'b00000000;
    ram[4841] = 8'b00000000;
    ram[4840] = 8'b00000000;
    ram[4839] = 8'b00000000;
    ram[4838] = 8'b00000000;
    ram[4837] = 8'b00000000;
    ram[4836] = 8'b00000000;
    ram[4835] = 8'b00000000;
    ram[4834] = 8'b00000000;
    ram[4833] = 8'b00000000;
    ram[4832] = 8'b00000000;
    ram[4831] = 8'b00000000;
    ram[4830] = 8'b00000000;
    ram[4829] = 8'b00000000;
    ram[4828] = 8'b00000000;
    ram[4827] = 8'b00000000;
    ram[4826] = 8'b00000000;
    ram[4825] = 8'b00000000;
    ram[4824] = 8'b00000000;
    ram[4823] = 8'b00000000;
    ram[4822] = 8'b00000000;
    ram[4821] = 8'b00000000;
    ram[4820] = 8'b00000000;
    ram[4819] = 8'b00000000;
    ram[4818] = 8'b00000000;
    ram[4817] = 8'b00000000;
    ram[4816] = 8'b00000000;
    ram[4815] = 8'b00000000;
    ram[4814] = 8'b00000000;
    ram[4813] = 8'b00000000;
    ram[4812] = 8'b00000000;
    ram[4811] = 8'b00000000;
    ram[4810] = 8'b00000000;
    ram[4809] = 8'b00000000;
    ram[4808] = 8'b00000000;
    ram[4807] = 8'b00000000;
    ram[4806] = 8'b00000000;
    ram[4805] = 8'b00000000;
    ram[4804] = 8'b00000000;
    ram[4803] = 8'b00000000;
    ram[4802] = 8'b00000000;
    ram[4801] = 8'b00000000;
    ram[4800] = 8'b00000000;
    ram[4799] = 8'b00000000;
    ram[4798] = 8'b00000000;
    ram[4797] = 8'b00000000;
    ram[4796] = 8'b00000000;
    ram[4795] = 8'b00000000;
    ram[4794] = 8'b00000000;
    ram[4793] = 8'b00000000;
    ram[4792] = 8'b00000000;
    ram[4791] = 8'b00000000;
    ram[4790] = 8'b00000000;
    ram[4789] = 8'b00000000;
    ram[4788] = 8'b00000000;
    ram[4787] = 8'b00000000;
    ram[4786] = 8'b00000000;
    ram[4785] = 8'b00000000;
    ram[4784] = 8'b00000000;
    ram[4783] = 8'b00000000;
    ram[4782] = 8'b00000000;
    ram[4781] = 8'b00000000;
    ram[4780] = 8'b00000000;
    ram[4779] = 8'b00000000;
    ram[4778] = 8'b00000000;
    ram[4777] = 8'b00000000;
    ram[4776] = 8'b00000000;
    ram[4775] = 8'b00000000;
    ram[4774] = 8'b00000000;
    ram[4773] = 8'b00000000;
    ram[4772] = 8'b00000000;
    ram[4771] = 8'b00000000;
    ram[4770] = 8'b00000000;
    ram[4769] = 8'b00000000;
    ram[4768] = 8'b00000000;
    ram[4767] = 8'b00000000;
    ram[4766] = 8'b00000000;
    ram[4765] = 8'b00000000;
    ram[4764] = 8'b00000000;
    ram[4763] = 8'b00000000;
    ram[4762] = 8'b00000000;
    ram[4761] = 8'b00000000;
    ram[4760] = 8'b00000000;
    ram[4759] = 8'b00000000;
    ram[4758] = 8'b00000000;
    ram[4757] = 8'b00000000;
    ram[4756] = 8'b00000000;
    ram[4755] = 8'b00000000;
    ram[4754] = 8'b00000000;
    ram[4753] = 8'b00000000;
    ram[4752] = 8'b00000000;
    ram[4751] = 8'b00000000;
    ram[4750] = 8'b00000000;
    ram[4749] = 8'b00000000;
    ram[4748] = 8'b00000000;
    ram[4747] = 8'b00000000;
    ram[4746] = 8'b00000000;
    ram[4745] = 8'b00000000;
    ram[4744] = 8'b00000000;
    ram[4743] = 8'b00000000;
    ram[4742] = 8'b00000000;
    ram[4741] = 8'b00000000;
    ram[4740] = 8'b00000000;
    ram[4739] = 8'b00000000;
    ram[4738] = 8'b00000000;
    ram[4737] = 8'b00000000;
    ram[4736] = 8'b00000000;
    ram[4735] = 8'b00000000;
    ram[4734] = 8'b00000000;
    ram[4733] = 8'b00000000;
    ram[4732] = 8'b00000000;
    ram[4731] = 8'b00000000;
    ram[4730] = 8'b00000000;
    ram[4729] = 8'b00000000;
    ram[4728] = 8'b00000000;
    ram[4727] = 8'b00000000;
    ram[4726] = 8'b00000000;
    ram[4725] = 8'b00000000;
    ram[4724] = 8'b00000000;
    ram[4723] = 8'b00000000;
    ram[4722] = 8'b00000000;
    ram[4721] = 8'b00000000;
    ram[4720] = 8'b00000000;
    ram[4719] = 8'b00000000;
    ram[4718] = 8'b00000000;
    ram[4717] = 8'b00000000;
    ram[4716] = 8'b00000000;
    ram[4715] = 8'b00000000;
    ram[4714] = 8'b00000000;
    ram[4713] = 8'b00000000;
    ram[4712] = 8'b00000000;
    ram[4711] = 8'b00000000;
    ram[4710] = 8'b00000000;
    ram[4709] = 8'b00000000;
    ram[4708] = 8'b00000000;
    ram[4707] = 8'b00000000;
    ram[4706] = 8'b00000000;
    ram[4705] = 8'b00000000;
    ram[4704] = 8'b00000000;
    ram[4703] = 8'b00000000;
    ram[4702] = 8'b00000000;
    ram[4701] = 8'b00000000;
    ram[4700] = 8'b00000000;
    ram[4699] = 8'b00000000;
    ram[4698] = 8'b00000000;
    ram[4697] = 8'b00000000;
    ram[4696] = 8'b00000000;
    ram[4695] = 8'b00000000;
    ram[4694] = 8'b00000000;
    ram[4693] = 8'b00000000;
    ram[4692] = 8'b00000000;
    ram[4691] = 8'b00000000;
    ram[4690] = 8'b00000000;
    ram[4689] = 8'b00000000;
    ram[4688] = 8'b00000000;
    ram[4687] = 8'b00000000;
    ram[4686] = 8'b00000000;
    ram[4685] = 8'b00000000;
    ram[4684] = 8'b00000000;
    ram[4683] = 8'b00000000;
    ram[4682] = 8'b00000000;
    ram[4681] = 8'b00000000;
    ram[4680] = 8'b00000000;
    ram[4679] = 8'b00000000;
    ram[4678] = 8'b00000000;
    ram[4677] = 8'b00000000;
    ram[4676] = 8'b00000000;
    ram[4675] = 8'b00000000;
    ram[4674] = 8'b00000000;
    ram[4673] = 8'b00000000;
    ram[4672] = 8'b00000000;
    ram[4671] = 8'b00000000;
    ram[4670] = 8'b00000000;
    ram[4669] = 8'b00000000;
    ram[4668] = 8'b00000000;
    ram[4667] = 8'b00000000;
    ram[4666] = 8'b00000000;
    ram[4665] = 8'b00000000;
    ram[4664] = 8'b00000000;
    ram[4663] = 8'b00000000;
    ram[4662] = 8'b00000000;
    ram[4661] = 8'b00000000;
    ram[4660] = 8'b00000000;
    ram[4659] = 8'b00000000;
    ram[4658] = 8'b00000000;
    ram[4657] = 8'b00000000;
    ram[4656] = 8'b00000000;
    ram[4655] = 8'b00000000;
    ram[4654] = 8'b00000000;
    ram[4653] = 8'b00000000;
    ram[4652] = 8'b00000000;
    ram[4651] = 8'b00000000;
    ram[4650] = 8'b00000000;
    ram[4649] = 8'b00000000;
    ram[4648] = 8'b00000000;
    ram[4647] = 8'b00000000;
    ram[4646] = 8'b00000000;
    ram[4645] = 8'b00000000;
    ram[4644] = 8'b00000000;
    ram[4643] = 8'b00000000;
    ram[4642] = 8'b00000000;
    ram[4641] = 8'b00000000;
    ram[4640] = 8'b00000000;
    ram[4639] = 8'b00000000;
    ram[4638] = 8'b00000000;
    ram[4637] = 8'b00000000;
    ram[4636] = 8'b00000000;
    ram[4635] = 8'b00000000;
    ram[4634] = 8'b00000000;
    ram[4633] = 8'b00000000;
    ram[4632] = 8'b00000000;
    ram[4631] = 8'b00000000;
    ram[4630] = 8'b00000000;
    ram[4629] = 8'b00000000;
    ram[4628] = 8'b00000000;
    ram[4627] = 8'b00000000;
    ram[4626] = 8'b00000000;
    ram[4625] = 8'b00000000;
    ram[4624] = 8'b00000000;
    ram[4623] = 8'b00000000;
    ram[4622] = 8'b00000000;
    ram[4621] = 8'b00000000;
    ram[4620] = 8'b00000000;
    ram[4619] = 8'b00000000;
    ram[4618] = 8'b00000000;
    ram[4617] = 8'b00000000;
    ram[4616] = 8'b00000000;
    ram[4615] = 8'b00000000;
    ram[4614] = 8'b00000000;
    ram[4613] = 8'b00000000;
    ram[4612] = 8'b00000000;
    ram[4611] = 8'b00000000;
    ram[4610] = 8'b00000000;
    ram[4609] = 8'b00000000;
    ram[4608] = 8'b00000000;
    ram[4607] = 8'b00000000;
    ram[4606] = 8'b00000000;
    ram[4605] = 8'b00000000;
    ram[4604] = 8'b00000000;
    ram[4603] = 8'b00000000;
    ram[4602] = 8'b00000000;
    ram[4601] = 8'b00000000;
    ram[4600] = 8'b00000000;
    ram[4599] = 8'b00000000;
    ram[4598] = 8'b00000000;
    ram[4597] = 8'b00000000;
    ram[4596] = 8'b00000000;
    ram[4595] = 8'b00000000;
    ram[4594] = 8'b00000000;
    ram[4593] = 8'b00000000;
    ram[4592] = 8'b00000000;
    ram[4591] = 8'b00000000;
    ram[4590] = 8'b00000000;
    ram[4589] = 8'b00000000;
    ram[4588] = 8'b00000000;
    ram[4587] = 8'b00000000;
    ram[4586] = 8'b00000000;
    ram[4585] = 8'b00000000;
    ram[4584] = 8'b00000000;
    ram[4583] = 8'b00000000;
    ram[4582] = 8'b00000000;
    ram[4581] = 8'b00000000;
    ram[4580] = 8'b00000000;
    ram[4579] = 8'b00000000;
    ram[4578] = 8'b00000000;
    ram[4577] = 8'b00000000;
    ram[4576] = 8'b00000000;
    ram[4575] = 8'b00000000;
    ram[4574] = 8'b00000000;
    ram[4573] = 8'b00000000;
    ram[4572] = 8'b00000000;
    ram[4571] = 8'b00000000;
    ram[4570] = 8'b00000000;
    ram[4569] = 8'b00000000;
    ram[4568] = 8'b00000000;
    ram[4567] = 8'b00000000;
    ram[4566] = 8'b00000000;
    ram[4565] = 8'b00000000;
    ram[4564] = 8'b00000000;
    ram[4563] = 8'b00000000;
    ram[4562] = 8'b00000000;
    ram[4561] = 8'b00000000;
    ram[4560] = 8'b00000000;
    ram[4559] = 8'b00000000;
    ram[4558] = 8'b00000000;
    ram[4557] = 8'b00000000;
    ram[4556] = 8'b00000000;
    ram[4555] = 8'b00000000;
    ram[4554] = 8'b00000000;
    ram[4553] = 8'b00000000;
    ram[4552] = 8'b00000000;
    ram[4551] = 8'b00000000;
    ram[4550] = 8'b00000000;
    ram[4549] = 8'b00000000;
    ram[4548] = 8'b00000000;
    ram[4547] = 8'b00000000;
    ram[4546] = 8'b00000000;
    ram[4545] = 8'b00000000;
    ram[4544] = 8'b00000000;
    ram[4543] = 8'b00000000;
    ram[4542] = 8'b00000000;
    ram[4541] = 8'b00000000;
    ram[4540] = 8'b00000000;
    ram[4539] = 8'b00000000;
    ram[4538] = 8'b00000000;
    ram[4537] = 8'b00000000;
    ram[4536] = 8'b00000000;
    ram[4535] = 8'b00000000;
    ram[4534] = 8'b00000000;
    ram[4533] = 8'b00000000;
    ram[4532] = 8'b00000000;
    ram[4531] = 8'b00000000;
    ram[4530] = 8'b00000000;
    ram[4529] = 8'b00000000;
    ram[4528] = 8'b00000000;
    ram[4527] = 8'b00000000;
    ram[4526] = 8'b00000000;
    ram[4525] = 8'b00000000;
    ram[4524] = 8'b00000000;
    ram[4523] = 8'b00000000;
    ram[4522] = 8'b00000000;
    ram[4521] = 8'b00000000;
    ram[4520] = 8'b00000000;
    ram[4519] = 8'b00000000;
    ram[4518] = 8'b00000000;
    ram[4517] = 8'b00000000;
    ram[4516] = 8'b00000000;
    ram[4515] = 8'b00000000;
    ram[4514] = 8'b00000000;
    ram[4513] = 8'b00000000;
    ram[4512] = 8'b00000000;
    ram[4511] = 8'b00000000;
    ram[4510] = 8'b00000000;
    ram[4509] = 8'b00000000;
    ram[4508] = 8'b00000000;
    ram[4507] = 8'b00000000;
    ram[4506] = 8'b00000000;
    ram[4505] = 8'b00000000;
    ram[4504] = 8'b00000000;
    ram[4503] = 8'b00000000;
    ram[4502] = 8'b00000000;
    ram[4501] = 8'b00000000;
    ram[4500] = 8'b00000000;
    ram[4499] = 8'b00000000;
    ram[4498] = 8'b00000000;
    ram[4497] = 8'b00000000;
    ram[4496] = 8'b00000000;
    ram[4495] = 8'b00000000;
    ram[4494] = 8'b00000000;
    ram[4493] = 8'b00000000;
    ram[4492] = 8'b00000000;
    ram[4491] = 8'b00000000;
    ram[4490] = 8'b00000000;
    ram[4489] = 8'b00000000;
    ram[4488] = 8'b00000000;
    ram[4487] = 8'b00000000;
    ram[4486] = 8'b00000000;
    ram[4485] = 8'b00000000;
    ram[4484] = 8'b00000000;
    ram[4483] = 8'b00000000;
    ram[4482] = 8'b00000000;
    ram[4481] = 8'b00000000;
    ram[4480] = 8'b00000000;
    ram[4479] = 8'b00000000;
    ram[4478] = 8'b00000000;
    ram[4477] = 8'b00000000;
    ram[4476] = 8'b00000000;
    ram[4475] = 8'b00000000;
    ram[4474] = 8'b00000000;
    ram[4473] = 8'b00000000;
    ram[4472] = 8'b00000000;
    ram[4471] = 8'b00000000;
    ram[4470] = 8'b00000000;
    ram[4469] = 8'b00000000;
    ram[4468] = 8'b00000000;
    ram[4467] = 8'b00000000;
    ram[4466] = 8'b00000000;
    ram[4465] = 8'b00000000;
    ram[4464] = 8'b00000000;
    ram[4463] = 8'b00000000;
    ram[4462] = 8'b00000000;
    ram[4461] = 8'b00000000;
    ram[4460] = 8'b00000000;
    ram[4459] = 8'b00000000;
    ram[4458] = 8'b00000000;
    ram[4457] = 8'b00000000;
    ram[4456] = 8'b00000000;
    ram[4455] = 8'b00000000;
    ram[4454] = 8'b00000000;
    ram[4453] = 8'b00000000;
    ram[4452] = 8'b00000000;
    ram[4451] = 8'b00000000;
    ram[4450] = 8'b00000000;
    ram[4449] = 8'b00000000;
    ram[4448] = 8'b00000000;
    ram[4447] = 8'b00000000;
    ram[4446] = 8'b00000000;
    ram[4445] = 8'b00000000;
    ram[4444] = 8'b00000000;
    ram[4443] = 8'b00000000;
    ram[4442] = 8'b00000000;
    ram[4441] = 8'b00000000;
    ram[4440] = 8'b00000000;
    ram[4439] = 8'b00000000;
    ram[4438] = 8'b00000000;
    ram[4437] = 8'b00000000;
    ram[4436] = 8'b00000000;
    ram[4435] = 8'b00000000;
    ram[4434] = 8'b00000000;
    ram[4433] = 8'b00000000;
    ram[4432] = 8'b00000000;
    ram[4431] = 8'b00000000;
    ram[4430] = 8'b00000000;
    ram[4429] = 8'b00000000;
    ram[4428] = 8'b00000000;
    ram[4427] = 8'b00000000;
    ram[4426] = 8'b00000000;
    ram[4425] = 8'b00000000;
    ram[4424] = 8'b00000000;
    ram[4423] = 8'b00000000;
    ram[4422] = 8'b00000000;
    ram[4421] = 8'b00000000;
    ram[4420] = 8'b00000000;
    ram[4419] = 8'b00000000;
    ram[4418] = 8'b00000000;
    ram[4417] = 8'b00000000;
    ram[4416] = 8'b00000000;
    ram[4415] = 8'b00000000;
    ram[4414] = 8'b00000000;
    ram[4413] = 8'b00000000;
    ram[4412] = 8'b00000000;
    ram[4411] = 8'b00000000;
    ram[4410] = 8'b00000000;
    ram[4409] = 8'b00000000;
    ram[4408] = 8'b00000000;
    ram[4407] = 8'b00000000;
    ram[4406] = 8'b00000000;
    ram[4405] = 8'b00000000;
    ram[4404] = 8'b00000000;
    ram[4403] = 8'b00000000;
    ram[4402] = 8'b00000000;
    ram[4401] = 8'b00000000;
    ram[4400] = 8'b00000000;
    ram[4399] = 8'b00000000;
    ram[4398] = 8'b00000000;
    ram[4397] = 8'b00000000;
    ram[4396] = 8'b00000000;
    ram[4395] = 8'b00000000;
    ram[4394] = 8'b00000000;
    ram[4393] = 8'b00000000;
    ram[4392] = 8'b00000000;
    ram[4391] = 8'b00000000;
    ram[4390] = 8'b00000000;
    ram[4389] = 8'b00000000;
    ram[4388] = 8'b00000000;
    ram[4387] = 8'b00000000;
    ram[4386] = 8'b00000000;
    ram[4385] = 8'b00000000;
    ram[4384] = 8'b00000000;
    ram[4383] = 8'b00000000;
    ram[4382] = 8'b00000000;
    ram[4381] = 8'b00000000;
    ram[4380] = 8'b00000000;
    ram[4379] = 8'b00000000;
    ram[4378] = 8'b00000000;
    ram[4377] = 8'b00000000;
    ram[4376] = 8'b00000000;
    ram[4375] = 8'b00000000;
    ram[4374] = 8'b00000000;
    ram[4373] = 8'b00000000;
    ram[4372] = 8'b00000000;
    ram[4371] = 8'b00000000;
    ram[4370] = 8'b00000000;
    ram[4369] = 8'b00000000;
    ram[4368] = 8'b00000000;
    ram[4367] = 8'b00000000;
    ram[4366] = 8'b00000000;
    ram[4365] = 8'b00000000;
    ram[4364] = 8'b00000000;
    ram[4363] = 8'b00000000;
    ram[4362] = 8'b00000000;
    ram[4361] = 8'b00000000;
    ram[4360] = 8'b00000000;
    ram[4359] = 8'b00000000;
    ram[4358] = 8'b00000000;
    ram[4357] = 8'b00000000;
    ram[4356] = 8'b00000000;
    ram[4355] = 8'b00000000;
    ram[4354] = 8'b00000000;
    ram[4353] = 8'b00000000;
    ram[4352] = 8'b00000000;
    ram[4351] = 8'b00000000;
    ram[4350] = 8'b00000000;
    ram[4349] = 8'b00000000;
    ram[4348] = 8'b00000000;
    ram[4347] = 8'b00000000;
    ram[4346] = 8'b00000000;
    ram[4345] = 8'b00000000;
    ram[4344] = 8'b00000000;
    ram[4343] = 8'b00000000;
    ram[4342] = 8'b00000000;
    ram[4341] = 8'b00000000;
    ram[4340] = 8'b00000000;
    ram[4339] = 8'b00000000;
    ram[4338] = 8'b00000000;
    ram[4337] = 8'b00000000;
    ram[4336] = 8'b00000000;
    ram[4335] = 8'b00000000;
    ram[4334] = 8'b00000000;
    ram[4333] = 8'b00000000;
    ram[4332] = 8'b00000000;
    ram[4331] = 8'b00000000;
    ram[4330] = 8'b00000000;
    ram[4329] = 8'b00000000;
    ram[4328] = 8'b00000000;
    ram[4327] = 8'b00000000;
    ram[4326] = 8'b00000000;
    ram[4325] = 8'b00000000;
    ram[4324] = 8'b00000000;
    ram[4323] = 8'b00000000;
    ram[4322] = 8'b00000000;
    ram[4321] = 8'b00000000;
    ram[4320] = 8'b00000000;
    ram[4319] = 8'b00000000;
    ram[4318] = 8'b00000000;
    ram[4317] = 8'b00000000;
    ram[4316] = 8'b00000000;
    ram[4315] = 8'b00000000;
    ram[4314] = 8'b00000000;
    ram[4313] = 8'b00000000;
    ram[4312] = 8'b00000000;
    ram[4311] = 8'b00000000;
    ram[4310] = 8'b00000000;
    ram[4309] = 8'b00000000;
    ram[4308] = 8'b00000000;
    ram[4307] = 8'b00000000;
    ram[4306] = 8'b00000000;
    ram[4305] = 8'b00000000;
    ram[4304] = 8'b00000000;
    ram[4303] = 8'b00000000;
    ram[4302] = 8'b00000000;
    ram[4301] = 8'b00000000;
    ram[4300] = 8'b00000000;
    ram[4299] = 8'b00000000;
    ram[4298] = 8'b00000000;
    ram[4297] = 8'b00000000;
    ram[4296] = 8'b00000000;
    ram[4295] = 8'b00000000;
    ram[4294] = 8'b00000000;
    ram[4293] = 8'b00000000;
    ram[4292] = 8'b00000000;
    ram[4291] = 8'b00000000;
    ram[4290] = 8'b00000000;
    ram[4289] = 8'b00000000;
    ram[4288] = 8'b00000000;
    ram[4287] = 8'b00000000;
    ram[4286] = 8'b00000000;
    ram[4285] = 8'b00000000;
    ram[4284] = 8'b00000000;
    ram[4283] = 8'b00000000;
    ram[4282] = 8'b00000000;
    ram[4281] = 8'b00000000;
    ram[4280] = 8'b00000000;
    ram[4279] = 8'b00000000;
    ram[4278] = 8'b00000000;
    ram[4277] = 8'b00000000;
    ram[4276] = 8'b00000000;
    ram[4275] = 8'b00000000;
    ram[4274] = 8'b00000000;
    ram[4273] = 8'b00000000;
    ram[4272] = 8'b00000000;
    ram[4271] = 8'b00000000;
    ram[4270] = 8'b00000000;
    ram[4269] = 8'b00000000;
    ram[4268] = 8'b00000000;
    ram[4267] = 8'b00000000;
    ram[4266] = 8'b00000000;
    ram[4265] = 8'b00000000;
    ram[4264] = 8'b00000000;
    ram[4263] = 8'b00000000;
    ram[4262] = 8'b00000000;
    ram[4261] = 8'b00000000;
    ram[4260] = 8'b00000000;
    ram[4259] = 8'b00000000;
    ram[4258] = 8'b00000000;
    ram[4257] = 8'b00000000;
    ram[4256] = 8'b00000000;
    ram[4255] = 8'b00000000;
    ram[4254] = 8'b00000000;
    ram[4253] = 8'b00000000;
    ram[4252] = 8'b00000000;
    ram[4251] = 8'b00000000;
    ram[4250] = 8'b00000000;
    ram[4249] = 8'b00000000;
    ram[4248] = 8'b00000000;
    ram[4247] = 8'b00000000;
    ram[4246] = 8'b00000000;
    ram[4245] = 8'b00000000;
    ram[4244] = 8'b00000000;
    ram[4243] = 8'b00000000;
    ram[4242] = 8'b00000000;
    ram[4241] = 8'b00000000;
    ram[4240] = 8'b00000000;
    ram[4239] = 8'b00000000;
    ram[4238] = 8'b00000000;
    ram[4237] = 8'b00000000;
    ram[4236] = 8'b00000000;
    ram[4235] = 8'b00000000;
    ram[4234] = 8'b00000000;
    ram[4233] = 8'b00000000;
    ram[4232] = 8'b00000000;
    ram[4231] = 8'b00000000;
    ram[4230] = 8'b00000000;
    ram[4229] = 8'b00000000;
    ram[4228] = 8'b00000000;
    ram[4227] = 8'b00000000;
    ram[4226] = 8'b00000000;
    ram[4225] = 8'b00000000;
    ram[4224] = 8'b00000000;
    ram[4223] = 8'b00000000;
    ram[4222] = 8'b00000000;
    ram[4221] = 8'b00000000;
    ram[4220] = 8'b00000000;
    ram[4219] = 8'b00000000;
    ram[4218] = 8'b00000000;
    ram[4217] = 8'b00000000;
    ram[4216] = 8'b00000000;
    ram[4215] = 8'b00000000;
    ram[4214] = 8'b00000000;
    ram[4213] = 8'b00000000;
    ram[4212] = 8'b00000000;
    ram[4211] = 8'b00000000;
    ram[4210] = 8'b00000000;
    ram[4209] = 8'b00000000;
    ram[4208] = 8'b00000000;
    ram[4207] = 8'b00000000;
    ram[4206] = 8'b00000000;
    ram[4205] = 8'b00000000;
    ram[4204] = 8'b00000000;
    ram[4203] = 8'b00000000;
    ram[4202] = 8'b00000000;
    ram[4201] = 8'b00000000;
    ram[4200] = 8'b00000000;
    ram[4199] = 8'b00000000;
    ram[4198] = 8'b00000000;
    ram[4197] = 8'b00000000;
    ram[4196] = 8'b00000000;
    ram[4195] = 8'b00000000;
    ram[4194] = 8'b00000000;
    ram[4193] = 8'b00000000;
    ram[4192] = 8'b00000000;
    ram[4191] = 8'b00000000;
    ram[4190] = 8'b00000000;
    ram[4189] = 8'b00000000;
    ram[4188] = 8'b00000000;
    ram[4187] = 8'b00000000;
    ram[4186] = 8'b00000000;
    ram[4185] = 8'b00000000;
    ram[4184] = 8'b00000000;
    ram[4183] = 8'b00000000;
    ram[4182] = 8'b00000000;
    ram[4181] = 8'b00000000;
    ram[4180] = 8'b00000000;
    ram[4179] = 8'b00000000;
    ram[4178] = 8'b00000000;
    ram[4177] = 8'b00000000;
    ram[4176] = 8'b00000000;
    ram[4175] = 8'b00000000;
    ram[4174] = 8'b00000000;
    ram[4173] = 8'b00000000;
    ram[4172] = 8'b00000000;
    ram[4171] = 8'b00000000;
    ram[4170] = 8'b00000000;
    ram[4169] = 8'b00000000;
    ram[4168] = 8'b00000000;
    ram[4167] = 8'b00000000;
    ram[4166] = 8'b00000000;
    ram[4165] = 8'b00000000;
    ram[4164] = 8'b00000000;
    ram[4163] = 8'b00000000;
    ram[4162] = 8'b00000000;
    ram[4161] = 8'b00000000;
    ram[4160] = 8'b00000000;
    ram[4159] = 8'b00000000;
    ram[4158] = 8'b00000000;
    ram[4157] = 8'b00000000;
    ram[4156] = 8'b00000000;
    ram[4155] = 8'b00000000;
    ram[4154] = 8'b00000000;
    ram[4153] = 8'b00000000;
    ram[4152] = 8'b00000000;
    ram[4151] = 8'b00000000;
    ram[4150] = 8'b00000000;
    ram[4149] = 8'b00000000;
    ram[4148] = 8'b00000000;
    ram[4147] = 8'b00000000;
    ram[4146] = 8'b00000000;
    ram[4145] = 8'b00000000;
    ram[4144] = 8'b00000000;
    ram[4143] = 8'b00000000;
    ram[4142] = 8'b00000000;
    ram[4141] = 8'b00000000;
    ram[4140] = 8'b00000000;
    ram[4139] = 8'b00000000;
    ram[4138] = 8'b00000000;
    ram[4137] = 8'b00000000;
    ram[4136] = 8'b00000000;
    ram[4135] = 8'b00000000;
    ram[4134] = 8'b00000000;
    ram[4133] = 8'b00000000;
    ram[4132] = 8'b00000000;
    ram[4131] = 8'b00000000;
    ram[4130] = 8'b00000000;
    ram[4129] = 8'b00000000;
    ram[4128] = 8'b00000000;
    ram[4127] = 8'b00000000;
    ram[4126] = 8'b00000000;
    ram[4125] = 8'b00000000;
    ram[4124] = 8'b00000000;
    ram[4123] = 8'b00000000;
    ram[4122] = 8'b00000000;
    ram[4121] = 8'b00000000;
    ram[4120] = 8'b00000000;
    ram[4119] = 8'b00000000;
    ram[4118] = 8'b00000000;
    ram[4117] = 8'b00000000;
    ram[4116] = 8'b00000000;
    ram[4115] = 8'b00000000;
    ram[4114] = 8'b00000000;
    ram[4113] = 8'b00000000;
    ram[4112] = 8'b00000000;
    ram[4111] = 8'b00000000;
    ram[4110] = 8'b00000000;
    ram[4109] = 8'b00000000;
    ram[4108] = 8'b00000000;
    ram[4107] = 8'b00000000;
    ram[4106] = 8'b00000000;
    ram[4105] = 8'b00000000;
    ram[4104] = 8'b00000000;
    ram[4103] = 8'b00000000;
    ram[4102] = 8'b00000000;
    ram[4101] = 8'b00000000;
    ram[4100] = 8'b00000000;
    ram[4099] = 8'b00000000;
    ram[4098] = 8'b00000000;
    ram[4097] = 8'b00000000;
    ram[4096] = 8'b00000000;
    ram[4095] = 8'b00000000;
    ram[4094] = 8'b00000000;
    ram[4093] = 8'b00000000;
    ram[4092] = 8'b00000000;
    ram[4091] = 8'b00000000;
    ram[4090] = 8'b00000000;
    ram[4089] = 8'b00000000;
    ram[4088] = 8'b00000000;
    ram[4087] = 8'b00000000;
    ram[4086] = 8'b00000000;
    ram[4085] = 8'b00000000;
    ram[4084] = 8'b00000000;
    ram[4083] = 8'b00000000;
    ram[4082] = 8'b00000000;
    ram[4081] = 8'b00000000;
    ram[4080] = 8'b00000000;
    ram[4079] = 8'b00000000;
    ram[4078] = 8'b00000000;
    ram[4077] = 8'b00000000;
    ram[4076] = 8'b00000000;
    ram[4075] = 8'b00000000;
    ram[4074] = 8'b00000000;
    ram[4073] = 8'b00000000;
    ram[4072] = 8'b00000000;
    ram[4071] = 8'b00000000;
    ram[4070] = 8'b00000000;
    ram[4069] = 8'b00000000;
    ram[4068] = 8'b00000000;
    ram[4067] = 8'b00000000;
    ram[4066] = 8'b00000000;
    ram[4065] = 8'b00000000;
    ram[4064] = 8'b00000000;
    ram[4063] = 8'b00000000;
    ram[4062] = 8'b00000000;
    ram[4061] = 8'b00000000;
    ram[4060] = 8'b00000000;
    ram[4059] = 8'b00000000;
    ram[4058] = 8'b00000000;
    ram[4057] = 8'b00000000;
    ram[4056] = 8'b00000000;
    ram[4055] = 8'b00000000;
    ram[4054] = 8'b00000000;
    ram[4053] = 8'b00000000;
    ram[4052] = 8'b00000000;
    ram[4051] = 8'b00000000;
    ram[4050] = 8'b00000000;
    ram[4049] = 8'b00000000;
    ram[4048] = 8'b00000000;
    ram[4047] = 8'b00000000;
    ram[4046] = 8'b00000000;
    ram[4045] = 8'b00000000;
    ram[4044] = 8'b00000000;
    ram[4043] = 8'b00000000;
    ram[4042] = 8'b00000000;
    ram[4041] = 8'b00000000;
    ram[4040] = 8'b00000000;
    ram[4039] = 8'b00000000;
    ram[4038] = 8'b00000000;
    ram[4037] = 8'b00000000;
    ram[4036] = 8'b00000000;
    ram[4035] = 8'b00000000;
    ram[4034] = 8'b00000000;
    ram[4033] = 8'b00000000;
    ram[4032] = 8'b00000000;
    ram[4031] = 8'b00000000;
    ram[4030] = 8'b00000000;
    ram[4029] = 8'b00000000;
    ram[4028] = 8'b00000000;
    ram[4027] = 8'b00000000;
    ram[4026] = 8'b00000000;
    ram[4025] = 8'b00000000;
    ram[4024] = 8'b00000000;
    ram[4023] = 8'b00000000;
    ram[4022] = 8'b00000000;
    ram[4021] = 8'b00000000;
    ram[4020] = 8'b00000000;
    ram[4019] = 8'b00000000;
    ram[4018] = 8'b00000000;
    ram[4017] = 8'b00000000;
    ram[4016] = 8'b00000000;
    ram[4015] = 8'b00000000;
    ram[4014] = 8'b00000000;
    ram[4013] = 8'b00000000;
    ram[4012] = 8'b00000000;
    ram[4011] = 8'b00000000;
    ram[4010] = 8'b00000000;
    ram[4009] = 8'b00000000;
    ram[4008] = 8'b00000000;
    ram[4007] = 8'b00000000;
    ram[4006] = 8'b00000000;
    ram[4005] = 8'b00000000;
    ram[4004] = 8'b00000000;
    ram[4003] = 8'b00000000;
    ram[4002] = 8'b00000000;
    ram[4001] = 8'b00000000;
    ram[4000] = 8'b00000000;
    ram[3999] = 8'b00000000;
    ram[3998] = 8'b00000000;
    ram[3997] = 8'b00000000;
    ram[3996] = 8'b00000000;
    ram[3995] = 8'b00000000;
    ram[3994] = 8'b00000000;
    ram[3993] = 8'b00000000;
    ram[3992] = 8'b00000000;
    ram[3991] = 8'b00000000;
    ram[3990] = 8'b00000000;
    ram[3989] = 8'b00000000;
    ram[3988] = 8'b00000000;
    ram[3987] = 8'b00000000;
    ram[3986] = 8'b00000000;
    ram[3985] = 8'b00000000;
    ram[3984] = 8'b00000000;
    ram[3983] = 8'b00000000;
    ram[3982] = 8'b00000000;
    ram[3981] = 8'b00000000;
    ram[3980] = 8'b00000000;
    ram[3979] = 8'b00000000;
    ram[3978] = 8'b00000000;
    ram[3977] = 8'b00000000;
    ram[3976] = 8'b00000000;
    ram[3975] = 8'b00000000;
    ram[3974] = 8'b00000000;
    ram[3973] = 8'b00000000;
    ram[3972] = 8'b00000000;
    ram[3971] = 8'b00000000;
    ram[3970] = 8'b00000000;
    ram[3969] = 8'b00000000;
    ram[3968] = 8'b00000000;
    ram[3967] = 8'b00000000;
    ram[3966] = 8'b00000000;
    ram[3965] = 8'b00000000;
    ram[3964] = 8'b00000000;
    ram[3963] = 8'b00000000;
    ram[3962] = 8'b00000000;
    ram[3961] = 8'b00000000;
    ram[3960] = 8'b00000000;
    ram[3959] = 8'b00000000;
    ram[3958] = 8'b00000000;
    ram[3957] = 8'b00000000;
    ram[3956] = 8'b00000000;
    ram[3955] = 8'b00000000;
    ram[3954] = 8'b00000000;
    ram[3953] = 8'b00000000;
    ram[3952] = 8'b00000000;
    ram[3951] = 8'b00000000;
    ram[3950] = 8'b00000000;
    ram[3949] = 8'b00000000;
    ram[3948] = 8'b00000000;
    ram[3947] = 8'b00000000;
    ram[3946] = 8'b00000000;
    ram[3945] = 8'b00000000;
    ram[3944] = 8'b00000000;
    ram[3943] = 8'b00000000;
    ram[3942] = 8'b00000000;
    ram[3941] = 8'b00000000;
    ram[3940] = 8'b00000000;
    ram[3939] = 8'b00000000;
    ram[3938] = 8'b00000000;
    ram[3937] = 8'b00000000;
    ram[3936] = 8'b00000000;
    ram[3935] = 8'b00000000;
    ram[3934] = 8'b00000000;
    ram[3933] = 8'b00000000;
    ram[3932] = 8'b00000000;
    ram[3931] = 8'b00000000;
    ram[3930] = 8'b00000000;
    ram[3929] = 8'b00000000;
    ram[3928] = 8'b00000000;
    ram[3927] = 8'b00000000;
    ram[3926] = 8'b00000000;
    ram[3925] = 8'b00000000;
    ram[3924] = 8'b00000000;
    ram[3923] = 8'b00000000;
    ram[3922] = 8'b00000000;
    ram[3921] = 8'b00000000;
    ram[3920] = 8'b00000000;
    ram[3919] = 8'b00000000;
    ram[3918] = 8'b00000000;
    ram[3917] = 8'b00000000;
    ram[3916] = 8'b00000000;
    ram[3915] = 8'b00000000;
    ram[3914] = 8'b00000000;
    ram[3913] = 8'b00000000;
    ram[3912] = 8'b00000000;
    ram[3911] = 8'b00000000;
    ram[3910] = 8'b00000000;
    ram[3909] = 8'b00000000;
    ram[3908] = 8'b00000000;
    ram[3907] = 8'b00000000;
    ram[3906] = 8'b00000000;
    ram[3905] = 8'b00000000;
    ram[3904] = 8'b00000000;
    ram[3903] = 8'b00000000;
    ram[3902] = 8'b00000000;
    ram[3901] = 8'b00000000;
    ram[3900] = 8'b00000000;
    ram[3899] = 8'b00000000;
    ram[3898] = 8'b00000000;
    ram[3897] = 8'b00000000;
    ram[3896] = 8'b00000000;
    ram[3895] = 8'b00000000;
    ram[3894] = 8'b00000000;
    ram[3893] = 8'b00000000;
    ram[3892] = 8'b00000000;
    ram[3891] = 8'b00000000;
    ram[3890] = 8'b00000000;
    ram[3889] = 8'b00000000;
    ram[3888] = 8'b00000000;
    ram[3887] = 8'b00000000;
    ram[3886] = 8'b00000000;
    ram[3885] = 8'b00000000;
    ram[3884] = 8'b00000000;
    ram[3883] = 8'b00000000;
    ram[3882] = 8'b00000000;
    ram[3881] = 8'b00000000;
    ram[3880] = 8'b00000000;
    ram[3879] = 8'b00000000;
    ram[3878] = 8'b00000000;
    ram[3877] = 8'b00000000;
    ram[3876] = 8'b00000000;
    ram[3875] = 8'b00000000;
    ram[3874] = 8'b00000000;
    ram[3873] = 8'b00000000;
    ram[3872] = 8'b00000000;
    ram[3871] = 8'b00000000;
    ram[3870] = 8'b00000000;
    ram[3869] = 8'b00000000;
    ram[3868] = 8'b00000000;
    ram[3867] = 8'b00000000;
    ram[3866] = 8'b00000000;
    ram[3865] = 8'b00000000;
    ram[3864] = 8'b00000000;
    ram[3863] = 8'b00000000;
    ram[3862] = 8'b00000000;
    ram[3861] = 8'b00000000;
    ram[3860] = 8'b00000000;
    ram[3859] = 8'b00000000;
    ram[3858] = 8'b00000000;
    ram[3857] = 8'b00000000;
    ram[3856] = 8'b00000000;
    ram[3855] = 8'b00000000;
    ram[3854] = 8'b00000000;
    ram[3853] = 8'b00000000;
    ram[3852] = 8'b00000000;
    ram[3851] = 8'b00000000;
    ram[3850] = 8'b00000000;
    ram[3849] = 8'b00000000;
    ram[3848] = 8'b00000000;
    ram[3847] = 8'b00000000;
    ram[3846] = 8'b00000000;
    ram[3845] = 8'b00000000;
    ram[3844] = 8'b00000000;
    ram[3843] = 8'b00000000;
    ram[3842] = 8'b00000000;
    ram[3841] = 8'b00000000;
    ram[3840] = 8'b00000000;
    ram[3839] = 8'b00000000;
    ram[3838] = 8'b00000000;
    ram[3837] = 8'b00000000;
    ram[3836] = 8'b00000000;
    ram[3835] = 8'b00000000;
    ram[3834] = 8'b00000000;
    ram[3833] = 8'b00000000;
    ram[3832] = 8'b00000000;
    ram[3831] = 8'b00000000;
    ram[3830] = 8'b00000000;
    ram[3829] = 8'b00000000;
    ram[3828] = 8'b00000000;
    ram[3827] = 8'b00000000;
    ram[3826] = 8'b00000000;
    ram[3825] = 8'b00000000;
    ram[3824] = 8'b00000000;
    ram[3823] = 8'b00000000;
    ram[3822] = 8'b00000000;
    ram[3821] = 8'b00000000;
    ram[3820] = 8'b00000000;
    ram[3819] = 8'b00000000;
    ram[3818] = 8'b00000000;
    ram[3817] = 8'b00000000;
    ram[3816] = 8'b00000000;
    ram[3815] = 8'b00000000;
    ram[3814] = 8'b00000000;
    ram[3813] = 8'b00000000;
    ram[3812] = 8'b00000000;
    ram[3811] = 8'b00000000;
    ram[3810] = 8'b00000000;
    ram[3809] = 8'b00000000;
    ram[3808] = 8'b00000000;
    ram[3807] = 8'b00000000;
    ram[3806] = 8'b00000000;
    ram[3805] = 8'b00000000;
    ram[3804] = 8'b00000000;
    ram[3803] = 8'b00000000;
    ram[3802] = 8'b00000000;
    ram[3801] = 8'b00000000;
    ram[3800] = 8'b00000000;
    ram[3799] = 8'b00000000;
    ram[3798] = 8'b00000000;
    ram[3797] = 8'b00000000;
    ram[3796] = 8'b00000000;
    ram[3795] = 8'b00000000;
    ram[3794] = 8'b00000000;
    ram[3793] = 8'b00000000;
    ram[3792] = 8'b00000000;
    ram[3791] = 8'b00000000;
    ram[3790] = 8'b00000000;
    ram[3789] = 8'b00000000;
    ram[3788] = 8'b00000000;
    ram[3787] = 8'b00000000;
    ram[3786] = 8'b00000000;
    ram[3785] = 8'b00000000;
    ram[3784] = 8'b00000000;
    ram[3783] = 8'b00000000;
    ram[3782] = 8'b00000000;
    ram[3781] = 8'b00000000;
    ram[3780] = 8'b00000000;
    ram[3779] = 8'b00000000;
    ram[3778] = 8'b00000000;
    ram[3777] = 8'b00000000;
    ram[3776] = 8'b00000000;
    ram[3775] = 8'b00000000;
    ram[3774] = 8'b00000000;
    ram[3773] = 8'b00000000;
    ram[3772] = 8'b00000000;
    ram[3771] = 8'b00000000;
    ram[3770] = 8'b00000000;
    ram[3769] = 8'b00000000;
    ram[3768] = 8'b00000000;
    ram[3767] = 8'b00000000;
    ram[3766] = 8'b00000000;
    ram[3765] = 8'b00000000;
    ram[3764] = 8'b00000000;
    ram[3763] = 8'b00000000;
    ram[3762] = 8'b00000000;
    ram[3761] = 8'b00000000;
    ram[3760] = 8'b00000000;
    ram[3759] = 8'b00000000;
    ram[3758] = 8'b00000000;
    ram[3757] = 8'b00000000;
    ram[3756] = 8'b00000000;
    ram[3755] = 8'b00000000;
    ram[3754] = 8'b00000000;
    ram[3753] = 8'b00000000;
    ram[3752] = 8'b00000000;
    ram[3751] = 8'b00000000;
    ram[3750] = 8'b00000000;
    ram[3749] = 8'b00000000;
    ram[3748] = 8'b00000000;
    ram[3747] = 8'b00000000;
    ram[3746] = 8'b00000000;
    ram[3745] = 8'b00000000;
    ram[3744] = 8'b00000000;
    ram[3743] = 8'b00000000;
    ram[3742] = 8'b00000000;
    ram[3741] = 8'b00000000;
    ram[3740] = 8'b00000000;
    ram[3739] = 8'b00000000;
    ram[3738] = 8'b00000000;
    ram[3737] = 8'b00000000;
    ram[3736] = 8'b00000000;
    ram[3735] = 8'b00000000;
    ram[3734] = 8'b00000000;
    ram[3733] = 8'b00000000;
    ram[3732] = 8'b00000000;
    ram[3731] = 8'b00000000;
    ram[3730] = 8'b00000000;
    ram[3729] = 8'b00000000;
    ram[3728] = 8'b00000000;
    ram[3727] = 8'b00000000;
    ram[3726] = 8'b00000000;
    ram[3725] = 8'b00000000;
    ram[3724] = 8'b00000000;
    ram[3723] = 8'b00000000;
    ram[3722] = 8'b00000000;
    ram[3721] = 8'b00000000;
    ram[3720] = 8'b00000000;
    ram[3719] = 8'b00000000;
    ram[3718] = 8'b00000000;
    ram[3717] = 8'b00000000;
    ram[3716] = 8'b00000000;
    ram[3715] = 8'b00000000;
    ram[3714] = 8'b00000000;
    ram[3713] = 8'b00000000;
    ram[3712] = 8'b00000000;
    ram[3711] = 8'b00000000;
    ram[3710] = 8'b00000000;
    ram[3709] = 8'b00000000;
    ram[3708] = 8'b00000000;
    ram[3707] = 8'b00000000;
    ram[3706] = 8'b00000000;
    ram[3705] = 8'b00000000;
    ram[3704] = 8'b00000000;
    ram[3703] = 8'b00000000;
    ram[3702] = 8'b00000000;
    ram[3701] = 8'b00000000;
    ram[3700] = 8'b00000000;
    ram[3699] = 8'b00000000;
    ram[3698] = 8'b00000000;
    ram[3697] = 8'b00000000;
    ram[3696] = 8'b00000000;
    ram[3695] = 8'b00000000;
    ram[3694] = 8'b00000000;
    ram[3693] = 8'b00000000;
    ram[3692] = 8'b00000000;
    ram[3691] = 8'b00000000;
    ram[3690] = 8'b00000000;
    ram[3689] = 8'b00000000;
    ram[3688] = 8'b00000000;
    ram[3687] = 8'b00000000;
    ram[3686] = 8'b00000000;
    ram[3685] = 8'b00000000;
    ram[3684] = 8'b00000000;
    ram[3683] = 8'b00000000;
    ram[3682] = 8'b00000000;
    ram[3681] = 8'b00000000;
    ram[3680] = 8'b00000000;
    ram[3679] = 8'b00000000;
    ram[3678] = 8'b00000000;
    ram[3677] = 8'b00000000;
    ram[3676] = 8'b00000000;
    ram[3675] = 8'b00000000;
    ram[3674] = 8'b00000000;
    ram[3673] = 8'b00000000;
    ram[3672] = 8'b00000000;
    ram[3671] = 8'b00000000;
    ram[3670] = 8'b00000000;
    ram[3669] = 8'b00000000;
    ram[3668] = 8'b00000000;
    ram[3667] = 8'b00000000;
    ram[3666] = 8'b00000000;
    ram[3665] = 8'b00000000;
    ram[3664] = 8'b00000000;
    ram[3663] = 8'b00000000;
    ram[3662] = 8'b00000000;
    ram[3661] = 8'b00000000;
    ram[3660] = 8'b00000000;
    ram[3659] = 8'b00000000;
    ram[3658] = 8'b00000000;
    ram[3657] = 8'b00000000;
    ram[3656] = 8'b00000000;
    ram[3655] = 8'b00000000;
    ram[3654] = 8'b00000000;
    ram[3653] = 8'b00000000;
    ram[3652] = 8'b00000000;
    ram[3651] = 8'b00000000;
    ram[3650] = 8'b00000000;
    ram[3649] = 8'b00000000;
    ram[3648] = 8'b00000000;
    ram[3647] = 8'b00000000;
    ram[3646] = 8'b00000000;
    ram[3645] = 8'b00000000;
    ram[3644] = 8'b00000000;
    ram[3643] = 8'b00000000;
    ram[3642] = 8'b00000000;
    ram[3641] = 8'b00000000;
    ram[3640] = 8'b00000000;
    ram[3639] = 8'b00000000;
    ram[3638] = 8'b00000000;
    ram[3637] = 8'b00000000;
    ram[3636] = 8'b00000000;
    ram[3635] = 8'b00000000;
    ram[3634] = 8'b00000000;
    ram[3633] = 8'b00000000;
    ram[3632] = 8'b00000000;
    ram[3631] = 8'b00000000;
    ram[3630] = 8'b00000000;
    ram[3629] = 8'b00000000;
    ram[3628] = 8'b00000000;
    ram[3627] = 8'b00000000;
    ram[3626] = 8'b00000000;
    ram[3625] = 8'b00000000;
    ram[3624] = 8'b00000000;
    ram[3623] = 8'b00000000;
    ram[3622] = 8'b00000000;
    ram[3621] = 8'b00000000;
    ram[3620] = 8'b00000000;
    ram[3619] = 8'b00000000;
    ram[3618] = 8'b00000000;
    ram[3617] = 8'b00000000;
    ram[3616] = 8'b00000000;
    ram[3615] = 8'b00000000;
    ram[3614] = 8'b00000000;
    ram[3613] = 8'b00000000;
    ram[3612] = 8'b00000000;
    ram[3611] = 8'b00000000;
    ram[3610] = 8'b00000000;
    ram[3609] = 8'b00000000;
    ram[3608] = 8'b00000000;
    ram[3607] = 8'b00000000;
    ram[3606] = 8'b00000000;
    ram[3605] = 8'b00000000;
    ram[3604] = 8'b00000000;
    ram[3603] = 8'b00000000;
    ram[3602] = 8'b00000000;
    ram[3601] = 8'b00000000;
    ram[3600] = 8'b00000000;
    ram[3599] = 8'b00000000;
    ram[3598] = 8'b00000000;
    ram[3597] = 8'b00000000;
    ram[3596] = 8'b00000000;
    ram[3595] = 8'b00000000;
    ram[3594] = 8'b00000000;
    ram[3593] = 8'b00000000;
    ram[3592] = 8'b00000000;
    ram[3591] = 8'b00000000;
    ram[3590] = 8'b00000000;
    ram[3589] = 8'b00000000;
    ram[3588] = 8'b00000000;
    ram[3587] = 8'b00000000;
    ram[3586] = 8'b00000000;
    ram[3585] = 8'b00000000;
    ram[3584] = 8'b00000000;
    ram[3583] = 8'b00000000;
    ram[3582] = 8'b00000000;
    ram[3581] = 8'b00000000;
    ram[3580] = 8'b00000000;
    ram[3579] = 8'b00000000;
    ram[3578] = 8'b00000000;
    ram[3577] = 8'b00000000;
    ram[3576] = 8'b00000000;
    ram[3575] = 8'b00000000;
    ram[3574] = 8'b00000000;
    ram[3573] = 8'b00000000;
    ram[3572] = 8'b00000000;
    ram[3571] = 8'b00000000;
    ram[3570] = 8'b00000000;
    ram[3569] = 8'b00000000;
    ram[3568] = 8'b00000000;
    ram[3567] = 8'b00000000;
    ram[3566] = 8'b00000000;
    ram[3565] = 8'b00000000;
    ram[3564] = 8'b00000000;
    ram[3563] = 8'b00000000;
    ram[3562] = 8'b00000000;
    ram[3561] = 8'b00000000;
    ram[3560] = 8'b00000000;
    ram[3559] = 8'b00000000;
    ram[3558] = 8'b00000000;
    ram[3557] = 8'b00000000;
    ram[3556] = 8'b00000000;
    ram[3555] = 8'b00000000;
    ram[3554] = 8'b00000000;
    ram[3553] = 8'b00000000;
    ram[3552] = 8'b00000000;
    ram[3551] = 8'b00000000;
    ram[3550] = 8'b00000000;
    ram[3549] = 8'b00000000;
    ram[3548] = 8'b00000000;
    ram[3547] = 8'b00000000;
    ram[3546] = 8'b00000000;
    ram[3545] = 8'b00000000;
    ram[3544] = 8'b00000000;
    ram[3543] = 8'b00000000;
    ram[3542] = 8'b00000000;
    ram[3541] = 8'b00000000;
    ram[3540] = 8'b00000000;
    ram[3539] = 8'b00000000;
    ram[3538] = 8'b00000000;
    ram[3537] = 8'b00000000;
    ram[3536] = 8'b00000000;
    ram[3535] = 8'b00000000;
    ram[3534] = 8'b00000000;
    ram[3533] = 8'b00000000;
    ram[3532] = 8'b00000000;
    ram[3531] = 8'b00000000;
    ram[3530] = 8'b00000000;
    ram[3529] = 8'b00000000;
    ram[3528] = 8'b00000000;
    ram[3527] = 8'b00000000;
    ram[3526] = 8'b00000000;
    ram[3525] = 8'b00000000;
    ram[3524] = 8'b00000000;
    ram[3523] = 8'b00000000;
    ram[3522] = 8'b00000000;
    ram[3521] = 8'b00000000;
    ram[3520] = 8'b00000000;
    ram[3519] = 8'b00000000;
    ram[3518] = 8'b00000000;
    ram[3517] = 8'b00000000;
    ram[3516] = 8'b00000000;
    ram[3515] = 8'b00000000;
    ram[3514] = 8'b00000000;
    ram[3513] = 8'b00000000;
    ram[3512] = 8'b00000000;
    ram[3511] = 8'b00000000;
    ram[3510] = 8'b00000000;
    ram[3509] = 8'b00000000;
    ram[3508] = 8'b00000000;
    ram[3507] = 8'b00000000;
    ram[3506] = 8'b00000000;
    ram[3505] = 8'b00000000;
    ram[3504] = 8'b00000000;
    ram[3503] = 8'b00000000;
    ram[3502] = 8'b00000000;
    ram[3501] = 8'b00000000;
    ram[3500] = 8'b00000000;
    ram[3499] = 8'b00000000;
    ram[3498] = 8'b00000000;
    ram[3497] = 8'b00000000;
    ram[3496] = 8'b00000000;
    ram[3495] = 8'b00000000;
    ram[3494] = 8'b00000000;
    ram[3493] = 8'b00000000;
    ram[3492] = 8'b00000000;
    ram[3491] = 8'b00000000;
    ram[3490] = 8'b00000000;
    ram[3489] = 8'b00000000;
    ram[3488] = 8'b00000000;
    ram[3487] = 8'b00000000;
    ram[3486] = 8'b00000000;
    ram[3485] = 8'b00000000;
    ram[3484] = 8'b00000000;
    ram[3483] = 8'b00000000;
    ram[3482] = 8'b00000000;
    ram[3481] = 8'b00000000;
    ram[3480] = 8'b00000000;
    ram[3479] = 8'b00000000;
    ram[3478] = 8'b00000000;
    ram[3477] = 8'b00000000;
    ram[3476] = 8'b00000000;
    ram[3475] = 8'b00000000;
    ram[3474] = 8'b00000000;
    ram[3473] = 8'b00000000;
    ram[3472] = 8'b00000000;
    ram[3471] = 8'b00000000;
    ram[3470] = 8'b00000000;
    ram[3469] = 8'b00000000;
    ram[3468] = 8'b00000000;
    ram[3467] = 8'b00000000;
    ram[3466] = 8'b00000000;
    ram[3465] = 8'b00000000;
    ram[3464] = 8'b00000000;
    ram[3463] = 8'b00000000;
    ram[3462] = 8'b00000000;
    ram[3461] = 8'b00000000;
    ram[3460] = 8'b00000000;
    ram[3459] = 8'b00000000;
    ram[3458] = 8'b00000000;
    ram[3457] = 8'b00000000;
    ram[3456] = 8'b00000000;
    ram[3455] = 8'b00000000;
    ram[3454] = 8'b00000000;
    ram[3453] = 8'b00000000;
    ram[3452] = 8'b00000000;
    ram[3451] = 8'b00000000;
    ram[3450] = 8'b00000000;
    ram[3449] = 8'b00000000;
    ram[3448] = 8'b00000000;
    ram[3447] = 8'b00000000;
    ram[3446] = 8'b00000000;
    ram[3445] = 8'b00000000;
    ram[3444] = 8'b00000000;
    ram[3443] = 8'b00000000;
    ram[3442] = 8'b00000000;
    ram[3441] = 8'b00000000;
    ram[3440] = 8'b00000000;
    ram[3439] = 8'b00000000;
    ram[3438] = 8'b00000000;
    ram[3437] = 8'b00000000;
    ram[3436] = 8'b00000000;
    ram[3435] = 8'b00000000;
    ram[3434] = 8'b00000000;
    ram[3433] = 8'b00000000;
    ram[3432] = 8'b00000000;
    ram[3431] = 8'b00000000;
    ram[3430] = 8'b00000000;
    ram[3429] = 8'b00000000;
    ram[3428] = 8'b00000000;
    ram[3427] = 8'b00000000;
    ram[3426] = 8'b00000000;
    ram[3425] = 8'b00000000;
    ram[3424] = 8'b00000000;
    ram[3423] = 8'b00000000;
    ram[3422] = 8'b00000000;
    ram[3421] = 8'b00000000;
    ram[3420] = 8'b00000000;
    ram[3419] = 8'b00000000;
    ram[3418] = 8'b00000000;
    ram[3417] = 8'b00000000;
    ram[3416] = 8'b00000000;
    ram[3415] = 8'b00000000;
    ram[3414] = 8'b00000000;
    ram[3413] = 8'b00000000;
    ram[3412] = 8'b00000000;
    ram[3411] = 8'b00000000;
    ram[3410] = 8'b00000000;
    ram[3409] = 8'b00000000;
    ram[3408] = 8'b00000000;
    ram[3407] = 8'b00000000;
    ram[3406] = 8'b00000000;
    ram[3405] = 8'b00000000;
    ram[3404] = 8'b00000000;
    ram[3403] = 8'b00000000;
    ram[3402] = 8'b00000000;
    ram[3401] = 8'b00000000;
    ram[3400] = 8'b00000000;
    ram[3399] = 8'b00000000;
    ram[3398] = 8'b00000000;
    ram[3397] = 8'b00000000;
    ram[3396] = 8'b00000000;
    ram[3395] = 8'b00000000;
    ram[3394] = 8'b00000000;
    ram[3393] = 8'b00000000;
    ram[3392] = 8'b00000000;
    ram[3391] = 8'b00000000;
    ram[3390] = 8'b00000000;
    ram[3389] = 8'b00000000;
    ram[3388] = 8'b00000000;
    ram[3387] = 8'b00000000;
    ram[3386] = 8'b00000000;
    ram[3385] = 8'b00000000;
    ram[3384] = 8'b00000000;
    ram[3383] = 8'b00000000;
    ram[3382] = 8'b00000000;
    ram[3381] = 8'b00000000;
    ram[3380] = 8'b00000000;
    ram[3379] = 8'b00000000;
    ram[3378] = 8'b00000000;
    ram[3377] = 8'b00000000;
    ram[3376] = 8'b00000000;
    ram[3375] = 8'b00000000;
    ram[3374] = 8'b00000000;
    ram[3373] = 8'b00000000;
    ram[3372] = 8'b00000000;
    ram[3371] = 8'b00000000;
    ram[3370] = 8'b00000000;
    ram[3369] = 8'b00000000;
    ram[3368] = 8'b00000000;
    ram[3367] = 8'b00000000;
    ram[3366] = 8'b00000000;
    ram[3365] = 8'b00000000;
    ram[3364] = 8'b00000000;
    ram[3363] = 8'b00000000;
    ram[3362] = 8'b00000000;
    ram[3361] = 8'b00000000;
    ram[3360] = 8'b00000000;
    ram[3359] = 8'b00000000;
    ram[3358] = 8'b00000000;
    ram[3357] = 8'b00000000;
    ram[3356] = 8'b00000000;
    ram[3355] = 8'b00000000;
    ram[3354] = 8'b00000000;
    ram[3353] = 8'b00000000;
    ram[3352] = 8'b00000000;
    ram[3351] = 8'b00000000;
    ram[3350] = 8'b00000000;
    ram[3349] = 8'b00000000;
    ram[3348] = 8'b00000000;
    ram[3347] = 8'b00000000;
    ram[3346] = 8'b00000000;
    ram[3345] = 8'b00000000;
    ram[3344] = 8'b00000000;
    ram[3343] = 8'b00000000;
    ram[3342] = 8'b00000000;
    ram[3341] = 8'b00000000;
    ram[3340] = 8'b00000000;
    ram[3339] = 8'b00000000;
    ram[3338] = 8'b00000000;
    ram[3337] = 8'b00000000;
    ram[3336] = 8'b00000000;
    ram[3335] = 8'b00000000;
    ram[3334] = 8'b00000000;
    ram[3333] = 8'b00000000;
    ram[3332] = 8'b00000000;
    ram[3331] = 8'b00000000;
    ram[3330] = 8'b00000000;
    ram[3329] = 8'b00000000;
    ram[3328] = 8'b00000000;
    ram[3327] = 8'b00000000;
    ram[3326] = 8'b00000000;
    ram[3325] = 8'b00000000;
    ram[3324] = 8'b00000000;
    ram[3323] = 8'b00000000;
    ram[3322] = 8'b00000000;
    ram[3321] = 8'b00000000;
    ram[3320] = 8'b00000000;
    ram[3319] = 8'b00000000;
    ram[3318] = 8'b00000000;
    ram[3317] = 8'b00000000;
    ram[3316] = 8'b00000000;
    ram[3315] = 8'b00000000;
    ram[3314] = 8'b00000000;
    ram[3313] = 8'b00000000;
    ram[3312] = 8'b00000000;
    ram[3311] = 8'b00000000;
    ram[3310] = 8'b00000000;
    ram[3309] = 8'b00000000;
    ram[3308] = 8'b00000000;
    ram[3307] = 8'b00000000;
    ram[3306] = 8'b00000000;
    ram[3305] = 8'b00000000;
    ram[3304] = 8'b00000000;
    ram[3303] = 8'b00000000;
    ram[3302] = 8'b00000000;
    ram[3301] = 8'b00000000;
    ram[3300] = 8'b00000000;
    ram[3299] = 8'b00000000;
    ram[3298] = 8'b00000000;
    ram[3297] = 8'b00000000;
    ram[3296] = 8'b00000000;
    ram[3295] = 8'b00000000;
    ram[3294] = 8'b00000000;
    ram[3293] = 8'b00000000;
    ram[3292] = 8'b00000000;
    ram[3291] = 8'b00000000;
    ram[3290] = 8'b00000000;
    ram[3289] = 8'b00000000;
    ram[3288] = 8'b00000000;
    ram[3287] = 8'b00000000;
    ram[3286] = 8'b00000000;
    ram[3285] = 8'b00000000;
    ram[3284] = 8'b00000000;
    ram[3283] = 8'b00000000;
    ram[3282] = 8'b00000000;
    ram[3281] = 8'b00000000;
    ram[3280] = 8'b00000000;
    ram[3279] = 8'b00000000;
    ram[3278] = 8'b00000000;
    ram[3277] = 8'b00000000;
    ram[3276] = 8'b00000000;
    ram[3275] = 8'b00000000;
    ram[3274] = 8'b00000000;
    ram[3273] = 8'b00000000;
    ram[3272] = 8'b00000000;
    ram[3271] = 8'b00000000;
    ram[3270] = 8'b00000000;
    ram[3269] = 8'b00000000;
    ram[3268] = 8'b00000000;
    ram[3267] = 8'b00000000;
    ram[3266] = 8'b00000000;
    ram[3265] = 8'b00000000;
    ram[3264] = 8'b00000000;
    ram[3263] = 8'b00000000;
    ram[3262] = 8'b00000000;
    ram[3261] = 8'b00000000;
    ram[3260] = 8'b00000000;
    ram[3259] = 8'b00000000;
    ram[3258] = 8'b00000000;
    ram[3257] = 8'b00000000;
    ram[3256] = 8'b00000000;
    ram[3255] = 8'b00000000;
    ram[3254] = 8'b00000000;
    ram[3253] = 8'b00000000;
    ram[3252] = 8'b00000000;
    ram[3251] = 8'b00000000;
    ram[3250] = 8'b00000000;
    ram[3249] = 8'b00000000;
    ram[3248] = 8'b00000000;
    ram[3247] = 8'b00000000;
    ram[3246] = 8'b00000000;
    ram[3245] = 8'b00000000;
    ram[3244] = 8'b00000000;
    ram[3243] = 8'b00000000;
    ram[3242] = 8'b00000000;
    ram[3241] = 8'b00000000;
    ram[3240] = 8'b00000000;
    ram[3239] = 8'b00000000;
    ram[3238] = 8'b00000000;
    ram[3237] = 8'b00000000;
    ram[3236] = 8'b00000000;
    ram[3235] = 8'b00000000;
    ram[3234] = 8'b00000000;
    ram[3233] = 8'b00000000;
    ram[3232] = 8'b00000000;
    ram[3231] = 8'b00000000;
    ram[3230] = 8'b00000000;
    ram[3229] = 8'b00000000;
    ram[3228] = 8'b00000000;
    ram[3227] = 8'b00000000;
    ram[3226] = 8'b00000000;
    ram[3225] = 8'b00000000;
    ram[3224] = 8'b00000000;
    ram[3223] = 8'b00000000;
    ram[3222] = 8'b00000000;
    ram[3221] = 8'b00000000;
    ram[3220] = 8'b00000000;
    ram[3219] = 8'b00000000;
    ram[3218] = 8'b00000000;
    ram[3217] = 8'b00000000;
    ram[3216] = 8'b00000000;
    ram[3215] = 8'b00000000;
    ram[3214] = 8'b00000000;
    ram[3213] = 8'b00000000;
    ram[3212] = 8'b00000000;
    ram[3211] = 8'b00000000;
    ram[3210] = 8'b00000000;
    ram[3209] = 8'b00000000;
    ram[3208] = 8'b00000000;
    ram[3207] = 8'b00000000;
    ram[3206] = 8'b00000000;
    ram[3205] = 8'b00000000;
    ram[3204] = 8'b00000000;
    ram[3203] = 8'b00000000;
    ram[3202] = 8'b00000000;
    ram[3201] = 8'b00000000;
    ram[3200] = 8'b00000000;
    ram[3199] = 8'b00000000;
    ram[3198] = 8'b00000000;
    ram[3197] = 8'b00000000;
    ram[3196] = 8'b00000000;
    ram[3195] = 8'b00000000;
    ram[3194] = 8'b00000000;
    ram[3193] = 8'b00000000;
    ram[3192] = 8'b00000000;
    ram[3191] = 8'b00000000;
    ram[3190] = 8'b00000000;
    ram[3189] = 8'b00000000;
    ram[3188] = 8'b00000000;
    ram[3187] = 8'b00000000;
    ram[3186] = 8'b00000000;
    ram[3185] = 8'b00000000;
    ram[3184] = 8'b00000000;
    ram[3183] = 8'b00000000;
    ram[3182] = 8'b00000000;
    ram[3181] = 8'b00000000;
    ram[3180] = 8'b00000000;
    ram[3179] = 8'b00000000;
    ram[3178] = 8'b00000000;
    ram[3177] = 8'b00000000;
    ram[3176] = 8'b00000000;
    ram[3175] = 8'b00000000;
    ram[3174] = 8'b00000000;
    ram[3173] = 8'b00000000;
    ram[3172] = 8'b00000000;
    ram[3171] = 8'b00000000;
    ram[3170] = 8'b00000000;
    ram[3169] = 8'b00000000;
    ram[3168] = 8'b00000000;
    ram[3167] = 8'b00000000;
    ram[3166] = 8'b00000000;
    ram[3165] = 8'b00000000;
    ram[3164] = 8'b00000000;
    ram[3163] = 8'b00000000;
    ram[3162] = 8'b00000000;
    ram[3161] = 8'b00000000;
    ram[3160] = 8'b00000000;
    ram[3159] = 8'b00000000;
    ram[3158] = 8'b00000000;
    ram[3157] = 8'b00000000;
    ram[3156] = 8'b00000000;
    ram[3155] = 8'b00000000;
    ram[3154] = 8'b00000000;
    ram[3153] = 8'b00000000;
    ram[3152] = 8'b00000000;
    ram[3151] = 8'b00000000;
    ram[3150] = 8'b00000000;
    ram[3149] = 8'b00000000;
    ram[3148] = 8'b00000000;
    ram[3147] = 8'b00000000;
    ram[3146] = 8'b00000000;
    ram[3145] = 8'b00000000;
    ram[3144] = 8'b00000000;
    ram[3143] = 8'b00000000;
    ram[3142] = 8'b00000000;
    ram[3141] = 8'b00000000;
    ram[3140] = 8'b00000000;
    ram[3139] = 8'b00000000;
    ram[3138] = 8'b00000000;
    ram[3137] = 8'b00000000;
    ram[3136] = 8'b00000000;
    ram[3135] = 8'b00000000;
    ram[3134] = 8'b00000000;
    ram[3133] = 8'b00000000;
    ram[3132] = 8'b00000000;
    ram[3131] = 8'b00000000;
    ram[3130] = 8'b00000000;
    ram[3129] = 8'b00000000;
    ram[3128] = 8'b00000000;
    ram[3127] = 8'b00000000;
    ram[3126] = 8'b00000000;
    ram[3125] = 8'b00000000;
    ram[3124] = 8'b00000000;
    ram[3123] = 8'b00000000;
    ram[3122] = 8'b00000000;
    ram[3121] = 8'b00000000;
    ram[3120] = 8'b00000000;
    ram[3119] = 8'b00000000;
    ram[3118] = 8'b00000000;
    ram[3117] = 8'b00000000;
    ram[3116] = 8'b00000000;
    ram[3115] = 8'b00000000;
    ram[3114] = 8'b00000000;
    ram[3113] = 8'b00000000;
    ram[3112] = 8'b00000000;
    ram[3111] = 8'b00000000;
    ram[3110] = 8'b00000000;
    ram[3109] = 8'b00000000;
    ram[3108] = 8'b00000000;
    ram[3107] = 8'b00000000;
    ram[3106] = 8'b00000000;
    ram[3105] = 8'b00000000;
    ram[3104] = 8'b00000000;
    ram[3103] = 8'b00000000;
    ram[3102] = 8'b00000000;
    ram[3101] = 8'b00000000;
    ram[3100] = 8'b00000000;
    ram[3099] = 8'b00000000;
    ram[3098] = 8'b00000000;
    ram[3097] = 8'b00000000;
    ram[3096] = 8'b00000000;
    ram[3095] = 8'b00000000;
    ram[3094] = 8'b00000000;
    ram[3093] = 8'b00000000;
    ram[3092] = 8'b00000000;
    ram[3091] = 8'b00000000;
    ram[3090] = 8'b00000000;
    ram[3089] = 8'b00000000;
    ram[3088] = 8'b00000000;
    ram[3087] = 8'b00000000;
    ram[3086] = 8'b00000000;
    ram[3085] = 8'b00000000;
    ram[3084] = 8'b00000000;
    ram[3083] = 8'b00000000;
    ram[3082] = 8'b00000000;
    ram[3081] = 8'b00000000;
    ram[3080] = 8'b00000000;
    ram[3079] = 8'b00000000;
    ram[3078] = 8'b00000000;
    ram[3077] = 8'b00000000;
    ram[3076] = 8'b00000000;
    ram[3075] = 8'b00000000;
    ram[3074] = 8'b00000000;
    ram[3073] = 8'b00000000;
    ram[3072] = 8'b00000000;
    ram[3071] = 8'b00000000;
    ram[3070] = 8'b00000000;
    ram[3069] = 8'b00000000;
    ram[3068] = 8'b00000000;
    ram[3067] = 8'b00000000;
    ram[3066] = 8'b00000000;
    ram[3065] = 8'b00000000;
    ram[3064] = 8'b00000000;
    ram[3063] = 8'b00000000;
    ram[3062] = 8'b00000000;
    ram[3061] = 8'b00000000;
    ram[3060] = 8'b00000000;
    ram[3059] = 8'b00000000;
    ram[3058] = 8'b00000000;
    ram[3057] = 8'b00000000;
    ram[3056] = 8'b00000000;
    ram[3055] = 8'b00000000;
    ram[3054] = 8'b00000000;
    ram[3053] = 8'b00000000;
    ram[3052] = 8'b00000000;
    ram[3051] = 8'b00000000;
    ram[3050] = 8'b00000000;
    ram[3049] = 8'b00000000;
    ram[3048] = 8'b00000000;
    ram[3047] = 8'b00000000;
    ram[3046] = 8'b00000000;
    ram[3045] = 8'b00000000;
    ram[3044] = 8'b00000000;
    ram[3043] = 8'b00000000;
    ram[3042] = 8'b00000000;
    ram[3041] = 8'b00000000;
    ram[3040] = 8'b00000000;
    ram[3039] = 8'b00000000;
    ram[3038] = 8'b00000000;
    ram[3037] = 8'b00000000;
    ram[3036] = 8'b00000000;
    ram[3035] = 8'b00000000;
    ram[3034] = 8'b00000000;
    ram[3033] = 8'b00000000;
    ram[3032] = 8'b00000000;
    ram[3031] = 8'b00000000;
    ram[3030] = 8'b00000000;
    ram[3029] = 8'b00000000;
    ram[3028] = 8'b00000000;
    ram[3027] = 8'b00000000;
    ram[3026] = 8'b00000000;
    ram[3025] = 8'b00000000;
    ram[3024] = 8'b00000000;
    ram[3023] = 8'b00000000;
    ram[3022] = 8'b00000000;
    ram[3021] = 8'b00000000;
    ram[3020] = 8'b00000000;
    ram[3019] = 8'b00000000;
    ram[3018] = 8'b00000000;
    ram[3017] = 8'b00000000;
    ram[3016] = 8'b00000000;
    ram[3015] = 8'b00000000;
    ram[3014] = 8'b00000000;
    ram[3013] = 8'b00000000;
    ram[3012] = 8'b00000000;
    ram[3011] = 8'b00000000;
    ram[3010] = 8'b00000000;
    ram[3009] = 8'b00000000;
    ram[3008] = 8'b00000000;
    ram[3007] = 8'b00000000;
    ram[3006] = 8'b00000000;
    ram[3005] = 8'b00000000;
    ram[3004] = 8'b00000000;
    ram[3003] = 8'b00000000;
    ram[3002] = 8'b00000000;
    ram[3001] = 8'b00000000;
    ram[3000] = 8'b00000000;
    ram[2999] = 8'b00000000;
    ram[2998] = 8'b00000000;
    ram[2997] = 8'b00000000;
    ram[2996] = 8'b00000000;
    ram[2995] = 8'b00000000;
    ram[2994] = 8'b00000000;
    ram[2993] = 8'b00000000;
    ram[2992] = 8'b00000000;
    ram[2991] = 8'b00000000;
    ram[2990] = 8'b00000000;
    ram[2989] = 8'b00000000;
    ram[2988] = 8'b00000000;
    ram[2987] = 8'b00000000;
    ram[2986] = 8'b00000000;
    ram[2985] = 8'b00000000;
    ram[2984] = 8'b00000000;
    ram[2983] = 8'b00000000;
    ram[2982] = 8'b00000000;
    ram[2981] = 8'b00000000;
    ram[2980] = 8'b00000000;
    ram[2979] = 8'b00000000;
    ram[2978] = 8'b00000000;
    ram[2977] = 8'b00000000;
    ram[2976] = 8'b00000000;
    ram[2975] = 8'b00000000;
    ram[2974] = 8'b00000000;
    ram[2973] = 8'b00000000;
    ram[2972] = 8'b00000000;
    ram[2971] = 8'b00000000;
    ram[2970] = 8'b00000000;
    ram[2969] = 8'b00000000;
    ram[2968] = 8'b00000000;
    ram[2967] = 8'b00000000;
    ram[2966] = 8'b00000000;
    ram[2965] = 8'b00000000;
    ram[2964] = 8'b00000000;
    ram[2963] = 8'b00000000;
    ram[2962] = 8'b00000000;
    ram[2961] = 8'b00000000;
    ram[2960] = 8'b00000000;
    ram[2959] = 8'b00000000;
    ram[2958] = 8'b00000000;
    ram[2957] = 8'b00000000;
    ram[2956] = 8'b00000000;
    ram[2955] = 8'b00000000;
    ram[2954] = 8'b00000000;
    ram[2953] = 8'b00000000;
    ram[2952] = 8'b00000000;
    ram[2951] = 8'b00000000;
    ram[2950] = 8'b00000000;
    ram[2949] = 8'b00000000;
    ram[2948] = 8'b00000000;
    ram[2947] = 8'b00000000;
    ram[2946] = 8'b00000000;
    ram[2945] = 8'b00000000;
    ram[2944] = 8'b00000000;
    ram[2943] = 8'b00000000;
    ram[2942] = 8'b00000000;
    ram[2941] = 8'b00000000;
    ram[2940] = 8'b00000000;
    ram[2939] = 8'b00000000;
    ram[2938] = 8'b00000000;
    ram[2937] = 8'b00000000;
    ram[2936] = 8'b00000000;
    ram[2935] = 8'b00000000;
    ram[2934] = 8'b00000000;
    ram[2933] = 8'b00000000;
    ram[2932] = 8'b00000000;
    ram[2931] = 8'b00000000;
    ram[2930] = 8'b00000000;
    ram[2929] = 8'b00000000;
    ram[2928] = 8'b00000000;
    ram[2927] = 8'b00000000;
    ram[2926] = 8'b00000000;
    ram[2925] = 8'b00000000;
    ram[2924] = 8'b00000000;
    ram[2923] = 8'b00000000;
    ram[2922] = 8'b00000000;
    ram[2921] = 8'b00000000;
    ram[2920] = 8'b00000000;
    ram[2919] = 8'b00000000;
    ram[2918] = 8'b00000000;
    ram[2917] = 8'b00000000;
    ram[2916] = 8'b00000000;
    ram[2915] = 8'b00000000;
    ram[2914] = 8'b00000000;
    ram[2913] = 8'b00000000;
    ram[2912] = 8'b00000000;
    ram[2911] = 8'b00000000;
    ram[2910] = 8'b00000000;
    ram[2909] = 8'b00000000;
    ram[2908] = 8'b00000000;
    ram[2907] = 8'b00000000;
    ram[2906] = 8'b00000000;
    ram[2905] = 8'b00000000;
    ram[2904] = 8'b00000000;
    ram[2903] = 8'b00000000;
    ram[2902] = 8'b00000000;
    ram[2901] = 8'b00000000;
    ram[2900] = 8'b00000000;
    ram[2899] = 8'b00000000;
    ram[2898] = 8'b00000000;
    ram[2897] = 8'b00000000;
    ram[2896] = 8'b00000000;
    ram[2895] = 8'b00000000;
    ram[2894] = 8'b00000000;
    ram[2893] = 8'b00000000;
    ram[2892] = 8'b00000000;
    ram[2891] = 8'b00000000;
    ram[2890] = 8'b00000000;
    ram[2889] = 8'b00000000;
    ram[2888] = 8'b00000000;
    ram[2887] = 8'b00000000;
    ram[2886] = 8'b00000000;
    ram[2885] = 8'b00000000;
    ram[2884] = 8'b00000000;
    ram[2883] = 8'b00000000;
    ram[2882] = 8'b00000000;
    ram[2881] = 8'b00000000;
    ram[2880] = 8'b00000000;
    ram[2879] = 8'b00000000;
    ram[2878] = 8'b00000000;
    ram[2877] = 8'b00000000;
    ram[2876] = 8'b00000000;
    ram[2875] = 8'b00000000;
    ram[2874] = 8'b00000000;
    ram[2873] = 8'b00000000;
    ram[2872] = 8'b00000000;
    ram[2871] = 8'b00000000;
    ram[2870] = 8'b00000000;
    ram[2869] = 8'b00000000;
    ram[2868] = 8'b00000000;
    ram[2867] = 8'b00000000;
    ram[2866] = 8'b00000000;
    ram[2865] = 8'b00000000;
    ram[2864] = 8'b00000000;
    ram[2863] = 8'b00000000;
    ram[2862] = 8'b00000000;
    ram[2861] = 8'b00000000;
    ram[2860] = 8'b00000000;
    ram[2859] = 8'b00000000;
    ram[2858] = 8'b00000000;
    ram[2857] = 8'b00000000;
    ram[2856] = 8'b00000000;
    ram[2855] = 8'b00000000;
    ram[2854] = 8'b00000000;
    ram[2853] = 8'b00000000;
    ram[2852] = 8'b00000000;
    ram[2851] = 8'b00000000;
    ram[2850] = 8'b00000000;
    ram[2849] = 8'b00000000;
    ram[2848] = 8'b00000000;
    ram[2847] = 8'b00000000;
    ram[2846] = 8'b00000000;
    ram[2845] = 8'b00000000;
    ram[2844] = 8'b00000000;
    ram[2843] = 8'b00000000;
    ram[2842] = 8'b00000000;
    ram[2841] = 8'b00000000;
    ram[2840] = 8'b00000000;
    ram[2839] = 8'b00000000;
    ram[2838] = 8'b00000000;
    ram[2837] = 8'b00000000;
    ram[2836] = 8'b00000000;
    ram[2835] = 8'b00000000;
    ram[2834] = 8'b00000000;
    ram[2833] = 8'b00000000;
    ram[2832] = 8'b00000000;
    ram[2831] = 8'b00000000;
    ram[2830] = 8'b00000000;
    ram[2829] = 8'b00000000;
    ram[2828] = 8'b00000000;
    ram[2827] = 8'b00000000;
    ram[2826] = 8'b00000000;
    ram[2825] = 8'b00000000;
    ram[2824] = 8'b00000000;
    ram[2823] = 8'b00000000;
    ram[2822] = 8'b00000000;
    ram[2821] = 8'b00000000;
    ram[2820] = 8'b00000000;
    ram[2819] = 8'b00000000;
    ram[2818] = 8'b00000000;
    ram[2817] = 8'b00000000;
    ram[2816] = 8'b00000000;
    ram[2815] = 8'b00000000;
    ram[2814] = 8'b00000000;
    ram[2813] = 8'b00000000;
    ram[2812] = 8'b00000000;
    ram[2811] = 8'b00000000;
    ram[2810] = 8'b00000000;
    ram[2809] = 8'b00000000;
    ram[2808] = 8'b00000000;
    ram[2807] = 8'b00000000;
    ram[2806] = 8'b00000000;
    ram[2805] = 8'b00000000;
    ram[2804] = 8'b00000000;
    ram[2803] = 8'b00000000;
    ram[2802] = 8'b00000000;
    ram[2801] = 8'b00000000;
    ram[2800] = 8'b00000000;
    ram[2799] = 8'b00000000;
    ram[2798] = 8'b00000000;
    ram[2797] = 8'b00000000;
    ram[2796] = 8'b00000000;
    ram[2795] = 8'b00000000;
    ram[2794] = 8'b00000000;
    ram[2793] = 8'b00000000;
    ram[2792] = 8'b00000000;
    ram[2791] = 8'b00000000;
    ram[2790] = 8'b00000000;
    ram[2789] = 8'b00000000;
    ram[2788] = 8'b00000000;
    ram[2787] = 8'b00000000;
    ram[2786] = 8'b00000000;
    ram[2785] = 8'b00000000;
    ram[2784] = 8'b00000000;
    ram[2783] = 8'b00000000;
    ram[2782] = 8'b00000000;
    ram[2781] = 8'b00000000;
    ram[2780] = 8'b00000000;
    ram[2779] = 8'b00000000;
    ram[2778] = 8'b00000000;
    ram[2777] = 8'b00000000;
    ram[2776] = 8'b00000000;
    ram[2775] = 8'b00000000;
    ram[2774] = 8'b00000000;
    ram[2773] = 8'b00000000;
    ram[2772] = 8'b00000000;
    ram[2771] = 8'b00000000;
    ram[2770] = 8'b00000000;
    ram[2769] = 8'b00000000;
    ram[2768] = 8'b00000000;
    ram[2767] = 8'b00000000;
    ram[2766] = 8'b00000000;
    ram[2765] = 8'b00000000;
    ram[2764] = 8'b00000000;
    ram[2763] = 8'b00000000;
    ram[2762] = 8'b00000000;
    ram[2761] = 8'b00000000;
    ram[2760] = 8'b00000000;
    ram[2759] = 8'b00000000;
    ram[2758] = 8'b00000000;
    ram[2757] = 8'b00000000;
    ram[2756] = 8'b00000000;
    ram[2755] = 8'b00000000;
    ram[2754] = 8'b00000000;
    ram[2753] = 8'b00000000;
    ram[2752] = 8'b00000000;
    ram[2751] = 8'b00000000;
    ram[2750] = 8'b00000000;
    ram[2749] = 8'b00000000;
    ram[2748] = 8'b00000000;
    ram[2747] = 8'b00000000;
    ram[2746] = 8'b00000000;
    ram[2745] = 8'b00000000;
    ram[2744] = 8'b00000000;
    ram[2743] = 8'b00000000;
    ram[2742] = 8'b00000000;
    ram[2741] = 8'b00000000;
    ram[2740] = 8'b00000000;
    ram[2739] = 8'b00000000;
    ram[2738] = 8'b00000000;
    ram[2737] = 8'b00000000;
    ram[2736] = 8'b00000000;
    ram[2735] = 8'b00000000;
    ram[2734] = 8'b00000000;
    ram[2733] = 8'b00000000;
    ram[2732] = 8'b00000000;
    ram[2731] = 8'b00000000;
    ram[2730] = 8'b00000000;
    ram[2729] = 8'b00000000;
    ram[2728] = 8'b00000000;
    ram[2727] = 8'b00000000;
    ram[2726] = 8'b00000000;
    ram[2725] = 8'b00000000;
    ram[2724] = 8'b00000000;
    ram[2723] = 8'b00000000;
    ram[2722] = 8'b00000000;
    ram[2721] = 8'b00000000;
    ram[2720] = 8'b00000000;
    ram[2719] = 8'b00000000;
    ram[2718] = 8'b00000000;
    ram[2717] = 8'b00000000;
    ram[2716] = 8'b00000000;
    ram[2715] = 8'b00000000;
    ram[2714] = 8'b00000000;
    ram[2713] = 8'b00000000;
    ram[2712] = 8'b00000000;
    ram[2711] = 8'b00000000;
    ram[2710] = 8'b00000000;
    ram[2709] = 8'b00000000;
    ram[2708] = 8'b00000000;
    ram[2707] = 8'b00000000;
    ram[2706] = 8'b00000000;
    ram[2705] = 8'b00000000;
    ram[2704] = 8'b00000000;
    ram[2703] = 8'b00000000;
    ram[2702] = 8'b00000000;
    ram[2701] = 8'b00000000;
    ram[2700] = 8'b00000000;
    ram[2699] = 8'b00000000;
    ram[2698] = 8'b00000000;
    ram[2697] = 8'b00000000;
    ram[2696] = 8'b00000000;
    ram[2695] = 8'b00000000;
    ram[2694] = 8'b00000000;
    ram[2693] = 8'b00000000;
    ram[2692] = 8'b00000000;
    ram[2691] = 8'b00000000;
    ram[2690] = 8'b00000000;
    ram[2689] = 8'b00000000;
    ram[2688] = 8'b00000000;
    ram[2687] = 8'b00000000;
    ram[2686] = 8'b00000000;
    ram[2685] = 8'b00000000;
    ram[2684] = 8'b00000000;
    ram[2683] = 8'b00000000;
    ram[2682] = 8'b00000000;
    ram[2681] = 8'b00000000;
    ram[2680] = 8'b00000000;
    ram[2679] = 8'b00000000;
    ram[2678] = 8'b00000000;
    ram[2677] = 8'b00000000;
    ram[2676] = 8'b00000000;
    ram[2675] = 8'b00000000;
    ram[2674] = 8'b00000000;
    ram[2673] = 8'b00000000;
    ram[2672] = 8'b00000000;
    ram[2671] = 8'b00000000;
    ram[2670] = 8'b00000000;
    ram[2669] = 8'b00000000;
    ram[2668] = 8'b00000000;
    ram[2667] = 8'b00000000;
    ram[2666] = 8'b00000000;
    ram[2665] = 8'b00000000;
    ram[2664] = 8'b00000000;
    ram[2663] = 8'b00000000;
    ram[2662] = 8'b00000000;
    ram[2661] = 8'b00000000;
    ram[2660] = 8'b00000000;
    ram[2659] = 8'b00000000;
    ram[2658] = 8'b00000000;
    ram[2657] = 8'b00000000;
    ram[2656] = 8'b00000000;
    ram[2655] = 8'b00000000;
    ram[2654] = 8'b00000000;
    ram[2653] = 8'b00000000;
    ram[2652] = 8'b00000000;
    ram[2651] = 8'b00000000;
    ram[2650] = 8'b00000000;
    ram[2649] = 8'b00000000;
    ram[2648] = 8'b00000000;
    ram[2647] = 8'b00000000;
    ram[2646] = 8'b00000000;
    ram[2645] = 8'b00000000;
    ram[2644] = 8'b00000000;
    ram[2643] = 8'b00000000;
    ram[2642] = 8'b00000000;
    ram[2641] = 8'b00000000;
    ram[2640] = 8'b00000000;
    ram[2639] = 8'b00000000;
    ram[2638] = 8'b00000000;
    ram[2637] = 8'b00000000;
    ram[2636] = 8'b00000000;
    ram[2635] = 8'b00000000;
    ram[2634] = 8'b00000000;
    ram[2633] = 8'b00000000;
    ram[2632] = 8'b00000000;
    ram[2631] = 8'b00000000;
    ram[2630] = 8'b00000000;
    ram[2629] = 8'b00000000;
    ram[2628] = 8'b00000000;
    ram[2627] = 8'b00000000;
    ram[2626] = 8'b00000000;
    ram[2625] = 8'b00000000;
    ram[2624] = 8'b00000000;
    ram[2623] = 8'b00000000;
    ram[2622] = 8'b00000000;
    ram[2621] = 8'b00000000;
    ram[2620] = 8'b00000000;
    ram[2619] = 8'b00000000;
    ram[2618] = 8'b00000000;
    ram[2617] = 8'b00000000;
    ram[2616] = 8'b00000000;
    ram[2615] = 8'b00000000;
    ram[2614] = 8'b00000000;
    ram[2613] = 8'b00000000;
    ram[2612] = 8'b00000000;
    ram[2611] = 8'b00000000;
    ram[2610] = 8'b00000000;
    ram[2609] = 8'b00000000;
    ram[2608] = 8'b00000000;
    ram[2607] = 8'b00000000;
    ram[2606] = 8'b00000000;
    ram[2605] = 8'b00000000;
    ram[2604] = 8'b00000000;
    ram[2603] = 8'b00000000;
    ram[2602] = 8'b00000000;
    ram[2601] = 8'b00000000;
    ram[2600] = 8'b00000000;
    ram[2599] = 8'b00000000;
    ram[2598] = 8'b00000000;
    ram[2597] = 8'b00000000;
    ram[2596] = 8'b00000000;
    ram[2595] = 8'b00000000;
    ram[2594] = 8'b00000000;
    ram[2593] = 8'b00000000;
    ram[2592] = 8'b00000000;
    ram[2591] = 8'b00000000;
    ram[2590] = 8'b00000000;
    ram[2589] = 8'b00000000;
    ram[2588] = 8'b00000000;
    ram[2587] = 8'b00000000;
    ram[2586] = 8'b00000000;
    ram[2585] = 8'b00000000;
    ram[2584] = 8'b00000000;
    ram[2583] = 8'b00000000;
    ram[2582] = 8'b00000000;
    ram[2581] = 8'b00000000;
    ram[2580] = 8'b00000000;
    ram[2579] = 8'b00000000;
    ram[2578] = 8'b00000000;
    ram[2577] = 8'b00000000;
    ram[2576] = 8'b00000000;
    ram[2575] = 8'b00000000;
    ram[2574] = 8'b00000000;
    ram[2573] = 8'b00000000;
    ram[2572] = 8'b00000000;
    ram[2571] = 8'b00000000;
    ram[2570] = 8'b00000000;
    ram[2569] = 8'b00000000;
    ram[2568] = 8'b00000000;
    ram[2567] = 8'b00000000;
    ram[2566] = 8'b00000000;
    ram[2565] = 8'b00000000;
    ram[2564] = 8'b00000000;
    ram[2563] = 8'b00000000;
    ram[2562] = 8'b00000000;
    ram[2561] = 8'b00000000;
    ram[2560] = 8'b00000000;
    ram[2559] = 8'b00000000;
    ram[2558] = 8'b00000000;
    ram[2557] = 8'b00000000;
    ram[2556] = 8'b00000000;
    ram[2555] = 8'b00000000;
    ram[2554] = 8'b00000000;
    ram[2553] = 8'b00000000;
    ram[2552] = 8'b00000000;
    ram[2551] = 8'b00000000;
    ram[2550] = 8'b00000000;
    ram[2549] = 8'b00000000;
    ram[2548] = 8'b00000000;
    ram[2547] = 8'b00000000;
    ram[2546] = 8'b00000000;
    ram[2545] = 8'b00000000;
    ram[2544] = 8'b00000000;
    ram[2543] = 8'b00000000;
    ram[2542] = 8'b00000000;
    ram[2541] = 8'b00000000;
    ram[2540] = 8'b00000000;
    ram[2539] = 8'b00000000;
    ram[2538] = 8'b00000000;
    ram[2537] = 8'b00000000;
    ram[2536] = 8'b00000000;
    ram[2535] = 8'b00000000;
    ram[2534] = 8'b00000000;
    ram[2533] = 8'b00000000;
    ram[2532] = 8'b00000000;
    ram[2531] = 8'b00000000;
    ram[2530] = 8'b00000000;
    ram[2529] = 8'b00000000;
    ram[2528] = 8'b00000000;
    ram[2527] = 8'b00000000;
    ram[2526] = 8'b00000000;
    ram[2525] = 8'b00000000;
    ram[2524] = 8'b00000000;
    ram[2523] = 8'b00000000;
    ram[2522] = 8'b00000000;
    ram[2521] = 8'b00000000;
    ram[2520] = 8'b00000000;
    ram[2519] = 8'b00000000;
    ram[2518] = 8'b00000000;
    ram[2517] = 8'b00000000;
    ram[2516] = 8'b00000000;
    ram[2515] = 8'b00000000;
    ram[2514] = 8'b00000000;
    ram[2513] = 8'b00000000;
    ram[2512] = 8'b00000000;
    ram[2511] = 8'b00000000;
    ram[2510] = 8'b00000000;
    ram[2509] = 8'b00000000;
    ram[2508] = 8'b00000000;
    ram[2507] = 8'b00000000;
    ram[2506] = 8'b00000000;
    ram[2505] = 8'b00000000;
    ram[2504] = 8'b00000000;
    ram[2503] = 8'b00000000;
    ram[2502] = 8'b00000000;
    ram[2501] = 8'b00000000;
    ram[2500] = 8'b00000000;
    ram[2499] = 8'b00000000;
    ram[2498] = 8'b00000000;
    ram[2497] = 8'b00000000;
    ram[2496] = 8'b00000000;
    ram[2495] = 8'b00000000;
    ram[2494] = 8'b00000000;
    ram[2493] = 8'b00000000;
    ram[2492] = 8'b00000000;
    ram[2491] = 8'b00000000;
    ram[2490] = 8'b00000000;
    ram[2489] = 8'b00000000;
    ram[2488] = 8'b00000000;
    ram[2487] = 8'b00000000;
    ram[2486] = 8'b00000000;
    ram[2485] = 8'b00000000;
    ram[2484] = 8'b00000000;
    ram[2483] = 8'b00000000;
    ram[2482] = 8'b00000000;
    ram[2481] = 8'b00000000;
    ram[2480] = 8'b00000000;
    ram[2479] = 8'b00000000;
    ram[2478] = 8'b00000000;
    ram[2477] = 8'b00000000;
    ram[2476] = 8'b00000000;
    ram[2475] = 8'b00000000;
    ram[2474] = 8'b00000000;
    ram[2473] = 8'b00000000;
    ram[2472] = 8'b00000000;
    ram[2471] = 8'b00000000;
    ram[2470] = 8'b00000000;
    ram[2469] = 8'b00000000;
    ram[2468] = 8'b00000000;
    ram[2467] = 8'b00000000;
    ram[2466] = 8'b00000000;
    ram[2465] = 8'b00000000;
    ram[2464] = 8'b00000000;
    ram[2463] = 8'b00000000;
    ram[2462] = 8'b00000000;
    ram[2461] = 8'b00000000;
    ram[2460] = 8'b00000000;
    ram[2459] = 8'b00000000;
    ram[2458] = 8'b00000000;
    ram[2457] = 8'b00000000;
    ram[2456] = 8'b00000000;
    ram[2455] = 8'b00000000;
    ram[2454] = 8'b00000000;
    ram[2453] = 8'b00000000;
    ram[2452] = 8'b00000000;
    ram[2451] = 8'b00000000;
    ram[2450] = 8'b00000000;
    ram[2449] = 8'b00000000;
    ram[2448] = 8'b00000000;
    ram[2447] = 8'b00000000;
    ram[2446] = 8'b00000000;
    ram[2445] = 8'b00000000;
    ram[2444] = 8'b00000000;
    ram[2443] = 8'b00000000;
    ram[2442] = 8'b00000000;
    ram[2441] = 8'b00000000;
    ram[2440] = 8'b00000000;
    ram[2439] = 8'b00000000;
    ram[2438] = 8'b00000000;
    ram[2437] = 8'b00000000;
    ram[2436] = 8'b00000000;
    ram[2435] = 8'b00000000;
    ram[2434] = 8'b00000000;
    ram[2433] = 8'b00000000;
    ram[2432] = 8'b00000000;
    ram[2431] = 8'b00000000;
    ram[2430] = 8'b00000000;
    ram[2429] = 8'b00000000;
    ram[2428] = 8'b00000000;
    ram[2427] = 8'b00000000;
    ram[2426] = 8'b00000000;
    ram[2425] = 8'b00000000;
    ram[2424] = 8'b00000000;
    ram[2423] = 8'b00000000;
    ram[2422] = 8'b00000000;
    ram[2421] = 8'b00000000;
    ram[2420] = 8'b00000000;
    ram[2419] = 8'b00000000;
    ram[2418] = 8'b00000000;
    ram[2417] = 8'b00000000;
    ram[2416] = 8'b00000000;
    ram[2415] = 8'b00000000;
    ram[2414] = 8'b00000000;
    ram[2413] = 8'b00000000;
    ram[2412] = 8'b00000000;
    ram[2411] = 8'b00000000;
    ram[2410] = 8'b00000000;
    ram[2409] = 8'b00000000;
    ram[2408] = 8'b00000000;
    ram[2407] = 8'b00000000;
    ram[2406] = 8'b00000000;
    ram[2405] = 8'b00000000;
    ram[2404] = 8'b00000000;
    ram[2403] = 8'b00000000;
    ram[2402] = 8'b00000000;
    ram[2401] = 8'b00000000;
    ram[2400] = 8'b00000000;
    ram[2399] = 8'b00000000;
    ram[2398] = 8'b00000000;
    ram[2397] = 8'b00000000;
    ram[2396] = 8'b00000000;
    ram[2395] = 8'b00000000;
    ram[2394] = 8'b00000000;
    ram[2393] = 8'b00000000;
    ram[2392] = 8'b00000000;
    ram[2391] = 8'b00000000;
    ram[2390] = 8'b00000000;
    ram[2389] = 8'b00000000;
    ram[2388] = 8'b00000000;
    ram[2387] = 8'b00000000;
    ram[2386] = 8'b00000000;
    ram[2385] = 8'b00000000;
    ram[2384] = 8'b00000000;
    ram[2383] = 8'b00000000;
    ram[2382] = 8'b00000000;
    ram[2381] = 8'b00000000;
    ram[2380] = 8'b00000000;
    ram[2379] = 8'b00000000;
    ram[2378] = 8'b00000000;
    ram[2377] = 8'b00000000;
    ram[2376] = 8'b00000000;
    ram[2375] = 8'b00000000;
    ram[2374] = 8'b00000000;
    ram[2373] = 8'b00000000;
    ram[2372] = 8'b00000000;
    ram[2371] = 8'b00000000;
    ram[2370] = 8'b00000000;
    ram[2369] = 8'b00000000;
    ram[2368] = 8'b00000000;
    ram[2367] = 8'b00000000;
    ram[2366] = 8'b00000000;
    ram[2365] = 8'b00000000;
    ram[2364] = 8'b00000000;
    ram[2363] = 8'b00000000;
    ram[2362] = 8'b00000000;
    ram[2361] = 8'b00000000;
    ram[2360] = 8'b00000000;
    ram[2359] = 8'b00000000;
    ram[2358] = 8'b00000000;
    ram[2357] = 8'b00000000;
    ram[2356] = 8'b00000000;
    ram[2355] = 8'b00000000;
    ram[2354] = 8'b00000000;
    ram[2353] = 8'b00000000;
    ram[2352] = 8'b00000000;
    ram[2351] = 8'b00000000;
    ram[2350] = 8'b00000000;
    ram[2349] = 8'b00000000;
    ram[2348] = 8'b00000000;
    ram[2347] = 8'b00000000;
    ram[2346] = 8'b00000000;
    ram[2345] = 8'b00000000;
    ram[2344] = 8'b00000000;
    ram[2343] = 8'b00000000;
    ram[2342] = 8'b00000000;
    ram[2341] = 8'b00000000;
    ram[2340] = 8'b00000000;
    ram[2339] = 8'b00000000;
    ram[2338] = 8'b00000000;
    ram[2337] = 8'b00000000;
    ram[2336] = 8'b00000000;
    ram[2335] = 8'b00000000;
    ram[2334] = 8'b00000000;
    ram[2333] = 8'b00000000;
    ram[2332] = 8'b00000000;
    ram[2331] = 8'b00000000;
    ram[2330] = 8'b00000000;
    ram[2329] = 8'b00000000;
    ram[2328] = 8'b00000000;
    ram[2327] = 8'b00000000;
    ram[2326] = 8'b00000000;
    ram[2325] = 8'b00000000;
    ram[2324] = 8'b00000000;
    ram[2323] = 8'b00000000;
    ram[2322] = 8'b00000000;
    ram[2321] = 8'b00000000;
    ram[2320] = 8'b00000000;
    ram[2319] = 8'b00000000;
    ram[2318] = 8'b00000000;
    ram[2317] = 8'b00000000;
    ram[2316] = 8'b00000000;
    ram[2315] = 8'b00000000;
    ram[2314] = 8'b00000000;
    ram[2313] = 8'b00000000;
    ram[2312] = 8'b00000000;
    ram[2311] = 8'b00000000;
    ram[2310] = 8'b00000000;
    ram[2309] = 8'b00000000;
    ram[2308] = 8'b00000000;
    ram[2307] = 8'b00000000;
    ram[2306] = 8'b00000000;
    ram[2305] = 8'b00000000;
    ram[2304] = 8'b00000000;
    ram[2303] = 8'b00000000;
    ram[2302] = 8'b00000000;
    ram[2301] = 8'b00000000;
    ram[2300] = 8'b00000000;
    ram[2299] = 8'b00000000;
    ram[2298] = 8'b00000000;
    ram[2297] = 8'b00000000;
    ram[2296] = 8'b00000000;
    ram[2295] = 8'b00000000;
    ram[2294] = 8'b00000000;
    ram[2293] = 8'b00000000;
    ram[2292] = 8'b00000000;
    ram[2291] = 8'b00000000;
    ram[2290] = 8'b00000000;
    ram[2289] = 8'b00000000;
    ram[2288] = 8'b00000000;
    ram[2287] = 8'b00000000;
    ram[2286] = 8'b00000000;
    ram[2285] = 8'b00000000;
    ram[2284] = 8'b00000000;
    ram[2283] = 8'b00000000;
    ram[2282] = 8'b00000000;
    ram[2281] = 8'b00000000;
    ram[2280] = 8'b00000000;
    ram[2279] = 8'b00000000;
    ram[2278] = 8'b00000000;
    ram[2277] = 8'b00000000;
    ram[2276] = 8'b00000000;
    ram[2275] = 8'b00000000;
    ram[2274] = 8'b00000000;
    ram[2273] = 8'b00000000;
    ram[2272] = 8'b00000000;
    ram[2271] = 8'b00000000;
    ram[2270] = 8'b00000000;
    ram[2269] = 8'b00000000;
    ram[2268] = 8'b00000000;
    ram[2267] = 8'b00000000;
    ram[2266] = 8'b00000000;
    ram[2265] = 8'b00000000;
    ram[2264] = 8'b00000000;
    ram[2263] = 8'b00000000;
    ram[2262] = 8'b00000000;
    ram[2261] = 8'b00000000;
    ram[2260] = 8'b00000000;
    ram[2259] = 8'b00000000;
    ram[2258] = 8'b00000000;
    ram[2257] = 8'b00000000;
    ram[2256] = 8'b00000000;
    ram[2255] = 8'b00000000;
    ram[2254] = 8'b00000000;
    ram[2253] = 8'b00000000;
    ram[2252] = 8'b00000000;
    ram[2251] = 8'b00000000;
    ram[2250] = 8'b00000000;
    ram[2249] = 8'b00000000;
    ram[2248] = 8'b00000000;
    ram[2247] = 8'b00000000;
    ram[2246] = 8'b00000000;
    ram[2245] = 8'b00000000;
    ram[2244] = 8'b00000000;
    ram[2243] = 8'b00000000;
    ram[2242] = 8'b00000000;
    ram[2241] = 8'b00000000;
    ram[2240] = 8'b00000000;
    ram[2239] = 8'b00000000;
    ram[2238] = 8'b00000000;
    ram[2237] = 8'b00000000;
    ram[2236] = 8'b00000000;
    ram[2235] = 8'b00000000;
    ram[2234] = 8'b00000000;
    ram[2233] = 8'b00000000;
    ram[2232] = 8'b00000000;
    ram[2231] = 8'b00000000;
    ram[2230] = 8'b00000000;
    ram[2229] = 8'b00000000;
    ram[2228] = 8'b00000000;
    ram[2227] = 8'b00000000;
    ram[2226] = 8'b00000000;
    ram[2225] = 8'b00000000;
    ram[2224] = 8'b00000000;
    ram[2223] = 8'b00000000;
    ram[2222] = 8'b00000000;
    ram[2221] = 8'b00000000;
    ram[2220] = 8'b00000000;
    ram[2219] = 8'b00000000;
    ram[2218] = 8'b00000000;
    ram[2217] = 8'b00000000;
    ram[2216] = 8'b00000000;
    ram[2215] = 8'b00000000;
    ram[2214] = 8'b00000000;
    ram[2213] = 8'b00000000;
    ram[2212] = 8'b00000000;
    ram[2211] = 8'b00000000;
    ram[2210] = 8'b00000000;
    ram[2209] = 8'b00000000;
    ram[2208] = 8'b00000000;
    ram[2207] = 8'b00000000;
    ram[2206] = 8'b00000000;
    ram[2205] = 8'b00000000;
    ram[2204] = 8'b00000000;
    ram[2203] = 8'b00000000;
    ram[2202] = 8'b00000000;
    ram[2201] = 8'b00000000;
    ram[2200] = 8'b00000000;
    ram[2199] = 8'b00000000;
    ram[2198] = 8'b00000000;
    ram[2197] = 8'b00000000;
    ram[2196] = 8'b00000000;
    ram[2195] = 8'b00000000;
    ram[2194] = 8'b00000000;
    ram[2193] = 8'b00000000;
    ram[2192] = 8'b00000000;
    ram[2191] = 8'b00000000;
    ram[2190] = 8'b00000000;
    ram[2189] = 8'b00000000;
    ram[2188] = 8'b00000000;
    ram[2187] = 8'b00000000;
    ram[2186] = 8'b00000000;
    ram[2185] = 8'b00000000;
    ram[2184] = 8'b00000000;
    ram[2183] = 8'b00000000;
    ram[2182] = 8'b00000000;
    ram[2181] = 8'b00000000;
    ram[2180] = 8'b00000000;
    ram[2179] = 8'b00000000;
    ram[2178] = 8'b00000000;
    ram[2177] = 8'b00000000;
    ram[2176] = 8'b00000000;
    ram[2175] = 8'b00000000;
    ram[2174] = 8'b00000000;
    ram[2173] = 8'b00000000;
    ram[2172] = 8'b00000000;
    ram[2171] = 8'b00000000;
    ram[2170] = 8'b00000000;
    ram[2169] = 8'b00000000;
    ram[2168] = 8'b00000000;
    ram[2167] = 8'b00000000;
    ram[2166] = 8'b00000000;
    ram[2165] = 8'b00000000;
    ram[2164] = 8'b00000000;
    ram[2163] = 8'b00000000;
    ram[2162] = 8'b00000000;
    ram[2161] = 8'b00000000;
    ram[2160] = 8'b00000000;
    ram[2159] = 8'b00000000;
    ram[2158] = 8'b00000000;
    ram[2157] = 8'b00000000;
    ram[2156] = 8'b00000000;
    ram[2155] = 8'b00000000;
    ram[2154] = 8'b00000000;
    ram[2153] = 8'b00000000;
    ram[2152] = 8'b00000000;
    ram[2151] = 8'b00000000;
    ram[2150] = 8'b00000000;
    ram[2149] = 8'b00000000;
    ram[2148] = 8'b00000000;
    ram[2147] = 8'b00000000;
    ram[2146] = 8'b00000000;
    ram[2145] = 8'b00000000;
    ram[2144] = 8'b00000000;
    ram[2143] = 8'b00000000;
    ram[2142] = 8'b00000000;
    ram[2141] = 8'b00000000;
    ram[2140] = 8'b00000000;
    ram[2139] = 8'b00000000;
    ram[2138] = 8'b00000000;
    ram[2137] = 8'b00000000;
    ram[2136] = 8'b00000000;
    ram[2135] = 8'b00000000;
    ram[2134] = 8'b00000000;
    ram[2133] = 8'b00000000;
    ram[2132] = 8'b00000000;
    ram[2131] = 8'b00000000;
    ram[2130] = 8'b00000000;
    ram[2129] = 8'b00000000;
    ram[2128] = 8'b00000000;
    ram[2127] = 8'b00000000;
    ram[2126] = 8'b00000000;
    ram[2125] = 8'b00000000;
    ram[2124] = 8'b00000000;
    ram[2123] = 8'b00000000;
    ram[2122] = 8'b00000000;
    ram[2121] = 8'b00000000;
    ram[2120] = 8'b00000000;
    ram[2119] = 8'b00000000;
    ram[2118] = 8'b00000000;
    ram[2117] = 8'b00000000;
    ram[2116] = 8'b00000000;
    ram[2115] = 8'b00000000;
    ram[2114] = 8'b00000000;
    ram[2113] = 8'b00000000;
    ram[2112] = 8'b00000000;
    ram[2111] = 8'b00000000;
    ram[2110] = 8'b00000000;
    ram[2109] = 8'b00000000;
    ram[2108] = 8'b00000000;
    ram[2107] = 8'b00000000;
    ram[2106] = 8'b00000000;
    ram[2105] = 8'b00000000;
    ram[2104] = 8'b00000000;
    ram[2103] = 8'b00000000;
    ram[2102] = 8'b00000000;
    ram[2101] = 8'b00000000;
    ram[2100] = 8'b00000000;
    ram[2099] = 8'b00000000;
    ram[2098] = 8'b00000000;
    ram[2097] = 8'b00000000;
    ram[2096] = 8'b00000000;
    ram[2095] = 8'b00000000;
    ram[2094] = 8'b00000000;
    ram[2093] = 8'b00000000;
    ram[2092] = 8'b00000000;
    ram[2091] = 8'b00000000;
    ram[2090] = 8'b00000000;
    ram[2089] = 8'b00000000;
    ram[2088] = 8'b00000000;
    ram[2087] = 8'b00000000;
    ram[2086] = 8'b00000000;
    ram[2085] = 8'b00000000;
    ram[2084] = 8'b00000000;
    ram[2083] = 8'b00000000;
    ram[2082] = 8'b00000000;
    ram[2081] = 8'b00000000;
    ram[2080] = 8'b00000000;
    ram[2079] = 8'b00000000;
    ram[2078] = 8'b00000000;
    ram[2077] = 8'b00000000;
    ram[2076] = 8'b00000000;
    ram[2075] = 8'b00000000;
    ram[2074] = 8'b00000000;
    ram[2073] = 8'b00000000;
    ram[2072] = 8'b00000000;
    ram[2071] = 8'b00000000;
    ram[2070] = 8'b00000000;
    ram[2069] = 8'b00000000;
    ram[2068] = 8'b00000000;
    ram[2067] = 8'b00000000;
    ram[2066] = 8'b00000000;
    ram[2065] = 8'b00000000;
    ram[2064] = 8'b00000000;
    ram[2063] = 8'b00000000;
    ram[2062] = 8'b00000000;
    ram[2061] = 8'b00000000;
    ram[2060] = 8'b00000000;
    ram[2059] = 8'b00000000;
    ram[2058] = 8'b00000000;
    ram[2057] = 8'b00000000;
    ram[2056] = 8'b00000000;
    ram[2055] = 8'b00000000;
    ram[2054] = 8'b00000000;
    ram[2053] = 8'b00000000;
    ram[2052] = 8'b00000000;
    ram[2051] = 8'b00000000;
    ram[2050] = 8'b00000000;
    ram[2049] = 8'b00000000;
    ram[2048] = 8'b00000000;
    ram[2047] = 8'b00000000;
    ram[2046] = 8'b00000000;
    ram[2045] = 8'b00000000;
    ram[2044] = 8'b00000000;
    ram[2043] = 8'b00000000;
    ram[2042] = 8'b00000000;
    ram[2041] = 8'b00000000;
    ram[2040] = 8'b00000000;
    ram[2039] = 8'b00000000;
    ram[2038] = 8'b00000000;
    ram[2037] = 8'b00000000;
    ram[2036] = 8'b00000000;
    ram[2035] = 8'b00000000;
    ram[2034] = 8'b00000000;
    ram[2033] = 8'b00000000;
    ram[2032] = 8'b00000000;
    ram[2031] = 8'b00000000;
    ram[2030] = 8'b00000000;
    ram[2029] = 8'b00000000;
    ram[2028] = 8'b00000000;
    ram[2027] = 8'b00000000;
    ram[2026] = 8'b00000000;
    ram[2025] = 8'b00000000;
    ram[2024] = 8'b00000000;
    ram[2023] = 8'b00000000;
    ram[2022] = 8'b00000000;
    ram[2021] = 8'b00000000;
    ram[2020] = 8'b00000000;
    ram[2019] = 8'b00000000;
    ram[2018] = 8'b00000000;
    ram[2017] = 8'b00000000;
    ram[2016] = 8'b00000000;
    ram[2015] = 8'b00000000;
    ram[2014] = 8'b00000000;
    ram[2013] = 8'b00000000;
    ram[2012] = 8'b00000000;
    ram[2011] = 8'b00000000;
    ram[2010] = 8'b00000000;
    ram[2009] = 8'b00000000;
    ram[2008] = 8'b00000000;
    ram[2007] = 8'b00000000;
    ram[2006] = 8'b00000000;
    ram[2005] = 8'b00000000;
    ram[2004] = 8'b00000000;
    ram[2003] = 8'b00000000;
    ram[2002] = 8'b00000000;
    ram[2001] = 8'b00000000;
    ram[2000] = 8'b00000000;
    ram[1999] = 8'b00000000;
    ram[1998] = 8'b00000000;
    ram[1997] = 8'b00000000;
    ram[1996] = 8'b00000000;
    ram[1995] = 8'b00000000;
    ram[1994] = 8'b00000000;
    ram[1993] = 8'b00000000;
    ram[1992] = 8'b00000000;
    ram[1991] = 8'b00000000;
    ram[1990] = 8'b00000000;
    ram[1989] = 8'b00000000;
    ram[1988] = 8'b00000000;
    ram[1987] = 8'b00000000;
    ram[1986] = 8'b00000000;
    ram[1985] = 8'b00000000;
    ram[1984] = 8'b00000000;
    ram[1983] = 8'b00000000;
    ram[1982] = 8'b00000000;
    ram[1981] = 8'b00000000;
    ram[1980] = 8'b00000000;
    ram[1979] = 8'b00000000;
    ram[1978] = 8'b00000000;
    ram[1977] = 8'b00000000;
    ram[1976] = 8'b00000000;
    ram[1975] = 8'b00000000;
    ram[1974] = 8'b00000000;
    ram[1973] = 8'b00000000;
    ram[1972] = 8'b00000000;
    ram[1971] = 8'b00000000;
    ram[1970] = 8'b00000000;
    ram[1969] = 8'b00000000;
    ram[1968] = 8'b00000000;
    ram[1967] = 8'b00000000;
    ram[1966] = 8'b00000000;
    ram[1965] = 8'b00000000;
    ram[1964] = 8'b00000000;
    ram[1963] = 8'b00000000;
    ram[1962] = 8'b00000000;
    ram[1961] = 8'b00000000;
    ram[1960] = 8'b00000000;
    ram[1959] = 8'b00000000;
    ram[1958] = 8'b00000000;
    ram[1957] = 8'b00000000;
    ram[1956] = 8'b00000000;
    ram[1955] = 8'b00000000;
    ram[1954] = 8'b00000000;
    ram[1953] = 8'b00000000;
    ram[1952] = 8'b00000000;
    ram[1951] = 8'b00000000;
    ram[1950] = 8'b00000000;
    ram[1949] = 8'b00000000;
    ram[1948] = 8'b00000000;
    ram[1947] = 8'b00000000;
    ram[1946] = 8'b00000000;
    ram[1945] = 8'b00000000;
    ram[1944] = 8'b00000000;
    ram[1943] = 8'b00000000;
    ram[1942] = 8'b00000000;
    ram[1941] = 8'b00000000;
    ram[1940] = 8'b00000000;
    ram[1939] = 8'b00000000;
    ram[1938] = 8'b00000000;
    ram[1937] = 8'b00000000;
    ram[1936] = 8'b00000000;
    ram[1935] = 8'b00000000;
    ram[1934] = 8'b00000000;
    ram[1933] = 8'b00000000;
    ram[1932] = 8'b00000000;
    ram[1931] = 8'b00000000;
    ram[1930] = 8'b00000000;
    ram[1929] = 8'b00000000;
    ram[1928] = 8'b00000000;
    ram[1927] = 8'b00000000;
    ram[1926] = 8'b00000000;
    ram[1925] = 8'b00000000;
    ram[1924] = 8'b00000000;
    ram[1923] = 8'b00000000;
    ram[1922] = 8'b00000000;
    ram[1921] = 8'b00000000;
    ram[1920] = 8'b00000000;
    ram[1919] = 8'b00000000;
    ram[1918] = 8'b00000000;
    ram[1917] = 8'b00000000;
    ram[1916] = 8'b00000000;
    ram[1915] = 8'b00000000;
    ram[1914] = 8'b00000000;
    ram[1913] = 8'b00000000;
    ram[1912] = 8'b00000000;
    ram[1911] = 8'b00000000;
    ram[1910] = 8'b00000000;
    ram[1909] = 8'b00000000;
    ram[1908] = 8'b00000000;
    ram[1907] = 8'b00000000;
    ram[1906] = 8'b00000000;
    ram[1905] = 8'b00000000;
    ram[1904] = 8'b00000000;
    ram[1903] = 8'b00000000;
    ram[1902] = 8'b00000000;
    ram[1901] = 8'b00000000;
    ram[1900] = 8'b00000000;
    ram[1899] = 8'b00000000;
    ram[1898] = 8'b00000000;
    ram[1897] = 8'b00000000;
    ram[1896] = 8'b00000000;
    ram[1895] = 8'b00000000;
    ram[1894] = 8'b00000000;
    ram[1893] = 8'b00000000;
    ram[1892] = 8'b00000000;
    ram[1891] = 8'b00000000;
    ram[1890] = 8'b00000000;
    ram[1889] = 8'b00000000;
    ram[1888] = 8'b00000000;
    ram[1887] = 8'b00000000;
    ram[1886] = 8'b00000000;
    ram[1885] = 8'b00000000;
    ram[1884] = 8'b00000000;
    ram[1883] = 8'b00000000;
    ram[1882] = 8'b00000000;
    ram[1881] = 8'b00000000;
    ram[1880] = 8'b00000000;
    ram[1879] = 8'b00000000;
    ram[1878] = 8'b00000000;
    ram[1877] = 8'b00000000;
    ram[1876] = 8'b00000000;
    ram[1875] = 8'b00000000;
    ram[1874] = 8'b00000000;
    ram[1873] = 8'b00000000;
    ram[1872] = 8'b00000000;
    ram[1871] = 8'b00000000;
    ram[1870] = 8'b00000000;
    ram[1869] = 8'b00000000;
    ram[1868] = 8'b00000000;
    ram[1867] = 8'b00000000;
    ram[1866] = 8'b00000000;
    ram[1865] = 8'b00000000;
    ram[1864] = 8'b00000000;
    ram[1863] = 8'b00000000;
    ram[1862] = 8'b00000000;
    ram[1861] = 8'b00000000;
    ram[1860] = 8'b00000000;
    ram[1859] = 8'b00000000;
    ram[1858] = 8'b00000000;
    ram[1857] = 8'b00000000;
    ram[1856] = 8'b00000000;
    ram[1855] = 8'b00000000;
    ram[1854] = 8'b00000000;
    ram[1853] = 8'b00000000;
    ram[1852] = 8'b00000000;
    ram[1851] = 8'b00000000;
    ram[1850] = 8'b00000000;
    ram[1849] = 8'b00000000;
    ram[1848] = 8'b00000000;
    ram[1847] = 8'b00000000;
    ram[1846] = 8'b00000000;
    ram[1845] = 8'b00000000;
    ram[1844] = 8'b00000000;
    ram[1843] = 8'b00000000;
    ram[1842] = 8'b00000000;
    ram[1841] = 8'b00000000;
    ram[1840] = 8'b00000000;
    ram[1839] = 8'b00000000;
    ram[1838] = 8'b00000000;
    ram[1837] = 8'b00000000;
    ram[1836] = 8'b00000000;
    ram[1835] = 8'b00000000;
    ram[1834] = 8'b00000000;
    ram[1833] = 8'b00000000;
    ram[1832] = 8'b00000000;
    ram[1831] = 8'b00000000;
    ram[1830] = 8'b00000000;
    ram[1829] = 8'b00000000;
    ram[1828] = 8'b00000000;
    ram[1827] = 8'b00000000;
    ram[1826] = 8'b00000000;
    ram[1825] = 8'b00000000;
    ram[1824] = 8'b00000000;
    ram[1823] = 8'b00000000;
    ram[1822] = 8'b00000000;
    ram[1821] = 8'b00000000;
    ram[1820] = 8'b00000000;
    ram[1819] = 8'b00000000;
    ram[1818] = 8'b00000000;
    ram[1817] = 8'b00000000;
    ram[1816] = 8'b00000000;
    ram[1815] = 8'b00000000;
    ram[1814] = 8'b00000000;
    ram[1813] = 8'b00000000;
    ram[1812] = 8'b00000000;
    ram[1811] = 8'b00000000;
    ram[1810] = 8'b00000000;
    ram[1809] = 8'b00000000;
    ram[1808] = 8'b00000000;
    ram[1807] = 8'b00000000;
    ram[1806] = 8'b00000000;
    ram[1805] = 8'b00000000;
    ram[1804] = 8'b00000000;
    ram[1803] = 8'b00000000;
    ram[1802] = 8'b00000000;
    ram[1801] = 8'b00000000;
    ram[1800] = 8'b00000000;
    ram[1799] = 8'b00000000;
    ram[1798] = 8'b00000000;
    ram[1797] = 8'b00000000;
    ram[1796] = 8'b00000000;
    ram[1795] = 8'b00000000;
    ram[1794] = 8'b00000000;
    ram[1793] = 8'b00000000;
    ram[1792] = 8'b00000000;
    ram[1791] = 8'b00000000;
    ram[1790] = 8'b00000000;
    ram[1789] = 8'b00000000;
    ram[1788] = 8'b00000000;
    ram[1787] = 8'b00000000;
    ram[1786] = 8'b00000000;
    ram[1785] = 8'b00000000;
    ram[1784] = 8'b00000000;
    ram[1783] = 8'b00000000;
    ram[1782] = 8'b00000000;
    ram[1781] = 8'b00000000;
    ram[1780] = 8'b00000000;
    ram[1779] = 8'b00000000;
    ram[1778] = 8'b00000000;
    ram[1777] = 8'b00000000;
    ram[1776] = 8'b00000000;
    ram[1775] = 8'b00000000;
    ram[1774] = 8'b00000000;
    ram[1773] = 8'b00000000;
    ram[1772] = 8'b00000000;
    ram[1771] = 8'b00000000;
    ram[1770] = 8'b00000000;
    ram[1769] = 8'b00000000;
    ram[1768] = 8'b00000000;
    ram[1767] = 8'b00000000;
    ram[1766] = 8'b00000000;
    ram[1765] = 8'b00000000;
    ram[1764] = 8'b00000000;
    ram[1763] = 8'b00000000;
    ram[1762] = 8'b00000000;
    ram[1761] = 8'b00000000;
    ram[1760] = 8'b00000000;
    ram[1759] = 8'b00000000;
    ram[1758] = 8'b00000000;
    ram[1757] = 8'b00000000;
    ram[1756] = 8'b00000000;
    ram[1755] = 8'b00000000;
    ram[1754] = 8'b00000000;
    ram[1753] = 8'b00000000;
    ram[1752] = 8'b00000000;
    ram[1751] = 8'b00000000;
    ram[1750] = 8'b00000000;
    ram[1749] = 8'b00000000;
    ram[1748] = 8'b00000000;
    ram[1747] = 8'b00000000;
    ram[1746] = 8'b00000000;
    ram[1745] = 8'b00000000;
    ram[1744] = 8'b00000000;
    ram[1743] = 8'b00000000;
    ram[1742] = 8'b00000000;
    ram[1741] = 8'b00000000;
    ram[1740] = 8'b00000000;
    ram[1739] = 8'b00000000;
    ram[1738] = 8'b00000000;
    ram[1737] = 8'b00000000;
    ram[1736] = 8'b00000000;
    ram[1735] = 8'b00000000;
    ram[1734] = 8'b00000000;
    ram[1733] = 8'b00000000;
    ram[1732] = 8'b00000000;
    ram[1731] = 8'b00000000;
    ram[1730] = 8'b00000000;
    ram[1729] = 8'b00000000;
    ram[1728] = 8'b00000000;
    ram[1727] = 8'b00000000;
    ram[1726] = 8'b00000000;
    ram[1725] = 8'b00000000;
    ram[1724] = 8'b00000000;
    ram[1723] = 8'b00000000;
    ram[1722] = 8'b00000000;
    ram[1721] = 8'b00000000;
    ram[1720] = 8'b00000000;
    ram[1719] = 8'b00000000;
    ram[1718] = 8'b00000000;
    ram[1717] = 8'b00000000;
    ram[1716] = 8'b00000000;
    ram[1715] = 8'b00000000;
    ram[1714] = 8'b00000000;
    ram[1713] = 8'b00000000;
    ram[1712] = 8'b00000000;
    ram[1711] = 8'b00000000;
    ram[1710] = 8'b00000000;
    ram[1709] = 8'b00000000;
    ram[1708] = 8'b00000000;
    ram[1707] = 8'b00000000;
    ram[1706] = 8'b00000000;
    ram[1705] = 8'b00000000;
    ram[1704] = 8'b00000000;
    ram[1703] = 8'b00000000;
    ram[1702] = 8'b00000000;
    ram[1701] = 8'b00000000;
    ram[1700] = 8'b00000000;
    ram[1699] = 8'b00000000;
    ram[1698] = 8'b00000000;
    ram[1697] = 8'b00000000;
    ram[1696] = 8'b00000000;
    ram[1695] = 8'b00000000;
    ram[1694] = 8'b00000000;
    ram[1693] = 8'b00000000;
    ram[1692] = 8'b00000000;
    ram[1691] = 8'b00000000;
    ram[1690] = 8'b00000000;
    ram[1689] = 8'b00000000;
    ram[1688] = 8'b00000000;
    ram[1687] = 8'b00000000;
    ram[1686] = 8'b00000000;
    ram[1685] = 8'b00000000;
    ram[1684] = 8'b00000000;
    ram[1683] = 8'b00000000;
    ram[1682] = 8'b00000000;
    ram[1681] = 8'b00000000;
    ram[1680] = 8'b00000000;
    ram[1679] = 8'b00000000;
    ram[1678] = 8'b00000000;
    ram[1677] = 8'b00000000;
    ram[1676] = 8'b00000000;
    ram[1675] = 8'b00000000;
    ram[1674] = 8'b00000000;
    ram[1673] = 8'b00000000;
    ram[1672] = 8'b00000000;
    ram[1671] = 8'b00000000;
    ram[1670] = 8'b00000000;
    ram[1669] = 8'b00000000;
    ram[1668] = 8'b00000000;
    ram[1667] = 8'b00000000;
    ram[1666] = 8'b00000000;
    ram[1665] = 8'b00000000;
    ram[1664] = 8'b00000000;
    ram[1663] = 8'b00000000;
    ram[1662] = 8'b00000000;
    ram[1661] = 8'b00000000;
    ram[1660] = 8'b00000000;
    ram[1659] = 8'b00000000;
    ram[1658] = 8'b00000000;
    ram[1657] = 8'b00000000;
    ram[1656] = 8'b00000000;
    ram[1655] = 8'b00000000;
    ram[1654] = 8'b00000000;
    ram[1653] = 8'b00000000;
    ram[1652] = 8'b00000000;
    ram[1651] = 8'b00000000;
    ram[1650] = 8'b00000000;
    ram[1649] = 8'b00000000;
    ram[1648] = 8'b00000000;
    ram[1647] = 8'b00000000;
    ram[1646] = 8'b00000000;
    ram[1645] = 8'b00000000;
    ram[1644] = 8'b00000000;
    ram[1643] = 8'b00000000;
    ram[1642] = 8'b00000000;
    ram[1641] = 8'b00000000;
    ram[1640] = 8'b00000000;
    ram[1639] = 8'b00000000;
    ram[1638] = 8'b00000000;
    ram[1637] = 8'b00000000;
    ram[1636] = 8'b00000000;
    ram[1635] = 8'b00000000;
    ram[1634] = 8'b00000000;
    ram[1633] = 8'b00000000;
    ram[1632] = 8'b00000000;
    ram[1631] = 8'b00000000;
    ram[1630] = 8'b00000000;
    ram[1629] = 8'b00000000;
    ram[1628] = 8'b00000000;
    ram[1627] = 8'b00000000;
    ram[1626] = 8'b00000000;
    ram[1625] = 8'b00000000;
    ram[1624] = 8'b00000000;
    ram[1623] = 8'b00000000;
    ram[1622] = 8'b00000000;
    ram[1621] = 8'b00000000;
    ram[1620] = 8'b00000000;
    ram[1619] = 8'b00000000;
    ram[1618] = 8'b00000000;
    ram[1617] = 8'b00000000;
    ram[1616] = 8'b00000000;
    ram[1615] = 8'b00000000;
    ram[1614] = 8'b00000000;
    ram[1613] = 8'b00000000;
    ram[1612] = 8'b00000000;
    ram[1611] = 8'b00000000;
    ram[1610] = 8'b00000000;
    ram[1609] = 8'b00000000;
    ram[1608] = 8'b00000000;
    ram[1607] = 8'b00000000;
    ram[1606] = 8'b00000000;
    ram[1605] = 8'b00000000;
    ram[1604] = 8'b00000000;
    ram[1603] = 8'b00000000;
    ram[1602] = 8'b00000000;
    ram[1601] = 8'b00000000;
    ram[1600] = 8'b00000000;
    ram[1599] = 8'b00000000;
    ram[1598] = 8'b00000000;
    ram[1597] = 8'b00000000;
    ram[1596] = 8'b00000000;
    ram[1595] = 8'b00000000;
    ram[1594] = 8'b00000000;
    ram[1593] = 8'b00000000;
    ram[1592] = 8'b00000000;
    ram[1591] = 8'b00000000;
    ram[1590] = 8'b00000000;
    ram[1589] = 8'b00000000;
    ram[1588] = 8'b00000000;
    ram[1587] = 8'b00000000;
    ram[1586] = 8'b00000000;
    ram[1585] = 8'b00000000;
    ram[1584] = 8'b00000000;
    ram[1583] = 8'b00000000;
    ram[1582] = 8'b00000000;
    ram[1581] = 8'b00000000;
    ram[1580] = 8'b00000000;
    ram[1579] = 8'b00000000;
    ram[1578] = 8'b00000000;
    ram[1577] = 8'b00000000;
    ram[1576] = 8'b00000000;
    ram[1575] = 8'b00000000;
    ram[1574] = 8'b00000000;
    ram[1573] = 8'b00000000;
    ram[1572] = 8'b00000000;
    ram[1571] = 8'b00000000;
    ram[1570] = 8'b00000000;
    ram[1569] = 8'b00000000;
    ram[1568] = 8'b00000000;
    ram[1567] = 8'b00000000;
    ram[1566] = 8'b00000000;
    ram[1565] = 8'b00000000;
    ram[1564] = 8'b00000000;
    ram[1563] = 8'b00000000;
    ram[1562] = 8'b00000000;
    ram[1561] = 8'b00000000;
    ram[1560] = 8'b00000000;
    ram[1559] = 8'b00000000;
    ram[1558] = 8'b00000000;
    ram[1557] = 8'b00000000;
    ram[1556] = 8'b00000000;
    ram[1555] = 8'b00000000;
    ram[1554] = 8'b00000000;
    ram[1553] = 8'b00000000;
    ram[1552] = 8'b00000000;
    ram[1551] = 8'b00000000;
    ram[1550] = 8'b00000000;
    ram[1549] = 8'b00000000;
    ram[1548] = 8'b00000000;
    ram[1547] = 8'b00000000;
    ram[1546] = 8'b00000000;
    ram[1545] = 8'b00000000;
    ram[1544] = 8'b00000000;
    ram[1543] = 8'b00000000;
    ram[1542] = 8'b00000000;
    ram[1541] = 8'b00000000;
    ram[1540] = 8'b00000000;
    ram[1539] = 8'b00000000;
    ram[1538] = 8'b00000000;
    ram[1537] = 8'b00000000;
    ram[1536] = 8'b00000000;
    ram[1535] = 8'b00000000;
    ram[1534] = 8'b00000000;
    ram[1533] = 8'b00000000;
    ram[1532] = 8'b00000000;
    ram[1531] = 8'b00000000;
    ram[1530] = 8'b00000000;
    ram[1529] = 8'b00000000;
    ram[1528] = 8'b00000000;
    ram[1527] = 8'b00000000;
    ram[1526] = 8'b00000000;
    ram[1525] = 8'b00000000;
    ram[1524] = 8'b00000000;
    ram[1523] = 8'b00000000;
    ram[1522] = 8'b00000000;
    ram[1521] = 8'b00000000;
    ram[1520] = 8'b00000000;
    ram[1519] = 8'b00000000;
    ram[1518] = 8'b00000000;
    ram[1517] = 8'b00000000;
    ram[1516] = 8'b00000000;
    ram[1515] = 8'b00000000;
    ram[1514] = 8'b00000000;
    ram[1513] = 8'b00000000;
    ram[1512] = 8'b00000000;
    ram[1511] = 8'b00000000;
    ram[1510] = 8'b00000000;
    ram[1509] = 8'b00000000;
    ram[1508] = 8'b00000000;
    ram[1507] = 8'b00000000;
    ram[1506] = 8'b00000000;
    ram[1505] = 8'b00000000;
    ram[1504] = 8'b00000000;
    ram[1503] = 8'b00000000;
    ram[1502] = 8'b00000000;
    ram[1501] = 8'b00000000;
    ram[1500] = 8'b00000000;
    ram[1499] = 8'b00000000;
    ram[1498] = 8'b00000000;
    ram[1497] = 8'b00000000;
    ram[1496] = 8'b00000000;
    ram[1495] = 8'b00000000;
    ram[1494] = 8'b00000000;
    ram[1493] = 8'b00000000;
    ram[1492] = 8'b00000000;
    ram[1491] = 8'b00000000;
    ram[1490] = 8'b00000000;
    ram[1489] = 8'b00000000;
    ram[1488] = 8'b00000000;
    ram[1487] = 8'b00000000;
    ram[1486] = 8'b00000000;
    ram[1485] = 8'b00000000;
    ram[1484] = 8'b00000000;
    ram[1483] = 8'b00000000;
    ram[1482] = 8'b00000000;
    ram[1481] = 8'b00000000;
    ram[1480] = 8'b00000000;
    ram[1479] = 8'b00000000;
    ram[1478] = 8'b00000000;
    ram[1477] = 8'b00000000;
    ram[1476] = 8'b00000000;
    ram[1475] = 8'b00000000;
    ram[1474] = 8'b00000000;
    ram[1473] = 8'b00000000;
    ram[1472] = 8'b00000000;
    ram[1471] = 8'b00000000;
    ram[1470] = 8'b00000000;
    ram[1469] = 8'b00000000;
    ram[1468] = 8'b00000000;
    ram[1467] = 8'b00000000;
    ram[1466] = 8'b00000000;
    ram[1465] = 8'b00000000;
    ram[1464] = 8'b00000000;
    ram[1463] = 8'b00000000;
    ram[1462] = 8'b00000000;
    ram[1461] = 8'b00000000;
    ram[1460] = 8'b00000000;
    ram[1459] = 8'b00000000;
    ram[1458] = 8'b00000000;
    ram[1457] = 8'b00000000;
    ram[1456] = 8'b00000000;
    ram[1455] = 8'b00000000;
    ram[1454] = 8'b00000000;
    ram[1453] = 8'b00000000;
    ram[1452] = 8'b00000000;
    ram[1451] = 8'b00000000;
    ram[1450] = 8'b00000000;
    ram[1449] = 8'b00000000;
    ram[1448] = 8'b00000000;
    ram[1447] = 8'b00000000;
    ram[1446] = 8'b00000000;
    ram[1445] = 8'b00000000;
    ram[1444] = 8'b00000000;
    ram[1443] = 8'b00000000;
    ram[1442] = 8'b00000000;
    ram[1441] = 8'b00000000;
    ram[1440] = 8'b00000000;
    ram[1439] = 8'b00000000;
    ram[1438] = 8'b00000000;
    ram[1437] = 8'b00000000;
    ram[1436] = 8'b00000000;
    ram[1435] = 8'b00000000;
    ram[1434] = 8'b00000000;
    ram[1433] = 8'b00000000;
    ram[1432] = 8'b00000000;
    ram[1431] = 8'b00000000;
    ram[1430] = 8'b00000000;
    ram[1429] = 8'b00000000;
    ram[1428] = 8'b00000000;
    ram[1427] = 8'b00000000;
    ram[1426] = 8'b00000000;
    ram[1425] = 8'b00000000;
    ram[1424] = 8'b00000000;
    ram[1423] = 8'b00000000;
    ram[1422] = 8'b00000000;
    ram[1421] = 8'b00000000;
    ram[1420] = 8'b00000000;
    ram[1419] = 8'b00000000;
    ram[1418] = 8'b00000000;
    ram[1417] = 8'b00000000;
    ram[1416] = 8'b00000000;
    ram[1415] = 8'b00000000;
    ram[1414] = 8'b00000000;
    ram[1413] = 8'b00000000;
    ram[1412] = 8'b00000000;
    ram[1411] = 8'b00000000;
    ram[1410] = 8'b00000000;
    ram[1409] = 8'b00000000;
    ram[1408] = 8'b00000000;
    ram[1407] = 8'b00000000;
    ram[1406] = 8'b00000000;
    ram[1405] = 8'b00000000;
    ram[1404] = 8'b00000000;
    ram[1403] = 8'b00000000;
    ram[1402] = 8'b00000000;
    ram[1401] = 8'b00000000;
    ram[1400] = 8'b00000000;
    ram[1399] = 8'b00000000;
    ram[1398] = 8'b00000000;
    ram[1397] = 8'b00000000;
    ram[1396] = 8'b00000000;
    ram[1395] = 8'b00000000;
    ram[1394] = 8'b00000000;
    ram[1393] = 8'b00000000;
    ram[1392] = 8'b00000000;
    ram[1391] = 8'b00000000;
    ram[1390] = 8'b00000000;
    ram[1389] = 8'b00000000;
    ram[1388] = 8'b00000000;
    ram[1387] = 8'b00000000;
    ram[1386] = 8'b00000000;
    ram[1385] = 8'b00000000;
    ram[1384] = 8'b00000000;
    ram[1383] = 8'b00000000;
    ram[1382] = 8'b00000000;
    ram[1381] = 8'b00000000;
    ram[1380] = 8'b00000000;
    ram[1379] = 8'b00000000;
    ram[1378] = 8'b00000000;
    ram[1377] = 8'b00000000;
    ram[1376] = 8'b00000000;
    ram[1375] = 8'b00000000;
    ram[1374] = 8'b00000000;
    ram[1373] = 8'b00000000;
    ram[1372] = 8'b00000000;
    ram[1371] = 8'b00000000;
    ram[1370] = 8'b00000000;
    ram[1369] = 8'b00000000;
    ram[1368] = 8'b00000000;
    ram[1367] = 8'b00000000;
    ram[1366] = 8'b00000000;
    ram[1365] = 8'b00000000;
    ram[1364] = 8'b00000000;
    ram[1363] = 8'b00000000;
    ram[1362] = 8'b00000000;
    ram[1361] = 8'b00000000;
    ram[1360] = 8'b00000000;
    ram[1359] = 8'b00000000;
    ram[1358] = 8'b00000000;
    ram[1357] = 8'b00000000;
    ram[1356] = 8'b00000000;
    ram[1355] = 8'b00000000;
    ram[1354] = 8'b00000000;
    ram[1353] = 8'b00000000;
    ram[1352] = 8'b00000000;
    ram[1351] = 8'b00000000;
    ram[1350] = 8'b00000000;
    ram[1349] = 8'b00000000;
    ram[1348] = 8'b00000000;
    ram[1347] = 8'b00000000;
    ram[1346] = 8'b00000000;
    ram[1345] = 8'b00000000;
    ram[1344] = 8'b00000000;
    ram[1343] = 8'b00000000;
    ram[1342] = 8'b00000000;
    ram[1341] = 8'b00000000;
    ram[1340] = 8'b00000000;
    ram[1339] = 8'b00000000;
    ram[1338] = 8'b00000000;
    ram[1337] = 8'b00000000;
    ram[1336] = 8'b00000000;
    ram[1335] = 8'b00000000;
    ram[1334] = 8'b00000000;
    ram[1333] = 8'b00000000;
    ram[1332] = 8'b00000000;
    ram[1331] = 8'b00000000;
    ram[1330] = 8'b00000000;
    ram[1329] = 8'b00000000;
    ram[1328] = 8'b00000000;
    ram[1327] = 8'b00000000;
    ram[1326] = 8'b00000000;
    ram[1325] = 8'b00000000;
    ram[1324] = 8'b00000000;
    ram[1323] = 8'b00000000;
    ram[1322] = 8'b00000000;
    ram[1321] = 8'b00000000;
    ram[1320] = 8'b00000000;
    ram[1319] = 8'b00000000;
    ram[1318] = 8'b00000000;
    ram[1317] = 8'b00000000;
    ram[1316] = 8'b00000000;
    ram[1315] = 8'b00000000;
    ram[1314] = 8'b00000000;
    ram[1313] = 8'b00000000;
    ram[1312] = 8'b00000000;
    ram[1311] = 8'b00000000;
    ram[1310] = 8'b00000000;
    ram[1309] = 8'b00000000;
    ram[1308] = 8'b00000000;
    ram[1307] = 8'b00000000;
    ram[1306] = 8'b00000000;
    ram[1305] = 8'b00000000;
    ram[1304] = 8'b00000000;
    ram[1303] = 8'b00000000;
    ram[1302] = 8'b00000000;
    ram[1301] = 8'b00000000;
    ram[1300] = 8'b00000000;
    ram[1299] = 8'b00000000;
    ram[1298] = 8'b00000000;
    ram[1297] = 8'b00000000;
    ram[1296] = 8'b00000000;
    ram[1295] = 8'b00000000;
    ram[1294] = 8'b00000000;
    ram[1293] = 8'b00000000;
    ram[1292] = 8'b00000000;
    ram[1291] = 8'b00000000;
    ram[1290] = 8'b00000000;
    ram[1289] = 8'b00000000;
    ram[1288] = 8'b00000000;
    ram[1287] = 8'b00000000;
    ram[1286] = 8'b00000000;
    ram[1285] = 8'b00000000;
    ram[1284] = 8'b00000000;
    ram[1283] = 8'b00000000;
    ram[1282] = 8'b00000000;
    ram[1281] = 8'b00000000;
    ram[1280] = 8'b00000000;
    ram[1279] = 8'b00000000;
    ram[1278] = 8'b00000000;
    ram[1277] = 8'b00000000;
    ram[1276] = 8'b00000000;
    ram[1275] = 8'b00000000;
    ram[1274] = 8'b00000000;
    ram[1273] = 8'b00000000;
    ram[1272] = 8'b00000000;
    ram[1271] = 8'b00000000;
    ram[1270] = 8'b00000000;
    ram[1269] = 8'b00000000;
    ram[1268] = 8'b00000000;
    ram[1267] = 8'b00000000;
    ram[1266] = 8'b00000000;
    ram[1265] = 8'b00000000;
    ram[1264] = 8'b00000000;
    ram[1263] = 8'b00000000;
    ram[1262] = 8'b00000000;
    ram[1261] = 8'b00000000;
    ram[1260] = 8'b00000000;
    ram[1259] = 8'b00000000;
    ram[1258] = 8'b00000000;
    ram[1257] = 8'b00000000;
    ram[1256] = 8'b00000000;
    ram[1255] = 8'b00000000;
    ram[1254] = 8'b00000000;
    ram[1253] = 8'b00000000;
    ram[1252] = 8'b00000000;
    ram[1251] = 8'b00000000;
    ram[1250] = 8'b00000000;
    ram[1249] = 8'b00000000;
    ram[1248] = 8'b00000000;
    ram[1247] = 8'b00000000;
    ram[1246] = 8'b00000000;
    ram[1245] = 8'b00000000;
    ram[1244] = 8'b00000000;
    ram[1243] = 8'b00000000;
    ram[1242] = 8'b00000000;
    ram[1241] = 8'b00000000;
    ram[1240] = 8'b00000000;
    ram[1239] = 8'b00000000;
    ram[1238] = 8'b00000000;
    ram[1237] = 8'b00000000;
    ram[1236] = 8'b00000000;
    ram[1235] = 8'b00000000;
    ram[1234] = 8'b00000000;
    ram[1233] = 8'b00000000;
    ram[1232] = 8'b00000000;
    ram[1231] = 8'b00000000;
    ram[1230] = 8'b00000000;
    ram[1229] = 8'b00000000;
    ram[1228] = 8'b00000000;
    ram[1227] = 8'b00000000;
    ram[1226] = 8'b00000000;
    ram[1225] = 8'b00000000;
    ram[1224] = 8'b00000000;
    ram[1223] = 8'b00000000;
    ram[1222] = 8'b00000000;
    ram[1221] = 8'b00000000;
    ram[1220] = 8'b00000000;
    ram[1219] = 8'b00000000;
    ram[1218] = 8'b00000000;
    ram[1217] = 8'b00000000;
    ram[1216] = 8'b00000000;
    ram[1215] = 8'b00000000;
    ram[1214] = 8'b00000000;
    ram[1213] = 8'b00000000;
    ram[1212] = 8'b00000000;
    ram[1211] = 8'b00000000;
    ram[1210] = 8'b00000000;
    ram[1209] = 8'b00000000;
    ram[1208] = 8'b00000000;
    ram[1207] = 8'b00000000;
    ram[1206] = 8'b00000000;
    ram[1205] = 8'b00000000;
    ram[1204] = 8'b00000000;
    ram[1203] = 8'b00000000;
    ram[1202] = 8'b00000000;
    ram[1201] = 8'b00000000;
    ram[1200] = 8'b00000000;
    ram[1199] = 8'b00000000;
    ram[1198] = 8'b00000000;
    ram[1197] = 8'b00000000;
    ram[1196] = 8'b00000000;
    ram[1195] = 8'b00000000;
    ram[1194] = 8'b00000000;
    ram[1193] = 8'b00000000;
    ram[1192] = 8'b00000000;
    ram[1191] = 8'b00000000;
    ram[1190] = 8'b00000000;
    ram[1189] = 8'b00000000;
    ram[1188] = 8'b00000000;
    ram[1187] = 8'b00000000;
    ram[1186] = 8'b00000000;
    ram[1185] = 8'b00000000;
    ram[1184] = 8'b00000000;
    ram[1183] = 8'b00000000;
    ram[1182] = 8'b00000000;
    ram[1181] = 8'b00000000;
    ram[1180] = 8'b00000000;
    ram[1179] = 8'b00000000;
    ram[1178] = 8'b00000000;
    ram[1177] = 8'b00000000;
    ram[1176] = 8'b00000000;
    ram[1175] = 8'b00000000;
    ram[1174] = 8'b00000000;
    ram[1173] = 8'b00000000;
    ram[1172] = 8'b00000000;
    ram[1171] = 8'b00000000;
    ram[1170] = 8'b00000000;
    ram[1169] = 8'b00000000;
    ram[1168] = 8'b00000000;
    ram[1167] = 8'b00000000;
    ram[1166] = 8'b00000000;
    ram[1165] = 8'b00000000;
    ram[1164] = 8'b00000000;
    ram[1163] = 8'b00000000;
    ram[1162] = 8'b00000000;
    ram[1161] = 8'b00000000;
    ram[1160] = 8'b00000000;
    ram[1159] = 8'b00000000;
    ram[1158] = 8'b00000000;
    ram[1157] = 8'b00000000;
    ram[1156] = 8'b00000000;
    ram[1155] = 8'b00000000;
    ram[1154] = 8'b00000000;
    ram[1153] = 8'b00000000;
    ram[1152] = 8'b00000000;
    ram[1151] = 8'b00000000;
    ram[1150] = 8'b00000000;
    ram[1149] = 8'b00000000;
    ram[1148] = 8'b00000000;
    ram[1147] = 8'b00000000;
    ram[1146] = 8'b00000000;
    ram[1145] = 8'b00000000;
    ram[1144] = 8'b00000000;
    ram[1143] = 8'b00000000;
    ram[1142] = 8'b00000000;
    ram[1141] = 8'b00000000;
    ram[1140] = 8'b00000000;
    ram[1139] = 8'b00000000;
    ram[1138] = 8'b00000000;
    ram[1137] = 8'b00000000;
    ram[1136] = 8'b00000000;
    ram[1135] = 8'b00000000;
    ram[1134] = 8'b00000000;
    ram[1133] = 8'b00000000;
    ram[1132] = 8'b00000000;
    ram[1131] = 8'b00000000;
    ram[1130] = 8'b00000000;
    ram[1129] = 8'b00000000;
    ram[1128] = 8'b00000000;
    ram[1127] = 8'b00000000;
    ram[1126] = 8'b00000000;
    ram[1125] = 8'b00000000;
    ram[1124] = 8'b00000000;
    ram[1123] = 8'b00000000;
    ram[1122] = 8'b00000000;
    ram[1121] = 8'b00000000;
    ram[1120] = 8'b00000000;
    ram[1119] = 8'b00000000;
    ram[1118] = 8'b00000000;
    ram[1117] = 8'b00000000;
    ram[1116] = 8'b00000000;
    ram[1115] = 8'b00000000;
    ram[1114] = 8'b00000000;
    ram[1113] = 8'b00000000;
    ram[1112] = 8'b00000000;
    ram[1111] = 8'b00000000;
    ram[1110] = 8'b00000000;
    ram[1109] = 8'b00000000;
    ram[1108] = 8'b00000000;
    ram[1107] = 8'b00000000;
    ram[1106] = 8'b00000000;
    ram[1105] = 8'b00000000;
    ram[1104] = 8'b00000000;
    ram[1103] = 8'b00000000;
    ram[1102] = 8'b00000000;
    ram[1101] = 8'b00000000;
    ram[1100] = 8'b00000000;
    ram[1099] = 8'b00000000;
    ram[1098] = 8'b00000000;
    ram[1097] = 8'b00000000;
    ram[1096] = 8'b00000000;
    ram[1095] = 8'b00000000;
    ram[1094] = 8'b00000000;
    ram[1093] = 8'b00000000;
    ram[1092] = 8'b00000000;
    ram[1091] = 8'b00000000;
    ram[1090] = 8'b00000000;
    ram[1089] = 8'b00000000;
    ram[1088] = 8'b00000000;
    ram[1087] = 8'b00000000;
    ram[1086] = 8'b00000000;
    ram[1085] = 8'b00000000;
    ram[1084] = 8'b00000000;
    ram[1083] = 8'b00000000;
    ram[1082] = 8'b00000000;
    ram[1081] = 8'b00000000;
    ram[1080] = 8'b00000000;
    ram[1079] = 8'b00000000;
    ram[1078] = 8'b00000000;
    ram[1077] = 8'b00000000;
    ram[1076] = 8'b00000000;
    ram[1075] = 8'b00000000;
    ram[1074] = 8'b00000000;
    ram[1073] = 8'b00000000;
    ram[1072] = 8'b00000000;
    ram[1071] = 8'b00000000;
    ram[1070] = 8'b00000000;
    ram[1069] = 8'b00000000;
    ram[1068] = 8'b00000000;
    ram[1067] = 8'b00000000;
    ram[1066] = 8'b00000000;
    ram[1065] = 8'b00000000;
    ram[1064] = 8'b00000000;
    ram[1063] = 8'b00000000;
    ram[1062] = 8'b00000000;
    ram[1061] = 8'b00000000;
    ram[1060] = 8'b00000000;
    ram[1059] = 8'b00000000;
    ram[1058] = 8'b00000000;
    ram[1057] = 8'b00000000;
    ram[1056] = 8'b00000000;
    ram[1055] = 8'b00000000;
    ram[1054] = 8'b00000000;
    ram[1053] = 8'b00000000;
    ram[1052] = 8'b00000000;
    ram[1051] = 8'b00000000;
    ram[1050] = 8'b00000000;
    ram[1049] = 8'b00000000;
    ram[1048] = 8'b00000000;
    ram[1047] = 8'b00000000;
    ram[1046] = 8'b00000000;
    ram[1045] = 8'b00000000;
    ram[1044] = 8'b00000000;
    ram[1043] = 8'b00000000;
    ram[1042] = 8'b00000000;
    ram[1041] = 8'b00000000;
    ram[1040] = 8'b00000000;
    ram[1039] = 8'b00000000;
    ram[1038] = 8'b00000000;
    ram[1037] = 8'b00000000;
    ram[1036] = 8'b00000000;
    ram[1035] = 8'b00000000;
    ram[1034] = 8'b00000000;
    ram[1033] = 8'b00000000;
    ram[1032] = 8'b00000000;
    ram[1031] = 8'b00000000;
    ram[1030] = 8'b00000000;
    ram[1029] = 8'b00000000;
    ram[1028] = 8'b00000000;
    ram[1027] = 8'b00000000;
    ram[1026] = 8'b00000000;
    ram[1025] = 8'b00000000;
    ram[1024] = 8'b00000000;
    ram[1023] = 8'b00000000;
    ram[1022] = 8'b00000000;
    ram[1021] = 8'b00000000;
    ram[1020] = 8'b00000000;
    ram[1019] = 8'b00000000;
    ram[1018] = 8'b00000000;
    ram[1017] = 8'b00000000;
    ram[1016] = 8'b00000000;
    ram[1015] = 8'b00000000;
    ram[1014] = 8'b00000000;
    ram[1013] = 8'b00000000;
    ram[1012] = 8'b00000000;
    ram[1011] = 8'b00000000;
    ram[1010] = 8'b00000000;
    ram[1009] = 8'b00000000;
    ram[1008] = 8'b00000000;
    ram[1007] = 8'b00000000;
    ram[1006] = 8'b00000000;
    ram[1005] = 8'b00000000;
    ram[1004] = 8'b00000000;
    ram[1003] = 8'b00000000;
    ram[1002] = 8'b00000000;
    ram[1001] = 8'b00000000;
    ram[1000] = 8'b00000000;
    ram[999] = 8'b00000000;
    ram[998] = 8'b00000000;
    ram[997] = 8'b00000000;
    ram[996] = 8'b00000000;
    ram[995] = 8'b00000000;
    ram[994] = 8'b00000000;
    ram[993] = 8'b00000000;
    ram[992] = 8'b00000000;
    ram[991] = 8'b00000000;
    ram[990] = 8'b00000000;
    ram[989] = 8'b00000000;
    ram[988] = 8'b00000000;
    ram[987] = 8'b00000000;
    ram[986] = 8'b00000000;
    ram[985] = 8'b00000000;
    ram[984] = 8'b00000000;
    ram[983] = 8'b00000000;
    ram[982] = 8'b00000000;
    ram[981] = 8'b00000000;
    ram[980] = 8'b00000000;
    ram[979] = 8'b00000000;
    ram[978] = 8'b00000000;
    ram[977] = 8'b00000000;
    ram[976] = 8'b00000000;
    ram[975] = 8'b00000000;
    ram[974] = 8'b00000000;
    ram[973] = 8'b00000000;
    ram[972] = 8'b00000000;
    ram[971] = 8'b00000000;
    ram[970] = 8'b00000000;
    ram[969] = 8'b00000000;
    ram[968] = 8'b00000000;
    ram[967] = 8'b00000000;
    ram[966] = 8'b00000000;
    ram[965] = 8'b00000000;
    ram[964] = 8'b00000000;
    ram[963] = 8'b00000000;
    ram[962] = 8'b00000000;
    ram[961] = 8'b00000000;
    ram[960] = 8'b00000000;
    ram[959] = 8'b00000000;
    ram[958] = 8'b00000000;
    ram[957] = 8'b00000000;
    ram[956] = 8'b00000000;
    ram[955] = 8'b00000000;
    ram[954] = 8'b00000000;
    ram[953] = 8'b00000000;
    ram[952] = 8'b00000000;
    ram[951] = 8'b00000000;
    ram[950] = 8'b00000000;
    ram[949] = 8'b00000000;
    ram[948] = 8'b00000000;
    ram[947] = 8'b00000000;
    ram[946] = 8'b00000000;
    ram[945] = 8'b00000000;
    ram[944] = 8'b00000000;
    ram[943] = 8'b00000000;
    ram[942] = 8'b00000000;
    ram[941] = 8'b00000000;
    ram[940] = 8'b00000000;
    ram[939] = 8'b00000000;
    ram[938] = 8'b00000000;
    ram[937] = 8'b00000000;
    ram[936] = 8'b00000000;
    ram[935] = 8'b00000000;
    ram[934] = 8'b00000000;
    ram[933] = 8'b00000000;
    ram[932] = 8'b00000000;
    ram[931] = 8'b00000000;
    ram[930] = 8'b00000000;
    ram[929] = 8'b00000000;
    ram[928] = 8'b00000000;
    ram[927] = 8'b00000000;
    ram[926] = 8'b00000000;
    ram[925] = 8'b00000000;
    ram[924] = 8'b00000000;
    ram[923] = 8'b00000000;
    ram[922] = 8'b00000000;
    ram[921] = 8'b00000000;
    ram[920] = 8'b00000000;
    ram[919] = 8'b00000000;
    ram[918] = 8'b00000000;
    ram[917] = 8'b00000000;
    ram[916] = 8'b00000000;
    ram[915] = 8'b00000000;
    ram[914] = 8'b00000000;
    ram[913] = 8'b00000000;
    ram[912] = 8'b00000000;
    ram[911] = 8'b00000000;
    ram[910] = 8'b00000000;
    ram[909] = 8'b00000000;
    ram[908] = 8'b00000000;
    ram[907] = 8'b00000000;
    ram[906] = 8'b00000000;
    ram[905] = 8'b00000000;
    ram[904] = 8'b00000000;
    ram[903] = 8'b00000000;
    ram[902] = 8'b00000000;
    ram[901] = 8'b00000000;
    ram[900] = 8'b00000000;
    ram[899] = 8'b00000000;
    ram[898] = 8'b00000000;
    ram[897] = 8'b00000000;
    ram[896] = 8'b00000000;
    ram[895] = 8'b00000000;
    ram[894] = 8'b00000000;
    ram[893] = 8'b00000000;
    ram[892] = 8'b00000000;
    ram[891] = 8'b00000000;
    ram[890] = 8'b00000000;
    ram[889] = 8'b00000000;
    ram[888] = 8'b00000000;
    ram[887] = 8'b00000000;
    ram[886] = 8'b00000000;
    ram[885] = 8'b00000000;
    ram[884] = 8'b00000000;
    ram[883] = 8'b00000000;
    ram[882] = 8'b00000000;
    ram[881] = 8'b00000000;
    ram[880] = 8'b00000000;
    ram[879] = 8'b00000000;
    ram[878] = 8'b00000000;
    ram[877] = 8'b00000000;
    ram[876] = 8'b00000000;
    ram[875] = 8'b00000000;
    ram[874] = 8'b00000000;
    ram[873] = 8'b00000000;
    ram[872] = 8'b00000000;
    ram[871] = 8'b00000000;
    ram[870] = 8'b00000000;
    ram[869] = 8'b00000000;
    ram[868] = 8'b00000000;
    ram[867] = 8'b00000000;
    ram[866] = 8'b00000000;
    ram[865] = 8'b00000000;
    ram[864] = 8'b00000000;
    ram[863] = 8'b00000000;
    ram[862] = 8'b00000000;
    ram[861] = 8'b00000000;
    ram[860] = 8'b00000000;
    ram[859] = 8'b00000000;
    ram[858] = 8'b00000000;
    ram[857] = 8'b00000000;
    ram[856] = 8'b00000000;
    ram[855] = 8'b00000000;
    ram[854] = 8'b00000000;
    ram[853] = 8'b00000000;
    ram[852] = 8'b00000000;
    ram[851] = 8'b00000000;
    ram[850] = 8'b00000000;
    ram[849] = 8'b00000000;
    ram[848] = 8'b00000000;
    ram[847] = 8'b00000000;
    ram[846] = 8'b00000000;
    ram[845] = 8'b00000000;
    ram[844] = 8'b00000000;
    ram[843] = 8'b00000000;
    ram[842] = 8'b00000000;
    ram[841] = 8'b00000000;
    ram[840] = 8'b00000000;
    ram[839] = 8'b00000000;
    ram[838] = 8'b00000000;
    ram[837] = 8'b00000000;
    ram[836] = 8'b00000000;
    ram[835] = 8'b00000000;
    ram[834] = 8'b00000000;
    ram[833] = 8'b00000000;
    ram[832] = 8'b00000000;
    ram[831] = 8'b00000000;
    ram[830] = 8'b00000000;
    ram[829] = 8'b00000000;
    ram[828] = 8'b00000000;
    ram[827] = 8'b00000000;
    ram[826] = 8'b00000000;
    ram[825] = 8'b00000000;
    ram[824] = 8'b00000000;
    ram[823] = 8'b00000000;
    ram[822] = 8'b00000000;
    ram[821] = 8'b00000000;
    ram[820] = 8'b00000000;
    ram[819] = 8'b00000000;
    ram[818] = 8'b00000000;
    ram[817] = 8'b00000000;
    ram[816] = 8'b00000000;
    ram[815] = 8'b00000000;
    ram[814] = 8'b00000000;
    ram[813] = 8'b00000000;
    ram[812] = 8'b00000000;
    ram[811] = 8'b00000000;
    ram[810] = 8'b00000000;
    ram[809] = 8'b00000000;
    ram[808] = 8'b00000000;
    ram[807] = 8'b00000000;
    ram[806] = 8'b00000000;
    ram[805] = 8'b00000000;
    ram[804] = 8'b00000000;
    ram[803] = 8'b00000000;
    ram[802] = 8'b00000000;
    ram[801] = 8'b00000000;
    ram[800] = 8'b00000000;
    ram[799] = 8'b00000000;
    ram[798] = 8'b00000000;
    ram[797] = 8'b00000000;
    ram[796] = 8'b00000000;
    ram[795] = 8'b00000000;
    ram[794] = 8'b00000000;
    ram[793] = 8'b00000000;
    ram[792] = 8'b00000000;
    ram[791] = 8'b00000000;
    ram[790] = 8'b00000000;
    ram[789] = 8'b00000000;
    ram[788] = 8'b00000000;
    ram[787] = 8'b00000000;
    ram[786] = 8'b00000000;
    ram[785] = 8'b00000000;
    ram[784] = 8'b00000000;
    ram[783] = 8'b00000000;
    ram[782] = 8'b00000000;
    ram[781] = 8'b00000000;
    ram[780] = 8'b00000000;
    ram[779] = 8'b00000000;
    ram[778] = 8'b00000000;
    ram[777] = 8'b00000000;
    ram[776] = 8'b00000000;
    ram[775] = 8'b00000000;
    ram[774] = 8'b00000000;
    ram[773] = 8'b00000000;
    ram[772] = 8'b00000000;
    ram[771] = 8'b00000000;
    ram[770] = 8'b00000000;
    ram[769] = 8'b00000000;
    ram[768] = 8'b00000000;
    ram[767] = 8'b00000000;
    ram[766] = 8'b00000000;
    ram[765] = 8'b00000000;
    ram[764] = 8'b00000000;
    ram[763] = 8'b00000000;
    ram[762] = 8'b00000000;
    ram[761] = 8'b00000000;
    ram[760] = 8'b00000000;
    ram[759] = 8'b00000000;
    ram[758] = 8'b00000000;
    ram[757] = 8'b00000000;
    ram[756] = 8'b00000000;
    ram[755] = 8'b00000000;
    ram[754] = 8'b00000000;
    ram[753] = 8'b00000000;
    ram[752] = 8'b00000000;
    ram[751] = 8'b00000000;
    ram[750] = 8'b00000000;
    ram[749] = 8'b00000000;
    ram[748] = 8'b00000000;
    ram[747] = 8'b00000000;
    ram[746] = 8'b00000000;
    ram[745] = 8'b00000000;
    ram[744] = 8'b00000000;
    ram[743] = 8'b00000000;
    ram[742] = 8'b00000000;
    ram[741] = 8'b00000000;
    ram[740] = 8'b00000000;
    ram[739] = 8'b00000000;
    ram[738] = 8'b00000000;
    ram[737] = 8'b00000000;
    ram[736] = 8'b00000000;
    ram[735] = 8'b00000000;
    ram[734] = 8'b00000000;
    ram[733] = 8'b00000000;
    ram[732] = 8'b00000000;
    ram[731] = 8'b00000000;
    ram[730] = 8'b00000000;
    ram[729] = 8'b00000000;
    ram[728] = 8'b00000000;
    ram[727] = 8'b00000000;
    ram[726] = 8'b00000000;
    ram[725] = 8'b00000000;
    ram[724] = 8'b00000000;
    ram[723] = 8'b00000000;
    ram[722] = 8'b00000000;
    ram[721] = 8'b00000000;
    ram[720] = 8'b00000000;
    ram[719] = 8'b00000000;
    ram[718] = 8'b00000000;
    ram[717] = 8'b00000000;
    ram[716] = 8'b00000000;
    ram[715] = 8'b00000000;
    ram[714] = 8'b00000000;
    ram[713] = 8'b00000000;
    ram[712] = 8'b00000000;
    ram[711] = 8'b00000000;
    ram[710] = 8'b00000000;
    ram[709] = 8'b00000000;
    ram[708] = 8'b00000000;
    ram[707] = 8'b00000000;
    ram[706] = 8'b00000000;
    ram[705] = 8'b00000000;
    ram[704] = 8'b00000000;
    ram[703] = 8'b00000000;
    ram[702] = 8'b00000000;
    ram[701] = 8'b00000000;
    ram[700] = 8'b00000000;
    ram[699] = 8'b00000000;
    ram[698] = 8'b00000000;
    ram[697] = 8'b00000000;
    ram[696] = 8'b00000000;
    ram[695] = 8'b00000000;
    ram[694] = 8'b00000000;
    ram[693] = 8'b00000000;
    ram[692] = 8'b00000000;
    ram[691] = 8'b00000000;
    ram[690] = 8'b00000000;
    ram[689] = 8'b00000000;
    ram[688] = 8'b00000000;
    ram[687] = 8'b00000000;
    ram[686] = 8'b00000000;
    ram[685] = 8'b00000000;
    ram[684] = 8'b00000000;
    ram[683] = 8'b00000000;
    ram[682] = 8'b00000000;
    ram[681] = 8'b00000000;
    ram[680] = 8'b00000000;
    ram[679] = 8'b00000000;
    ram[678] = 8'b00000000;
    ram[677] = 8'b00000000;
    ram[676] = 8'b00000000;
    ram[675] = 8'b00000000;
    ram[674] = 8'b00000000;
    ram[673] = 8'b00000000;
    ram[672] = 8'b00000000;
    ram[671] = 8'b00000000;
    ram[670] = 8'b00000000;
    ram[669] = 8'b00000000;
    ram[668] = 8'b00000000;
    ram[667] = 8'b00000000;
    ram[666] = 8'b00000000;
    ram[665] = 8'b00000000;
    ram[664] = 8'b00000000;
    ram[663] = 8'b00000000;
    ram[662] = 8'b00000000;
    ram[661] = 8'b00000000;
    ram[660] = 8'b00000000;
    ram[659] = 8'b00000000;
    ram[658] = 8'b00000000;
    ram[657] = 8'b00000000;
    ram[656] = 8'b00000000;
    ram[655] = 8'b00000000;
    ram[654] = 8'b00000000;
    ram[653] = 8'b00000000;
    ram[652] = 8'b00000000;
    ram[651] = 8'b00000000;
    ram[650] = 8'b00000000;
    ram[649] = 8'b00000000;
    ram[648] = 8'b00000000;
    ram[647] = 8'b00000000;
    ram[646] = 8'b00000000;
    ram[645] = 8'b00000000;
    ram[644] = 8'b00000000;
    ram[643] = 8'b00000000;
    ram[642] = 8'b00000000;
    ram[641] = 8'b00000000;
    ram[640] = 8'b00000000;
    ram[639] = 8'b00000000;
    ram[638] = 8'b00000000;
    ram[637] = 8'b00000000;
    ram[636] = 8'b00000000;
    ram[635] = 8'b00000000;
    ram[634] = 8'b00000000;
    ram[633] = 8'b00000000;
    ram[632] = 8'b00000000;
    ram[631] = 8'b00000000;
    ram[630] = 8'b00000000;
    ram[629] = 8'b00000000;
    ram[628] = 8'b00000000;
    ram[627] = 8'b00000000;
    ram[626] = 8'b00000000;
    ram[625] = 8'b00000000;
    ram[624] = 8'b00000000;
    ram[623] = 8'b00000000;
    ram[622] = 8'b00000000;
    ram[621] = 8'b00000000;
    ram[620] = 8'b00000000;
    ram[619] = 8'b00000000;
    ram[618] = 8'b00000000;
    ram[617] = 8'b00000000;
    ram[616] = 8'b00000000;
    ram[615] = 8'b00000000;
    ram[614] = 8'b00000000;
    ram[613] = 8'b00000000;
    ram[612] = 8'b00000000;
    ram[611] = 8'b00000000;
    ram[610] = 8'b00000000;
    ram[609] = 8'b00000000;
    ram[608] = 8'b00000000;
    ram[607] = 8'b00000000;
    ram[606] = 8'b00000000;
    ram[605] = 8'b00000000;
    ram[604] = 8'b00000000;
    ram[603] = 8'b00000000;
    ram[602] = 8'b00000000;
    ram[601] = 8'b00000000;
    ram[600] = 8'b00000000;
    ram[599] = 8'b00000000;
    ram[598] = 8'b00000000;
    ram[597] = 8'b00000000;
    ram[596] = 8'b00000000;
    ram[595] = 8'b00000000;
    ram[594] = 8'b00000000;
    ram[593] = 8'b00000000;
    ram[592] = 8'b00000000;
    ram[591] = 8'b00000000;
    ram[590] = 8'b00000000;
    ram[589] = 8'b00000000;
    ram[588] = 8'b00000000;
    ram[587] = 8'b00000000;
    ram[586] = 8'b00000000;
    ram[585] = 8'b00000000;
    ram[584] = 8'b00000000;
    ram[583] = 8'b00000000;
    ram[582] = 8'b00000000;
    ram[581] = 8'b00000000;
    ram[580] = 8'b00000000;
    ram[579] = 8'b00000000;
    ram[578] = 8'b00000000;
    ram[577] = 8'b00000000;
    ram[576] = 8'b00000000;
    ram[575] = 8'b00000000;
    ram[574] = 8'b00000000;
    ram[573] = 8'b00000000;
    ram[572] = 8'b00000000;
    ram[571] = 8'b00000000;
    ram[570] = 8'b00000000;
    ram[569] = 8'b00000000;
    ram[568] = 8'b00000000;
    ram[567] = 8'b00000000;
    ram[566] = 8'b00000000;
    ram[565] = 8'b00000000;
    ram[564] = 8'b00000000;
    ram[563] = 8'b00000000;
    ram[562] = 8'b00000000;
    ram[561] = 8'b00000000;
    ram[560] = 8'b00000000;
    ram[559] = 8'b00000000;
    ram[558] = 8'b00000000;
    ram[557] = 8'b00000000;
    ram[556] = 8'b00000000;
    ram[555] = 8'b00000000;
    ram[554] = 8'b00000000;
    ram[553] = 8'b00000000;
    ram[552] = 8'b00000000;
    ram[551] = 8'b00000000;
    ram[550] = 8'b00000000;
    ram[549] = 8'b00000000;
    ram[548] = 8'b00000000;
    ram[547] = 8'b00000000;
    ram[546] = 8'b00000000;
    ram[545] = 8'b00000000;
    ram[544] = 8'b00000000;
    ram[543] = 8'b00000000;
    ram[542] = 8'b00000000;
    ram[541] = 8'b00000000;
    ram[540] = 8'b00000000;
    ram[539] = 8'b00000000;
    ram[538] = 8'b00000000;
    ram[537] = 8'b00000000;
    ram[536] = 8'b00000000;
    ram[535] = 8'b00000000;
    ram[534] = 8'b00000000;
    ram[533] = 8'b00000000;
    ram[532] = 8'b00000000;
    ram[531] = 8'b00000000;
    ram[530] = 8'b00000000;
    ram[529] = 8'b00000000;
    ram[528] = 8'b00000000;
    ram[527] = 8'b00000000;
    ram[526] = 8'b00000000;
    ram[525] = 8'b00000000;
    ram[524] = 8'b00000000;
    ram[523] = 8'b00000000;
    ram[522] = 8'b00000000;
    ram[521] = 8'b00000000;
    ram[520] = 8'b00000000;
    ram[519] = 8'b00000000;
    ram[518] = 8'b00000000;
    ram[517] = 8'b00000000;
    ram[516] = 8'b00000000;
    ram[515] = 8'b00000000;
    ram[514] = 8'b00000000;
    ram[513] = 8'b00000000;
    ram[512] = 8'b00000000;
    ram[511] = 8'b00000000;
    ram[510] = 8'b00000000;
    ram[509] = 8'b00000000;
    ram[508] = 8'b00000000;
    ram[507] = 8'b00000000;
    ram[506] = 8'b00000000;
    ram[505] = 8'b00000000;
    ram[504] = 8'b00000000;
    ram[503] = 8'b00000000;
    ram[502] = 8'b00000000;
    ram[501] = 8'b00000000;
    ram[500] = 8'b00000000;
    ram[499] = 8'b00000000;
    ram[498] = 8'b00000000;
    ram[497] = 8'b00000000;
    ram[496] = 8'b00000000;
    ram[495] = 8'b00000000;
    ram[494] = 8'b00000000;
    ram[493] = 8'b00000000;
    ram[492] = 8'b00000000;
    ram[491] = 8'b00000000;
    ram[490] = 8'b00000000;
    ram[489] = 8'b00000000;
    ram[488] = 8'b00000000;
    ram[487] = 8'b00000000;
    ram[486] = 8'b00000000;
    ram[485] = 8'b00000000;
    ram[484] = 8'b00000000;
    ram[483] = 8'b00000000;
    ram[482] = 8'b00000000;
    ram[481] = 8'b00000000;
    ram[480] = 8'b00000000;
    ram[479] = 8'b00000000;
    ram[478] = 8'b00000000;
    ram[477] = 8'b00000000;
    ram[476] = 8'b00000000;
    ram[475] = 8'b00000000;
    ram[474] = 8'b00000000;
    ram[473] = 8'b00000000;
    ram[472] = 8'b00000000;
    ram[471] = 8'b00000000;
    ram[470] = 8'b00000000;
    ram[469] = 8'b00000000;
    ram[468] = 8'b00000000;
    ram[467] = 8'b00000000;
    ram[466] = 8'b00000000;
    ram[465] = 8'b00000000;
    ram[464] = 8'b00000000;
    ram[463] = 8'b00000000;
    ram[462] = 8'b00000000;
    ram[461] = 8'b00000000;
    ram[460] = 8'b00000000;
    ram[459] = 8'b00000000;
    ram[458] = 8'b00000000;
    ram[457] = 8'b00000000;
    ram[456] = 8'b00000000;
    ram[455] = 8'b00000000;
    ram[454] = 8'b00000000;
    ram[453] = 8'b00000000;
    ram[452] = 8'b00000000;
    ram[451] = 8'b00000000;
    ram[450] = 8'b00000000;
    ram[449] = 8'b00000000;
    ram[448] = 8'b00000000;
    ram[447] = 8'b00000000;
    ram[446] = 8'b00000000;
    ram[445] = 8'b00000000;
    ram[444] = 8'b00000000;
    ram[443] = 8'b00000000;
    ram[442] = 8'b00000000;
    ram[441] = 8'b00000000;
    ram[440] = 8'b00000000;
    ram[439] = 8'b00000000;
    ram[438] = 8'b00000000;
    ram[437] = 8'b00000000;
    ram[436] = 8'b00000000;
    ram[435] = 8'b00000000;
    ram[434] = 8'b00000000;
    ram[433] = 8'b00000000;
    ram[432] = 8'b00000000;
    ram[431] = 8'b00000000;
    ram[430] = 8'b00000000;
    ram[429] = 8'b00000000;
    ram[428] = 8'b00000000;
    ram[427] = 8'b00000000;
    ram[426] = 8'b00000000;
    ram[425] = 8'b00000000;
    ram[424] = 8'b00000000;
    ram[423] = 8'b00000000;
    ram[422] = 8'b00000000;
    ram[421] = 8'b00000000;
    ram[420] = 8'b00000000;
    ram[419] = 8'b00000000;
    ram[418] = 8'b00000000;
    ram[417] = 8'b00000000;
    ram[416] = 8'b00000000;
    ram[415] = 8'b00000000;
    ram[414] = 8'b00000000;
    ram[413] = 8'b00000000;
    ram[412] = 8'b00000000;
    ram[411] = 8'b00000000;
    ram[410] = 8'b00000000;
    ram[409] = 8'b00000000;
    ram[408] = 8'b00000000;
    ram[407] = 8'b00000000;
    ram[406] = 8'b00000000;
    ram[405] = 8'b00000000;
    ram[404] = 8'b00000000;
    ram[403] = 8'b00000000;
    ram[402] = 8'b00000000;
    ram[401] = 8'b00000000;
    ram[400] = 8'b00000000;
    ram[399] = 8'b00000000;
    ram[398] = 8'b00000000;
    ram[397] = 8'b00000000;
    ram[396] = 8'b00000000;
    ram[395] = 8'b00000000;
    ram[394] = 8'b00000000;
    ram[393] = 8'b00000000;
    ram[392] = 8'b00000000;
    ram[391] = 8'b00000000;
    ram[390] = 8'b00000000;
    ram[389] = 8'b00000000;
    ram[388] = 8'b00000000;
    ram[387] = 8'b00000000;
    ram[386] = 8'b00000000;
    ram[385] = 8'b00000000;
    ram[384] = 8'b00000000;
    ram[383] = 8'b00000000;
    ram[382] = 8'b00000000;
    ram[381] = 8'b00000000;
    ram[380] = 8'b00000000;
    ram[379] = 8'b00000000;
    ram[378] = 8'b00000000;
    ram[377] = 8'b00000000;
    ram[376] = 8'b00000000;
    ram[375] = 8'b00000000;
    ram[374] = 8'b00000000;
    ram[373] = 8'b00000000;
    ram[372] = 8'b00000000;
    ram[371] = 8'b00000000;
    ram[370] = 8'b00000000;
    ram[369] = 8'b00000000;
    ram[368] = 8'b00000000;
    ram[367] = 8'b00000000;
    ram[366] = 8'b00000000;
    ram[365] = 8'b00000000;
    ram[364] = 8'b00000000;
    ram[363] = 8'b00000000;
    ram[362] = 8'b00000000;
    ram[361] = 8'b00000000;
    ram[360] = 8'b00000000;
    ram[359] = 8'b00000000;
    ram[358] = 8'b00000000;
    ram[357] = 8'b00000000;
    ram[356] = 8'b00000000;
    ram[355] = 8'b00000000;
    ram[354] = 8'b00000000;
    ram[353] = 8'b00000000;
    ram[352] = 8'b00000000;
    ram[351] = 8'b00000000;
    ram[350] = 8'b00000000;
    ram[349] = 8'b00000000;
    ram[348] = 8'b00000000;
    ram[347] = 8'b00000000;
    ram[346] = 8'b00000000;
    ram[345] = 8'b00000000;
    ram[344] = 8'b00000000;
    ram[343] = 8'b00000000;
    ram[342] = 8'b00000000;
    ram[341] = 8'b00000000;
    ram[340] = 8'b00000000;
    ram[339] = 8'b00000000;
    ram[338] = 8'b00000000;
    ram[337] = 8'b00000000;
    ram[336] = 8'b00000000;
    ram[335] = 8'b00000000;
    ram[334] = 8'b00000000;
    ram[333] = 8'b00000000;
    ram[332] = 8'b00000000;
    ram[331] = 8'b00000000;
    ram[330] = 8'b00000000;
    ram[329] = 8'b00000000;
    ram[328] = 8'b00000000;
    ram[327] = 8'b00000000;
    ram[326] = 8'b00000000;
    ram[325] = 8'b00000000;
    ram[324] = 8'b00000000;
    ram[323] = 8'b00000000;
    ram[322] = 8'b00000000;
    ram[321] = 8'b00000000;
    ram[320] = 8'b00000000;
    ram[319] = 8'b00000000;
    ram[318] = 8'b00000000;
    ram[317] = 8'b00000000;
    ram[316] = 8'b00000000;
    ram[315] = 8'b00000000;
    ram[314] = 8'b00000000;
    ram[313] = 8'b00000000;
    ram[312] = 8'b00000000;
    ram[311] = 8'b00000000;
    ram[310] = 8'b00000000;
    ram[309] = 8'b00000000;
    ram[308] = 8'b00000000;
    ram[307] = 8'b00000000;
    ram[306] = 8'b00000000;
    ram[305] = 8'b00000000;
    ram[304] = 8'b00000000;
    ram[303] = 8'b00000000;
    ram[302] = 8'b00000000;
    ram[301] = 8'b00000000;
    ram[300] = 8'b00000000;
    ram[299] = 8'b00000000;
    ram[298] = 8'b00000000;
    ram[297] = 8'b00000000;
    ram[296] = 8'b00000000;
    ram[295] = 8'b00000000;
    ram[294] = 8'b00000000;
    ram[293] = 8'b00000000;
    ram[292] = 8'b00000000;
    ram[291] = 8'b00000000;
    ram[290] = 8'b00000000;
    ram[289] = 8'b00000000;
    ram[288] = 8'b00000000;
    ram[287] = 8'b00000000;
    ram[286] = 8'b00000000;
    ram[285] = 8'b00000000;
    ram[284] = 8'b00000000;
    ram[283] = 8'b00000000;
    ram[282] = 8'b00000000;
    ram[281] = 8'b00000000;
    ram[280] = 8'b00000000;
    ram[279] = 8'b00000000;
    ram[278] = 8'b00000000;
    ram[277] = 8'b00000000;
    ram[276] = 8'b00000000;
    ram[275] = 8'b00000000;
    ram[274] = 8'b00000000;
    ram[273] = 8'b00000000;
    ram[272] = 8'b00000000;
    ram[271] = 8'b00000000;
    ram[270] = 8'b00000000;
    ram[269] = 8'b00000000;
    ram[268] = 8'b00000000;
    ram[267] = 8'b00000000;
    ram[266] = 8'b00000000;
    ram[265] = 8'b00000000;
    ram[264] = 8'b00000000;
    ram[263] = 8'b00000000;
    ram[262] = 8'b00000000;
    ram[261] = 8'b00000000;
    ram[260] = 8'b00000000;
    ram[259] = 8'b00000000;
    ram[258] = 8'b00000000;
    ram[257] = 8'b00000000;
    ram[256] = 8'b00000000;
    ram[255] = 8'b00000000;
    ram[254] = 8'b00000000;
    ram[253] = 8'b00000000;
    ram[252] = 8'b00000000;
    ram[251] = 8'b00000000;
    ram[250] = 8'b00000000;
    ram[249] = 8'b00000000;
    ram[248] = 8'b00000000;
    ram[247] = 8'b00000000;
    ram[246] = 8'b00000000;
    ram[245] = 8'b00000000;
    ram[244] = 8'b00000000;
    ram[243] = 8'b00000000;
    ram[242] = 8'b00000000;
    ram[241] = 8'b00000000;
    ram[240] = 8'b00000000;
    ram[239] = 8'b00000000;
    ram[238] = 8'b00000000;
    ram[237] = 8'b00000000;
    ram[236] = 8'b00000000;
    ram[235] = 8'b00000000;
    ram[234] = 8'b00000000;
    ram[233] = 8'b00000000;
    ram[232] = 8'b00000000;
    ram[231] = 8'b00000000;
    ram[230] = 8'b00000000;
    ram[229] = 8'b00000000;
    ram[228] = 8'b00000000;
    ram[227] = 8'b00000000;
    ram[226] = 8'b00000000;
    ram[225] = 8'b00000000;
    ram[224] = 8'b00000000;
    ram[223] = 8'b00000000;
    ram[222] = 8'b00000000;
    ram[221] = 8'b00000000;
    ram[220] = 8'b00000000;
    ram[219] = 8'b00000000;
    ram[218] = 8'b00000000;
    ram[217] = 8'b00000000;
    ram[216] = 8'b00000000;
    ram[215] = 8'b00000000;
    ram[214] = 8'b00000000;
    ram[213] = 8'b00000000;
    ram[212] = 8'b00000000;
    ram[211] = 8'b00000000;
    ram[210] = 8'b00000000;
    ram[209] = 8'b00000000;
    ram[208] = 8'b00000000;
    ram[207] = 8'b00000000;
    ram[206] = 8'b00000000;
    ram[205] = 8'b00000000;
    ram[204] = 8'b00000000;
    ram[203] = 8'b00000000;
    ram[202] = 8'b00000000;
    ram[201] = 8'b00000000;
    ram[200] = 8'b00000000;
    ram[199] = 8'b00000000;
    ram[198] = 8'b00000000;
    ram[197] = 8'b00000000;
    ram[196] = 8'b00000000;
    ram[195] = 8'b00000000;
    ram[194] = 8'b00000000;
    ram[193] = 8'b00000000;
    ram[192] = 8'b00000000;
    ram[191] = 8'b00000000;
    ram[190] = 8'b00000000;
    ram[189] = 8'b00000000;
    ram[188] = 8'b00000000;
    ram[187] = 8'b00000000;
    ram[186] = 8'b00000000;
    ram[185] = 8'b00000000;
    ram[184] = 8'b00000000;
    ram[183] = 8'b00000000;
    ram[182] = 8'b00000000;
    ram[181] = 8'b00000000;
    ram[180] = 8'b00000000;
    ram[179] = 8'b00000000;
    ram[178] = 8'b00000000;
    ram[177] = 8'b00000000;
    ram[176] = 8'b00000000;
    ram[175] = 8'b00000000;
    ram[174] = 8'b00000000;
    ram[173] = 8'b00000000;
    ram[172] = 8'b00000000;
    ram[171] = 8'b00000000;
    ram[170] = 8'b00000000;
    ram[169] = 8'b00000000;
    ram[168] = 8'b00000000;
    ram[167] = 8'b00000000;
    ram[166] = 8'b00000000;
    ram[165] = 8'b00000000;
    ram[164] = 8'b00000000;
    ram[163] = 8'b00000000;
    ram[162] = 8'b00000000;
    ram[161] = 8'b00000000;
    ram[160] = 8'b00000000;
    ram[159] = 8'b00000000;
    ram[158] = 8'b00000000;
    ram[157] = 8'b00000000;
    ram[156] = 8'b00000000;
    ram[155] = 8'b00000000;
    ram[154] = 8'b00000000;
    ram[153] = 8'b00000000;
    ram[152] = 8'b00000000;
    ram[151] = 8'b00000000;
    ram[150] = 8'b00000000;
    ram[149] = 8'b00000000;
    ram[148] = 8'b00000000;
    ram[147] = 8'b00000000;
    ram[146] = 8'b00000000;
    ram[145] = 8'b00000000;
    ram[144] = 8'b00000000;
    ram[143] = 8'b00000000;
    ram[142] = 8'b00000000;
    ram[141] = 8'b00000000;
    ram[140] = 8'b00000000;
    ram[139] = 8'b00000000;
    ram[138] = 8'b00000000;
    ram[137] = 8'b00000000;
    ram[136] = 8'b00000000;
    ram[135] = 8'b00000000;
    ram[134] = 8'b00000000;
    ram[133] = 8'b00000000;
    ram[132] = 8'b00000000;
    ram[131] = 8'b00000000;
    ram[130] = 8'b00000000;
    ram[129] = 8'b00000000;
    ram[128] = 8'b00000000;
    ram[127] = 8'b00000000;
    ram[126] = 8'b00000000;
    ram[125] = 8'b00000000;
    ram[124] = 8'b00000000;
    ram[123] = 8'b00000000;
    ram[122] = 8'b00000000;
    ram[121] = 8'b00000000;
    ram[120] = 8'b00000000;
    ram[119] = 8'b00000000;
    ram[118] = 8'b00000000;
    ram[117] = 8'b00000000;
    ram[116] = 8'b00000000;
    ram[115] = 8'b00000000;
    ram[114] = 8'b00000000;
    ram[113] = 8'b00000000;
    ram[112] = 8'b00000000;
    ram[111] = 8'b00000000;
    ram[110] = 8'b00000000;
    ram[109] = 8'b00000000;
    ram[108] = 8'b00000000;
    ram[107] = 8'b00000000;
    ram[106] = 8'b00000000;
    ram[105] = 8'b00000000;
    ram[104] = 8'b00000000;
    ram[103] = 8'b00000000;
    ram[102] = 8'b00000000;
    ram[101] = 8'b00000000;
    ram[100] = 8'b00000000;
    ram[99] = 8'b00000000;
    ram[98] = 8'b00000000;
    ram[97] = 8'b00000000;
    ram[96] = 8'b00000000;
    ram[95] = 8'b00000000;
    ram[94] = 8'b00000000;
    ram[93] = 8'b00000000;
    ram[92] = 8'b00000000;
    ram[91] = 8'b00000000;
    ram[90] = 8'b00000000;
    ram[89] = 8'b00000000;
    ram[88] = 8'b00000000;
    ram[87] = 8'b00000000;
    ram[86] = 8'b00000000;
    ram[85] = 8'b00000000;
    ram[84] = 8'b00000000;
    ram[83] = 8'b00000000;
    ram[82] = 8'b00000000;
    ram[81] = 8'b00000000;
    ram[80] = 8'b00000000;
    ram[79] = 8'b00000000;
    ram[78] = 8'b00000000;
    ram[77] = 8'b00000000;
    ram[76] = 8'b00000000;
    ram[75] = 8'b00000000;
    ram[74] = 8'b00000000;
    ram[73] = 8'b00000000;
    ram[72] = 8'b00000000;
    ram[71] = 8'b00000000;
    ram[70] = 8'b00000000;
    ram[69] = 8'b00000000;
    ram[68] = 8'b00000000;
    ram[67] = 8'b00000000;
    ram[66] = 8'b00000000;
    ram[65] = 8'b00000000;
    ram[64] = 8'b00000000;
    ram[63] = 8'b00000000;
    ram[62] = 8'b00000000;
    ram[61] = 8'b00000000;
    ram[60] = 8'b00000000;
    ram[59] = 8'b00000000;
    ram[58] = 8'b00000000;
    ram[57] = 8'b00000000;
    ram[56] = 8'b00000000;
    ram[55] = 8'b00000000;
    ram[54] = 8'b00000000;
    ram[53] = 8'b00000000;
    ram[52] = 8'b00000000;
    ram[51] = 8'b00000000;
    ram[50] = 8'b00000000;
    ram[49] = 8'b00000000;
    ram[48] = 8'b00000000;
    ram[47] = 8'b00000000;
    ram[46] = 8'b00000000;
    ram[45] = 8'b00000000;
    ram[44] = 8'b00000000;
    ram[43] = 8'b00000000;
    ram[42] = 8'b00000000;
    ram[41] = 8'b00000000;
    ram[40] = 8'b00000000;
    ram[39] = 8'b00000000;
    ram[38] = 8'b00000000;
    ram[37] = 8'b00000000;
    ram[36] = 8'b00000000;
    ram[35] = 8'b00000000;
    ram[34] = 8'b00000000;
    ram[33] = 8'b00000000;
    ram[32] = 8'b00000000;
    ram[31] = 8'b00000000;
    ram[30] = 8'b00000000;
    ram[29] = 8'b00000000;
    ram[28] = 8'b00000000;
    ram[27] = 8'b00000000;
    ram[26] = 8'b00000000;
    ram[25] = 8'b00000000;
    ram[24] = 8'b00000000;
    ram[23] = 8'b00000000;
    ram[22] = 8'b00000000;
    ram[21] = 8'b00000000;
    ram[20] = 8'b00000000;
    ram[19] = 8'b00000000;
    ram[18] = 8'b00000000;
    ram[17] = 8'b00000000;
    ram[16] = 8'b00000000;
    ram[15] = 8'b00000000;
    ram[14] = 8'b00000000;
    ram[13] = 8'b00000000;
    ram[12] = 8'b00000000;
    ram[11] = 8'b00000000;
    ram[10] = 8'b00000000;
    ram[9] = 8'b00000000;
    ram[8] = 8'b00000000;
    ram[7] = 8'b00000000;
    ram[6] = 8'b00000000;
    ram[5] = 8'b00000000;
    ram[4] = 8'b00000000;
    ram[3] = 8'b00000000;
    ram[2] = 8'b00000000;
    ram[1] = 8'b00000000;
    ram[0] = 8'b00000000;
    end
  always @(posedge clk)
    if (1'b1)
      n1010 <= ram[addr];
  always @(posedge clk)
    if (n989)
      ram[addr] <= data_in;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/ram_sync.vhdl:70:21  */
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/ram_sync.vhdl:70:21  */
endmodule

module b8008_Bstructural_25000000
  (input  clk_in,
   input  reset,
   input  run_enable,
   output phi1_out,
   output phi2_out,
   output phi1_rising_out,
   output phi1_falling_out,
   output phi2_rising_out,
   output phi2_falling_out,
   input  [7:0] data_bus_in,
   output [7:0] data_bus_out,
   output data_bus_oe,
   output sync_out,
   output s0_out,
   output s1_out,
   output s2_out,
   input  ready_in,
   input  interrupt,
   output [7:0] debug_reg_a,
   output [7:0] debug_reg_b,
   output [7:0] debug_reg_c,
   output [7:0] debug_reg_d,
   output [7:0] debug_reg_e,
   output [7:0] debug_reg_h,
   output [7:0] debug_reg_l,
   output [1:0] debug_cycle,
   output [13:0] debug_pc,
   output [7:0] debug_ir,
   output debug_needs_address,
   output debug_int_pending,
   output [1:0] cycle_type,
   output debug_flag_carry,
   output debug_flag_zero,
   output debug_flag_sign,
   output debug_flag_parity,
   output debug_state_half);
  wire phi1;
  wire phi2;
  wire phi1_rising;
  wire phi1_falling;
  wire phi2_rising;
  wire phi2_falling;
  wire [7:0] internal_bus;
  wire [7:0] mem_mux_bus_out;
  wire mem_mux_bus_oe;
  wire [7:0] temp_reg_bus_out;
  wire temp_reg_bus_oe;
  wire [7:0] alu_bus_out;
  wire alu_bus_oe;
  wire [7:0] cond_flags_bus_out;
  wire cond_flags_bus_oe;
  wire [7:0] ir_bus_out;
  wire ir_bus_oe;
  wire [7:0] io_buffer_bus_out;
  wire io_buffer_bus_oe;
  wire state_t1;
  wire state_t2;
  wire state_t3;
  wire state_t4;
  wire state_t5;
  wire state_t1i;
  wire state_stopped;
  wire state_half;
  wire status_s0;
  wire status_s1;
  wire status_s2;
  wire sync;
  wire [1:0] current_cycle;
  wire [1:0] next_cycle;
  wire advance_state;
  wire cycle_done;
  wire instr_is_hlt_flag;
  wire [7:0] instr_byte;
  wire instr_needs_immediate;
  wire instr_needs_address;
  wire instr_is_io;
  wire instr_is_write;
  wire [2:0] instr_sss_field;
  wire [2:0] instr_ddd_field;
  wire instr_is_alu;
  wire instr_is_call;
  wire instr_is_ret;
  wire instr_is_rst;
  wire instr_is_hlt;
  wire instr_writes_reg;
  wire instr_reads_reg;
  wire instr_is_mem_indirect;
  wire instr_uses_temp_regs;
  wire instr_is_inr_dcr;
  wire instr_is_rotate;
  wire instr_needs_t4t5;
  wire [2:0] rst_vector;
  wire [1:0] condition_code;
  wire test_true;
  wire eval_condition;
  wire transition_to_stopped;
  wire condition_met;
  wire flag_carry;
  wire flag_zero;
  wire flag_sign;
  wire flag_parity;
  wire interrupt_pending;
  wire ready_status;
  wire int_clear;
  wire [13:0] pc_addr;
  wire [13:0] pc_data_in;
  wire [3:0] pc_control;
  wire pc_carry;
  wire [13:0] stack_addr;
  wire [13:0] selected_address;
  wire [7:0] regfile_data_out;
  wire [7:0] regfile_data_in;
  wire regfile_enable_a;
  wire regfile_enable_b;
  wire regfile_enable_c;
  wire regfile_enable_d;
  wire regfile_enable_e;
  wire regfile_enable_h;
  wire regfile_enable_l;
  wire regfile_read_enable;
  wire regfile_write_enable;
  wire [7:0] reg_a_out;
  wire [7:0] reg_b_out;
  wire [7:0] accumulator;
  wire [7:0] debug_reg_a_actual;
  wire [7:0] debug_reg_b_actual;
  wire [7:0] debug_reg_c_actual;
  wire [7:0] debug_reg_d_actual;
  wire [7:0] debug_reg_e_actual;
  wire [7:0] debug_reg_h_actual;
  wire [7:0] debug_reg_l_actual;
  wire [2:0] alu_opcode;
  wire alu_flag_carry;
  wire alu_flag_zero;
  wire alu_flag_sign;
  wire alu_flag_parity;
  wire [2:0] sp;
  wire load_ir;
  wire ir_output_enable;
  wire io_buffer_enable;
  wire io_buffer_direction;
  wire [7:0] io_buffer_data_out;
  wire io_buffer_oe;
  wire [2:0] scratchpad_select;
  wire scratchpad_read;
  wire scratchpad_write;
  wire regfile_to_bus;
  wire bus_to_regfile;
  wire select_pc;
  wire select_stack;
  wire [2:0] ahl_scratchpad_addr;
  wire ahl_active;
  wire [2:0] final_scratchpad_addr;
  wire pc_load_from_regs;
  wire pc_load_from_stack;
  wire pc_load_from_rst;
  wire stack_push;
  wire stack_pop;
  wire pc_increment_lower;
  wire pc_increment_upper;
  wire pc_load;
  wire pc_hold;
  wire load_reg_a;
  wire load_reg_b;
  wire alu_enable;
  wire update_flags;
  wire output_reg_a;
  wire output_reg_b;
  wire output_result;
  wire output_flags;
  wire [13:0] n478;
  wire [2:0] n479;
  wire [7:0] n480;
  wire n481;
  wire n482;
  wire [31:0] n483;
  wire n485;
  wire n486;
  wire n487;
  wire n488;
  wire [7:0] n489;
  wire [5:0] n490;
  wire [7:0] n491;
  wire n492;
  wire n493;
  wire [31:0] n494;
  wire n496;
  wire n497;
  wire n498;
  wire n499;
  wire [7:0] n500;
  wire [7:0] n501;
  wire n504;
  wire n505;
  wire [31:0] n506;
  wire n508;
  wire n509;
  wire n510;
  wire n511;
  wire n512;
  wire n514;
  wire n515;
  wire [31:0] n516;
  wire n518;
  wire n519;
  wire n520;
  wire n521;
  wire n522;
  wire n524;
  wire [2:0] n525;
  wire [2:0] n526;
  wire [7:0] n527;
  wire [7:0] n528;
  wire [7:0] n529;
  wire [7:0] n530;
  wire [7:0] n531;
  wire [7:0] n532;
  wire u_phase_clocks_n534;
  wire u_phase_clocks_n535;
  wire u_phase_clocks_n536;
  wire u_phase_clocks_n537;
  wire u_phase_clocks_n538;
  wire u_phase_clocks_n539;
  wire u_phase_clocks_n540;
  wire u_state_timing_n555;
  wire u_state_timing_n556;
  wire u_state_timing_n557;
  wire u_state_timing_n558;
  wire u_state_timing_n559;
  wire u_state_timing_n560;
  wire u_state_timing_n561;
  wire u_state_timing_n562;
  wire u_state_timing_n563;
  wire u_state_timing_n564;
  wire u_state_timing_n565;
  wire u_interrupt_ready_n588;
  wire u_interrupt_ready_n589;
  wire u_machine_cycle_n594;
  wire u_machine_cycle_n595;
  wire u_machine_cycle_n596;
  wire [1:0] u_machine_cycle_n597;
  wire [1:0] u_machine_cycle_n598;
  wire [1:0] u_machine_cycle_n599;
  wire u_instr_decoder_n612;
  wire u_instr_decoder_n613;
  wire u_instr_decoder_n614;
  wire u_instr_decoder_n615;
  wire [2:0] u_instr_decoder_n616;
  wire [2:0] u_instr_decoder_n617;
  wire u_instr_decoder_n618;
  wire u_instr_decoder_n619;
  wire u_instr_decoder_n620;
  wire u_instr_decoder_n621;
  wire u_instr_decoder_n622;
  wire u_instr_decoder_n623;
  wire u_instr_decoder_n624;
  wire u_instr_decoder_n625;
  wire u_instr_decoder_n626;
  wire u_instr_decoder_n627;
  wire u_instr_decoder_n629;
  wire u_instr_decoder_n630;
  wire [2:0] u_instr_decoder_n631;
  wire [1:0] u_instr_decoder_n632;
  wire u_instr_decoder_n633;
  wire u_instr_decoder_n634;
  wire u_instr_decoder_n635;
  wire \u_instr_decoder.instr_is_binary_alu ;
  wire u_memory_io_control_n684;
  wire u_memory_io_control_n685;
  wire u_memory_io_control_n686;
  wire u_memory_io_control_n687;
  wire [2:0] u_memory_io_control_n690;
  wire u_memory_io_control_n691;
  wire u_memory_io_control_n692;
  wire u_memory_io_control_n696;
  wire u_memory_io_control_n697;
  wire u_memory_io_control_n698;
  wire u_memory_io_control_n699;
  wire u_memory_io_control_n700;
  wire u_memory_io_control_n701;
  wire u_memory_io_control_n702;
  wire u_memory_io_control_n705;
  wire u_memory_io_control_n706;
  wire u_memory_io_control_n709;
  wire u_memory_io_control_n710;
  wire u_memory_io_control_n711;
  wire u_memory_io_control_n712;
  wire [7:0] n713;
  wire [2:0] \u_memory_io_control.addr_select_sss ;
  wire [2:0] \u_memory_io_control.addr_select_ddd ;
  wire \u_memory_io_control.memory_read ;
  wire \u_memory_io_control.memory_write ;
  wire \u_memory_io_control.memory_refresh ;
  wire \u_memory_io_control.refresh_increment ;
  wire \u_memory_io_control.stack_addr_select ;
  wire \u_memory_io_control.stack_read ;
  wire \u_memory_io_control.stack_write ;
  wire [2:0] u_ahl_pointer_n772;
  wire u_ahl_pointer_n773;
  wire [7:0] u_mem_mux_refresh_n778;
  wire [7:0] u_mem_mux_refresh_n779;
  wire u_mem_mux_refresh_n780;
  wire [13:0] u_mem_mux_refresh_n781;
  wire [2:0] u_stack_pointer_n790;
  wire [13:0] u_stack_memory_n793;
  wire u_stack_memory_n794;
  wire n795;
  wire n796;
  wire n797;
  wire n798;
  wire u_scratchpad_decoder_n803;
  wire u_scratchpad_decoder_n804;
  wire u_scratchpad_decoder_n805;
  wire u_scratchpad_decoder_n806;
  wire u_scratchpad_decoder_n807;
  wire u_scratchpad_decoder_n808;
  wire u_scratchpad_decoder_n809;
  wire u_scratchpad_decoder_n811;
  wire u_scratchpad_decoder_n812;
  wire \u_scratchpad_decoder.enable_m ;
  wire [7:0] u_register_file_n833;
  wire [7:0] u_register_file_n834;
  wire [7:0] u_register_file_n835;
  wire [7:0] u_register_file_n836;
  wire [7:0] u_register_file_n837;
  wire [7:0] u_register_file_n838;
  wire [7:0] u_register_file_n839;
  wire [7:0] u_register_file_n840;
  wire [7:0] u_register_file_n841;
  wire u_register_alu_control_n860;
  wire u_register_alu_control_n861;
  wire u_register_alu_control_n862;
  wire u_register_alu_control_n863;
  wire u_register_alu_control_n864;
  wire u_register_alu_control_n865;
  wire u_register_alu_control_n866;
  wire u_register_alu_control_n867;
  wire [7:0] u_temp_registers_n884;
  wire u_temp_registers_n885;
  wire [7:0] u_temp_registers_n886;
  wire [7:0] u_temp_registers_n887;
  wire [7:0] u_alu_n896;
  wire u_alu_n897;
  wire u_alu_n899;
  wire u_alu_n900;
  wire u_alu_n901;
  wire u_alu_n902;
  wire [8:0] \u_alu.result ;
  wire [7:0] u_condition_flags_n917;
  wire u_condition_flags_n918;
  wire u_condition_flags_n919;
  wire u_condition_flags_n920;
  wire u_condition_flags_n921;
  wire u_condition_flags_n922;
  wire u_condition_flags_n923;
  wire [7:0] u_instruction_register_n938;
  wire u_instruction_register_n939;
  wire u_instruction_register_n940;
  wire u_instruction_register_n941;
  wire u_instruction_register_n942;
  wire u_instruction_register_n943;
  wire u_instruction_register_n944;
  wire u_instruction_register_n945;
  wire u_instruction_register_n946;
  wire u_instruction_register_n947;
  wire [7:0] u_io_buffer_n968;
  wire u_io_buffer_n969;
  wire [7:0] u_io_buffer_n970;
  wire u_io_buffer_n971;
  wire [7:0] n980;
  wire [3:0] n981;
  assign phi1_out = phi1; //(module output)
  assign phi2_out = phi2; //(module output)
  assign phi1_rising_out = phi1_rising; //(module output)
  assign phi1_falling_out = phi1_falling; //(module output)
  assign phi2_rising_out = phi2_rising; //(module output)
  assign phi2_falling_out = phi2_falling; //(module output)
  assign data_bus_out = n489; //(module output)
  assign data_bus_oe = n512; //(module output)
  assign sync_out = sync; //(module output)
  assign s0_out = status_s0; //(module output)
  assign s1_out = status_s1; //(module output)
  assign s2_out = status_s2; //(module output)
  assign debug_reg_a = debug_reg_a_actual; //(module output)
  assign debug_reg_b = debug_reg_b_actual; //(module output)
  assign debug_reg_c = debug_reg_c_actual; //(module output)
  assign debug_reg_d = debug_reg_d_actual; //(module output)
  assign debug_reg_e = debug_reg_e_actual; //(module output)
  assign debug_reg_h = debug_reg_h_actual; //(module output)
  assign debug_reg_l = debug_reg_l_actual; //(module output)
  assign debug_cycle = current_cycle; //(module output)
  assign debug_pc = pc_addr; //(module output)
  assign debug_ir = instr_byte; //(module output)
  assign debug_needs_address = instr_needs_address; //(module output)
  assign debug_int_pending = interrupt_pending; //(module output)
  assign cycle_type = u_machine_cycle_n597; //(module output)
  assign debug_flag_carry = flag_carry; //(module output)
  assign debug_flag_zero = flag_zero; //(module output)
  assign debug_flag_sign = flag_sign; //(module output)
  assign debug_flag_parity = flag_parity; //(module output)
  assign debug_state_half = state_half; //(module output)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:560:12  */
  assign phi1 = u_phase_clocks_n534; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:561:12  */
  assign phi2 = u_phase_clocks_n535; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:562:12  */
  assign phi1_rising = u_phase_clocks_n537; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:563:12  */
  assign phi1_falling = u_phase_clocks_n538; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:564:12  */
  assign phi2_rising = u_phase_clocks_n539; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:565:12  */
  assign phi2_falling = u_phase_clocks_n540; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:569:12  */
  assign internal_bus = n527; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:572:12  */
  assign mem_mux_bus_out = u_mem_mux_refresh_n779; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:573:12  */
  assign mem_mux_bus_oe = u_mem_mux_refresh_n780; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:574:12  */
  assign temp_reg_bus_out = u_temp_registers_n884; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:575:12  */
  assign temp_reg_bus_oe = u_temp_registers_n885; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:576:12  */
  assign alu_bus_out = u_alu_n896; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:577:12  */
  assign alu_bus_oe = u_alu_n897; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:578:12  */
  assign cond_flags_bus_out = u_condition_flags_n917; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:579:12  */
  assign cond_flags_bus_oe = u_condition_flags_n918; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:580:12  */
  assign ir_bus_out = u_instruction_register_n938; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:581:12  */
  assign ir_bus_oe = u_instruction_register_n939; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:582:12  */
  assign io_buffer_bus_out = u_io_buffer_n970; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:583:12  */
  assign io_buffer_bus_oe = u_io_buffer_n971; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:586:12  */
  assign state_t1 = u_state_timing_n555; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:587:12  */
  assign state_t2 = u_state_timing_n556; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:588:12  */
  assign state_t3 = u_state_timing_n557; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:589:12  */
  assign state_t4 = u_state_timing_n558; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:590:12  */
  assign state_t5 = u_state_timing_n559; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:591:12  */
  assign state_t1i = u_state_timing_n560; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:592:12  */
  assign state_stopped = u_state_timing_n561; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:593:12  */
  assign state_half = u_state_timing_n562; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:594:12  */
  assign status_s0 = u_state_timing_n563; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:595:12  */
  assign status_s1 = u_state_timing_n564; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:596:12  */
  assign status_s2 = u_state_timing_n565; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:597:12  */
  assign sync = u_phase_clocks_n536; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:601:12  */
  assign current_cycle = u_machine_cycle_n598; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:602:12  */
  assign next_cycle = u_machine_cycle_n599; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:603:12  */
  assign advance_state = u_machine_cycle_n594; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:604:12  */
  assign cycle_done = u_machine_cycle_n595; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:605:12  */
  assign instr_is_hlt_flag = u_machine_cycle_n596; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:608:12  */
  assign instr_byte = n980; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:609:12  */
  assign instr_needs_immediate = u_instr_decoder_n612; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:610:12  */
  assign instr_needs_address = u_instr_decoder_n613; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:611:12  */
  assign instr_is_io = u_instr_decoder_n614; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:612:12  */
  assign instr_is_write = u_instr_decoder_n615; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:613:12  */
  assign instr_sss_field = u_instr_decoder_n616; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:614:12  */
  assign instr_ddd_field = u_instr_decoder_n617; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:615:12  */
  assign instr_is_alu = u_instr_decoder_n618; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:616:12  */
  assign instr_is_call = u_instr_decoder_n619; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:617:12  */
  assign instr_is_ret = u_instr_decoder_n620; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:618:12  */
  assign instr_is_rst = u_instr_decoder_n621; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:619:12  */
  assign instr_is_hlt = u_instr_decoder_n622; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:620:12  */
  assign instr_writes_reg = u_instr_decoder_n623; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:621:12  */
  assign instr_reads_reg = u_instr_decoder_n624; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:622:12  */
  assign instr_is_mem_indirect = u_instr_decoder_n625; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:623:12  */
  assign instr_uses_temp_regs = u_instr_decoder_n626; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:624:12  */
  assign instr_is_inr_dcr = u_instr_decoder_n627; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:626:12  */
  assign instr_is_rotate = u_instr_decoder_n629; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:627:12  */
  assign instr_needs_t4t5 = u_instr_decoder_n630; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:628:12  */
  assign rst_vector = u_instr_decoder_n631; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:629:12  */
  assign condition_code = u_instr_decoder_n632; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:630:12  */
  assign test_true = u_instr_decoder_n633; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:631:12  */
  assign eval_condition = u_instr_decoder_n634; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:632:12  */
  assign transition_to_stopped = u_instr_decoder_n635; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:635:12  */
  assign condition_met = u_condition_flags_n919; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:636:12  */
  assign flag_carry = u_condition_flags_n920; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:637:12  */
  assign flag_zero = u_condition_flags_n921; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:638:12  */
  assign flag_sign = u_condition_flags_n922; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:639:12  */
  assign flag_parity = u_condition_flags_n923; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:642:12  */
  assign interrupt_pending = u_interrupt_ready_n588; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:643:12  */
  assign ready_status = u_interrupt_ready_n589; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:644:12  */
  assign int_clear = state_t1i; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:647:12  */
  assign pc_addr = u_stack_memory_n793; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:648:12  */
  assign pc_data_in = u_mem_mux_refresh_n781; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:649:12  */
  assign pc_control = n981; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:650:12  */
  assign pc_carry = u_stack_memory_n794; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:653:12  */
  assign stack_addr = pc_addr; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:654:12  */
  assign selected_address = n478; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:657:12  */
  assign regfile_data_out = u_register_file_n833; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:658:12  */
  assign regfile_data_in = u_mem_mux_refresh_n778; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:659:12  */
  assign regfile_enable_a = u_scratchpad_decoder_n803; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:660:12  */
  assign regfile_enable_b = u_scratchpad_decoder_n804; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:661:12  */
  assign regfile_enable_c = u_scratchpad_decoder_n805; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:662:12  */
  assign regfile_enable_d = u_scratchpad_decoder_n806; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:663:12  */
  assign regfile_enable_e = u_scratchpad_decoder_n807; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:664:12  */
  assign regfile_enable_h = u_scratchpad_decoder_n808; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:665:12  */
  assign regfile_enable_l = u_scratchpad_decoder_n809; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:667:12  */
  assign regfile_read_enable = u_scratchpad_decoder_n811; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:668:12  */
  assign regfile_write_enable = u_scratchpad_decoder_n812; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:671:12  */
  assign reg_a_out = u_temp_registers_n886; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:672:12  */
  assign reg_b_out = u_temp_registers_n887; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:675:12  */
  assign accumulator = u_register_file_n834; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:678:12  */
  assign debug_reg_a_actual = u_register_file_n835; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:679:12  */
  assign debug_reg_b_actual = u_register_file_n836; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:680:12  */
  assign debug_reg_c_actual = u_register_file_n837; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:681:12  */
  assign debug_reg_d_actual = u_register_file_n838; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:682:12  */
  assign debug_reg_e_actual = u_register_file_n839; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:683:12  */
  assign debug_reg_h_actual = u_register_file_n840; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:684:12  */
  assign debug_reg_l_actual = u_register_file_n841; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:688:12  */
  assign alu_opcode = n525; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:690:12  */
  assign alu_flag_carry = u_alu_n899; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:691:12  */
  assign alu_flag_zero = u_alu_n900; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:692:12  */
  assign alu_flag_sign = u_alu_n901; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:693:12  */
  assign alu_flag_parity = u_alu_n902; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:696:12  */
  assign sp = u_stack_pointer_n790; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:702:12  */
  assign load_ir = u_memory_io_control_n684; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:703:12  */
  assign ir_output_enable = u_memory_io_control_n685; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:704:12  */
  assign io_buffer_enable = u_memory_io_control_n686; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:705:12  */
  assign io_buffer_direction = u_memory_io_control_n687; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:706:12  */
  assign io_buffer_data_out = u_io_buffer_n968; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:707:12  */
  assign io_buffer_oe = u_io_buffer_n969; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:710:12  */
  assign scratchpad_select = u_memory_io_control_n690; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:711:12  */
  assign scratchpad_read = u_memory_io_control_n691; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:712:12  */
  assign scratchpad_write = u_memory_io_control_n692; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:716:12  */
  assign regfile_to_bus = u_memory_io_control_n696; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:717:12  */
  assign bus_to_regfile = u_memory_io_control_n697; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:718:12  */
  assign select_pc = u_memory_io_control_n698; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:719:12  */
  assign select_stack = u_memory_io_control_n699; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:722:12  */
  assign ahl_scratchpad_addr = u_ahl_pointer_n772; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:723:12  */
  assign ahl_active = u_ahl_pointer_n773; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:724:12  */
  assign final_scratchpad_addr = n479; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:725:12  */
  assign pc_load_from_regs = u_memory_io_control_n700; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:726:12  */
  assign pc_load_from_stack = u_memory_io_control_n701; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:727:12  */
  assign pc_load_from_rst = u_memory_io_control_n702; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:730:12  */
  assign stack_push = u_memory_io_control_n705; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:731:12  */
  assign stack_pop = u_memory_io_control_n706; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:734:12  */
  assign pc_increment_lower = u_memory_io_control_n709; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:735:12  */
  assign pc_increment_upper = u_memory_io_control_n710; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:736:12  */
  assign pc_load = u_memory_io_control_n711; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:737:12  */
  assign pc_hold = u_memory_io_control_n712; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:743:12  */
  assign load_reg_a = u_register_alu_control_n860; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:744:12  */
  assign load_reg_b = u_register_alu_control_n861; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:745:12  */
  assign alu_enable = u_register_alu_control_n862; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:746:12  */
  assign update_flags = u_register_alu_control_n863; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:747:12  */
  assign output_reg_a = u_register_alu_control_n864; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:748:12  */
  assign output_reg_b = u_register_alu_control_n865; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:749:12  */
  assign output_result = u_register_alu_control_n866; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:750:12  */
  assign output_flags = u_register_alu_control_n867; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:777:36  */
  assign n478 = select_stack ? stack_addr : pc_addr;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:780:50  */
  assign n479 = ahl_active ? ahl_scratchpad_addr : scratchpad_select;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:795:54  */
  assign n480 = selected_address[7:0]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:795:93  */
  assign n481 = ~ahl_active;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:795:89  */
  assign n482 = n481 & state_t1;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:796:113  */
  assign n483 = {30'b0, next_cycle};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:796:113  */
  assign n485 = n483 == 32'b00000000000000000000000000000001;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:796:98  */
  assign n486 = n485 & instr_is_io;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:796:75  */
  assign n487 = ~n486;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:795:116  */
  assign n488 = n487 & n482;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:795:68  */
  assign n489 = n488 ? n480 : n500;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:797:68  */
  assign n490 = selected_address[13:8]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:797:33  */
  assign n491 = {u_machine_cycle_n597, n490};
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:797:109  */
  assign n492 = ~ahl_active;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:797:105  */
  assign n493 = n492 & state_t2;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:798:132  */
  assign n494 = {30'b0, current_cycle};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:798:132  */
  assign n496 = n494 == 32'b00000000000000000000000000000001;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:798:114  */
  assign n497 = n496 & instr_is_io;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:798:91  */
  assign n498 = ~n497;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:797:132  */
  assign n499 = n498 & n493;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:796:119  */
  assign n500 = n499 ? n491 : n501;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:798:138  */
  assign n501 = io_buffer_oe ? io_buffer_data_out : 8'b00000000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:805:49  */
  assign n504 = ~ahl_active;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:805:45  */
  assign n505 = n504 & state_t1;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:805:114  */
  assign n506 = {30'b0, next_cycle};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:805:114  */
  assign n508 = n506 == 32'b00000000000000000000000000000001;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:805:99  */
  assign n509 = n508 & instr_is_io;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:805:76  */
  assign n510 = ~n509;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:805:72  */
  assign n511 = n510 & n505;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:805:24  */
  assign n512 = n511 ? 1'b1 : n522;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:806:49  */
  assign n514 = ~ahl_active;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:806:45  */
  assign n515 = n514 & state_t2;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:806:117  */
  assign n516 = {30'b0, current_cycle};  //  uext
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:806:117  */
  assign n518 = n516 == 32'b00000000000000000000000000000001;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:806:99  */
  assign n519 = n518 & instr_is_io;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:806:76  */
  assign n520 = ~n519;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:806:72  */
  assign n521 = n520 & n515;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:805:120  */
  assign n522 = n521 ? 1'b1 : io_buffer_oe;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:838:64  */
  assign n524 = instr_is_inr_dcr | instr_is_rotate;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:838:35  */
  assign n525 = n524 ? instr_sss_field : n526;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:838:105  */
  assign n526 = instr_byte[5:3]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:848:40  */
  assign n527 = io_buffer_bus_oe ? io_buffer_bus_out : n528;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:848:69  */
  assign n528 = mem_mux_bus_oe ? mem_mux_bus_out : n529;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:849:69  */
  assign n529 = temp_reg_bus_oe ? temp_reg_bus_out : n530;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:850:69  */
  assign n530 = alu_bus_oe ? alu_bus_out : n531;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:851:69  */
  assign n531 = cond_flags_bus_oe ? cond_flags_bus_out : n532;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:852:69  */
  assign n532 = ir_bus_oe ? ir_bus_out : 8'b00000000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:868:5  */
  phase_clocks_Brtl_25000000 u_phase_clocks (
    .clk_in(clk_in),
    .reset(reset),
    .run_enable(run_enable),
    .phi1(u_phase_clocks_n534),
    .phi2(u_phase_clocks_n535),
    .sync(u_phase_clocks_n536),
    .phi1_rising(u_phase_clocks_n537),
    .phi1_falling(u_phase_clocks_n538),
    .phi2_rising(u_phase_clocks_n539),
    .phi2_falling(u_phase_clocks_n540));
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:885:5  */
  state_timing_generator_Brtl u_state_timing (
    .clk(clk_in),
    .phi2_falling(phi2_falling),
    .reset(reset),
    .advance_state(advance_state),
    .cycle_done(cycle_done),
    .interrupt_pending(interrupt_pending),
    .ready(ready_status),
    .instr_is_hlt_flag(instr_is_hlt_flag),
    .transition_to_stopped(transition_to_stopped),
    .state_t1(u_state_timing_n555),
    .state_t2(u_state_timing_n556),
    .state_t3(u_state_timing_n557),
    .state_t4(u_state_timing_n558),
    .state_t5(u_state_timing_n559),
    .state_t1i(u_state_timing_n560),
    .state_stopped(u_state_timing_n561),
    .state_half(u_state_timing_n562),
    .status_s0(u_state_timing_n563),
    .status_s1(u_state_timing_n564),
    .status_s2(u_state_timing_n565));
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:913:5  */
  interrupt_ready_ff_Brtl u_interrupt_ready (
    .clk(clk_in),
    .phi2_rising(phi2_rising),
    .reset(reset),
    .int_request(interrupt),
    .int_clear(int_clear),
    .ready_in(ready_in),
    .interrupt_pending(u_interrupt_ready_n588),
    .ready_status(u_interrupt_ready_n589));
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:925:5  */
  machine_cycle_control_Brtl u_machine_cycle (
    .clk(clk_in),
    .phi1_rising(phi1_rising),
    .reset(reset),
    .state_t1(state_t1),
    .state_t2(state_t2),
    .state_t3(state_t3),
    .state_t4(state_t4),
    .state_t5(state_t5),
    .state_t1i(state_t1i),
    .state_half(state_half),
    .instr_needs_immediate(instr_needs_immediate),
    .instr_needs_address(instr_needs_address),
    .instr_is_io(instr_is_io),
    .instr_is_write(instr_is_write),
    .instr_is_hlt(instr_is_hlt),
    .instr_needs_t4t5(instr_needs_t4t5),
    .instr_is_mem_indirect(instr_is_mem_indirect),
    .eval_condition(eval_condition),
    .condition_met(condition_met),
    .advance_state(u_machine_cycle_n594),
    .cycle_done(u_machine_cycle_n595),
    .instr_is_hlt_flag(u_machine_cycle_n596),
    .cycle_type(u_machine_cycle_n597),
    .current_cycle(u_machine_cycle_n598),
    .next_cycle(u_machine_cycle_n599));
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:954:5  */
  instruction_decoder_Brtl u_instr_decoder (
    .instruction_byte(instr_byte),
    .instr_needs_immediate(u_instr_decoder_n612),
    .instr_needs_address(u_instr_decoder_n613),
    .instr_is_io(u_instr_decoder_n614),
    .instr_is_write(u_instr_decoder_n615),
    .instr_sss_field(u_instr_decoder_n616),
    .instr_ddd_field(u_instr_decoder_n617),
    .instr_is_alu(u_instr_decoder_n618),
    .instr_is_call(u_instr_decoder_n619),
    .instr_is_ret(u_instr_decoder_n620),
    .instr_is_rst(u_instr_decoder_n621),
    .instr_is_hlt(u_instr_decoder_n622),
    .instr_writes_reg(u_instr_decoder_n623),
    .instr_reads_reg(u_instr_decoder_n624),
    .instr_is_mem_indirect(u_instr_decoder_n625),
    .instr_uses_temp_regs(u_instr_decoder_n626),
    .instr_is_inr_dcr(u_instr_decoder_n627),
    .instr_is_binary_alu(),
    .instr_is_rotate(u_instr_decoder_n629),
    .instr_needs_t4t5(u_instr_decoder_n630),
    .rst_vector(u_instr_decoder_n631),
    .condition_code(u_instr_decoder_n632),
    .test_true(u_instr_decoder_n633),
    .eval_condition(u_instr_decoder_n634),
    .transition_to_stopped(u_instr_decoder_n635));
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:1049:62  */
  assign n713 = pc_addr[7:0]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:983:5  */
  memory_io_control_Brtl u_memory_io_control (
    .clk(clk_in),
    .phi1_rising(phi1_rising),
    .reset(reset),
    .state_t1(state_t1),
    .state_t2(state_t2),
    .state_t3(state_t3),
    .state_t4(state_t4),
    .state_t5(state_t5),
    .state_t1i(state_t1i),
    .state_stopped(state_stopped),
    .state_half(state_half),
    .status_s0(status_s0),
    .status_s1(status_s1),
    .status_s2(status_s2),
    .cycle_type(u_machine_cycle_n597),
    .current_cycle(current_cycle),
    .next_cycle(next_cycle),
    .advance_state(advance_state),
    .instr_is_hlt_flag(instr_is_hlt_flag),
    .instr_needs_immediate(instr_needs_immediate),
    .instr_needs_address(instr_needs_address),
    .instr_is_io(instr_is_io),
    .instr_is_write(instr_is_write),
    .instr_sss_field(instr_sss_field),
    .instr_ddd_field(instr_ddd_field),
    .instr_is_alu(instr_is_alu),
    .instr_is_call(instr_is_call),
    .instr_is_ret(instr_is_ret),
    .instr_is_rst(instr_is_rst),
    .instr_writes_reg(instr_writes_reg),
    .instr_reads_reg(instr_reads_reg),
    .instr_is_mem_indirect(instr_is_mem_indirect),
    .eval_condition(eval_condition),
    .condition_met(condition_met),
    .interrupt_pending(interrupt_pending),
    .ready_status(ready_status),
    .pc_carry_in(pc_carry),
    .pc_lower_byte(n713),
    .ir_load(u_memory_io_control_n684),
    .ir_output_enable(u_memory_io_control_n685),
    .io_buffer_enable(u_memory_io_control_n686),
    .io_buffer_direction(u_memory_io_control_n687),
    .addr_select_sss(),
    .addr_select_ddd(),
    .scratchpad_select(u_memory_io_control_n690),
    .scratchpad_read(u_memory_io_control_n691),
    .scratchpad_write(u_memory_io_control_n692),
    .memory_read(),
    .memory_write(),
    .memory_refresh(),
    .regfile_to_bus(u_memory_io_control_n696),
    .bus_to_regfile(u_memory_io_control_n697),
    .select_pc(u_memory_io_control_n698),
    .select_stack(u_memory_io_control_n699),
    .pc_load_from_regs(u_memory_io_control_n700),
    .pc_load_from_stack(u_memory_io_control_n701),
    .pc_load_from_rst(u_memory_io_control_n702),
    .refresh_increment(),
    .stack_addr_select(),
    .stack_push(u_memory_io_control_n705),
    .stack_pop(u_memory_io_control_n706),
    .stack_read(),
    .stack_write(),
    .pc_increment_lower(u_memory_io_control_n709),
    .pc_increment_upper(u_memory_io_control_n710),
    .pc_load(u_memory_io_control_n711),
    .pc_hold(u_memory_io_control_n712));
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:1063:5  */
  ahl_pointer_Brtl u_ahl_pointer (
    .state_t1(state_t1),
    .state_t2(state_t2),
    .current_cycle(current_cycle),
    .next_cycle(next_cycle),
    .instr_is_mem_indirect(instr_is_mem_indirect),
    .instr_needs_address(instr_needs_address),
    .ahl_select(u_ahl_pointer_n772),
    .ahl_active(u_ahl_pointer_n773));
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:1075:5  */
  mem_mux_refresh_Brtl u_mem_mux_refresh (
    .pc_addr(pc_addr),
    .stack_addr(stack_addr),
    .reg_a(reg_a_out),
    .reg_b(reg_b_out),
    .rst_vector(rst_vector),
    .regfile_data_out(regfile_data_out),
    .internal_bus_in(internal_bus),
    .select_pc(select_pc),
    .select_stack(select_stack),
    .pc_load_from_regs(pc_load_from_regs),
    .pc_load_from_stack(pc_load_from_stack),
    .pc_load_from_rst(pc_load_from_rst),
    .regfile_to_bus(regfile_to_bus),
    .bus_to_regfile(bus_to_regfile),
    .regfile_data_in(u_mem_mux_refresh_n778),
    .internal_bus_out(u_mem_mux_refresh_n779),
    .internal_bus_oe(u_mem_mux_refresh_n780),
    .pc_data_in(u_mem_mux_refresh_n781));
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:1102:5  */
  stack_pointer_Brtl u_stack_pointer (
    .clk(clk_in),
    .phi1_rising(phi1_rising),
    .reset(reset),
    .stack_push(stack_push),
    .stack_pop(stack_pop),
    .sp_out(u_stack_pointer_n790));
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:1112:5  */
  stack_memory_Brtl u_stack_memory (
    .clk(clk_in),
    .phi1_rising(phi1_rising),
    .reset(reset),
    .sp_in(sp),
    .\control[increment_lower] (n795),
    .\control[increment_upper] (n796),
    .\control[load] (n797),
    .\control[hold] (n798),
    .data_in(pc_data_in),
    .addr_out(u_stack_memory_n793),
    .carry_out(u_stack_memory_n794));
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:1112:5  */
  assign n795 = pc_control[0]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:1112:5  */
  assign n796 = pc_control[1]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:1112:5  */
  assign n797 = pc_control[2]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:1112:5  */
  assign n798 = pc_control[3]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:1132:5  */
  scratchpad_decoder_Brtl u_scratchpad_decoder (
    .addr_in(final_scratchpad_addr),
    .read_enable(scratchpad_read),
    .write_enable(scratchpad_write),
    .enable_a(u_scratchpad_decoder_n803),
    .enable_b(u_scratchpad_decoder_n804),
    .enable_c(u_scratchpad_decoder_n805),
    .enable_d(u_scratchpad_decoder_n806),
    .enable_e(u_scratchpad_decoder_n807),
    .enable_h(u_scratchpad_decoder_n808),
    .enable_l(u_scratchpad_decoder_n809),
    .enable_m(),
    .read_out(u_scratchpad_decoder_n811),
    .write_out(u_scratchpad_decoder_n812));
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:1149:5  */
  register_file_Brtl u_register_file (
    .clk(clk_in),
    .phi2_rising(phi2_rising),
    .reset(reset),
    .data_in(regfile_data_in),
    .enable_a(regfile_enable_a),
    .enable_b(regfile_enable_b),
    .enable_c(regfile_enable_c),
    .enable_d(regfile_enable_d),
    .enable_e(regfile_enable_e),
    .enable_h(regfile_enable_h),
    .enable_l(regfile_enable_l),
    .read_enable(regfile_read_enable),
    .write_enable(regfile_write_enable),
    .data_out(u_register_file_n833),
    .accumulator_out(u_register_file_n834),
    .debug_reg_a(u_register_file_n835),
    .debug_reg_b(u_register_file_n836),
    .debug_reg_c(u_register_file_n837),
    .debug_reg_d(u_register_file_n838),
    .debug_reg_e(u_register_file_n839),
    .debug_reg_h(u_register_file_n840),
    .debug_reg_l(u_register_file_n841));
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:1179:5  */
  register_alu_control_Brtl u_register_alu_control (
    .clk(clk_in),
    .phi2_rising(phi2_rising),
    .status_s0(status_s0),
    .status_s1(status_s1),
    .status_s2(status_s2),
    .instr_is_alu_op(instr_is_alu),
    .instr_uses_temp_regs(instr_uses_temp_regs),
    .instr_needs_immediate(instr_needs_immediate),
    .instr_writes_reg(instr_writes_reg),
    .instr_is_write(instr_is_write),
    .instr_is_io(instr_is_io),
    .current_cycle(current_cycle),
    .state_half(state_half),
    .interrupt(interrupt_pending),
    .load_reg_a(u_register_alu_control_n860),
    .load_reg_b(u_register_alu_control_n861),
    .alu_enable(u_register_alu_control_n862),
    .update_flags(u_register_alu_control_n863),
    .output_reg_a(u_register_alu_control_n864),
    .output_reg_b(u_register_alu_control_n865),
    .output_result(u_register_alu_control_n866),
    .output_flags(u_register_alu_control_n867));
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:1205:5  */
  temp_registers_Brtl u_temp_registers (
    .clk(clk_in),
    .phi2_rising(phi2_rising),
    .reset(reset),
    .load_reg_a(load_reg_a),
    .load_reg_b(load_reg_b),
    .output_reg_a(output_reg_a),
    .output_reg_b(output_reg_b),
    .internal_bus_in(internal_bus),
    .internal_bus_out(u_temp_registers_n884),
    .internal_bus_oe(u_temp_registers_n885),
    .reg_a_out(u_temp_registers_n886),
    .reg_b_out(u_temp_registers_n887));
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:1225:5  */
  alu_Brtl u_alu (
    .clk(clk_in),
    .phi2_rising(phi2_rising),
    .accumulator_in(accumulator),
    .reg_b_in(reg_b_out),
    .opcode(alu_opcode),
    .is_inr_dcr(instr_is_inr_dcr),
    .is_rotate(instr_is_rotate),
    .carry_in(flag_carry),
    .enable(alu_enable),
    .output_result(output_result),
    .internal_bus_out(u_alu_n896),
    .internal_bus_oe(u_alu_n897),
    .result(),
    .flag_carry(u_alu_n899),
    .flag_zero(u_alu_n900),
    .flag_sign(u_alu_n901),
    .flag_parity(u_alu_n902));
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:1246:5  */
  condition_flags_Brtl u_condition_flags (
    .clk(clk_in),
    .phi2_rising(phi2_rising),
    .reset(reset),
    .flag_carry_in(alu_flag_carry),
    .flag_zero_in(alu_flag_zero),
    .flag_sign_in(alu_flag_sign),
    .flag_parity_in(alu_flag_parity),
    .update_flags(update_flags),
    .carry_only(instr_is_rotate),
    .condition_code(condition_code),
    .test_true(test_true),
    .eval_condition(eval_condition),
    .output_flags(output_flags),
    .internal_bus_out(u_condition_flags_n917),
    .internal_bus_oe(u_condition_flags_n918),
    .condition_met(u_condition_flags_n919),
    .flag_carry(u_condition_flags_n920),
    .flag_zero(u_condition_flags_n921),
    .flag_sign(u_condition_flags_n922),
    .flag_parity(u_condition_flags_n923));
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:1274:5  */
  instruction_register_Brtl u_instruction_register (
    .clk(clk_in),
    .phi1_falling(phi1_falling),
    .reset(reset),
    .internal_bus_in(internal_bus),
    .load_ir(load_ir),
    .output_ir(ir_output_enable),
    .internal_bus_out(u_instruction_register_n938),
    .internal_bus_oe(u_instruction_register_n939),
    .ir_bit_7(u_instruction_register_n940),
    .ir_bit_6(u_instruction_register_n941),
    .ir_bit_5(u_instruction_register_n942),
    .ir_bit_4(u_instruction_register_n943),
    .ir_bit_3(u_instruction_register_n944),
    .ir_bit_2(u_instruction_register_n945),
    .ir_bit_1(u_instruction_register_n946),
    .ir_bit_0(u_instruction_register_n947));
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:1294:5  */
  io_buffer_Brtl u_io_buffer (
    .external_data_in(data_bus_in),
    .internal_bus_in(internal_bus),
    .enable(io_buffer_enable),
    .direction(io_buffer_direction),
    .external_data_out(u_io_buffer_n968),
    .external_data_oe(u_io_buffer_n969),
    .internal_bus_out(u_io_buffer_n970),
    .internal_bus_oe(u_io_buffer_n971));
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:608:12  */
  assign n980 = {u_instruction_register_n940, u_instruction_register_n941, u_instruction_register_n942, u_instruction_register_n943, u_instruction_register_n944, u_instruction_register_n945, u_instruction_register_n946, u_instruction_register_n947};
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008.vhdl:649:12  */
  assign n981 = {pc_hold, pc_load, pc_increment_upper, pc_increment_lower};
endmodule

module b8008_top
  (input  clk_in,
   input  reset,
   input  run_enable,
   input  interrupt,
   input  [2:0] int_vector,
   input  ready_in,
   output phi1_out,
   output phi2_out,
   output sync_out,
   output s0_out,
   output s1_out,
   output s2_out,
   output [13:0] address_out,
   output [7:0] data_out,
   output [7:0] ram_byte_0,
   output [7:0] debug_reg_a,
   output [7:0] debug_reg_b,
   output [7:0] debug_reg_c,
   output [7:0] debug_reg_d,
   output [7:0] debug_reg_e,
   output [7:0] debug_reg_h,
   output [7:0] debug_reg_l,
   output [1:0] debug_cycle,
   output [13:0] debug_pc,
   output [7:0] debug_ir,
   output debug_needs_address,
   output debug_int_pending,
   output debug_flag_carry,
   output debug_flag_zero,
   output debug_flag_sign,
   output debug_flag_parity,
   output [7:0] debug_io_port_8,
   output [7:0] debug_io_port_9,
   output [7:0] debug_io_port_10,
   output debug_state_half,
   input  [7:0] io_port_in,
   input  [2:0] io_port_in_select,
   input  io_port_in_enable,
   output [7:0] io_port_out,
   output [4:0] io_port_num_out,
   output io_port_write,
   output io_port_read,
   output [13:0] rom_a,
   input  [7:0] rom_d,
   output rom_ce_n,
   output rom_oe_n);
  wire [7:0] data_bus;
  wire [7:0] cpu_data_in;
  wire [7:0] cpu_data_out;
  wire cpu_data_oe;
  wire phi1;
  wire phi2;
  wire phi1_rising;
  wire phi2_rising;
  wire rom_cs_n_int;
  wire ram_cs_n;
  wire [7:0] ram_data_in;
  wire [7:0] ram_data_out;
  wire ram_rw_n;
  wire rom_selected;
  wire ram_selected;
  wire is_write;
  wire is_io;
  wire [1:0] cycle_type;
  wire s0_int;
  wire s1_int;
  wire s2_int;
  wire sync_int;
  wire [7:0] io_input_data;
  wire [2:0] io_port_num;
  reg [7:0] io_output_port_8;
  reg [7:0] io_output_port_9;
  reg [7:0] io_output_port_10;
  reg [13:0] latched_address;
  (* keep=1'b1 *) reg io_write_strobe;
  (* keep=1'b1 *) reg io_read_strobe;
  (* keep=1'b1 *) reg [4:0] io_full_port_num;
  wire is_t1;
  wire is_t2;
  wire is_t3;
  wire is_t4;
  wire is_t5;
  wire n47;
  wire n48;
  wire n49;
  wire n50;
  wire n51;
  wire n54;
  wire n55;
  wire n56;
  wire n57;
  wire n58;
  wire n61;
  wire n62;
  wire n63;
  wire n64;
  wire n65;
  wire n68;
  wire n69;
  wire n70;
  wire n73;
  wire n74;
  wire n75;
  wire n76;
  wire n82;
  wire n83;
  wire [5:0] n84;
  wire [5:0] n85;
  wire [5:0] n86;
  wire [7:0] n87;
  wire [7:0] n88;
  wire [5:0] n89;
  wire [5:0] n90;
  wire [13:0] n91;
  wire u_cpu_n113;
  wire u_cpu_n114;
  wire u_cpu_n115;
  wire u_cpu_n117;
  wire [7:0] u_cpu_n119;
  wire u_cpu_n120;
  wire u_cpu_n121;
  wire u_cpu_n122;
  wire u_cpu_n123;
  wire u_cpu_n124;
  wire [7:0] u_cpu_n125;
  wire [7:0] u_cpu_n126;
  wire [7:0] u_cpu_n127;
  wire [7:0] u_cpu_n128;
  wire [7:0] u_cpu_n129;
  wire [7:0] u_cpu_n130;
  wire [7:0] u_cpu_n131;
  wire [1:0] u_cpu_n132;
  wire [13:0] u_cpu_n133;
  wire [7:0] u_cpu_n134;
  wire u_cpu_n135;
  wire u_cpu_n136;
  wire [1:0] u_cpu_n137;
  wire u_cpu_n138;
  wire u_cpu_n139;
  wire u_cpu_n140;
  wire u_cpu_n141;
  wire u_cpu_n142;
  wire \u_cpu.phi1_falling_out ;
  wire \u_cpu.phi2_falling_out ;
  wire [7:0] u_ram_n203;
  wire n209;
  wire n210;
  wire n211;
  wire n213;
  wire n214;
  wire [7:0] n215;
  wire [7:0] n217;
  wire u_decode_n220;
  wire u_decode_n221;
  wire u_decode_n222;
  wire u_decode_n223;
  wire [13:0] n233;
  wire n236;
  wire n237;
  wire n241;
  wire n242;
  wire n245;
  wire n246;
  wire n247;
  wire n248;
  wire n249;
  wire [2:0] n251;
  wire n253;
  wire n255;
  wire n256;
  wire [7:0] n257;
  wire n259;
  wire n261;
  wire n262;
  wire [7:0] n263;
  wire n265;
  wire n267;
  wire n268;
  wire [7:0] n269;
  wire n271;
  wire n273;
  wire n274;
  wire [7:0] n275;
  wire n277;
  wire n279;
  wire n280;
  wire [7:0] n281;
  wire n283;
  wire n285;
  wire n286;
  wire [7:0] n287;
  wire n289;
  wire n291;
  wire n292;
  wire [7:0] n293;
  wire n295;
  wire n297;
  wire n298;
  wire [7:0] n299;
  wire n300;
  wire n301;
  wire [7:0] n302;
  wire [1:0] n304;
  wire [4:0] n305;
  wire n313;
  wire n314;
  wire n315;
  wire n316;
  wire n317;
  wire n318;
  wire n319;
  wire n322;
  wire n324;
  wire n325;
  wire n326;
  wire n327;
  wire n328;
  wire [1:0] n334;
  wire n336;
  wire n338;
  wire n339;
  wire n346;
  wire n348;
  wire n350;
  wire [2:0] n351;
  reg [7:0] n352;
  reg [7:0] n353;
  reg [7:0] n354;
  wire [7:0] n355;
  wire [7:0] n356;
  wire [7:0] n357;
  wire n367;
  wire n370;
  wire n372;
  wire n374;
  wire [4:0] n405;
  wire [7:0] n407;
  wire n408;
  wire n409;
  wire n410;
  wire [7:0] n411;
  wire n412;
  wire n413;
  wire n414;
  wire [1:0] n415;
  wire n417;
  wire n418;
  wire [7:0] n419;
  wire n420;
  wire n421;
  wire n422;
  wire n423;
  wire n424;
  wire [7:0] n425;
  wire n426;
  wire n427;
  wire n428;
  wire n429;
  wire n430;
  wire [7:0] n431;
  wire [7:0] n433;
  reg [7:0] n435;
  wire [7:0] n436;
  reg [7:0] n437;
  wire [7:0] n438;
  reg [7:0] n439;
  wire [7:0] n440;
  reg [7:0] n441;
  wire [13:0] n442;
  reg [13:0] n443;
  wire n444;
  reg n445;
  wire n446;
  reg n447;
  assign phi1_out = phi1; //(module output)
  assign phi2_out = phi2; //(module output)
  assign sync_out = sync_int; //(module output)
  assign s0_out = s0_int; //(module output)
  assign s1_out = s1_int; //(module output)
  assign s2_out = s2_int; //(module output)
  assign address_out = latched_address; //(module output)
  assign data_out = data_bus; //(module output)
  assign ram_byte_0 = n435; //(module output)
  assign debug_reg_a = u_cpu_n125; //(module output)
  assign debug_reg_b = u_cpu_n126; //(module output)
  assign debug_reg_c = u_cpu_n127; //(module output)
  assign debug_reg_d = u_cpu_n128; //(module output)
  assign debug_reg_e = u_cpu_n129; //(module output)
  assign debug_reg_h = u_cpu_n130; //(module output)
  assign debug_reg_l = u_cpu_n131; //(module output)
  assign debug_cycle = u_cpu_n132; //(module output)
  assign debug_pc = u_cpu_n133; //(module output)
  assign debug_ir = u_cpu_n134; //(module output)
  assign debug_needs_address = u_cpu_n135; //(module output)
  assign debug_int_pending = u_cpu_n136; //(module output)
  assign debug_flag_carry = u_cpu_n138; //(module output)
  assign debug_flag_zero = u_cpu_n139; //(module output)
  assign debug_flag_sign = u_cpu_n140; //(module output)
  assign debug_flag_parity = u_cpu_n141; //(module output)
  assign debug_io_port_8 = io_output_port_8; //(module output)
  assign debug_io_port_9 = io_output_port_9; //(module output)
  assign debug_io_port_10 = io_output_port_10; //(module output)
  assign debug_state_half = u_cpu_n142; //(module output)
  assign io_port_out = data_bus; //(module output)
  assign io_port_num_out = io_full_port_num; //(module output)
  assign io_port_write = io_write_strobe; //(module output)
  assign io_port_read = io_read_strobe; //(module output)
  assign rom_a = n233; //(module output)
  assign rom_ce_n = rom_cs_n_int; //(module output)
  assign rom_oe_n = rom_cs_n_int; //(module output)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:197:12  */
  assign data_bus = n433; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:198:12  */
  assign cpu_data_in = n411; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:199:12  */
  assign cpu_data_out = u_cpu_n119; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:200:12  */
  assign cpu_data_oe = u_cpu_n120; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:201:12  */
  assign phi1 = u_cpu_n113; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:202:12  */
  assign phi2 = u_cpu_n114; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:203:12  */
  assign phi1_rising = u_cpu_n115; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:205:12  */
  assign phi2_rising = u_cpu_n117; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:209:12  */
  assign rom_cs_n_int = u_decode_n222; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:210:12  */
  assign ram_cs_n = u_decode_n223; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:211:12  */
  assign ram_data_in = data_bus; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:212:12  */
  assign ram_data_out = u_ram_n203; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:213:12  */
  assign ram_rw_n = n249; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:216:12  */
  assign rom_selected = u_decode_n220; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:217:12  */
  assign ram_selected = u_decode_n221; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:218:12  */
  assign is_write = n237; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:219:12  */
  assign is_io = n242; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:220:12  */
  assign cycle_type = u_cpu_n137; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:224:12  */
  assign s0_int = u_cpu_n122; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:225:12  */
  assign s1_int = u_cpu_n123; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:226:12  */
  assign s2_int = u_cpu_n124; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:227:12  */
  assign sync_int = u_cpu_n121; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:238:12  */
  assign io_input_data = n257; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:239:12  */
  assign io_port_num = n251; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:243:12  */
  always @*
    io_output_port_8 = n437; // (isignal)
  initial
    io_output_port_8 = 8'b00000000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:244:12  */
  always @*
    io_output_port_9 = n439; // (isignal)
  initial
    io_output_port_9 = 8'b00000000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:245:12  */
  always @*
    io_output_port_10 = n441; // (isignal)
  initial
    io_output_port_10 = 8'b00000000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:254:12  */
  always @*
    latched_address = n443; // (isignal)
  initial
    latched_address = 14'b00000000000000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:257:12  */
  always @*
    io_write_strobe = n445; // (isignal)
  initial
    io_write_strobe = 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:258:12  */
  always @*
    io_read_strobe = n447; // (isignal)
  initial
    io_read_strobe = 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:259:12  */
  always @*
    io_full_port_num = n305; // (isignal)
  initial
    io_full_port_num = 5'b00000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:268:12  */
  assign is_t1 = n51; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:269:12  */
  assign is_t2 = n58; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:270:12  */
  assign is_t3 = n65; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:271:12  */
  assign is_t4 = n70; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:272:12  */
  assign is_t5 = n76; // (signal)
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:295:31  */
  assign n47 = ~s2_int;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:295:37  */
  assign n48 = s1_int & n47;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:295:65  */
  assign n49 = ~s0_int;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:295:54  */
  assign n50 = n49 & n48;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:295:18  */
  assign n51 = n50 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:296:48  */
  assign n54 = ~s1_int;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:296:37  */
  assign n55 = n54 & s2_int;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:296:65  */
  assign n56 = ~s0_int;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:296:54  */
  assign n57 = n56 & n55;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:296:18  */
  assign n58 = n57 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:297:31  */
  assign n61 = ~s2_int;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:297:48  */
  assign n62 = ~s1_int;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:297:37  */
  assign n63 = n62 & n61;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:297:54  */
  assign n64 = s0_int & n63;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:297:18  */
  assign n65 = n64 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:298:37  */
  assign n68 = s1_int & s2_int;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:298:54  */
  assign n69 = s0_int & n68;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:298:18  */
  assign n70 = n69 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:299:48  */
  assign n73 = ~s1_int;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:299:37  */
  assign n74 = n73 & s2_int;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:299:54  */
  assign n75 = s0_int & n74;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:299:18  */
  assign n76 = n75 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:312:28  */
  assign n82 = sync_int & is_t1;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:316:31  */
  assign n83 = sync_int & is_t2;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:318:57  */
  assign n84 = data_bus[5:0]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:254:12  */
  assign n85 = latched_address[13:8]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:316:13  */
  assign n86 = n83 ? n84 : n85;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:254:12  */
  assign n87 = latched_address[7:0]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:312:13  */
  assign n88 = n82 ? data_bus : n87;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:254:12  */
  assign n89 = latched_address[13:8]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:312:13  */
  assign n90 = n82 ? n89 : n86;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:311:9  */
  assign n91 = {n90, n88};
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:348:5  */
  b8008_Bstructural_25000000 u_cpu (
    .clk_in(clk_in),
    .reset(reset),
    .run_enable(run_enable),
    .data_bus_in(cpu_data_in),
    .ready_in(ready_in),
    .interrupt(interrupt),
    .phi1_out(u_cpu_n113),
    .phi2_out(u_cpu_n114),
    .phi1_rising_out(u_cpu_n115),
    .phi1_falling_out(),
    .phi2_rising_out(u_cpu_n117),
    .phi2_falling_out(),
    .data_bus_out(u_cpu_n119),
    .data_bus_oe(u_cpu_n120),
    .sync_out(u_cpu_n121),
    .s0_out(u_cpu_n122),
    .s1_out(u_cpu_n123),
    .s2_out(u_cpu_n124),
    .debug_reg_a(u_cpu_n125),
    .debug_reg_b(u_cpu_n126),
    .debug_reg_c(u_cpu_n127),
    .debug_reg_d(u_cpu_n128),
    .debug_reg_e(u_cpu_n129),
    .debug_reg_h(u_cpu_n130),
    .debug_reg_l(u_cpu_n131),
    .debug_cycle(u_cpu_n132),
    .debug_pc(u_cpu_n133),
    .debug_ir(u_cpu_n134),
    .debug_needs_address(u_cpu_n135),
    .debug_int_pending(u_cpu_n136),
    .cycle_type(u_cpu_n137),
    .debug_flag_carry(u_cpu_n138),
    .debug_flag_zero(u_cpu_n139),
    .debug_flag_sign(u_cpu_n140),
    .debug_flag_parity(u_cpu_n141),
    .debug_state_half(u_cpu_n142));
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:400:5  */
  ram_sync_Brtl_14_da39a3ee5e6b4b0d3255bfef95601890afd80709 u_ram (
    .clk(clk_in),
    .addr(latched_address),
    .data_in(ram_data_in),
    .rw_n(ram_rw_n),
    .cs_n(ram_cs_n),
    .data_out(u_ram_n203));
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:421:28  */
  assign n209 = ~ram_cs_n;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:421:47  */
  assign n210 = ~ram_rw_n;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:421:34  */
  assign n211 = n210 & n209;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:422:71  */
  assign n213 = latched_address == 14'b00000000000000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:421:53  */
  assign n214 = n213 & n211;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:421:13  */
  assign n215 = n214 ? ram_data_in : n435;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:419:13  */
  assign n217 = reset ? 8'b00000000 : n215;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:434:5  */
  address_decoder_Brtl_0_4095_4096_16383 u_decode (
    .address(latched_address),
    .rom_sel(u_decode_n220),
    .ram_sel(u_decode_n221),
    .rom_cs_n(u_decode_n222),
    .ram_cs_n(u_decode_n223));
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:451:67  */
  assign n233 = latched_address - 14'b00000000000000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:463:37  */
  assign n236 = cycle_type == 2'b11;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:463:21  */
  assign n237 = n236 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:464:37  */
  assign n241 = cycle_type == 2'b10;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:464:21  */
  assign n242 = n241 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:468:42  */
  assign n245 = ram_selected & is_write;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:469:39  */
  assign n246 = is_t3 | is_t4;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:469:54  */
  assign n247 = n246 | is_t5;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:468:65  */
  assign n248 = n247 & n245;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:468:21  */
  assign n249 = n248 ? 1'b0 : 1'b1;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:484:35  */
  assign n251 = latched_address[11:9]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:490:52  */
  assign n253 = ~io_port_in_enable;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:490:74  */
  assign n255 = io_port_num == 3'b000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:490:58  */
  assign n256 = n255 & n253;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:490:28  */
  assign n257 = n256 ? 8'b01010101 : n263;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:491:52  */
  assign n259 = ~io_port_in_enable;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:491:74  */
  assign n261 = io_port_num == 3'b001;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:491:58  */
  assign n262 = n261 & n259;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:490:83  */
  assign n263 = n262 ? 8'b10101010 : n269;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:492:52  */
  assign n265 = ~io_port_in_enable;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:492:74  */
  assign n267 = io_port_num == 3'b010;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:492:58  */
  assign n268 = n267 & n265;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:491:83  */
  assign n269 = n268 ? 8'b01000010 : n275;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:493:52  */
  assign n271 = ~io_port_in_enable;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:493:74  */
  assign n273 = io_port_num == 3'b011;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:493:58  */
  assign n274 = n273 & n271;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:492:83  */
  assign n275 = n274 ? 8'b00000011 : n281;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:494:52  */
  assign n277 = ~io_port_in_enable;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:494:74  */
  assign n279 = io_port_num == 3'b100;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:494:58  */
  assign n280 = n279 & n277;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:493:83  */
  assign n281 = n280 ? 8'b00000100 : n287;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:495:52  */
  assign n283 = ~io_port_in_enable;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:495:74  */
  assign n285 = io_port_num == 3'b101;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:495:58  */
  assign n286 = n285 & n283;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:494:83  */
  assign n287 = n286 ? 8'b00000101 : n293;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:496:52  */
  assign n289 = ~io_port_in_enable;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:496:74  */
  assign n291 = io_port_num == 3'b110;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:496:58  */
  assign n292 = n291 & n289;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:495:83  */
  assign n293 = n292 ? 8'b00000110 : n299;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:497:52  */
  assign n295 = ~io_port_in_enable;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:497:74  */
  assign n297 = io_port_num == 3'b111;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:497:58  */
  assign n298 = n297 & n295;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:496:83  */
  assign n299 = n298 ? 8'b00000111 : n302;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:498:79  */
  assign n300 = io_port_num == io_port_in_select;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:498:63  */
  assign n301 = n300 & io_port_in_enable;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:497:83  */
  assign n302 = n301 ? io_port_in : 8'b00000000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:504:40  */
  assign n304 = latched_address[13:12]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:504:55  */
  assign n305 = {n304, io_port_num};
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:531:35  */
  assign n313 = is_io & is_t3;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:531:68  */
  assign n314 = latched_address[13]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:531:49  */
  assign n315 = ~n314;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:531:45  */
  assign n316 = n313 & n315;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:531:96  */
  assign n317 = latched_address[12]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:531:77  */
  assign n318 = ~n317;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:531:73  */
  assign n319 = n316 & n318;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:532:13  */
  assign n322 = n319 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:540:35  */
  assign n324 = is_io & is_t3;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:540:65  */
  assign n325 = latched_address[13]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:540:88  */
  assign n326 = latched_address[12]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:540:70  */
  assign n327 = n325 | n326;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:540:45  */
  assign n328 = n324 & n327;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:549:35  */
  assign n334 = latched_address[13:12]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:549:50  */
  assign n336 = n334 == 2'b11;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:549:73  */
  assign n338 = io_port_num == 3'b111;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:549:57  */
  assign n339 = n338 & n336;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:573:25  */
  assign n346 = io_port_num == 3'b000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:576:25  */
  assign n348 = io_port_num == 3'b001;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:579:25  */
  assign n350 = io_port_num == 3'b010;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:572:21  */
  assign n351 = {n350, n348, n346};
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:572:21  */
  always @*
    case (n351)
      3'b100: n352 = io_output_port_8;
      3'b010: n352 = io_output_port_8;
      3'b001: n352 = data_bus;
      default: n352 = io_output_port_8;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:572:21  */
  always @*
    case (n351)
      3'b100: n353 = io_output_port_9;
      3'b010: n353 = data_bus;
      3'b001: n353 = io_output_port_9;
      default: n353 = io_output_port_9;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:572:21  */
  always @*
    case (n351)
      3'b100: n354 = data_bus;
      3'b010: n354 = io_output_port_10;
      3'b001: n354 = io_output_port_10;
      default: n354 = io_output_port_10;
    endcase
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:549:17  */
  assign n355 = n339 ? io_output_port_8 : n352;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:549:17  */
  assign n356 = n339 ? io_output_port_9 : n353;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:549:17  */
  assign n357 = n339 ? io_output_port_10 : n354;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:541:13  */
  assign n367 = n328 ? 1'b1 : 1'b0;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:523:35  */
  assign n370 = n328 & phi2_rising;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:523:35  */
  assign n372 = n328 & phi2_rising;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:523:35  */
  assign n374 = n328 & phi2_rising;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:609:26  */
  assign n405 = {2'b00, int_vector};
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:609:39  */
  assign n407 = {n405, 3'b101};
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:609:67  */
  assign n408 = s1_int & s2_int;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:609:95  */
  assign n409 = ~s0_int;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:609:84  */
  assign n410 = n409 & n408;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:609:48  */
  assign n411 = n410 ? n407 : n419;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:610:69  */
  assign n412 = is_t3 | is_t4;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:610:84  */
  assign n413 = n412 | is_t5;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:610:52  */
  assign n414 = n413 & is_io;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:611:55  */
  assign n415 = latched_address[13:12]; // extract
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:611:70  */
  assign n417 = n415 == 2'b00;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:610:100  */
  assign n418 = n417 & n414;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:609:102  */
  assign n419 = n418 ? io_input_data : n425;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:612:38  */
  assign n420 = ~is_io;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:612:44  */
  assign n421 = rom_selected & n420;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:612:84  */
  assign n422 = is_t3 | is_t4;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:612:99  */
  assign n423 = n422 | is_t5;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:612:67  */
  assign n424 = n423 & n421;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:611:78  */
  assign n425 = n424 ? rom_d : n431;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:613:45  */
  assign n426 = ~is_io;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:613:51  */
  assign n427 = ram_selected & n426;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:613:91  */
  assign n428 = is_t3 | is_t4;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:613:106  */
  assign n429 = n428 | is_t5;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:613:74  */
  assign n430 = n429 & n427;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:612:116  */
  assign n431 = n430 ? ram_data_out : 8'b00000000;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:617:30  */
  assign n433 = cpu_data_oe ? cpu_data_out : cpu_data_in;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:418:9  */
  always @(posedge clk_in)
    n435 <= n217;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:523:9  */
  assign n436 = n370 ? n355 : io_output_port_8;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:523:9  */
  always @(posedge clk_in or posedge reset)
    if (reset)
      n437 <= 8'b00000000;
    else
      n437 <= n436;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:523:9  */
  assign n438 = n372 ? n356 : io_output_port_9;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:523:9  */
  always @(posedge clk_in or posedge reset)
    if (reset)
      n439 <= 8'b00000000;
    else
      n439 <= n438;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:523:9  */
  assign n440 = n374 ? n357 : io_output_port_10;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:523:9  */
  always @(posedge clk_in or posedge reset)
    if (reset)
      n441 <= 8'b00000000;
    else
      n441 <= n440;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:311:9  */
  assign n442 = phi1_rising ? n91 : latched_address;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:311:9  */
  always @(posedge clk_in or posedge reset)
    if (reset)
      n443 <= 14'b00000000000000;
    else
      n443 <= n442;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:523:9  */
  assign n444 = phi2_rising ? n367 : io_write_strobe;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:523:9  */
  always @(posedge clk_in or posedge reset)
    if (reset)
      n445 <= 1'b0;
    else
      n445 <= n444;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:523:9  */
  assign n446 = phi2_rising ? n322 : io_read_strobe;
  /* /Users/hambook/Development/intel-8008-vhdl/src/b8008/b8008_top.vhdl:523:9  */
  always @(posedge clk_in or posedge reset)
    if (reset)
      n447 <= 1'b0;
    else
      n447 <= n446;
endmodule

