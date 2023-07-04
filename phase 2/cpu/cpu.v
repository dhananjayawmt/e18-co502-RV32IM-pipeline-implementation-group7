`include "modules/alu_module/alu.v"

`include "modules/controlUnit_module/control_unit.v"

`include "modules/dataMemory_module/dcache.v"
`include "modules/dataMemory_module/dcache.v"

`include "modules/hazardHandling_module/alu_hazard_unit.v"
`include "modules/hazardHandling_module/branch_hazard_unit.v"
`include "modules/hazardHandling_module/load_hazard_unit.v"

`include "modules/ImmediateEncoder_module/Immediate_encoder.v"

`include "modules/instructionMemory_module/icache.v"
`include "modules/instructionMemory_module/instr_memory.v"

`include "modules/pipeLine_registers/EX_MEM_pipeline_reg.v"
`include "modules/pipeLine_registers/ID_EX_pipeline_reg.v"
`include "modules/pipeLine_registers/IF_ID_pipeline_reg.v"
`include "modules/pipeLine_registers/MEM_WB_pipeline_reg.v"

`include "modules/regFile_module/reg_file.v"

module CPU(
    input [31:0] INSTRUCTION,
    input CLK,
    input RESET,
    input INS_CACHE_BUSY_WAIT,
    input [31:0] DATA_CACHE_READ_DATA,
    input DATA_CACHE_BUSY_WAIT,
    output reg[31:0] PC,
    output [3:0] memReadEn,
    output [2:0] memWriteEn,
    output [31:0] DATA_CACHE_ADDR,
    output [31:0] DATA_CACHE_DATA,
    output reg insReadEn
);

  wire [31:0] INSTRUCTION_CACHE_READ_DATA;
  wire [31:0] REGISTER_FILE_DATA1;
  wire [31:0] REGISTER_FILE_DATA2;

  alu ALU_inst(
    .DATA1(REGISTER_FILE_DATA1),
    .DATA2(ID_EX_pipeline_reg_inst.OUT_DATA2),
    .RESULT(ALU_result),
    .SELECT(ID_EX_pipeline_reg_inst.OUT_ALU_OP)
  );

  control_unit control_unit_inst(
    .INSTRUCTION(INSTRUCTION_CACHE_READ_DATA),
    .RESET(RESET),
    .OP1_SEL(ID_EX_pipeline_reg_inst.OUT_OP1_SEL),
    .OP2_SEL(ID_EX_pipeline_reg_inst.OUT_OP2_SEL),
    .REG_WRITE_EN(ID_EX_pipeline_reg_inst.OUT_REG_WRITE_EN),
    .IMM_SEL(ID_EX_pipeline_reg_inst.OUT_IMMEDIATE[11:9]),
    .BR_SEL(ID_EX_pipeline_reg_inst.OUT_BR_SEL),
    .ALU_OP(ID_EX_pipeline_reg_inst.OUT_ALU_OP),
    .MEM_WRITE(ID_EX_pipeline_reg_inst.OUT_MEM_WRITE),
    .MEM_READ(ID_EX_pipeline_reg_inst.OUT_MEM_READ),
    .REG_WRITE_SEL(ID_EX_pipeline_reg_inst.OUT_REG_WRITE_SEL)
  );

  dcache dcache_inst(
    .CLK(CLK),
    .RESET(RESET),
    .READ(memReadEn),
    .WRITE(memWriteEn),
    .ADDRESS(DATA_CACHE_ADDR),
    .WRITE_DATA(DATA_CACHE_DATA),
    .MEM_READ_DATA(DATA_CACHE_READ_DATA),
    .MEM_BUSY_WAIT(DATA_CACHE_BUSY_WAIT),
    .READ_DATA(REGISTER_FILE_DATA2),
    .MEM_ADDRESS(ID_EX_pipeline_reg_inst.OUT_DATA2[11:4]),
    .BUSY_WAIT(DATA_CACHE_BUSY_WAIT),
    .MAIN_MEM_READ(memReadEn[3]),
    .MAIN_MEM_WRITE(memWriteEn[2])
  );

  alu_hazard_unit alu_hazard_unit_inst(
    .CLK(CLK),
    .RESET(RESET),
    .DEST_MEM(ex_mem_pipeline_reg_inst.OUT_DESTINATION_REG_ADDRESS),
    .DEST_ALU(ex_mem_pipeline_reg_inst.OUT_DESTINATION_REG_ADDRESS),
    .RS1_ID(ID_EX_pipeline_reg_inst.OUT_DESTINATION_REG_ADDRESS),
    .RS2_ID(ID_EX_pipeline_reg_inst.OUT_OP2_SEL),
    .ENABLE_RS1_MEM_STAGE(alu_rs1_mem_stage_enable),
    .ENABLE_RS2_MEM_STAGE(alu_rs2_mem_stage_enable),
    .ENABLE_RS1_WB_STAGE(alu_rs1_wb_stage_enable),
    .ENABLE_RS2_WB_STAGE(alu_rs2_wb_stage_enable)
  );

  branch_hazard_unit branch_hazard_unit_inst(
    .ID_pc(IF_ID_pipeline_reg_inst.OUT_PC[2:0]),
    .ALU_pc(ID_EX_pipeline_reg_inst.OUT_PC[2:0]),
    .reset(RESET),
    .ID_stage_branch(control_unit_inst.OUT_ID_STAGE_BRANCH),
    .ALU_stage_branch(control_unit_inst.OUT_ALU_STAGE_BRANCH),
    .ALU_stage_branch_result(ALU_result[0]),
    .flush(branch_flush),
    .early_prediction_is_branch_taken(branch_taken_early_prediction),
    .signal_to_take_branch(signal_to_take_branch)
  );

  load_use_hazard_unit load_use_hazard_unit_inst(
    .CLK(CLK),
    .RESET(RESET),
    .LOAD_SIG(ID_EX_pipeline_reg_inst.OUT_MEM_READ),
    .MEM_Rd(ex_mem_pipeline_reg_inst.OUT_DESTINATION_REG_ADDRESS),
    .ALU_RS1(ID_EX_pipeline_reg_inst.OUT_DESTINATION_REG_ADDRESS),
    .ALU_RS2(ID_EX_pipeline_reg_inst.OUT_OP2_SEL),
    .FRWD_RS1_WB(forward_rs1_wb),
    .FRWD_RS2_WB(forward_rs2_wb),
    .BUBBLE(bubble)
  );

  immediate_encoder immediate_encoder_inst(
    .INSTRUCTION(INSTRUCTION),
    .IMM_SEL(ID_EX_pipeline_reg_inst.OUT_IMMEDIATE[11:9]),
    .IMMEDIATE(ID_EX_pipeline_reg_inst.OUT_IMMEDIATE)
  );

  ins_cache_memory ins_cache_memory_inst(
    .clock(CLK),
    .reset(RESET),
    .in_read(insReadEn),
    .in_address(PC),
    .out_readdata(INSTRUCTION_CACHE_READ_DATA),
    .out_busywait(INS_CACHE_BUSY_WAIT),
    .out_MAIN_MEM_READ(memReadEn[0]),
    .out_MAIN_MEM_ADDRESS(PC[27:0]),
    .in_MAIN_MEM_READ_DATA(DATA_CACHE_READ_DATA),
    .in_MAIN_MEM_BUSY_WAIT(DATA_CACHE_BUSY_WAIT)
  );

  reg_file REGISTER_FILE_inst(
    .CLK(CLK),
    .RESET(RESET),
    .IN(ID_EX_pipeline_reg_inst.OUT_DESTINATION_REG_ADDRESS),
    .OUT1(REGISTER_FILE_DATA1),
    .OUT2(REGISTER_FILE_DATA2),
    .INADDRESS(ID_EX_pipeline_reg_inst.OUT_DESTINATION_REG_ADDRESS),
    .OUT1ADDRESS(ID_EX_pipeline_reg_inst.OUT_DESTINATION_REG_ADDRESS),
    .OUT2ADDRESS(ID_EX_pipeline_reg_inst.OUT_OP2_SEL),
    .WRITE(mem_wb_pipeline_reg_inst.OUT_REG_WRITE_EN),
    .RW_Write_Register(mem_wb_pipeline_reg_inst.OUT_DATA_OUT)
  );

  // Connect pipeline stages
  IF_ID_pipeline_reg IF_ID_pipeline_reg_inst(
    .IN_INSTRUCTION(INSTRUCTION_CACHE_READ_DATA),
    .IN_PC(PC),
    .CLK(CLK),
    .RESET(RESET),
    .BUSY_WAIT(INS_CACHE_BUSY_WAIT),
    .OUT_INSTRUCTION(IF_ID_instruction),
    .OUT_PC(IF_ID_pc)
  );

  ID_EX_pipeline_reg ID_EX_pipeline_reg_inst(
    .IN_DESTINATION_REG_ADDRESS(IF_ID_pipeline_reg_inst.OUT_DESTINATION_REG_ADDRESS),
    .IN_ALU_OP(control_unit_inst.OUT_ALU_OP),
    .IN_PC(IF_ID_pipeline_reg_inst.OUT_PC),
    .IN_DATA1(REGISTER_FILE_DATA1),
    .IN_DATA2(REGISTER_FILE_DATA2),
    .IN_IMMEDIATE(immediate_encoder_inst.OUT_IMMEDIATE),
    .IN_BR_SEL(control_unit_inst.OUT_BR_SEL),
    .IN_MEM_READ(control_unit_inst.OUT_MEM_READ),
    .IN_MEM_WRITE(control_unit_inst.OUT_MEM_WRITE),
    .IN_REG_WRITE_SEL(control_unit_inst.OUT_REG_WRITE_SEL),
    .IN_OP1_SEL(control_unit_inst.OUT_OP1_SEL),
    .IN_OP2_SEL(control_unit_inst.OUT_OP2_SEL),
    .IN_REG_WRITE_EN(control_unit_inst.OUT_REG_WRITE_EN),
    .CLK(CLK),
    .RESET(RESET),
    .BUSY_WAIT(branch_flush),
    .OUT_DESTINATION_REG_ADDRESS(ID_EX_dest_reg_address),
    .OUT_ALU_OP(ID_EX_alu_op),
    .OUT_PC(ID_EX_pc),
    .OUT_DATA1(ID_EX_data1),
    .OUT_DATA2(ID_EX_data2),
    .OUT_IMMEDIATE(ID_EX_immediate),
    .OUT_BR_SEL(ID_EX_br_sel),
    .OUT_MEM_READ(ID_EX_mem_read),
    .OUT_MEM_WRITE(ID_EX_mem_write),
    .OUT_REG_WRITE_SEL(ID_EX_reg_write_sel),
    .OUT_OP1_SEL(ID_EX_op1_sel),
    .OUT_OP2_SEL(ID_EX_op2_sel),
    .OUT_REG_WRITE_EN(ID_EX_reg_write_en)
  );

  EX_MEM_pipeline_reg ex_mem_pipeline_reg_inst(
    .IN_DESTINATION_REG_ADDRESS(ID_EX_pipeline_reg_inst.OUT_DESTINATION_REG_ADDRESS),
    .IN_PC(ID_EX_pipeline_reg_inst.OUT_PC),
    .IN_ALU_OUT(ALU_result),
    .IN_DATA2(ID_EX_pipeline_reg_inst.OUT_DATA2),
    .IN_MEM_WRITE(ID_EX_pipeline_reg_inst.OUT_MEM_WRITE),
    .IN_MEM_READ(ID_EX_pipeline_reg_inst.OUT_MEM_READ),
    .IN_REG_WRITE_SEL(ID_EX_pipeline_reg_inst.OUT_REG_WRITE_SEL),
    .IN_REG_WRITE_EN(ID_EX_pipeline_reg_inst.OUT_REG_WRITE_EN),
    .CLK(CLK),
    .RESET(RESET),
    .BUSY_WAIT(bubble),
    .OUT_DESTINATION_REG_ADDRESS(EX_MEM_dest_reg_address),
    .OUT_PC(EX_MEM_pc),
    .OUT_ALU_OUT(EX_MEM_alu_out),
    .OUT_DATA2(EX_MEM_data2),
    .OUT_MEM_WRITE(EX_MEM_mem_write),
    .OUT_MEM_READ(EX_MEM_mem_read),
    .OUT_REG_WRITE_SEL(EX_MEM_reg_write_sel),
    .OUT_REG_WRITE_EN(EX_MEM_reg_write_en)
  );

  MEM_WB_pipeline_reg mem_wb_pipeline_reg_inst(
    .IN_DESTINATION_REG_ADDRESS(EX_MEM_pipeline_reg_inst.OUT_DESTINATION_REG_ADDRESS),
    .IN_UPDATED_PC(EX_MEM_pipeline_reg_inst.OUT_PC),
    .IN_ALU_OUT(EX_MEM_pipeline_reg_inst.OUT_ALU_OUT),
    .IN_DATA_OUT(DATA_CACHE_READ_DATA),
    .IN_REG_WRITE_SEL(EX_MEM_pipeline_reg_inst.OUT_REG_WRITE_SEL),
    .IN_REG_WRITE_EN(EX_MEM_pipeline_reg_inst.OUT_REG_WRITE_EN),
    .CLK(CLK),
    .RESET(RESET),
    .BUSY_WAIT(bubble),
    .OUT_DESTINATION_REG_ADDRESS(MEM_WB_dest_reg_address),
    .OUT_UPDATED_PC(MEM_WB_updated_pc),
    .OUT_ALU_OUT(MEM_WB_alu_out),
    .OUT_DATA_OUT(MEM_WB_data_out),
    .OUT_REG_WRITE_SEL(MEM_WB_reg_write_sel),
    .OUT_REG_WRITE_EN(MEM_WB_reg_write_en)
  );

  always @(posedge CLK) begin
    if (RESET)
      PC <= 0;
    else if (branch_flush)
      PC <= signal_to_take_branch ? ALU_result : PC + 4;
    else if (!bubble)
      PC <= PC + 4;
  end

  assign memReadEn = {1'b0, ex_mem_pipeline_reg_inst.OUT_MEM_READ, ex_mem_pipeline_reg_inst.OUT_MEM_READ, ex_mem_pipeline_reg_inst.OUT_MEM_READ};
  assign memWriteEn = {1'b0, ex_mem_pipeline_reg_inst.OUT_MEM_WRITE, ex_mem_pipeline_reg_inst.OUT_MEM_WRITE};

  assign DATA_CACHE_ADDR = ex_mem_pipeline_reg_inst.OUT_ALU_OUT;
  assign DATA_CACHE_DATA = ex_mem_pipeline_reg_inst.OUT_DATA2;

  assign insReadEn = IF_ID_pipeline_reg_inst.OUT_INSTRUCTION_VALID;

endmodule




