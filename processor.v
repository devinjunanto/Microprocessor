`timescale 1ns / 1ps

module processor(
	input clock,
	input reset,

	//these ports are used for serial IO and 
	//must be wired up to the data_memory module
	input [7:0] serial_in,
	input serial_valid_in,
	input serial_ready_in,
	output [7:0] serial_out,
	output serial_rden_out,
	output serial_wren_out
);

wire [31:0] addr_in;
wire [31:0] data_out;
wire [31:0] writedata_in;
wire re_in;
wire we_in;
wire [31:0] readdata_out;
wire [1:0] size_in;
wire [5:0] Func_in;
wire [31:0] A_in;
wire [31:0] B_in;
wire [31:0] O_out;
wire Branch_out;
wire Jump_out;
wire [31:0] PCchoose;
wire [4:0] ifItIsjal;
wire [31:0] ifItIsjalLong;
wire [31:0] ifItIslui;

wire Jump;
wire RegDst;
wire ALUSrc;
wire MemToReg;
wire write_en;
wire orrerResult;
wire notXor;
wire isjr;
wire isjal;
wire islui;
wire isLoad;
wire isLB;
wire notZero;

wire [31:0] extended;
wire [4:0] write_register;
wire [31:0] write_data;
wire [31:0] outputPC;
wire [31:0] branchTargetPC;
wire [31:0] adderResult;
wire [31:0] jumpTargetPC;
wire [31:0] PC_next;
wire [31:0] shiftLeft2Result;
wire [31:0] extendedLB;
wire [31:0] extendedLH;
wire [31:0] LBorLH;
wire [31:0] isLoadResult;
wire [7:0] beforeExtendedLB;
wire [15:0] beforeExtendedLH;

control control(
	.clock(clock),
	.inst_1(data_out[31:26]),
	.inst_2(data_out[5:0]),
	.inst_3(data_out[19:16]),
	.RegDst(RegDst),
	.Jump(Jump),
	.MemRead(re_in),
	.MemToReg(MemToReg),
	.Func_in(Func_in),
	.MemWrite(we_in),
	.ALUSrc(ALUSrc),
	.RegWrite(write_en),
	.size_in(size_in),
	.notXor(notXor),
	.isjr(isjr),
	.isjal(isjal),
	.islui(islui),
	.isLoad(isLoad),
	.isLB(isLB),
	.notZero(notZero)
);

pc pc(
	.clock(clock),
	.reset(reset),
	.PC_next(PC_next),
	.PC(outputPC)
);

adder adder(
	.clock(clock),
	.outputPC(outputPC),
	.out(addr_in)
);

shiftLeft2Add2bits shiftLeft2Add2bits(
	.theInput(data_out),
	.resultOfShiftLeft2Add2bits(jumpTargetPC[27:0])
);
fourinputmux fourinputmux(
	.clock(clock),
	.reset(reset),
	.condition(O_out[1:0]),
	.output1(readdata_out[31:0]),
	.theOutput(beforeExtendedLB)
);

muxForLB muxForLB(
	.clock(clock),
	.reset(reset),
	.condition(O_out[0]),
	.output1(readdata_out[31:0]),
	.theOutput(beforeExtendedLH)
);


assign jumpTargetPC[31:28] = addr_in[31:28];

shiftLeft2 shiftLeft2(
	.theInput(extended),
	.resultOfShiftLeft2(shiftLeft2Result)
);

adder2 adder2(
	.clock(clock),
	.inA(outputPC),
	.inB(shiftLeft2Result),
	.out(adderResult)
);

mux mux4(
	.clock(clock),
	.reset(reset),
	.condition(Jump),
	.output1(jumpTargetPC[31:0]),
	.output2(adderResult[31:0]),
	.theOutput(branchTargetPC[31:0])
);

mux mux5(
	.clock(clock),
	.reset(reset),
	.condition(orrerResult),
	.output1(branchTargetPC[31:0]),
	.output2(addr_in[31:0]),
	.theOutput(PCchoose[31:0])
);

inst_rom inst_rom(
	.clock(clock),
	.reset(reset),
	.addr_in(outputPC),
	.data_out(data_out)
);

smallMux mux1 (
	.clock(clock),
	.reset(reset),
	.condition(RegDst),
	.output1(data_out[15:11]),
	.output2(data_out[20:16]),
	.theOutput(ifItIsjal)
);
//assign write_register = (RegDst) ? data_out[15:11] : data_out[20:16];

smallMux mux7(
	.clock(clock),
	.reset(reset),
	.condition(isjal),
	.output1(5'b11111),
	.output2(ifItIsjal[4:0]),
	.theOutput(write_register)
);

mux mux8(
	.clock(clock),
	.reset(reset),
	.condition(isjal),
	.output1(outputPC),
	.output2(ifItIsjalLong),
	.theOutput(ifItIslui)
);

mux mux9(
	.clock(clock),
	.reset(reset),
	.condition(islui),
	.output1({data_out[15:0],16'b0}),
	.output2(ifItIslui),
	.theOutput(write_data)
);


register register(
	.clock(clock),
	.reset(reset),
	.read_register_1(data_out[25:21]),
	.read_register_2(data_out[20:16]),
	.write_register(write_register),
	.write_data(write_data),
	.write_en(write_en),
	.read_data_1(A_in[31:0]),
	.read_data_2(writedata_in[31:0])
);
mux mux6(
	.clock(clock),
	.reset(reset),
	.condition(isjr),
	.output1(A_in[31:0]),
	.output2(PCchoose[31:0]),
	.theOutput(PC_next)
);

extension extension(
	.data_out(data_out[15:0]),
	.notXor(~notXor),
	.extended(extended[31:0])
);

extensionForLB extensionForLB(
	.data_out(beforeExtendedLB[7:0]),
	.notXor(~notZero),
	.extended(extendedLB)
);

extension extensionForLH(
	.data_out(beforeExtendedLH[15:0]),
	.notXor(~notZero),
	.extended(extendedLH)
);

//assign extended[31:16] = 16'b0;
//assign extended[15:0] = data_out[15:0];

mux mux2 (
	.clock(clock),
	.reset(reset),
	.condition(ALUSrc),
	.output1(extended[31:0]),
	.output2(writedata_in[31:0]),
	.theOutput(B_in)
);
//assign B_in = (ALUSrc) ? extended[31:0] : writedata_in[31:0];

mux mux10(
	.clock(clock),
	.reset(reset),
	.condition(isLoad),
	.output1(LBorLH[31:0]),
	.output2(readdata_out[31:0]),
	.theOutput(isLoadResult)
);

mux mux11(
	.clock(clock),
	.reset(reset),
	.condition(isLB),
	.output1(extendedLB[31:0]),
	.output2(extendedLH[31:0]),
	.theOutput(LBorLH)
);

alu alu(
	.Func_in(Func_in),
	.A_in(A_in),
	.B_in(B_in),
	.O_out(O_out),
	.Branch_out(Branch_out),
	.Jump_out(Jump_out)
);

orrer orrer(
	.theInput1(Branch_out),
	.theInput2(Jump_out),
	.theOutputOfTheOrrer(orrerResult)
);


data_memory data_memory(
	.clock(clock),
	.reset(reset),
	.addr_in(O_out),
	.writedata_in(writedata_in),
	.re_in(re_in),
	.we_in(we_in),
	.readdata_out(readdata_out),
	.size_in(size_in),
	.serial_in(serial_in),
	.serial_ready_in(serial_ready_in),
	.serial_valid_in(serial_valid_in),
	.serial_out(serial_out),
	.serial_rden_out(serial_rden_out),
	.serial_wren_out(serial_wren_out)
);

mux mux3 (
	.clock(clock),
	.reset(reset),
	.condition(MemToReg),
	.output1(isLoadResult[31:0]),
	.output2(O_out[31:0]),
	.theOutput(ifItIsjalLong)
);

//assign write_data = (MemToReg) ? readdata_out[31:0] : O_out[31:0];

endmodule //processor


module pc(
	input clock,
	input reset,
	input [31:0] PC_next,
	output[31:0] PC
);
reg [31:0] regPC;
wire [31:0] PCnext;
assign PCnext = PC_next;
always@(posedge clock)
	begin
		if(reset)
		begin
			regPC <= 23'b10000000000000000000000;
		end
		else
		begin
			regPC <= PCnext;
		end

	end
assign PC = regPC;
endmodule //pc

module extension(
	input [15:0] data_out,
	input notXor,
	output [31:0] extended
);
	assign extended [31:16] = (data_out[15] & notXor) ? 16'b1111111111111111 : 16'b0;
	assign extended [15:0] = data_out[15:0];
endmodule

module fourinputmux(
	input clock,
	input reset,
	input [1:0] condition,
	input [31:0] output1,
	output reg [7:0] theOutput
);
always @(*) begin
	if(condition == 2'b0) begin
		theOutput <= output1[7:0];
	end
	else if(condition == 2'b01) begin
		theOutput <= output1[15:8];
	end
	else if(condition == 2'b10) begin
		theOutput <= output1[24:16];
	end
	else if(condition == 2'b11) begin
		theOutput <= output1[31:24];
	end
end
endmodule

module muxForLB(
	input clock,
	input reset,
	input condition,
	input [31:0] output1,
	output reg [15:0] theOutput
);
always @(*) begin
	theOutput <= (condition) ? output1[31:16] : output1[15:0];
end
endmodule

module adder(
	input clock,
	input [31:0] outputPC,
	output[31:0] out
);
reg [31:0] regOutputPC;
reg [31:0] regOut;
wire [31:0] wireOut;

assign wireOut = regOutputPC + 4;
assign out = regOut;

always@(*)
	begin
		regOutputPC <= outputPC;
		regOut <= wireOut;
	end
endmodule //adder


module register(
	input clock,
	input reset,
	input [4:0]read_register_1,
	input [4:0]read_register_2,
	input [4:0]write_register,
	input [31:0]write_data,
	input write_en,
	output [31:0]read_data_1,
	output [31:0]read_data_2
);

reg [31:0] result[31:0];
always@(posedge clock)
begin
	if(reset) begin
		result[0] <= 0;
		result[1] <= 0;
		result[2] <= 0;
		result[3] <= 0;
		result[4] <= 0;
		result[5] <= 0;
		result[6] <= 0;
		result[7] <= 0;
		result[8] <= 0;
		result[9] <= 0;
		result[10] <= 0;
		result[11] <= 0;
		result[12] <= 0;
		result[13] <= 0;
		result[14] <= 0;
		result[15] <= 0;
		result[16] <= 0;
		result[17] <= 0;
		result[18] <= 0;
		result[19] <= 0;
		result[20] <= 0;
		result[21] <= 0;
		result[22] <= 0;
		result[23] <= 0;
		result[24] <= 0;
		result[25] <= 0;
		result[26] <= 0;
		result[27] <= 0;
		result[28] <= 0;
		result[29] <= 0;
		result[30] <= 0;
		result[31] <= 0;
	end
	else begin
		if (write_en) begin
			result[write_register] <= write_data;
		end
	end
	result[0] <= 0;
	
end
assign read_data_1 = result[read_register_1];
assign read_data_2 = result[read_register_2];
endmodule //register

module mux(
	input clock,
	input reset,
	input condition,
	input [31:0] output1,
	input [31:0] output2,
	output reg [31:0] theOutput
);

always @(*) begin
	theOutput <= (condition) ? output1 : output2;
end
endmodule

module smallMux(
	input clock,
	input reset,
	input condition,
	input [4:0] output1,
	input [4:0] output2,
	output reg [4:0] theOutput
);

always @(*) begin
	theOutput <= (condition) ? output1 : output2;
end

endmodule


module control(
	input clock,
	input [5:0] inst_1,
	input [5:0] inst_2,
	input [3:0] inst_3,
	output RegDst,
	output Jump,
	output MemRead,
	output MemToReg,
	output [5:0] Func_in,
	output MemWrite,
	output ALUSrc,
	output RegWrite,
	output [1:0] size_in,
	output notXor,
	output isjr,
	output isjal,
	output islui,
	output isLoad,
	output isLB,
	output notZero
);

reg [5:0] regFunc_in;
reg regRegDst;
reg regALUSrc;
reg regRegWrite;
reg regMemRead;
reg regMemWrite;
reg regMemToReg;
reg regJump;
reg [1:0] regSize_in;
reg regnotXor;
reg regisjr;
reg regisjal;
reg regislui;
reg regisLoad;
reg regisLB;
reg regnotZero;

always @(*) begin
	if(inst_1 == 6'b100011) begin //0x23
		regFunc_in = 6'b100000;
		regRegDst = 1'b0;
		regALUSrc = 1'b1;
		regRegWrite = 1'b1;
		regMemRead = 1'b1;
		regMemWrite = 1'b0;
		regMemToReg = 1'b1;
		regJump = 1'b0;
		regSize_in = 2'b11;
		regnotXor = 1'b0;
		regisjr = 1'b0;
		regisjal = 1'b0;
		regislui = 1'b0;
		regisLoad = 1'b0;
		regisLB = 1'b0;
		regnotZero = 1'b0;
	end
	else if(inst_1 == 6'b101011) begin //0x2b
		regFunc_in = 6'b100000;
		regRegDst = 1'b0;
		regALUSrc = 1'b1;
		regRegWrite = 1'b0;
		regMemRead = 1'b0;
		regMemWrite = 1'b1;
		regMemToReg = 1'b0;
		regJump = 1'b0;
		regSize_in = 2'b11;
		regnotXor = 1'b0;
		regisjr = 1'b0;
		regisjal = 1'b0;
		regislui = 1'b0;
		regisLoad = 1'b0;
		regisLB = 1'b0;
		regnotZero = 1'b0;
	end
	else if(inst_1 == 6'b001000) begin //0x8
		regFunc_in = 6'b100000;
		regRegDst = 1'b0;
		regALUSrc = 1'b1;
		regRegWrite = 1'b1;
		regMemRead = 1'b0;
		regMemWrite = 1'b0;
		regMemToReg = 1'b0;
		regJump = 1'b0;
		regSize_in = 2'b11;
		regnotXor = 1'b0;
		regisjr = 1'b0;
		regisjal = 1'b0;
		regislui = 1'b0;
		regisLoad = 1'b0;
		regisLB = 1'b0;
		regnotZero = 1'b0;
	end
	else if(inst_1 == 6'b100000) begin //0x20
		regFunc_in = 6'b100000;
		regRegDst = 1'b0;
		regALUSrc = 1'b1;
		regRegWrite = 1'b1;
		regMemRead = 1'b1;
		regMemWrite = 1'b0;
		regMemToReg = 1'b1;
		regJump = 1'b0;
		regSize_in = 2'b00;
		regnotXor = 1'b0;
		regisjr = 1'b0;
		regisjal = 1'b0;
		regislui = 1'b0;
		regisLoad = 1'b1;
		regisLB = 1'b1;
		regnotZero = 1'b0;
	end
	else if(inst_1 == 6'b100001) begin //0x21
		regFunc_in = 6'b100000;
		regRegDst = 1'b0;
		regALUSrc = 1'b1;
		regRegWrite = 1'b1;
		regMemRead = 1'b1;
		regMemWrite = 1'b0;
		regMemToReg = 1'b1;
		regJump = 1'b0;
		regSize_in = 2'b01;
		regnotXor = 1'b0;
		regisjr = 1'b0;
		regisjal = 1'b0;
		regislui = 1'b0;
		regisLoad = 1'b1;
		regisLB = 1'b0;
		regnotZero = 1'b0;
	end
	else if(inst_1 == 6'b101000) begin //0x28
		regFunc_in = 6'b100000;
		regRegDst = 1'b0;
		regALUSrc = 1'b1;
		regRegWrite = 1'b0;
		regMemRead = 1'b0;
		regMemWrite = 1'b1;
		regMemToReg = 1'b0;
		regJump = 1'b0;
		regSize_in = 2'b00;
		regnotXor = 1'b0;
		regisjr = 1'b0;
		regisjal = 1'b0;
		regislui = 1'b0;
		regisLoad = 1'b0;
		regisLB = 1'b0;
		regnotZero = 1'b0;
	end
	else if(inst_1 == 6'b101001) begin //0x29
		regFunc_in = 6'b100000;
		regRegDst = 1'b0;
		regALUSrc = 1'b1;
		regRegWrite = 1'b0;
		regMemRead = 1'b0;
		regMemWrite = 1'b1;
		regMemToReg = 1'b0;
		regJump = 1'b0;
		regSize_in = 2'b01;
		regnotXor = 1'b0;
		regisjr = 1'b0;
		regisjal = 1'b0;
		regislui = 1'b0;
		regisLoad = 1'b0;
		regisLB = 1'b0;
		regnotZero = 1'b0;
	end
	else if(inst_1 == 6'b100100) begin //0x24
		regFunc_in = 6'b100000;
		regRegDst = 1'b0;
		regALUSrc = 1'b1;
		regRegWrite = 1'b1;
		regMemRead = 1'b1;
		regMemWrite = 1'b0;
		regMemToReg = 1'b1;
		regJump = 1'b0;
		regSize_in = 2'b00;
		regnotXor = 1'b0;
		regisjr = 1'b0;
		regisjal = 1'b0;
		regislui = 1'b0;
		regisLoad = 1'b1;
		regisLB = 1'b1;
		regnotZero = 1'b1;
	end
	else if(inst_1 == 6'b100101) begin //0x25
		regFunc_in = 6'b100000;
		regRegDst = 1'b0;
		regALUSrc = 1'b1;
		regRegWrite = 1'b1;
		regMemRead = 1'b1;
		regMemWrite = 1'b0;
		regMemToReg = 1'b1;
		regJump = 1'b0;
		regSize_in = 2'b01;
		regnotXor = 1'b0;
		regisjr = 1'b0;
		regisjal = 1'b0;
		regislui = 1'b0;
		regisLoad = 1'b1;
		regisLB = 1'b0;
		regnotZero = 1'b1;
	end
	else if(inst_1 == 6'b000100) begin //0x4
		regFunc_in = 6'b111100;
		regRegDst = 1'b0;
		regALUSrc = 1'b0;
		regRegWrite = 1'b0;
		regMemRead = 1'b0;
		regMemWrite = 1'b0;
		regMemToReg = 1'b0;
		regJump = 1'b0;
		regSize_in = 2'b11;
		regnotXor = 1'b0;
		regisjr = 1'b0;
		regisjal = 1'b0;
		regislui = 1'b0;
		regisLoad = 1'b0;
		regisLB = 1'b0;
		regnotZero = 1'b0;
	end
	else if(inst_1 == 6'b000101) begin //0x5
		regFunc_in = 6'b111101;
		regRegDst = 1'b0;
		regALUSrc = 1'b0;
		regRegWrite = 1'b0;
		regMemRead = 1'b0;
		regMemWrite = 1'b0;
		regMemToReg = 1'b0;
		regJump = 1'b0;
		regSize_in = 2'b11;
		regnotXor = 1'b0;
		regisjr = 1'b0;
		regisjal = 1'b0;
		regislui = 1'b0;
		regisLoad = 1'b0;
		regisLB = 1'b0;
		regnotZero = 1'b0;
	end
	else if(inst_1 == 6'b000001) begin //0x1
		if(inst_3 == 4'b0000) begin //bltz
			regFunc_in = 6'b111000;
			regRegDst = 1'b0;
			regALUSrc = 1'b0;
			regRegWrite = 1'b0;
			regMemRead = 1'b0;
			regMemWrite = 1'b0;
			regMemToReg = 1'b0;
			regJump = 1'b0;
			regSize_in = 2'b11;
			regnotXor = 1'b0;
			regisjr = 1'b0;
			regisjal = 1'b0;
			regislui = 1'b0;
			regisLoad = 1'b0;
			regisLB = 1'b0;
			regnotZero = 1'b0;
		end
		else if(inst_3 == 4'b0001) begin //bgez
			regFunc_in = 6'b111001;
			regRegDst = 1'b0;
			regALUSrc = 1'b0;
			regRegWrite = 1'b0;
			regMemRead = 1'b0;
			regMemWrite = 1'b0;
			regMemToReg = 1'b0;
			regJump = 1'b0;
			regSize_in = 2'b11;
			regnotXor = 1'b0;
			regisjr = 1'b0;
			regisjal = 1'b0;
			regislui = 1'b0;
			regisLoad = 1'b0;
			regisLB = 1'b0;
			regnotZero = 1'b0;
		end
	end
	else if(inst_1 == 6'b000110) begin //0x6
		regFunc_in = 6'b111110;
		regRegDst = 1'b0;
		regALUSrc = 1'b0;
		regRegWrite = 1'b0;
		regMemRead = 1'b0;
		regMemWrite = 1'b0;
		regMemToReg = 1'b0;
		regJump = 1'b0;
		regSize_in = 2'b11;
		regnotXor = 1'b0;
		regisjr = 1'b0;
		regisjal = 1'b0;
		regislui = 1'b0;
		regisLoad = 1'b0;
		regisLB = 1'b0;
		regnotZero = 1'b0;
	end
	else if(inst_1 == 6'b000111) begin //0x7
		regFunc_in = 6'b111111;
		regRegDst = 1'b0;
		regALUSrc = 1'b0;
		regRegWrite = 1'b0;
		regMemRead = 1'b0;
		regMemWrite = 1'b0;
		regMemToReg = 1'b0;
		regJump = 1'b0;
		regSize_in = 2'b11;
		regnotXor = 1'b0;
		regisjr = 1'b0;
		regisjal = 1'b0;
		regislui = 1'b0;
		regisLoad = 1'b0;
		regisLB = 1'b0;
		regnotZero = 1'b0;
	end
	else if(inst_1 == 6'b001001) begin //0x9
		regFunc_in = 6'b100001;
		regRegDst = 1'b0;
		regALUSrc = 1'b1;
		regRegWrite = 1'b1;
		regMemRead = 1'b0;
		regMemWrite = 1'b0;
		regMemToReg = 1'b0;
		regJump = 1'b0;
		regSize_in = 2'b11;
		regnotXor = 1'b0;
		regisjr = 1'b0;
		regisjal = 1'b0;
		regislui = 1'b0;
		regisLoad = 1'b0;
		regisLB = 1'b0;
		regnotZero = 1'b0;
	end
	else if(inst_1 == 6'b001100) begin //0xC
		regFunc_in = 6'b100100;
		regRegDst = 1'b0;
		regALUSrc = 1'b1;
		regRegWrite = 1'b1;
		regMemRead = 1'b0;
		regMemWrite = 1'b0;
		regMemToReg = 1'b0;
		regJump = 1'b0;
		regSize_in = 2'b11;
		regnotXor = 1'b1;
		regisjr = 1'b0;
		regisjal = 1'b0;
		regislui = 1'b0;
		regisLoad = 1'b0;
		regisLB = 1'b0;
		regnotZero = 1'b0;
	end
	else if(inst_1 == 6'b001101) begin //0xD
		regFunc_in = 6'b100101;
		regRegDst = 1'b0;
		regALUSrc = 1'b1;
		regRegWrite = 1'b1;
		regMemRead = 1'b0;
		regMemWrite = 1'b0;
		regMemToReg = 1'b0;
		regJump = 1'b0;
		regSize_in = 2'b11;
		regnotXor = 1'b1;
		regisjr = 1'b0;
		regisjal = 1'b0;
		regislui = 1'b0;
		regisLoad = 1'b0;
		regisLB = 1'b0;
		regnotZero = 1'b0;
	end
	else if(inst_1 == 6'b001110) begin //0xE
		regFunc_in = 6'b100110;
		regRegDst = 1'b0;
		regALUSrc = 1'b1;
		regRegWrite = 1'b1;
		regMemRead = 1'b0;
		regMemWrite = 1'b0;
		regMemToReg = 1'b0;
		regJump = 1'b0;
		regSize_in = 2'b11;
		regnotXor = 1'b1;
		regisjr = 1'b0;
		regisjal = 1'b0;
		regislui = 1'b0;
		regisLoad = 1'b0;
		regisLB = 1'b0;
		regnotZero = 1'b0;
	end
	else if(inst_1 == 6'b000010) begin //0x2
		regFunc_in = 6'b111010;
		regRegDst = 1'b0;
		regALUSrc = 1'b0;
		regRegWrite = 1'b0;
		regMemRead = 1'b0;
		regMemWrite = 1'b0;
		regMemToReg = 1'b0;
		regJump = 1'b1;
		regSize_in = 2'b11;
		regnotXor = 1'b0;
		regisjr = 1'b0;
		regisjal = 1'b0;
		regislui = 1'b0;
		regisLoad = 1'b0;
		regisLB = 1'b0;
		regnotZero = 1'b0;
	end
	else if(inst_1 == 6'b001111) begin //0xF
		regFunc_in = 6'b100111;
		regRegDst = 1'b0;
		regALUSrc = 1'b1;
		regRegWrite = 1'b1;
		regMemRead = 1'b0;
		regMemWrite = 1'b0;
		regMemToReg = 1'b0;
		regJump = 1'b0;
		regSize_in = 2'b11;
		regnotXor = 1'b0;
		regisjr = 1'b0;
		regisjal = 1'b0;
		regislui = 1'b1;
		regisLoad = 1'b0;
		regisLB = 1'b0;
		regnotZero = 1'b0;
	end
	else if(inst_1 == 6'b000011) begin //0x3
		regFunc_in = 6'b111011;
		regRegDst = 1'b1;
		regALUSrc = 1'b0;
		regRegWrite = 1'b1;
		regMemRead = 1'b0;
		regMemWrite = 1'b0;
		regMemToReg = 1'b0;
		regJump = 1'b1;
		regSize_in = 2'b11;
		regnotXor = 1'b0;
		regisjr = 1'b0;
		regisjal = 1'b1;
		regislui = 1'b0;
		regisLoad = 1'b0;
		regisLB = 1'b0;
		regnotZero = 1'b0;
	end
	else if(inst_1 == 0) begin //0x0
		if(inst_2 == 6'b100000) begin //0x20
			regFunc_in = 6'b100000;
			regRegDst = 1'b1;
			regALUSrc = 1'b0;
			regRegWrite = 1'b1;
			regMemRead = 1'b0;
			regMemWrite = 1'b0;
			regMemToReg = 1'b0;
			regJump = 1'b0;
			regSize_in = 2'b11;
			regnotXor = 1'b0;
			regisjr = 1'b0;
			regisjal = 1'b0;
			regislui = 1'b0;
			regisLoad = 1'b0;
			regisLB = 1'b0;
			regnotZero = 1'b0;
		end
		else if(inst_2 == 6'b100010) begin //0x22
			regFunc_in = 6'b100010;
			regRegDst = 1'b1;
			regALUSrc = 1'b0;
			regRegWrite = 1'b1;
			regMemRead = 1'b0;
			regMemWrite = 1'b0;
			regMemToReg = 1'b0;
			regJump = 1'b0;
			regSize_in = 2'b11;
			regnotXor = 1'b0;
			regisjr = 1'b0;
			regisjal = 1'b0;
			regislui = 1'b0;
			regisLoad = 1'b0;
			regisLB = 1'b0;
			regnotZero = 1'b0;
		end
		else if(inst_2 == 6'b100100) begin //0x24
			regFunc_in = 6'b100100;
			regRegDst = 1'b1;
			regALUSrc = 1'b0;
			regRegWrite = 1'b1;
			regMemRead = 1'b0;
			regMemWrite = 1'b0;
			regMemToReg = 1'b0;
			regJump = 1'b0;
			regSize_in = 2'b11;
			regnotXor = 1'b0;
			regisjr = 1'b0;
			regisjal = 1'b0;
			regislui = 1'b0;
			regisLoad = 1'b0;
			regisLB = 1'b0;
			regnotZero = 1'b0;
		end
		else if(inst_2 == 6'b100101) begin //0x25
			regFunc_in = 6'b100101;
			regRegDst = 1'b1;
			regALUSrc = 1'b0;
			regRegWrite = 1'b1;
			regMemRead = 1'b0;
			regMemWrite = 1'b0;
			regMemToReg = 1'b0;
			regJump = 1'b0;
			regSize_in = 2'b11;
			regnotXor = 1'b0;
			regisjr = 1'b0;
			regisjal = 1'b0;
			regislui = 1'b0;
			regisLoad = 1'b0;
			regisLB = 1'b0;
			regnotZero = 1'b0;
		end
		else if(inst_2 == 6'b100111) begin //0x27
			regFunc_in = 6'b100111;
			regRegDst = 1'b1;
			regALUSrc = 1'b0;
			regRegWrite = 1'b1;
			regMemRead = 1'b0;
			regMemWrite = 1'b0;
			regMemToReg = 1'b0;
			regJump = 1'b0;
			regSize_in = 2'b11;
			regnotXor = 1'b0;
			regisjr = 1'b0;
			regisjal = 1'b0;
			regislui = 1'b0;
			regisLoad = 1'b0;
			regisLB = 1'b0;
			regnotZero = 1'b0;
		end
		else if(inst_2 == 6'b100110) begin //0x26
			regFunc_in = 6'b100110;
			regRegDst = 1'b1;
			regALUSrc = 1'b0;
			regRegWrite = 1'b1;
			regMemRead = 1'b0;
			regMemWrite = 1'b0;
			regMemToReg = 1'b0;
			regJump = 1'b0;
			regSize_in = 2'b11;
			regnotXor = 1'b0;
			regisjr = 1'b0;
			regisjal = 1'b0;
			regislui = 1'b0;
			regisLoad = 1'b0;
			regisLB = 1'b0;
			regnotZero = 1'b0;
		end
		else if(inst_2 == 6'b100001) begin //0x21
			regFunc_in = 6'b100001;
			regRegDst = 1'b1;
			regALUSrc = 1'b0;
			regRegWrite = 1'b1;
			regMemRead = 1'b0;
			regMemWrite = 1'b0;
			regMemToReg = 1'b0;
			regJump = 1'b0;
			regSize_in = 2'b11;
			regnotXor = 1'b0;
			regisjr = 1'b0;
			regisjal = 1'b0;
			regislui = 1'b0;
			regisLoad = 1'b0;
			regisLB = 1'b0;
			regnotZero = 1'b0;
		end
		else if(inst_2 == 6'b100011) begin //0x23
			regFunc_in = 6'b100011;
			regRegDst = 1'b1;
			regALUSrc = 1'b0;
			regRegWrite = 1'b1;
			regMemRead = 1'b0;
			regMemWrite = 1'b0;
			regMemToReg = 1'b0;
			regJump = 1'b0;
			regSize_in = 2'b11;
			regnotXor = 1'b0;
			regisjr = 1'b0;
			regisjal = 1'b0;
			regislui = 1'b0;
			regisLoad = 1'b0;
			regisLB = 1'b0;
			regnotZero = 1'b0;
		end
		else if(inst_2 == 6'b101010) begin //0x2A
			regFunc_in = 6'b101000;
			regRegDst = 1'b1;
			regALUSrc = 1'b0;
			regRegWrite = 1'b1;
			regMemRead = 1'b0;
			regMemWrite = 1'b0;
			regMemToReg = 1'b0;
			regJump = 1'b0;
			regSize_in = 2'b11;
			regnotXor = 1'b0;
			regisjr = 1'b0;
			regisjal = 1'b0;
			regislui = 1'b0;
			regisLoad = 1'b0;
			regisLB = 1'b0;
			regnotZero = 1'b0;
		end
		else if(inst_2 == 6'b101011) begin //0x2B
			regFunc_in = 6'b101001;
			regRegDst = 1'b1;
			regALUSrc = 1'b0;
			regRegWrite = 1'b1;
			regMemRead = 1'b0;
			regMemWrite = 1'b0;
			regMemToReg = 1'b0;
			regJump = 1'b0;
			regSize_in = 2'b11;
			regnotXor = 1'b0;
			regisjr = 1'b0;
			regisjal = 1'b0;
			regislui = 1'b0;
			regisLoad = 1'b0;
			regisLB = 1'b0;
			regnotZero = 1'b0;
		end
		else if(inst_2 == 6'b000000) begin //0x0
			regFunc_in = 6'b100000;
			regRegDst = 1'b0;
			regALUSrc = 1'b0;
			regRegWrite = 1'b0;
			regMemRead = 1'b0;
			regMemWrite = 1'b0;
			regMemToReg = 1'b0;
			regJump = 1'b0;
			regSize_in = 2'b11;
			regnotXor = 1'b0;
			regisjr = 1'b0;
			regisjal = 1'b0;
			regislui = 1'b0;
			regisLoad = 1'b0;
			regisLB = 1'b0;
			regnotZero = 1'b0;
		end
		else if(inst_2 == 6'b001000) begin //0x8
			regFunc_in = 6'b111010;
			regRegDst = 1'b1;
			regALUSrc = 1'b0;
			regRegWrite = 1'b0;
			regMemRead = 1'b0;
			regMemWrite = 1'b0;
			regMemToReg = 1'b0;
			regJump = 1'b1;
			regSize_in = 2'b11;
			regnotXor = 1'b0;
			regisjr = 1'b1;
			regisjal = 1'b0;
			regislui = 1'b0;
			regisLoad = 1'b0;
			regisLB = 1'b0;
			regnotZero = 1'b0;
		end
		else if(inst_2 == 6'b001001) begin //0x9
			regFunc_in = 6'b111011;
			regRegDst = 1'b1;
			regALUSrc = 1'b0;
			regRegWrite = 1'b1;
			regMemRead = 1'b0;
			regMemWrite = 1'b0;
			regMemToReg = 1'b0;
			regJump = 1'b1;
			regSize_in = 2'b11;
			regnotXor = 1'b0;
			regisjr = 1'b1;
			regisjal = 1'b1;
			regislui = 1'b0;
			regisLoad = 1'b0;
			regisLB = 1'b0;
			regnotZero = 1'b0;
		end
		else begin
			regFunc_in = 6'b000000;
			regRegDst = 1'b0;
			regALUSrc = 1'b0;
			regRegWrite = 1'b0;
			regMemRead = 1'b0;
			regMemWrite = 1'b0;
			regMemToReg = 1'b0;
			regJump = 1'b0;
			regSize_in = 2'b11;
			regnotXor = 1'b0;
			regisjr = 1'b0;
			regisjal = 1'b0;
			regislui = 1'b0;
			regisLoad = 1'b0;
			regisLB = 1'b0;
			regnotZero = 1'b0;
		end
	end
	else begin
		regFunc_in = 6'b000000;
		regRegDst = 1'b0;
		regALUSrc = 1'b0;
		regRegWrite = 1'b0;
		regMemRead = 1'b0;
		regMemWrite = 1'b0;
		regMemToReg = 1'b0;
		regJump = 1'b0;
		regSize_in = 2'b11;
		regnotXor = 1'b0;
		regisjr = 1'b0;
		regisjal = 1'b0;
		regislui = 1'b0;
		regisLoad = 1'b0;
		regisLB = 1'b0;
		regnotZero = 1'b0;
	end
end
assign Func_in = regFunc_in;
assign RegDst = regRegDst;
assign ALUSrc = regALUSrc;
assign RegWrite = regRegWrite;
assign MemRead = regMemRead;
assign MemWrite = regMemWrite;
assign MemToReg = regMemToReg;
assign Jump = regJump;
assign size_in = regSize_in;
assign notXor = regnotXor;
assign isjr = regisjr;
assign isjal = regisjal;
assign islui = regislui;
assign isLoad = regisLoad;
assign isLB = regisLB;
assign notZero = regnotZero;
endmodule //control

module shiftLeft2Add2bits(
	input [25:0] theInput,
	output [27:0] resultOfShiftLeft2Add2bits
);
	assign resultOfShiftLeft2Add2bits [27:0] = theInput [25:0] << 2;
endmodule 

module shiftLeft2(
	input [31:0] theInput,
	output [31:0] resultOfShiftLeft2
);
	assign resultOfShiftLeft2 [31:0] = theInput [31:0] << 2;
endmodule

module adder2(
	input clock,
	input [31:0] inA, inB,
	output [31:0] out
);

reg [31:0] regA, regB;
reg [31:0] regOut;
wire [31:0] wireOut;

assign wireOut = regA + regB;
assign out = regOut;

always@(*) begin
	regA <= inA;
	regB <= inB;
	regOut <= wireOut;
end
endmodule 

module orrer(
	input theInput1,
	input theInput2,
	output theOutputOfTheOrrer
);
	assign theOutputOfTheOrrer = theInput1 | theInput2;
endmodule 

module extensionForLB(
	input [7:0] data_out,
	input notXor,
	output [31:0] extended
);
	assign extended [31:8] = (data_out[7] & notXor) ? 24'b111111111111111111111111 : 24'b0;
	assign extended [7:0] = data_out[7:0];
endmodule