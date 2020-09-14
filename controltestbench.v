`timescale 1ns/1ps

module controltestbench();

reg clock;

reg [5:0] inst_1;
reg [5:0] inst_2;
wire RegDst;
wire Branch;
wire MemRead;
wire MemToReg;
wire [5:0] Func_in;
wire MemWrite;
wire ALUSrc;
wire RegWrite;

//Generate clock at 100 MHz
initial begin
	clock <= 1'b0;
	forever #10 clock <= ~clock;
end
	
//instantiate the processor  "DUT"
control dut(
	.clock(clock),
	.inst_1(inst_1),
	.inst_2(inst_2),
	.RegDst(RegDst),
	.Branch(Branch),
	.MemRead(MemRead),
	.MemToReg(MemToReg),
	.Func_in(Func_in),
	.MemWrite(MemWrite),
	.ALUSrc(ALUSrc),
	.RegWrite(RegWrite)
);


//This will print out a message whenever the serial port is written to
initial begin
	inst_1 = 6'b100011;
	#20
	inst_1 = 6'b101011;
	#20
	inst_1 = 6'b001000;
	#20
	inst_1 = 0;
	inst_2 = 6'b100000;
	#20
	inst_1 = 0;
	inst_2 = 6'b100010;
	#20
	inst_1 = 0;
	inst_2 = 6'b100100;
	#20
	inst_1 = 0;
	inst_2 = 6'b100101;
	#20
	inst_1 = 0;
	inst_2 = 6'b100111;
	#20
	inst_1 = 0;
	inst_2 = 6'b100110;
end


endmodule 