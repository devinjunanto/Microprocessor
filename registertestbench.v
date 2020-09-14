`timescale 1ns/1ps

module registertestbench();

reg clock;
reg reset;

reg [4:0] read_register_1;
reg [4:0] read_register_2;
reg [4:0] write_register;
reg [31:0] write_data;

wire [31:0] read_data_1;
wire [31:0] read_data_2;

//Generate clock at 100 MHz
initial begin
	clock <= 1'b0;
	forever #10 clock <= ~clock;
end
	
//instantiate the processor  "DUT"
register dut(
	.clock(clock),
	.reset(reset),
	.read_register_1(read_register_1),
	.read_register_2(read_register_2),
	.write_register(write_register),
	.write_data(write_data),
	.read_data_1(read_data_1),
	.read_data_2(read_data_2)
);


//This will print out a message whenever the serial port is written to
initial begin
	read_register_1 = 5'b0; //read 0
	read_register_2 = 5'b0; //read 0
	write_register = 5'b00001;
	write_data = 1;
	#20
	read_register_1 = 5'b00001; //read 1
	read_register_2 = 5'b0; //read 0
	write_register = 5'b0;
	write_data = 2;
	#20
	read_register_1 = 5'b00001; //read 1
	read_register_2 = 5'b0; //read 0
	write_register = 5'b00010;
	write_data = 6;
	#20
	read_register_1 = 5'b00010; //read 6
	read_register_2 = 5'b00100; //read error
	write_register = 5'b00011;
	write_data = 7;
	#20
	read_register_1 = 5'b00011; //read 7
	read_register_2 = 5'b00011; //read 7
	write_register = 5'b00001;
	write_data = 3;
	#20
	read_register_1 = 5'b00001;
	read_register_2 = 5'b00001;
end


endmodule 