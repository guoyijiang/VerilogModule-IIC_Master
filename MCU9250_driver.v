`timescale 1ns / 1ps

module  MCU9250_driver(
	input wire clk,
	input wire rst,
	
	output wire signed[15:0]accelY,

    input  wire        scl_i,
    output wire        scl_o,
    output wire        scl_t,
    input  wire        sda_i,
    output wire        sda_o,
    output wire        sda_t

);

reg [6:0] cmd_address = 0;
reg cmd_start = 0;
reg cmd_read = 0;
reg cmd_write = 0;
reg cmd_write_multiple = 0;
reg cmd_stop = 0;
reg cmd_valid = 0;
reg [7:0] data_in = 0;
reg data_in_valid = 0;
reg data_in_last = 0;
reg data_out_ready = 1;
reg [15:0] prescale = 2500000/800;//125000/800;
reg stop_on_idle = 1'b1;

// Outputs
wire cmd_ready;
wire data_in_ready;
wire [7:0] data_out;
wire data_out_valid;
wire data_out_last;
wire busy;
wire bus_control;
wire bus_active;
wire missed_ack;

reg circul_counter_en = 1'b1;
wire circul_counter_co;
wire [$clog2(400000) - 1 : 0] circul_counter_cnt;
Counter #(.M(400000))circul_counter(clk, rst, circul_counter_en, circul_counter_cnt, circul_counter_co);

parameter GYRO_ADDRESS = 7'b1101000;

parameter STATE0_IDLE = 0;
parameter STATE1_WRITE_REGL_ADDR = 1;
parameter STATE2_READ_ACC_LOW = 2;
parameter STATE3_WRITE_REGH_ADDR = 3;
parameter STATE4_READ_ACC_HIGH = 4;

parameter ACCEL_XOUT_L = 8'h3E;
parameter ACCEL_XOUT_H = 8'h3D;

reg [3:0] state = STATE1_WRITE_REGL_ADDR;
reg signed[15:0] accelData = 16'd0;
assign accelY = accelData;

reg accelDataValid = 1'b0;
//main tate
always@(posedge clk) begin
	if(rst) begin
		cmd_address <= GYRO_ADDRESS;
		cmd_start <= 1'b1;
		cmd_read <= 1'b1;
		cmd_write <= 1'b0;
		cmd_write_multiple <= 1'b0;
		cmd_stop <= 1'b0;
		cmd_valid <= 1'b0;	
		data_in	<= 16'd0;	
		data_in_valid <= 1'b0;
		stop_on_idle <= 1'b1;
	end
	else begin
		if(cmd_valid) cmd_valid <= 1'b0;
		if(accelDataValid) accelDataValid <= 1'b0;
		//if(data_in_valid) data_in_valid <= 1'b0;
		if(circul_counter_co) begin
			case(state)
				STATE1_WRITE_REGL_ADDR:
					begin
						accelData[15:8] <= data_out[7:0];
						accelDataValid <= 1'b1;
					
						data_in <= ACCEL_XOUT_L;
						data_in_valid <= 1'b1;
						cmd_read  <= 1'b0;
						cmd_write <= 1'b1;
						cmd_valid <= 1'b1;
						
						state <= STATE2_READ_ACC_LOW;
					end
				STATE2_READ_ACC_LOW:
					begin
						cmd_read  <= 1'b1;
						cmd_write <= 1'b0;
						cmd_valid <= 1'b1;
						state <= STATE3_WRITE_REGH_ADDR;
					end
				STATE3_WRITE_REGH_ADDR:
					begin
						accelData[7:0] <= data_out[7:0];
						
						data_in <= ACCEL_XOUT_H;
						data_in_valid <= 1'b1;
						cmd_read  <= 1'b0;
						cmd_write <= 1'b1;
						cmd_valid <= 1'b1;
						state <= STATE4_READ_ACC_HIGH;
					end				
				STATE4_READ_ACC_HIGH:
					begin
						cmd_read  <= 1'b1;
						cmd_write <= 1'b0;
						cmd_valid <= 1'b1;
						state <= STATE1_WRITE_REGL_ADDR;
					end
			endcase
		end	
  end
end


i2c_master i2c_master_mcu9250_inst (
    .clk(clk),
    .rst(rst),
    .cmd_address(7'b1101000),
    .cmd_start(cmd_start),
    .cmd_read(cmd_read),
    .cmd_write(cmd_write),
    .cmd_write_multiple(cmd_write_multiple),
    .cmd_stop(cmd_stop),
    .cmd_valid(cmd_valid),
    .cmd_ready(cmd_ready),
    .data_in(data_in),
    .data_in_valid(data_in_valid),
    .data_in_ready(data_in_ready),
    .data_in_last(data_in_last),
    .data_out(data_out),
    .data_out_valid(data_out_valid),
    .data_out_ready(data_out_ready),
    .data_out_last(data_out_last),
    .scl_i(scl_i),
    .scl_o(scl_o),
    .scl_t(scl_t),
    .sda_i(sda_i),
    .sda_o(sda_o),
    .sda_t(sda_t),
    .busy(busy),
    .bus_control(bus_control),
    .bus_active(bus_active),
    .missed_ack(missed_ack),
    .prescale(prescale),
    .stop_on_idle(stop_on_idle)
);

endmodule
