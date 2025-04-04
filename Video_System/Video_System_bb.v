
module Video_System (
	I2C_SDAT_to_and_from_the_AV_Config,
	I2C_SCLK_from_the_AV_Config,
	SRAM_DQ_to_and_from_the_Pixel_Buffer,
	SRAM_ADDR_from_the_Pixel_Buffer,
	SRAM_LB_N_from_the_Pixel_Buffer,
	SRAM_UB_N_from_the_Pixel_Buffer,
	SRAM_CE_N_from_the_Pixel_Buffer,
	SRAM_OE_N_from_the_Pixel_Buffer,
	SRAM_WE_N_from_the_Pixel_Buffer,
	VGA_CLK_from_the_VGA_Controller,
	VGA_HS_from_the_VGA_Controller,
	VGA_VS_from_the_VGA_Controller,
	VGA_BLANK_from_the_VGA_Controller,
	VGA_SYNC_from_the_VGA_Controller,
	VGA_R_from_the_VGA_Controller,
	VGA_G_from_the_VGA_Controller,
	VGA_B_from_the_VGA_Controller,
	TD_CLK27_to_the_Video_In_Decoder,
	TD_DATA_to_the_Video_In_Decoder,
	TD_HS_to_the_Video_In_Decoder,
	TD_VS_to_the_Video_In_Decoder,
	Video_In_Decoder_external_interface_clk27_reset,
	TD_RESET_from_the_Video_In_Decoder,
	overflow_flag_from_the_Video_In_Decoder,
	clk,
	reset_n);	

	inout		I2C_SDAT_to_and_from_the_AV_Config;
	output		I2C_SCLK_from_the_AV_Config;
	inout	[15:0]	SRAM_DQ_to_and_from_the_Pixel_Buffer;
	output	[19:0]	SRAM_ADDR_from_the_Pixel_Buffer;
	output		SRAM_LB_N_from_the_Pixel_Buffer;
	output		SRAM_UB_N_from_the_Pixel_Buffer;
	output		SRAM_CE_N_from_the_Pixel_Buffer;
	output		SRAM_OE_N_from_the_Pixel_Buffer;
	output		SRAM_WE_N_from_the_Pixel_Buffer;
	output		VGA_CLK_from_the_VGA_Controller;
	output		VGA_HS_from_the_VGA_Controller;
	output		VGA_VS_from_the_VGA_Controller;
	output		VGA_BLANK_from_the_VGA_Controller;
	output		VGA_SYNC_from_the_VGA_Controller;
	output	[7:0]	VGA_R_from_the_VGA_Controller;
	output	[7:0]	VGA_G_from_the_VGA_Controller;
	output	[7:0]	VGA_B_from_the_VGA_Controller;
	input		TD_CLK27_to_the_Video_In_Decoder;
	input	[7:0]	TD_DATA_to_the_Video_In_Decoder;
	input		TD_HS_to_the_Video_In_Decoder;
	input		TD_VS_to_the_Video_In_Decoder;
	input		Video_In_Decoder_external_interface_clk27_reset;
	output		TD_RESET_from_the_Video_In_Decoder;
	output		overflow_flag_from_the_Video_In_Decoder;
	input		clk;
	input		reset_n;
endmodule
