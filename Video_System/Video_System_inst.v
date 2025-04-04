	Video_System u0 (
		.I2C_SDAT_to_and_from_the_AV_Config              (<connected-to-I2C_SDAT_to_and_from_the_AV_Config>),              //        AV_Config_external_interface.SDAT
		.I2C_SCLK_from_the_AV_Config                     (<connected-to-I2C_SCLK_from_the_AV_Config>),                     //                                    .SCLK
		.SRAM_DQ_to_and_from_the_Pixel_Buffer            (<connected-to-SRAM_DQ_to_and_from_the_Pixel_Buffer>),            //     Pixel_Buffer_external_interface.DQ
		.SRAM_ADDR_from_the_Pixel_Buffer                 (<connected-to-SRAM_ADDR_from_the_Pixel_Buffer>),                 //                                    .ADDR
		.SRAM_LB_N_from_the_Pixel_Buffer                 (<connected-to-SRAM_LB_N_from_the_Pixel_Buffer>),                 //                                    .LB_N
		.SRAM_UB_N_from_the_Pixel_Buffer                 (<connected-to-SRAM_UB_N_from_the_Pixel_Buffer>),                 //                                    .UB_N
		.SRAM_CE_N_from_the_Pixel_Buffer                 (<connected-to-SRAM_CE_N_from_the_Pixel_Buffer>),                 //                                    .CE_N
		.SRAM_OE_N_from_the_Pixel_Buffer                 (<connected-to-SRAM_OE_N_from_the_Pixel_Buffer>),                 //                                    .OE_N
		.SRAM_WE_N_from_the_Pixel_Buffer                 (<connected-to-SRAM_WE_N_from_the_Pixel_Buffer>),                 //                                    .WE_N
		.VGA_CLK_from_the_VGA_Controller                 (<connected-to-VGA_CLK_from_the_VGA_Controller>),                 //   VGA_Controller_external_interface.CLK
		.VGA_HS_from_the_VGA_Controller                  (<connected-to-VGA_HS_from_the_VGA_Controller>),                  //                                    .HS
		.VGA_VS_from_the_VGA_Controller                  (<connected-to-VGA_VS_from_the_VGA_Controller>),                  //                                    .VS
		.VGA_BLANK_from_the_VGA_Controller               (<connected-to-VGA_BLANK_from_the_VGA_Controller>),               //                                    .BLANK
		.VGA_SYNC_from_the_VGA_Controller                (<connected-to-VGA_SYNC_from_the_VGA_Controller>),                //                                    .SYNC
		.VGA_R_from_the_VGA_Controller                   (<connected-to-VGA_R_from_the_VGA_Controller>),                   //                                    .R
		.VGA_G_from_the_VGA_Controller                   (<connected-to-VGA_G_from_the_VGA_Controller>),                   //                                    .G
		.VGA_B_from_the_VGA_Controller                   (<connected-to-VGA_B_from_the_VGA_Controller>),                   //                                    .B
		.TD_CLK27_to_the_Video_In_Decoder                (<connected-to-TD_CLK27_to_the_Video_In_Decoder>),                // Video_In_Decoder_external_interface.TD_CLK27
		.TD_DATA_to_the_Video_In_Decoder                 (<connected-to-TD_DATA_to_the_Video_In_Decoder>),                 //                                    .TD_DATA
		.TD_HS_to_the_Video_In_Decoder                   (<connected-to-TD_HS_to_the_Video_In_Decoder>),                   //                                    .TD_HS
		.TD_VS_to_the_Video_In_Decoder                   (<connected-to-TD_VS_to_the_Video_In_Decoder>),                   //                                    .TD_VS
		.Video_In_Decoder_external_interface_clk27_reset (<connected-to-Video_In_Decoder_external_interface_clk27_reset>), //                                    .clk27_reset
		.TD_RESET_from_the_Video_In_Decoder              (<connected-to-TD_RESET_from_the_Video_In_Decoder>),              //                                    .TD_RESET
		.overflow_flag_from_the_Video_In_Decoder         (<connected-to-overflow_flag_from_the_Video_In_Decoder>),         //                                    .overflow_flag
		.clk                                             (<connected-to-clk>),                                             //                          clk_clk_in.clk
		.reset_n                                         (<connected-to-reset_n>)                                          //                    clk_clk_in_reset.reset_n
	);

