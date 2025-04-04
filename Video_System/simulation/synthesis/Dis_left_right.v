module Video_System_video_edge_detection_0 (
    // Inputs
    input clk,
    input reset,

    input [7:0] avalon_streaming_sink_data,
    input avalon_streaming_sink_startofpacket,
    input avalon_streaming_sink_endofpacket,
    input avalon_streaming_sink_empty,
    input avalon_streaming_sink_valid,

    input avalon_streaming_source_ready,

    // Outputs
    output avalon_streaming_sink_ready,

    output reg [7:0] avalon_streaming_source_data,
    output reg avalon_streaming_source_startofpacket,
    output reg avalon_streaming_source_endofpacket,
    output reg avalon_streaming_source_empty,
    output reg avalon_streaming_source_valid,
    output reg [15:0] avalon_distance_from_bottom // Updated to 16-bit output
);

parameter WIDTH = 720; // Image width in pixels

// Internal Wires
wire transfer_data;
wire [8:0] filter_1_data_out;   // Gaussian_Smoothing
wire [9:0] filter_2_data_out;   // Sobel operator
wire [7:0] filter_3_data_out;   // Nonmaximum Suppression
wire [7:0] filter_4_data_out;   // Hysteresis
wire [7:0] final_value;         // Intensity Correction
wire [1:0] pixel_info_in;
wire [1:0] pixel_info_out;
wire [9:0] current_row;
wire found;

// Internal Registers
reg [7:0] data;
reg startofpacket;
reg endofpacket;
reg empty;
reg valid;
reg flush_pipeline;

// Output Registers
always @(posedge clk) begin
    if (reset) begin
        avalon_streaming_source_data            <= 8'h00;
        avalon_streaming_source_startofpacket   <= 1'b0;
        avalon_streaming_source_endofpacket     <= 1'b0;
        avalon_streaming_source_empty           <= 1'b0;
        avalon_streaming_source_valid           <= 1'b0;
    end else if (transfer_data) begin
        avalon_streaming_source_data            <= final_value;
        avalon_streaming_source_startofpacket   <= pixel_info_out[1] & ~(&(pixel_info_out));
        avalon_streaming_source_endofpacket     <= pixel_info_out[0] & ~(&(pixel_info_out));
        avalon_streaming_source_empty           <= 1'b0;
        avalon_streaming_source_valid           <= (|(pixel_info_out));
    end else if (avalon_streaming_source_ready) begin
        avalon_streaming_source_valid           <= 1'b0;
    end
end

// Internal Registers
always @(posedge clk) begin
    if (reset) begin
        data                <= 8'h00;
        startofpacket       <= 1'b0;
        endofpacket         <= 1'b0;
        empty               <= 1'b0;
        valid               <= 1'b0;
    end else if (avalon_streaming_sink_ready) begin
        data                <= avalon_streaming_sink_data;
        startofpacket       <= avalon_streaming_sink_startofpacket;
        endofpacket         <= avalon_streaming_sink_endofpacket;
        empty               <= avalon_streaming_sink_empty;
        valid               <= avalon_streaming_sink_valid;
    end
end

always @(posedge clk) begin
    if (reset) begin
        flush_pipeline      <= 1'b0;
    end else if (avalon_streaming_sink_ready & avalon_streaming_sink_endofpacket) begin
        flush_pipeline      <= 1'b1;
    end else if (avalon_streaming_sink_ready & avalon_streaming_sink_startofpacket) begin
        flush_pipeline      <= 1'b0;
    end
end

// Output Assignments
assign avalon_streaming_sink_ready = avalon_streaming_sink_valid & (avalon_streaming_source_ready | ~avalon_streaming_source_valid);

// Internal Assignments
assign transfer_data = avalon_streaming_sink_ready | (flush_pipeline & (avalon_streaming_source_ready | ~avalon_streaming_source_valid));
assign final_value = {filter_4_data_out[6:0], 1'b0} | {8{filter_4_data_out[7]}};
assign pixel_info_in[1] = avalon_streaming_sink_valid & ~avalon_streaming_sink_endofpacket;
assign pixel_info_in[0] = avalon_streaming_sink_valid & ~avalon_streaming_sink_startofpacket;

// Distance Calculation
always @(posedge clk) begin
    if (reset) begin
        avalon_distance_from_bottom <= 16'd0;
    end else if (found) begin
        avalon_distance_from_bottom <= 16'd480 - current_row; // Assuming image height is 480
    end
end

/*****************************************************************************
 *                              Internal Modules                             *
 *****************************************************************************/

// Gaussian Smoothing Filter
altera_up_edge_detection_gaussian_smoothing_filter Filter_1 (
    .clk        (clk),
    .reset      (reset),
    .data_in    (data),
    .data_en    (transfer_data),
    .data_out   (filter_1_data_out)
);
defparam Filter_1.WIDTH = WIDTH;    

// Sobel Operator
altera_up_edge_detection_sobel_operator Filter_2 (
    .clk        (clk),
    .reset      (reset),
    .data_in    (filter_1_data_out),
    .data_en    (transfer_data),
    .data_out   (filter_2_data_out)
);
defparam Filter_2.WIDTH = WIDTH;    

// Nonmaximum Suppression
altera_up_edge_detection_nonmaximum_suppression Filter_3 (
    .clk        (clk),
    .reset      (reset),
    .data_in    (filter_2_data_out),
    .data_en    (transfer_data),
    .data_out   (filter_3_data_out)
);
defparam Filter_3.WIDTH = WIDTH;    

// Hysteresis
altera_up_edge_detection_hysteresis Filter_4 (
    .clk        (clk),
    .reset      (reset),
    .data_in    (filter_3_data_out),
    .data_en    (transfer_data),
    .data_out   (filter_4_data_out)
);
defparam Filter_4.WIDTH = WIDTH;    

// Pixel Info Shift Register
altera_up_edge_detection_pixel_info_shift_register Pixel_Info_Shift_Register (
    .clock      (clk),
    .clken      (transfer_data),
    .shiftin    (pixel_info_in),
    .shiftout   (pixel_info_out),
    .taps       ()
);
defparam Pixel_Info_Shift_Register.SIZE = (WIDTH * 5) + 28;    

// Find Lower Edge Module
Find_Lower_Edge Find_Lower_Edge_Module (
    .clk        (clk),
    .reset      (reset),
    .in_data    (filter_4_data_out),
    .in_valid   (transfer_data),
    .row        (current_row),
    .col        (/* Your column logic */),
    .lower_edge_row (current_row),
    .done       (found)
);

endmodule

module Find_Lower_Edge (
    input wire clk,
    input wire reset,
    input wire [7:0] in_data, // Edge-detected image data
    input wire in_valid, // Signal to validate input data
    input wire [9:0] row, // Current row index
    input wire [9:0] col, // Current column index
    output reg [9:0] lower_edge_row, // Row index of the car's lower edge
    output reg done // Signal indicating completion
);

parameter IMG_HEIGHT = 480; // Adjust based on your image resolution
parameter IMG_WIDTH = 640;  // Adjust based on your image resolution

reg [9:0] current_row;
reg found;

always @(posedge clk or posedge reset) begin
    if (reset) begin
        lower_edge_row <= 10'd0;
        current_row <= IMG_HEIGHT;
        found <= 1'b0;
        done <= 1'b0;
    end else if (in_valid && !found) begin
        if (in_data > 8'h00 && row < current_row) begin
            lower_edge_row <= row;
            found <= 1'b1;
        end
    end else if (row == 10'd0 && col == IMG_WIDTH - 1) begin
        done <= 1'b1;
    end
end

endmodule

