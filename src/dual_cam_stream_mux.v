module dual_cam_stream_mux #(
    parameter IN_W   = 1280,
    parameter IN_H   = 720,
    parameter SCALE  = 3
)(
    // cam0 write side (camera domain)
    input                 wr0_clk,
    input                 wr0_rst_n,
    input                 wr0_vs,
    input                 wr0_href,
    input                 wr0_de,
    input  [15:0]         wr0_data,   // RGB565

    // cam1 write side (camera domain)
    input                 wr1_clk,
    input                 wr1_rst_n,
    input                 wr1_vs,
    input                 wr1_href,
    input                 wr1_de,
    input  [15:0]         wr1_data,   // RGB565

    // read side (video domain)
    input                 rd_clk,
    input                 rd_rst_n,
    input                 rd0_en,      // region enable for cam0 window
    input                 rd1_en,      // region enable for cam1 window
    input                 rd_de,       // active video from timing
    output [15:0]         rd0_data,    // decimated RGB565 for cam0
    output [15:0]         rd1_data,    // decimated RGB565 for cam1

    // status
    output                fifo0_empty,
    output                fifo0_full,
    output                fifo1_empty,
    output                fifo1_full
);

    // decimation counters for cam0
    localparam OUT_W = IN_W / SCALE;
    localparam OUT_H = IN_H / SCALE;

    reg [1:0] x0_mod;  // 0..SCALE-1
    reg [1:0] y0_mod;  // 0..SCALE-1
    reg       href0_d;
    wire      href0_rise =  wr0_href & ~href0_d;
    wire      href0_fall = ~wr0_href &  href0_d;

    reg [1:0] x1_mod;  // 0..SCALE-1
    reg [1:0] y1_mod;  // 0..SCALE-1
    reg       href1_d;
    wire      href1_rise =  wr1_href & ~href1_d;
    wire      href1_fall = ~wr1_href &  href1_d;

    // simple gate: sample when both mods are zero
    wire wr0_sample = (x0_mod == 2'd0) && (y0_mod == 2'd0) && wr0_de && wr0_href;
    wire wr1_sample = (x1_mod == 2'd0) && (y1_mod == 2'd0) && wr1_de && wr1_href;

    // manage counters for cam0
    always @(posedge wr0_clk or negedge wr0_rst_n) begin
        if(!wr0_rst_n) begin
            x0_mod  <= 2'd0;
            y0_mod  <= 2'd0;
            href0_d <= 1'b0;
        end else begin
            href0_d <= wr0_href;
            if(wr0_vs) begin
                x0_mod <= 2'd0; y0_mod <= 2'd0;
            end else if(href0_rise) begin
                x0_mod <= 2'd0;
            end else if(wr0_href) begin
                x0_mod <= (x0_mod == (SCALE-1)) ? 2'd0 : (x0_mod + 1'b1);
            end
            if(href0_fall) begin
                y0_mod <= (y0_mod == (SCALE-1)) ? 2'd0 : (y0_mod + 1'b1);
            end
        end
    end

    // manage counters for cam1
    always @(posedge wr1_clk or negedge wr1_rst_n) begin
        if(!wr1_rst_n) begin
            x1_mod  <= 2'd0;
            y1_mod  <= 2'd0;
            href1_d <= 1'b0;
        end else begin
            href1_d <= wr1_href;
            if(wr1_vs) begin
                x1_mod <= 2'd0; y1_mod <= 2'd0;
            end else if(href1_rise) begin
                x1_mod <= 2'd0;
            end else if(wr1_href) begin
                x1_mod <= (x1_mod == (SCALE-1)) ? 2'd0 : (x1_mod + 1'b1);
            end
            if(href1_fall) begin
                y1_mod <= (y1_mod == (SCALE-1)) ? 2'd0 : (y1_mod + 1'b1);
            end
        end
    end

    // two async FIFOs for cam0/cam1
    wire [15:0] fifo0_q;
    wire [15:0] fifo1_q;
    wire [12:0] fifo0_rnum;
    wire [12:0] fifo1_rnum;

    // reset FIFOs: generate a short pulse on VSYNC rising edge (write domains)
    reg wr0_vs_d, wr1_vs_d;
    wire wr0_vs_rise = wr0_vs & ~wr0_vs_d;
    wire wr1_vs_rise = wr1_vs & ~wr1_vs_d;
    reg [1:0] rst0_cnt, rst1_cnt; // 2-cycle reset pulses
    always @(posedge wr0_clk or negedge wr0_rst_n) begin
        if(!wr0_rst_n) begin
            wr0_vs_d <= 1'b0; rst0_cnt <= 2'd0;
        end else begin
            wr0_vs_d <= wr0_vs;
            if(wr0_vs_rise)       rst0_cnt <= 2'd2;
            else if(rst0_cnt!=0)  rst0_cnt <= rst0_cnt - 1'b1;
        end
    end
    always @(posedge wr1_clk or negedge wr1_rst_n) begin
        if(!wr1_rst_n) begin
            wr1_vs_d <= 1'b0; rst1_cnt <= 2'd0;
        end else begin
            wr1_vs_d <= wr1_vs;
            if(wr1_vs_rise)       rst1_cnt <= 2'd2;
            else if(rst1_cnt!=0)  rst1_cnt <= rst1_cnt - 1'b1;
        end
    end
    wire rst0_ff = (rst0_cnt != 2'd0);
    wire rst1_ff = (rst1_cnt != 2'd0);
    wire rst1_ff = (rst1_cnt != 2'd0);

    video_fifo u_fifo0 (
        .Data  (wr0_data),
        .Reset (~wr0_rst_n | rst0_ff),
        .WrClk (wr0_clk),
        .RdClk (rd_clk),
        .WrEn  (wr0_sample),
        .RdEn  (rd0_en && rd_de),
        .Rnum  (fifo0_rnum),
        .Q     (fifo0_q),
        .Empty (fifo0_empty),
        .Full  (fifo0_full)
    );

    video_fifo u_fifo1 (
        .Data  (wr1_data),
        .Reset (~wr1_rst_n | rst1_ff),
        .WrClk (wr1_clk),
        .RdClk (rd_clk),
        .WrEn  (wr1_sample),
        .RdEn  (rd1_en && rd_de),
        .Rnum  (fifo1_rnum),
        .Q     (fifo1_q),
        .Empty (fifo1_empty),
        .Full  (fifo1_full)
    );

    // output mapping: if empty, output black
    // 读端输出：当FIFO为空时输出黑色，否则输出FIFO数据
    assign rd0_data = fifo0_empty ? 16'h0000 : fifo0_q;
    assign rd1_data = fifo1_empty ? 16'h0000 : fifo1_q;

endmodule