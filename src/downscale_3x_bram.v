module downscale_3x_bram #(
    parameter IN_W   = 1280,
    parameter IN_H   = 720,
    parameter SCALE  = 3,
    parameter OUT_W  = IN_W / SCALE,
    parameter OUT_H  = IN_H / SCALE,
    parameter USE_PINGPONG = 1'b0,  // 可选乒乓双缓冲，不改接口
    parameter [11:0] Y_OFFSET = 12'd0, // 读端行偏移，用于校正上下错位
    parameter LPF_H2 = 1'b0,           // 水平2点低通（抽样前对相邻像素求平均）
    parameter LPF_V2 = 1'b0            // 垂直2点低通（对当前抽样与上一抽样行的同x求平均）
)(
    // write port (camera domain)
    input                 wr_clk,
    input                 wr_rst_n,
    input                 wr_vs,
    input                 wr_href,
    input                 wr_de,
    input [15:0]          wr_data,   // RGB565

    // read port (video domain)
    input                 rd_clk,
    input                 rd_rst_n,
    input                 rd_en,
    input                 rd_busy,    // 新增：读端忙，写端需冻结，避免撕裂
    input                 rd_vs,      // 新增：读端帧VS，用于在帧边界切换乒乓缓冲
    input [11:0]          rd_x,      // up to OUT_W-1
    input [11:0]          rd_y,      // up to OUT_H-1
    output reg [15:0]     rd_data
);

    // simple modulo counters to avoid divider/modulo logic
    reg [1:0] x_mod;  // 0..(SCALE-1) for SCALE<=4
    reg [1:0] y_mod;  // 0..(SCALE-1) for SCALE<=4
    reg [11:0] sx;    // sampled x (0..OUT_W-1)
    reg [11:0] sy;    // sampled y (0..OUT_H-1)

    reg        wr_de_d;
    reg        wr_href_d;
    reg [1:0]  wr_vs_sync;   // 双级打拍同步VSYNC到wr_clk域
    reg [1:0]  rd_busy_sync; // 双级打拍同步读忙到写域（用于单缓冲时的写冻结）
    reg [15:0] prev_wr_data; // 水平相邻像素缓存（用于H2低通）
    reg        first_dec_y;  // 首个抽样行标记（V2低通在首抽样行禁用）

    wire href_rise =  wr_href & ~wr_href_d;
    wire href_fall = ~wr_href &  wr_href_d;
    // 帧开始/结束边沿（保持原语义：当前表达式用于“帧开始”）
    wire vs_rise   =  wr_vs_sync[1] & ~wr_vs_sync[0];
    wire vs_begin  =  vs_rise;
    wire vs_end    =  wr_vs_sync[0] & ~wr_vs_sync[1];

    always @(posedge wr_clk or negedge wr_rst_n) begin
        if(!wr_rst_n) begin
            x_mod    <= 2'd0;

            sx       <= 12'd0;
            sy       <= 12'd0;

            wr_href_d <= 1'b0;
            wr_vs_sync <= 2'b00;
        end else begin
            wr_de_d   <= wr_de;
            wr_href_d <= wr_href;
            wr_vs_sync   <= {wr_vs_sync[0], wr_vs};     // 双级打拍
            rd_busy_sync <= {rd_busy_sync[0], rd_busy}; // 双级打拍
            if (wr_de) begin
                prev_wr_data <= wr_data; // 始终更新为最近的有效像素
            end

            if (vs_begin) begin
                x_mod <= 2'd0;
                y_mod <= 2'd0;
                sx    <= 12'd0;
                sy    <= 12'd0;
                first_dec_y <= 1'b1; // 新帧首抽样行
            end else begin
                // line begin
                if (href_rise) begin
                    x_mod <= 2'd0;
                    sx    <= 12'd0;
                end
                // within active pixels
                if (wr_de) begin
                    // sample when x_mod == 0
                    if (x_mod == 2'd0) begin
                        if (sx < OUT_W) begin
                            sx <= sx + 12'd1;
                        end
                    end
                    // advance x_mod 0->1->...->(SCALE-1)->0
                    x_mod <= (x_mod == (SCALE-1)) ? 2'd0 : (x_mod + 2'd1);
                end
                // line end
                if (href_fall) begin
                    sx    <= 12'd0;
                    x_mod <= 2'd0;
                    // advance y_mod and sy every SCALE lines
                    if (y_mod == 2'd0) begin
                        if (sy < OUT_H - 12'd1) begin
                            sy <= sy + 12'd1;
                        end else begin
                        end
                    end
                    // 0->1->...->(SCALE-1)->0
                    y_mod <= (y_mod == (SCALE-1)) ? 2'd0 : (y_mod + 2'd1);
                    if (y_mod == (SCALE-1)) begin
                        // 下一行进入抽样行，取消首行标记
                        first_dec_y <= 1'b0;
                    end
                end
            end
        end
    end

    // memory + 可选乒乓双缓冲
    localparam integer MEM_DEPTH    = OUT_W * OUT_H;
    localparam integer OFFSET_BASE  = OUT_W * Y_OFFSET; // 行偏移对应的线性基址（常量）

    // 写采样使能：采样到像素且为抽点行（x_mod==0 && y_mod==0）
    // 地址推进始终跟随采样使能；
    // 单缓冲：在读忙期间冻结写入以避免混帧；
    // 乒乓缓冲：完全取消写冻结，持续写入下一帧，避免半帧断续导致下半撕裂。
    wire we_px              = wr_de && (x_mod == 2'd0) && (y_mod == 2'd0) && (sx < OUT_W);
    wire we_write_single    = we_px && ~rd_busy_sync[1];
    wire we_write_pingpong  = we_px;
    // 线性写地址累加器：避免乘法寻址（sy*OUT_W）占用DSP
    reg  [31:0] waddr;
    always @(posedge wr_clk or negedge wr_rst_n) begin
        if(!wr_rst_n) begin
            waddr <= 32'd0;
        end else if (vs_begin) begin
            waddr <= 32'd0; // 每帧从0开始线性写入
        end else if (we_px) begin
            // 地址推进仅跟随采样命中，读忙期间只冻结写入，不停止地址推进，保持行/帧映射的连续性
            waddr <= (waddr == (MEM_DEPTH-1)) ? 32'd0 : (waddr + 32'd1);
        end
    end

    // 抽样前的轻量滤波
    wire [15:0] wr_data_h2 = LPF_H2 ? ((wr_data + prev_wr_data) >> 1) : wr_data;
    // 行缓冲：上一抽样行的像素，用于V2低通
    (* ram_style = "distributed" *) reg [15:0] line_prev [0:OUT_W-1];
    // 安全索引，防止在行末/非采样周期访问越界
    wire [11:0] sx_safe = (sx < OUT_W) ? sx : (OUT_W-1);
    wire [15:0] prev_line_val = line_prev[sx_safe];
    // 仅在采样命中时做垂直平均，否则直接透传，避免组合越界访问导致噪点
    wire [15:0] wr_data_v2 = (LPF_V2 && !first_dec_y && we_px) ? ((wr_data_h2 + prev_line_val) >> 1) : wr_data_h2;

    // 读端：恢复按坐标计算行偏移（rd_y_off*OUT_W + rd_x），更稳健不会受 rd_en 边沿抖动影响
    // 应用行偏移的环绕：
    localparam [11:0] Y_WRAP = OUT_H - Y_OFFSET;
    wire [11:0] rd_y_off = (rd_y < Y_WRAP) ? (rd_y + Y_OFFSET) : (rd_y - Y_WRAP);

    generate
    if (USE_PINGPONG == 1'b0) begin : g_single
        // infer block RAM
        (* ram_style = "block", syn_ramstyle = "block_ram" *) reg [15:0] mem [0:MEM_DEPTH-1];

        // write（线性地址累加）
        always @(posedge wr_clk) begin
            if (we_write_single) begin
                mem[waddr]      <= wr_data_v2;
                line_prev[sx_safe]   <= wr_data_h2; // 行缓冲记录当前抽样行原始（或H2）像素
            end
        end

        // read side：按坐标计算 raddr = rd_y_off * OUT_W + rd_x（常量乘法，综合为移位加法）
        wire [31:0] raddr = rd_y_off * OUT_W + rd_x;
        always @(posedge rd_clk or negedge rd_rst_n) begin
            if(!rd_rst_n) begin
                rd_data <= 16'h0000;
            end else if (rd_en) begin
                rd_data <= mem[raddr];
            end
        end
    end else begin : g_pingpong
        // 两块独立BRAM，综合器能正确推断块RAM
        (* ram_style = "block", syn_ramstyle = "block_ram" *) reg [15:0] mem0 [0:MEM_DEPTH-1];
        (* ram_style = "block", syn_ramstyle = "block_ram" *) reg [15:0] mem1 [0:MEM_DEPTH-1];

        reg wr_buf_sel;  // 0->写mem0，1->写mem1
        reg wr_toggle;
        always @(posedge wr_clk or negedge wr_rst_n) begin
            if(!wr_rst_n) begin
                wr_buf_sel <= 1'b0;
                wr_toggle  <= 1'b0;
            end else if (vs_begin) begin
                wr_buf_sel <= ~wr_buf_sel;   // 新一帧开始，翻转写缓冲
                wr_toggle  <= ~wr_toggle;    // 写缓冲翻转指示（供调试）
            end
        end

        // 写端帧完成指示：在帧结束边界翻转，用于读端在下一帧边界切换缓冲
        reg wr_done_toggle;
        always @(posedge wr_clk or negedge wr_rst_n) begin
            if(!wr_rst_n) begin
                wr_done_toggle <= 1'b0;
            end else if (vs_end) begin
                wr_done_toggle <= ~wr_done_toggle;
            end
        end

        // 写入到选择的BRAM（线性地址累加）
        always @(posedge wr_clk) begin
            if (we_write_pingpong) begin
                if (wr_buf_sel == 1'b0) mem0[waddr] <= wr_data_v2;
                else                    mem1[waddr] <= wr_data_v2;
                line_prev[sx_safe] <= wr_data_h2; // 行缓冲记录当前抽样行原始（或H2）像素
            end
        end

        // 读端缓冲选择：同步写端toggle，避免跨域毛刺
        reg [1:0] wr_toggle_sync;
        reg [1:0] wr_done_sync;
        reg [1:0] rd_vs_sync;          // 读端VS同步用于边沿检测
        reg       rd_buf_sel;          // 0->读mem0，1->读mem1
        reg       rd_buf_sel_pending;  // 等待在帧边界切换
        always @(posedge rd_clk or negedge rd_rst_n) begin
            if(!rd_rst_n) begin
                wr_toggle_sync     <= 2'b00;
                wr_done_sync       <= 2'b00;
                rd_vs_sync         <= 2'b00;
                rd_buf_sel         <= 1'b1; // 初始读与写相反
                rd_buf_sel_pending <= 1'b0;
            end else begin
                wr_toggle_sync <= {wr_toggle_sync[0], wr_toggle};
                wr_done_sync   <= {wr_done_sync[0],   wr_done_toggle};
                rd_vs_sync     <= {rd_vs_sync[0], rd_vs};
                // 仅在写端“帧完成”后申请切换，避免读到未写完整的缓冲
                if (wr_done_sync[1] ^ wr_done_sync[0]) begin
                    rd_buf_sel_pending <= 1'b1;
                end
                // 仅在读端VS边界切换，避免一帧内多次切换造成滚动条
                if ((rd_vs_sync[1] ^ rd_vs_sync[0]) && rd_buf_sel_pending) begin
                    rd_buf_sel         <= ~rd_buf_sel; // 在帧边界完成切换
                    rd_buf_sel_pending <= 1'b0;
                end
            end
        end

        // 从选择的BRAM读出：按坐标计算 raddr
        wire [31:0] raddr = rd_y_off * OUT_W + rd_x;
        always @(posedge rd_clk or negedge rd_rst_n) begin
            if(!rd_rst_n) begin
                rd_data <= 16'h0000;
            end else if (rd_en) begin
                rd_data <= (rd_buf_sel == 1'b0) ? mem0[raddr] : mem1[raddr];
            end
        end
    end
    endgenerate

endmodule