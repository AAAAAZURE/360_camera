module top #(
    parameter USE_TPG = "false", // 切换回摄像头模式进行颜色调试
    parameter DDR_BYPASS = "true"
)(
	input                  clk,
	input                  rst_n,
	inout                  cmos_scl,       //cmos i2c clock
	inout                  cmos_sda,       //cmos i2c data
	input                  cmos_vsync,     //cmos vsync
	input                  cmos_href,      //cmos hsync refrence,data valid
	input                  cmos_pclk,      //cmos pxiel clock
	input  [7:0]           cmos_db,        //cmos data
    // cam1 interface (second OV5640)
    input                  c1_vsync,     //cam1 vsync
    input                  c1_href,      //cam1 href/data valid
    input                  c1_pclk,      //cam1 pixel clock
    input  [7:0]           c1_db,        //cam1 data
    // cam2 interface (third OV5640 via 40pin header)
    input                  c2_vsync,     //cam2 vsync
    input                  c2_href,      //cam2 href/data valid
    input                  c2_pclk,      //cam2 pixel clock
    input  [7:0]           c2_db,        //cam2 data
    output                 cmos_rst_n,     //cmos reset 
    output                 cmos_pwdn,      //cmos power down

	output [15:0]          ddr_addr,       //ROW_WIDTH=16
	output [2:0]           ddr_bank,       //BANK_WIDTH=3
	output                 ddr_cs,
	output                 ddr_ras,
	output                 ddr_cas,
	output                 ddr_we,
	output                 ddr_ck,
	output                 ddr_ck_n,
	output                 ddr_cke,
	output                 ddr_odt,
	output                 ddr_reset_n,
    output [31:0]          ddr_dm,         //DM_WIDTH=4
    inout  [31:0]          ddr_dq,         //DQ_WIDTH=32
	inout  [3:0]           ddr_dqs,        //DQS_WIDTH=4
	inout  [3:0]           ddr_dqs_n,      //DQS_WIDTH=4
  
    output                 tmds_clk_n_0,
    output                 tmds_clk_p_0,
    output [2:0]           tmds_d_n_0, //{r,g,b}
    output [2:0]           tmds_d_p_0,
    // Bluetooth + Servo control (HC-05 + 270° servo)
    input                  bt_state,
    input                  bt_rxd,
    output                 bt_txd,
    output                 bt_en,
    output                 servo_pwm
);

//memory interface
    wire                   memory_clk         ;
    wire                   dma_clk         	  ;
    wire                   DDR_pll_lock       ;
    wire                   cmd_ready          ;
    wire[2:0]              cmd                ;
    wire                   cmd_en             ;
    //wire[5:0]              app_burst_number   ;
    wire[ADDR_WIDTH-1:0]   addr               ;
    wire                   wr_data_rdy        ;
    wire                   wr_data_en         ;
    wire                   wr_data_end        ;
    wire[DATA_WIDTH-1:0]   wr_data            ;   
    wire[DATA_WIDTH/8-1:0] wr_data_mask       ;   
    wire                   rd_data_valid      ;  
    wire                   rd_data_end        ;//unused 
    wire[DATA_WIDTH-1:0]   rd_data            ;   
    wire                   init_calib_complete;
    wire                   err;
    wire                   TMDS_DDR_pll_lock  ;

    //According to IP parameters to choose
    `define	    WR_VIDEO_WIDTH_32
    `define	DEF_WR_VIDEO_WIDTH 32

    `define	    RD_VIDEO_WIDTH_32
    `define	DEF_RD_VIDEO_WIDTH 32

    `define	USE_THREE_FRAME_BUFFER

    `define	DEF_ADDR_WIDTH 29 
    `define	DEF_SRAM_DATA_WIDTH 256
    
    //=========================================================
    //SRAM parameters
    localparam ADDR_WIDTH          = `DEF_ADDR_WIDTH;        //存储单元是byte，总容量=2^29*16bit = 8Gbit,增加1位rank地址，{rank[0],bank[2:0],row[15:0],cloumn[9:0]}
    localparam DATA_WIDTH          = `DEF_SRAM_DATA_WIDTH;   //与生成DDR3IP有关，此ddr3 4Gbit, x32， 时钟比例1:4 ，则固定256bit
    localparam WR_VIDEO_WIDTH      = `DEF_WR_VIDEO_WIDTH;  
    localparam RD_VIDEO_WIDTH      = `DEF_RD_VIDEO_WIDTH;
    
    // 添加缺失的参数声明
    localparam H_TOTAL             = 12'd1650;
    localparam V_TOTAL             = 12'd750;  

    wire                            video_clk;  //video pixel clock
    //-------------------
    //syn_code
    wire                      syn_off0_re;      // ofifo read enable signal
    wire                      syn_off0_vs;
    wire                      syn_off0_hs;

    wire                      off0_syn_de  ;
    wire [RD_VIDEO_WIDTH-1:0] off0_syn_data;

    wire[15:0]                      cmos_16bit_data;
    wire                            cmos_16bit_de;
wire[15:0] 						write_data;
wire [15:0]                 c1_16bit_data;
wire                        c1_16bit_de;
// 为c1/c2引入YUV到RGB转换后的信号，保持与c0一致的RGB显示
// 三路YUV422->RGB565转换输出（统一颜色管线）
// 已回退为RGB直通路径，不再使用YUV转换输出信号

    wire[9:0]                       lut_index;
    wire[31:0]                      lut_data;
    wire i2c_done;
    wire i2c_err;

    assign cmos_pwdn = 1'b0;
//    assign cmos_rst_n = 1'b1;
    assign cmos_rst_n = cmos_reset;
    // 顶层直接使用传感器的RGB565（与LUT的0x4300=0x61一致）
    // C0 颜色校正：轻量通道增益微调，压黄偏，抑制暗部蓝偏
    wire [4:0] c0_r5 = cmos_16bit_data[15:11];
    wire [5:0] c0_g6 = cmos_16bit_data[10:5];
    wire [4:0] c0_b5 = cmos_16bit_data[4:0];
    // 扩展至8位近似：重复高位
    wire [7:0] c0_r8 = {c0_r5, c0_r5[4:2]};
    wire [7:0] c0_g8 = {c0_g6, c0_g6[5:4]};
    wire [7:0] c0_b8 = {c0_b5, c0_b5[4:2]};
    // 增益调整：R/G -1/16（约0.9375），B +1/16（约1.0625，带饱和）
    wire [7:0] c0_r8_adj = c0_r8 - (c0_r8 >> 4);
    wire [7:0] c0_g8_adj = c0_g8 - (c0_g8 >> 4);
    wire [8:0] c0_b9_sum = {1'b0, c0_b8} + {1'b0, (c0_b8 >> 4)};
    wire [7:0] c0_b8_adj = c0_b9_sum[8] ? 8'hFF : c0_b9_sum[7:0];
    // 压回RGB565
    wire [4:0] c0_r5_o = c0_r8_adj[7:3];
    wire [5:0] c0_g6_o = c0_g8_adj[7:2];
    wire [4:0] c0_b5_o = c0_b8_adj[7:3];
    wire [15:0] c0_rgb565_cc = {c0_r5_o, c0_g6_o, c0_b5_o};
    // 回退为原始RGB565直通，避免色彩处理引入额外量化纹理
    assign write_data = cmos_16bit_data;
    //assign write_data = {cmos_16bit_data[10:5],cmos_16bit_data[15:11],cmos_16bit_data[4:0]}; // GRB->RGB映射 - 结果：绿色品红色间隔
    //assign write_data = {cmos_16bit_data[4:0],cmos_16bit_data[10:5],cmos_16bit_data[15:11]}; // BGR映射 - 结果：黑灰白色
    //assign write_data = {cmos_16bit_data[4:0],cmos_16bit_data[10:5],cmos_16bit_data[15:11]}; // 尝试BGR->RGB重映射
    //assign write_data = {cmos_16bit_data[15:11],cmos_16bit_data[10:5],cmos_16bit_data[4:0]}; // 尝试RGB->BGR重映射
    //assign write_data = {cmos_16bit_data[10:5],cmos_16bit_data[15:11],cmos_16bit_data[4:0]}; // 尝试GRB重映射 - 结果：绿色和品红色
    //assign write_data = {cmos_16bit_data[15:11],cmos_16bit_data[4:0],cmos_16bit_data[10:5]}; // 尝试RBG重映射 - 结果：蓝黄色调
    //assign write_data = cmos_16bit_data; // 原始RGB映射 - 结果：天花板自然，但肤色为绿色

    reg [4:0] cmos_vs_cnt;
    always@(posedge cmos_vsync) 
        cmos_vs_cnt <= cmos_vs_cnt + 1;

    // LED调试信号 - 显示摄像头数据流状态
    reg [25:0] led_counter;
    reg led_vsync_toggle;
    reg led_data_activity;
    reg [15:0] prev_cmos_data;
    
    always@(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            led_counter <= 0;
            led_vsync_toggle <= 0;
            led_data_activity <= 0;
            prev_cmos_data <= 0;
        end else begin
            led_counter <= led_counter + 1;
            
            // 检测VSYNC变化 - LED闪烁表示帧同步正常
            if(cmos_vsync && !led_vsync_toggle) begin
                led_vsync_toggle <= 1;
            end else if(led_counter[24]) begin  // 约每0.67秒复位一次
                led_vsync_toggle <= 0;
            end
            
            // 检测数据变化 - LED亮表示数据在变化
            if(cmos_16bit_data != prev_cmos_data && cmos_16bit_de) begin
                led_data_activity <= 1;
                prev_cmos_data <= cmos_16bit_data;
            end else if(led_counter[22]) begin  // 约每0.17秒检查一次
                led_data_activity <= 0;
            end
        end
    end

    //generate the CMOS sensor clock and the SDRAM controller, I2C controller clock
    Gowin_PLL Gowin_PLL_m0(
    	.clkin                     (clk                         ),
    	.clkout0                   (cmos_clk 	              	),
        .clkout1                   (aux_clk 	              	),
        .clkout2                   (memory_clk 	              	),
    	.lock 					   (DDR_pll_lock 				),
        .reset                     (1'b0                        ),
        .enclk0                    (1'b1                        ), //input enclk0
        .enclk1                    (1'b1                        ), //input enclk1
        .enclk2                    (pll_stop                    ) //input enclk2
	);

    // 改进的复位序列管理
    
    
    

    


    
    

    

    
    

    
    
    
    
    // 帧缓冲区状态监控和保护机制
    wire fifo_full, fifo_empty;
    reg [3:0] fifo_full_cnt, fifo_empty_cnt;
    reg fifo_overflow_flag, fifo_underflow_flag;
    
    // FIFO状态计数器 - 防止偶发的满/空状态触发错误保护
    always @(posedge dma_clk or negedge rst_n) begin
        if (!rst_n) begin
            fifo_full_cnt <= 0;
            fifo_empty_cnt <= 0;
            fifo_overflow_flag <= 0;
            fifo_underflow_flag <= 0;
        end else begin
            // FIFO满状态计数
            if (fifo_full) begin
                if (fifo_full_cnt < 4'hF)
                    fifo_full_cnt <= fifo_full_cnt + 1;
                if (fifo_full_cnt > 4'h8)  // 连续满状态超过8个时钟周期
                    fifo_overflow_flag <= 1;
            end else begin
                fifo_full_cnt <= 0;
                fifo_overflow_flag <= 0;
            end
            
            // FIFO空状态计数
            if (fifo_empty) begin
                if (fifo_empty_cnt < 4'hF)
                    fifo_empty_cnt <= fifo_empty_cnt + 1;
                if (fifo_empty_cnt > 4'h8)  // 连续空状态超过8个时钟周期
                    fifo_underflow_flag <= 1;
            end else begin
                fifo_empty_cnt <= 0;
                fifo_underflow_flag <= 0;
            end
        end
    end

    // 为功耗管理添加fb_vout_de信号连接与输入视频总线定义
    wire fb_vin_clk;
    wire fb_vin_vsync;
    wire [15:0] fb_vin_data;
    wire fb_vin_de;
    wire fb_vout_de;

    assign fb_vout_de = off0_syn_de;

    // 帧缓冲输入切换为视频域的“拼接后帧流”，确保写入DDR与显示时序一致
    // 若启用TPG，则将TPG作为帧缓冲输入；否则使用三窗口合成的final_rgb565
    generate if(USE_TPG == "true")
    begin
        assign fb_vin_clk      = video_clk;
        assign fb_vin_vsync    = tp0_vs_in;
        assign fb_vin_data     = {tp0_data_r[7:3],tp0_data_g[7:2],tp0_data_b[7:3]};
        assign fb_vin_de       = tp0_de_in;
    end else begin
        assign fb_vin_clk      = video_clk;        // 统一在视频时钟域组帧并写入DDR
        assign fb_vin_vsync    = syn_off0_vs;      // 与显示VS保持一致，便于读写对齐
        assign fb_vin_data     = final_rgb565;     // 三窗口合成后的RGB565帧流
        assign fb_vin_de       = out_de;           // 数据有效沿用视频域DE
    end
    endgenerate

    // 时钟域交叉同步器 - 解决VSYNC信号跨时钟域传递的时序问题
    reg [2:0] vsync_sync_dma;
    reg [2:0] vsync_sync_video;
    wire vsync_dma_sync, vsync_video_sync;
    
    // 将VSYNC同步到DMA时钟域
    always @(posedge dma_clk or negedge rst_n) begin
        if (!rst_n)
            vsync_sync_dma <= 3'b0;
        else
            vsync_sync_dma <= {vsync_sync_dma[1:0], fb_vin_vsync};
    end
    assign vsync_dma_sync = vsync_sync_dma[2];
    
    // 将输出VSYNC同步到视频时钟域
    always @(posedge video_clk or negedge rst_n) begin
        if (!rst_n)
            vsync_sync_video <= 3'b0;
        else
            vsync_sync_video <= {vsync_sync_video[1:0], syn_off0_vs};
    end
    assign vsync_video_sync = vsync_sync_video[2];

    

    


    // DDR Output video Timing Align - 改进时序对齐
    //---------------------------------------------
    wire [4:0] lcd_r,lcd_b;
    wire [5:0] lcd_g;
    wire lcd_vs,lcd_de,lcd_hs,lcd_dclk;
    
    // 改进RGB数据映射，确保正确的RGB565颜色通道顺序
    // RGB565格式：[15:11]=R[4:0], [10:5]=G[5:0], [4:0]=B[4:0]
    assign lcd_r    = off0_syn_de ? off0_syn_data[15:11] : 5'h00;  // R通道：位[15:11]
    assign lcd_g    = off0_syn_de ? off0_syn_data[10:5]  : 6'h00;  // G通道：位[10:5]
    assign lcd_b    = off0_syn_de ? off0_syn_data[4:0]   : 5'h00;  // B通道：位[4:0]
    assign lcd_vs      			  = Pout_vs_dn[2];//使用更深的延迟对齐
    assign lcd_hs      			  = Pout_hs_dn[2];//使用更深的延迟对齐
    assign lcd_de      			  = Pout_de_dn[2];//使用更深的延迟对齐
    assign lcd_dclk    			  = video_clk;//video_clk_phs;

    // 注意：Pout_*_dn 在前文已定义并移位，这里不再重复声明

    

    

// 添加电源管理和时钟门控
reg power_save_mode;
reg [15:0] idle_counter;
wire system_idle;

// 检测系统空闲状态
assign system_idle = (idle_counter > 16'd1000);

always @(posedge dma_clk or negedge rst_n) begin
    if (!rst_n) begin
        idle_counter <= 16'd0;
        power_save_mode <= 1'b0;
    end else begin
        if (fb_vin_de || fb_vout_de) begin
            idle_counter <= 16'd0;
            power_save_mode <= 1'b0;
        end else begin
            if (idle_counter < 16'd2000)
                idle_counter <= idle_counter + 1'b1;
            else
                power_save_mode <= 1'b1;
        end
    end
end

// 时钟门控以降低功耗
wire gated_cmos_pclk;
wire gated_video_clk;

assign gated_cmos_pclk = cmos_pclk & (~power_save_mode);
assign gated_video_clk = video_clk & (~power_save_mode);

// 添加系统状态监控和调试信号
reg [31:0] debug_counter;
reg [7:0] system_status;
reg [15:0] error_flags;

// 系统状态编码
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        debug_counter <= 32'd0;
        system_status <= 8'd0;
        error_flags <= 16'd0;
    end else begin
        debug_counter <= debug_counter + 1'b1;
        
        // 系统状态监控
        system_status <= {
            system_stable,           // bit 7: 系统稳定
            init_calib_complete,     // bit 6: DDR初始化完成
            DDR_pll_lock,           // bit 5: PLL锁定
            TMDS_DDR_pll_lock,      // bit 4: HDMI PLL锁定
            i2c_done,               // bit 3: I2C配置完成
            cmos_start_config,      // bit 2: 摄像头开始配置
            fb_vin_de,              // bit 1: 输入数据有效
            fb_vout_de              // bit 0: 输出数据有效
        };
        
        // 错误标志监控
        error_flags <= {
            i2c_err,                // bit 15: I2C错误
            fifo_overflow_flag,     // bit 14: FIFO溢出
            fifo_underflow_flag,    // bit 13: FIFO下溢
            ~DDR_pll_lock & system_stable, // bit 12: PLL失锁
            ~TMDS_DDR_pll_lock & system_stable, // bit 11: HDMI PLL失锁
            11'd0                   // 保留位
        };
    end
end

// 改进的复位序列管理
    reg [31:0] system_reset_counter;
    reg [3:0] reset_state;
    reg system_stable;
    reg ddr_reset_stable;
    reg cmos_reset_stable;
    
    // 复位状态机
    localparam RST_IDLE = 4'd0;
    localparam RST_SYSTEM = 4'd1;
    localparam RST_DDR_WAIT = 4'd2;
    localparam RST_CMOS_WAIT = 4'd3;
    localparam RST_STABLE = 4'd4;
    
    always@(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            system_reset_counter <= 32'd0;
            reset_state <= RST_IDLE;
            system_stable <= 1'b0;
            ddr_reset_stable <= 1'b0;
            cmos_reset_stable <= 1'b0;
        end else begin
            case(reset_state)
                RST_IDLE: begin
                    system_reset_counter <= system_reset_counter + 1'b1;
                    if(system_reset_counter >= 32'd1_000_000) begin // 20ms@50MHz
                        reset_state <= RST_SYSTEM;
                        system_reset_counter <= 32'd0;
                    end
                end
                
                RST_SYSTEM: begin
                    system_reset_counter <= system_reset_counter + 1'b1;
                    if(system_reset_counter >= 32'd2_500_000 && DDR_pll_lock) begin // 50ms@50MHz
                        reset_state <= RST_DDR_WAIT;
                        system_reset_counter <= 32'd0;
                        ddr_reset_stable <= 1'b1;
                    end
                end
                
                RST_DDR_WAIT: begin
                    system_reset_counter <= system_reset_counter + 1'b1;
                    if(system_reset_counter >= 32'd5_000_000 && init_calib_complete) begin // 100ms@50MHz
                        reset_state <= RST_CMOS_WAIT;
                        system_reset_counter <= 32'd0;
                        cmos_reset_stable <= 1'b1;
                    end
                end
                
                RST_CMOS_WAIT: begin
                    system_reset_counter <= system_reset_counter + 1'b1;
                    if(system_reset_counter >= 32'd2_500_000) begin // 50ms@50MHz
                        reset_state <= RST_STABLE;
                        system_stable <= 1'b1;
                    end
                end
                
                RST_STABLE: begin
                    // 系统稳定运行状态
                    system_stable <= 1'b1;
                end
                
                default: reset_state <= RST_IDLE;
            endcase
        end
    end

    // Reset for camera sensor - 使用改进的复位序列
    reg [31:0] cmos_reset_delay_cnt;
    reg cmos_reset;
    reg cmos_start_config;
    // I2C配置重试：收紧配置触发与时序
    localparam [31:0] I2C_RESTART_GAP      = 32'd500_000;   // STOP到下一次START的间隔
    localparam [2:0]  I2C_RECONFIG_PASSES  = 3;             // 整表写入重复次数
    reg [2:0]  i2c_pass_cnt;
    reg [31:0] i2c_restart_cnt;
    always@(posedge clk or negedge rst_n)
    begin
        if(!rst_n)
        begin
            cmos_reset_delay_cnt <= 32'd0;
            cmos_reset <= 1'b0;
            cmos_start_config <= 1'b0;
            i2c_pass_cnt <= 3'd0;
            i2c_restart_cnt <= 32'd0;
        end else begin
            if(cmos_reset_stable) begin
                // Phase A: 复位保持
                if(cmos_reset_delay_cnt < 32'd5_000_000) begin
                    cmos_reset_delay_cnt <= cmos_reset_delay_cnt + 1;
                    cmos_reset <= 1'b0;
                    cmos_start_config <= 1'b0;
                    i2c_pass_cnt <= 3'd0;
                    i2c_restart_cnt <= 32'd0;
                end
                // Phase B: 释放复位，等待上电稳定
                else if(cmos_reset_delay_cnt < 32'd10_000_000) begin
                    cmos_reset_delay_cnt <= cmos_reset_delay_cnt + 1;
                    cmos_reset <= 1'b1;
                    cmos_start_config <= 1'b0;
                end
                // Phase C: 启动配置，并做重复写入（带间隔）
                else begin
                    cmos_reset_delay_cnt <= cmos_reset_delay_cnt;
                    cmos_reset <= 1'b1;
                    if(i2c_done) begin
                        if(i2c_pass_cnt < (I2C_RECONFIG_PASSES - 1)) begin
                            if(i2c_restart_cnt < I2C_RESTART_GAP) begin
                                i2c_restart_cnt   <= i2c_restart_cnt + 1;
                                cmos_start_config <= 1'b0; // 复位I2C控制器，拉出STOP
                            end else begin
                                i2c_restart_cnt   <= 32'd0;
                                i2c_pass_cnt      <= i2c_pass_cnt + 3'd1;
                                cmos_start_config <= 1'b1; // 下一轮整表写入
                            end
                        end else begin
                            cmos_start_config <= 1'b0; // 完成所有重复后释放
                        end
                    end else begin
                        cmos_start_config <= 1'b1; // 正在写入
                    end
                end
            end else begin
                cmos_reset_delay_cnt <= 32'd0;
                cmos_reset <= 1'b0;
                cmos_start_config <= 1'b0;
                i2c_pass_cnt <= 3'd0;
                i2c_restart_cnt <= 32'd0;
            end
        end
    end

    //configure look-up table
    lut_ov5640_rgb565 #(
    	.HActive(12'd1280),
    	.VActive(12'd720),
    	.HTotal(13'd2200),
    	.VTotal(13'd750),    // 修复：720+5+5+20=750，匹配标准720p时序
        .USE_4vs3_frame("false")  // 修复：使用16:9的1280x720分辨率，而非4:3
    )lut_ov5640_rgb565_m0(
    	.lut_index(lut_index),
    	.lut_data(lut_data)
    );


    //I2C master controller（与c0/c1同一总线，仅导出cmos_scl/cmos_sda）
    wire i2c_scl_bus;
    wire i2c_sda_bus;
    assign cmos_scl = i2c_scl_bus;
    assign cmos_sda = i2c_sda_bus;

    i2c_config i2c_config_m0(
        .rst                        (~cmos_start_config       ),
        .clk                        (clk                      ),
        .clk_div_cnt                (16'd5000                  ),
        .i2c_addr_2byte             (1'b1                     ),
        .lut_index                  (lut_index                ),
        .lut_dev_addr               (lut_data[31:24]          ),
        .lut_reg_addr               (lut_data[23:8]           ),
        .lut_reg_data               (lut_data[7:0]            ),
        .error                      (i2c_err                  ),
        .done                       (i2c_done                 ),
        .i2c_scl                    (i2c_scl_bus              ),
        .i2c_sda                    (i2c_sda_bus              )
    );
    

    //CMOS sensor 8bit data is converted to 16bit data
    // C0：对 href/vsync 进行三取二多数判决的去毛刺过滤（仅作用于C0）
    reg  [2:0]  c0_href_sr, c0_vsync_sr;
    wire        c0_href_filt  = (c0_href_sr[2] & c0_href_sr[1]) | (c0_href_sr[1] & c0_href_sr[0]) | (c0_href_sr[2] & c0_href_sr[0]);
    wire        c0_vsync_filt = (c0_vsync_sr[2] & c0_vsync_sr[1]) | (c0_vsync_sr[1] & c0_vsync_sr[0]) | (c0_vsync_sr[2] & c0_vsync_sr[0]);
    always @(posedge cmos_pclk or negedge rst_n) begin
        if(!rst_n) begin
            c0_href_sr  <= 3'b000;
            c0_vsync_sr <= 3'b000;
        end else begin
            c0_href_sr  <= {c0_href_sr[1:0],  cmos_href};
            c0_vsync_sr <= {c0_vsync_sr[1:0], cmos_vsync};
        end
    end
    cmos_8_16bit #(.SWAP_BYTES(1'b1), .RESET_ON_BLANK(1'b1)) cmos_8_16bit_m0(
        .rst        (~rst_n        ),
        .pclk       (cmos_pclk     ),
        .pdata_i    (cmos_db       ),
        .de_i       (c0_href_filt  ),
        .pdata_o    (cmos_16bit_data),
        .hblank     (cmos_16bit_wr ),
        .de_o       (cmos_16bit_de )
    );
    // 直接RGB565路径，无需YUV转换

    // cam1: CMOS sensor 8bit to 16bit RGB565
    // C1：对 href/vsync 进行三取二多数判决的去毛刺过滤（与C2一致）
    reg  [2:0]  c1_href_sr, c1_vsync_sr;
    wire        c1_href_filt  = (c1_href_sr[2] & c1_href_sr[1]) | (c1_href_sr[1] & c1_href_sr[0]) | (c1_href_sr[2] & c1_href_sr[0]);
    wire        c1_vsync_filt = (c1_vsync_sr[2] & c1_vsync_sr[1]) | (c1_vsync_sr[1] & c1_vsync_sr[0]) | (c1_vsync_sr[2] & c1_vsync_sr[0]);
    always @(posedge c1_pclk or negedge rst_n) begin
        if(!rst_n) begin
            c1_href_sr  <= 3'b000;
            c1_vsync_sr <= 3'b000;
        end else begin
            c1_href_sr  <= {c1_href_sr[1:0],  c1_href};
            c1_vsync_sr <= {c1_vsync_sr[1:0], c1_vsync};
        end
    end
    cmos_8_16bit #(.SWAP_BYTES(1'b1), .RESET_ON_BLANK(1'b1)) cmos_8_16bit_m1(
        .rst                        (~rst_n                   ),
        .pclk                       (c1_pclk                  ),
        .pdata_i                    (c1_db                    ),
        .de_i                       (c1_href_filt             ),
        .pdata_o                    (c1_16bit_data            ),
        .hblank                     (/*unused*/               ),
        .de_o                       (c1_16bit_de              )
    );
    // 直接RGB565路径，无需YUV转换
    // 直接RGB565路径，无需YUV转换

    // cam2: CMOS sensor 8bit to 16bit RGB565
    wire [15:0] c2_16bit_data; 
    wire        c2_16bit_de;
    // C2：对 href/vsync 进行三取二多数判决的去毛刺过滤（仅作用于C2）
    reg  [2:0]  c2_href_sr, c2_vsync_sr;
    wire        c2_href_filt  = (c2_href_sr[2] & c2_href_sr[1]) | (c2_href_sr[1] & c2_href_sr[0]) | (c2_href_sr[2] & c2_href_sr[0]);
    wire        c2_vsync_filt = (c2_vsync_sr[2] & c2_vsync_sr[1]) | (c2_vsync_sr[1] & c2_vsync_sr[0]) | (c2_vsync_sr[2] & c2_vsync_sr[0]);
    always @(posedge c2_pclk or negedge rst_n) begin
        if(!rst_n) begin
            c2_href_sr  <= 3'b000;
            c2_vsync_sr <= 3'b000;
        end else begin
            c2_href_sr  <= {c2_href_sr[1:0],  c2_href};
            c2_vsync_sr <= {c2_vsync_sr[1:0], c2_vsync};
        end
    end
    cmos_8_16bit #(.SWAP_BYTES(1'b1), .RESET_ON_BLANK(1'b1)) cmos_8_16bit_m2(
        .rst                        (~rst_n                   ),
        .pclk                       (c2_pclk                  ),
        .pdata_i                    (c2_db                    ),
        .de_i                       (c2_href_filt             ),
        .pdata_o                    (c2_16bit_data            ),
        .hblank                     (/*unused*/               ),
        .de_o                       (c2_16bit_de              )
    );
    // 直接RGB565路径，无需YUV转换

    //The video output timing generator and generate a frame read data request
    //输出
    wire out_de;
    wire [10:0] lcd_x,lcd_y;

    vga_timing #(
        .H_ACTIVE(16'd1280), 
        .H_FP(16'd110),
        .H_SYNC(16'd40),
        .H_BP(16'd220),
        .V_ACTIVE(16'd720),
        .V_FP(16'd5),
        .V_SYNC(16'd5),
        .V_BP(16'd20), 	
        .HS_POL(1'b1),   	
        .VS_POL(1'b1)
    ) vga_timing_m0(
        .clk (video_clk),
        .rst (~rst_n),
        .active_x(lcd_x),
        .active_y(lcd_y),
        .hs(syn_off0_hs),
        .vs(syn_off0_vs),
        .de(out_de)
    );

    // =============================================
    // 三摄像头缩放 + 横向并排（尺寸不变），垂直居中
    // =============================================
    localparam integer SCALE      = 4;
    localparam integer OUT_W      = 1280 / SCALE; // 实际=320
    localparam integer OUT_H      = 720  / SCALE; // 实际=180
    // 横向间隔像素（窗口之间），默认0
    localparam integer GAP_PIX    = 0;
    // 垂直居中：窗口上边缘位置
    localparam integer Y0         = (720 - OUT_H) / 2;
    // 三路裁剪（单位：下采样后的像素数），裁剪量与上次一致
    localparam integer C0_CROP_L  = 0;
    localparam integer C0_CROP_R  = OUT_W/5; // 右裁 1/5
    localparam integer C1_CROP_L  = 12;   // 左裁剪宽度
    localparam integer C1_CROP_R  = 12;   // 右裁剪宽度
    localparam integer C2_CROP_L  = OUT_W/5; // 左裁 1/5
    localparam integer C2_CROP_R  = 0;
    localparam integer C0_W_EFF   = OUT_W - C0_CROP_L - C0_CROP_R;
    localparam integer C1_W_EFF   = OUT_W - C1_CROP_L - C1_CROP_R; // c1有效显示宽度
    localparam integer C2_W_EFF   = OUT_W - C2_CROP_L - C2_CROP_R;
    // 羽化宽度（边缘过渡更自然），选择16像素，便于移位实现线性混合
    localparam integer FEATHER    = 16;
    localparam integer FEATHER_SHIFT = 4; // 除以16
    // 计算总宽度，并将三幅图像整体居中（考虑裁剪后宽度）
    localparam integer W_TOTAL    = C0_W_EFF + C1_W_EFF + C2_W_EFF + GAP_PIX*2;
    // 三个窗口的水平起点（从左到右，居中）
    localparam integer X0         = (1280 - W_TOTAL) / 2;
    localparam integer X1         = X0 + C0_W_EFF + GAP_PIX;
    localparam integer X2         = X1 + C1_W_EFF + GAP_PIX;
    // 12位坐标常量与当前像素坐标
    localparam [11:0] OUT_W_12 = OUT_W;
    localparam [11:0] OUT_H_12 = OUT_H;
    localparam [11:0] C0_W_EFF_12 = C0_W_EFF;
    localparam [11:0] C1_W_EFF_12 = C1_W_EFF;
    localparam [11:0] C2_W_EFF_12 = C2_W_EFF;
    localparam [11:0] C0_CROP_L_12= C0_CROP_L;
    localparam [11:0] C1_CROP_L_12= C1_CROP_L;
    localparam [11:0] C2_CROP_L_12= C2_CROP_L;
    localparam [11:0] FEATHER_12  = FEATHER;
    localparam [11:0] X0_12    = X0;
    localparam [11:0] X1_12    = X1;
    localparam [11:0] X2_12    = X2;
    localparam [11:0] Y0_12    = Y0;
    wire [11:0] lcd_x12 = {1'b0, lcd_x};
    wire [11:0] lcd_y12 = {1'b0, lcd_y};

    // cam0 缩放写入（摄像头域） + 坐标读（视频域）
    wire [15:0] cam0_pix;
    // 读端窗口起始脉冲（用于乒乓在窗口边界切换）

    // C0：单缓冲，行偏移校正 2 行（为稳态，回退乒乓）
    downscale_3x_bram #(
        .IN_W(1280), .IN_H(720), .SCALE(SCALE), .USE_PINGPONG(1'b0), .Y_OFFSET(12'd2),
        .LPF_H2(1'b0), .LPF_V2(1'b0)
    ) u_cam0_ds (
        .wr_clk   (cmos_pclk),
        .wr_rst_n (rst_n),
        .wr_vs    (c0_vsync_filt),
        .wr_href  (c0_href_filt),
        .wr_de    (cmos_16bit_de),
        .wr_data  (write_data),

        .rd_clk   (video_clk),
        .rd_rst_n (rst_n),
        // c0：窗口读取使用裁剪后宽度；在 c0→c1 边界的羽化带内也读取 c0 最后 FEATHER 列
        .rd_en    ( ((lcd_x >= X0) && (lcd_x < (X0 + C0_W_EFF)) && (lcd_y >= Y0) && (lcd_y < (Y0 + OUT_H)) && out_de)
                  ||((lcd_x >= X1) && (lcd_x < (X1 + FEATHER))   && (lcd_y >= Y0) && (lcd_y < (Y0 + OUT_H)) && out_de) ),
        .rd_busy  ( ((lcd_x >= X0) && (lcd_x < (X0 + C0_W_EFF)) && (lcd_y >= Y0) && (lcd_y < (Y0 + OUT_H)) && out_de)
                  ||((lcd_x >= X1) && (lcd_x < (X1 + FEATHER))   && (lcd_y >= Y0) && (lcd_y < (Y0 + OUT_H)) && out_de) ),
        .rd_vs    (syn_off0_vs),
        .rd_x     ( (lcd_x < (X0 + C0_W_EFF)) ? (lcd_x12 - X0_12 + C0_CROP_L_12)
                    : ((C0_CROP_L_12 + C0_W_EFF_12 - FEATHER_12) + (lcd_x12 - X1_12)) ),
        .rd_y     (lcd_y12 - Y0_12),
        .rd_data  (cam0_pix)
    );

    // cam1 缩放写入（摄像头域） + 坐标读（视频域）
    wire [15:0] cam1_pix;
    // C1回退为单缓冲，配合读忙期写冻结，避免资源超限
    // C1：单缓冲，行偏移校正 1 行（可根据效果再调）
    downscale_3x_bram #(
        .IN_W(1280), .IN_H(720), .SCALE(SCALE), .USE_PINGPONG(1'b0), .Y_OFFSET(12'd1)
    ) u_cam1_ds (
        .wr_clk   (c1_pclk),
        .wr_rst_n (rst_n),
        .wr_vs    (c1_vsync_filt),
        .wr_href  (c1_href_filt),
        .wr_de    (c1_16bit_de),
        .wr_data  (c1_16bit_data),

        .rd_clk   (video_clk),
        .rd_rst_n (rst_n),
        // c1：窗口读取使用裁剪后宽度；在 c1→c2 边界的羽化带内也读取 c1 最后 FEATHER 列
        .rd_en    ( ((lcd_x >= X1) && (lcd_x < (X1 + C1_W_EFF)) && (lcd_y >= Y0) && (lcd_y < (Y0 + OUT_H)) && out_de)
                  ||((lcd_x >= X2) && (lcd_x < (X2 + FEATHER))   && (lcd_y >= Y0) && (lcd_y < (Y0 + OUT_H)) && out_de) ),
        .rd_busy  ( ((lcd_x >= X1) && (lcd_x < (X1 + C1_W_EFF)) && (lcd_y >= Y0) && (lcd_y < (Y0 + OUT_H)) && out_de)
                  ||((lcd_x >= X2) && (lcd_x < (X2 + FEATHER))   && (lcd_y >= Y0) && (lcd_y < (Y0 + OUT_H)) && out_de) ),
        .rd_vs    (syn_off0_vs),
        .rd_x     ( (lcd_x < (X1 + C1_W_EFF)) ? ((lcd_x12 - X1_12) + C1_CROP_L_12)
                    : ((C1_CROP_L_12 + C1_W_EFF_12 - FEATHER_12) + (lcd_x12 - X2_12)) ),
        .rd_y     (lcd_y12 - Y0_12),
        .rd_data  (cam1_pix)
    );

    // cam2 缩放写入（摄像头域） + 坐标读（视频域） — 与c0/c1一致
    wire [15:0] cam2_pix;
    // cam2 使用相同的垂直偏移与其对应的水平偏移
    // 读取窗口的行场同步沿用 DDR 输出 VSYNC，以保持与视频域一致
    // C2：单缓冲，行偏移校正 2 行
    downscale_3x_bram #(
        .IN_W(1280), .IN_H(720), .SCALE(SCALE), .USE_PINGPONG(1'b0), .Y_OFFSET(12'd2)
    ) u_cam2_ds (
        .wr_clk   (c2_pclk),
        .wr_rst_n (rst_n),
        .wr_vs    (c2_vsync_filt),
        .wr_href  (c2_href_filt),
        .wr_de    (c2_16bit_de),
        .wr_data  (c2_16bit_data),

        .rd_clk   (video_clk),
        .rd_rst_n (rst_n),
        .rd_en    ((lcd_x >= X2) && (lcd_x < (X2 + C2_W_EFF)) && (lcd_y >= Y0) && (lcd_y < (Y0 + OUT_H)) && out_de),
        .rd_busy  ((lcd_x >= X2) && (lcd_x < (X2 + C2_W_EFF)) && (lcd_y >= Y0) && (lcd_y < (Y0 + OUT_H)) && out_de),
        .rd_vs    (syn_off0_vs),
        .rd_x     (lcd_x12 - X2_12 + C2_CROP_L_12),
        .rd_y     (lcd_y12 - Y0_12),
        .rd_data  (cam2_pix)
    );

    // 拼接区域判定 + 边界羽化区域
    wire cam0_region = (lcd_x >= X0) && (lcd_x < (X0 + C0_W_EFF)) && (lcd_y >= Y0) && (lcd_y < (Y0 + OUT_H));
    wire cam1_region = (lcd_x >= X1) && (lcd_x < (X1 + C1_W_EFF)) && (lcd_y >= Y0) && (lcd_y < (Y0 + OUT_H));
    wire cam2_region = (lcd_x >= X2) && (lcd_x < (X2 + C2_W_EFF)) && (lcd_y >= Y0) && (lcd_y < (Y0 + OUT_H));
    wire blend01_zone = (lcd_x >= X1) && (lcd_x < (X1 + FEATHER)) && (lcd_y >= Y0) && (lcd_y < (Y0 + OUT_H));
    wire blend12_zone = (lcd_x >= X2) && (lcd_x < (X2 + FEATHER)) && (lcd_y >= Y0) && (lcd_y < (Y0 + OUT_H));

    // 基于 RGB565 分量的线性混合（除以16，移位实现）
    wire [4:0] c0_r = cam0_pix[15:11];
    wire [5:0] c0_g = cam0_pix[10:5];
    wire [4:0] c0_b = cam0_pix[4:0];
    wire [4:0] c1_r = cam1_pix[15:11];
    wire [5:0] c1_g = cam1_pix[10:5];
    wire [4:0] c1_b = cam1_pix[4:0];
    wire [4:0] c2_r = cam2_pix[15:11];
    wire [5:0] c2_g = cam2_pix[10:5];
    wire [4:0] c2_b = cam2_pix[4:0];

    wire [11:0] t01   = {1'b0, lcd_x} - X1_12; // 0..FEATHER-1
    wire [11:0] w01_a = FEATHER_12 - t01;
    wire [11:0] w01_b = t01;
    wire [4:0]  mix01_r = ((c0_r * w01_a) + (c1_r * w01_b)) >> FEATHER_SHIFT;
    wire [5:0]  mix01_g = ((c0_g * w01_a) + (c1_g * w01_b)) >> FEATHER_SHIFT;
    wire [4:0]  mix01_b = ((c0_b * w01_a) + (c1_b * w01_b)) >> FEATHER_SHIFT;
    wire [15:0] blend01_rgb565 = {mix01_r, mix01_g, mix01_b};

    wire [11:0] t12   = {1'b0, lcd_x} - X2_12; // 0..FEATHER-1
    wire [11:0] w12_a = FEATHER_12 - t12;
    wire [11:0] w12_b = t12;
    wire [4:0]  mix12_r = ((c1_r * w12_a) + (c2_r * w12_b)) >> FEATHER_SHIFT;
    wire [5:0]  mix12_g = ((c1_g * w12_a) + (c2_g * w12_b)) >> FEATHER_SHIFT;
    wire [4:0]  mix12_b = ((c1_b * w12_a) + (c2_b * w12_b)) >> FEATHER_SHIFT;
    wire [15:0] blend12_rgb565 = {mix12_r, mix12_g, mix12_b};

    // 输出：窗口区域显示对应缩放画面；边界采用羽化混合；外区恢复为黑色
    wire [15:0] final_rgb565 = blend01_zone ? blend01_rgb565
                              : (blend12_zone ? blend12_rgb565
                              : (cam0_region ? cam0_pix
                              : (cam1_region ? cam1_pix
                              : (cam2_region ? cam2_pix : 16'h0000))));
    

    //Test pattern generate
    ///--------------------------
    wire        tp0_vs_in  ;
    wire        tp0_hs_in  ;
    wire        tp0_de_in ;
    wire [ 7:0] tp0_data_r;
    wire [ 7:0] tp0_data_g;
    wire [ 7:0] tp0_data_b;

    generate if(USE_TPG == "true")         
    begin
        testpattern testpattern_inst_1280
        (
            .I_pxl_clk   (video_clk    ),//pixel clock
            .I_rst_n     (rst_n        ),//low active 
            .I_mode      (3'b000       ),//data select
            .I_single_r  (8'd255       ),
            .I_single_g  (8'd255       ),
            .I_single_b  (8'd255       ),                  //800x600    //1024x768   //1280x720   //1920x1080 
            .I_h_total   (12'd1650     ),//hor total time  // 12'd1056  // 12'd1344  // 12'd1650  // 12'd2200
            .I_h_sync    (12'd40       ),//hor sync time   // 12'd128   // 12'd136   // 12'd40    // 12'd44  
            .I_h_bporch  (12'd220      ),//hor back porch  // 12'd88    // 12'd160   // 12'd220   // 12'd148 
            .I_h_res     (12'd1280     ),//hor resolution  // 12'd800   // 12'd1024  // 12'd1280  // 12'd1920
            .I_v_total   (12'd750      ),//ver total time  // 12'd628   // 12'd806   // 12'd750   // 12'd1125 
            .I_v_sync    (12'd5        ),//ver sync time   // 12'd4     // 12'd6     // 12'd5     // 12'd5   
            .I_v_bporch  (12'd20       ),//ver back porch  // 12'd23    // 12'd29    // 12'd20    // 12'd36  
            .I_v_res     (12'd720      ),//ver resolution  // 12'd600   // 12'd768   // 12'd720   // 12'd1080 
            .I_hs_pol    (1'b1         ),//0,负极性;1,正极性
            .I_vs_pol    (1'b1         ),//0,负极性;1,正极性
            .O_de        (tp0_de_in    ),   
            .O_hs        (tp0_hs_in    ),
            .O_vs        (tp0_vs_in    ),
            .O_data_r    (tp0_data_r   ),   
            .O_data_g    (tp0_data_g   ),
            .O_data_b    (tp0_data_b   )
        );
    end
    endgenerate
    
    
    
    // 帧缓冲区状态监控和保护机制
    wire fifo_full, fifo_empty;
    reg [3:0] fifo_full_cnt, fifo_empty_cnt;
    reg fifo_overflow_flag, fifo_underflow_flag;
    
    // FIFO状态计数器 - 防止偶发的满/空状态触发错误保护
    always @(posedge dma_clk or negedge rst_n) begin
        if (!rst_n) begin
            fifo_full_cnt <= 0;
            fifo_empty_cnt <= 0;
            fifo_overflow_flag <= 0;
            fifo_underflow_flag <= 0;
        end else begin
            // FIFO满状态计数
            if (fifo_full) begin
                if (fifo_full_cnt < 4'hF)
                    fifo_full_cnt <= fifo_full_cnt + 1;
                if (fifo_full_cnt > 4'h8)  // 连续满状态超过8个时钟周期
                    fifo_overflow_flag <= 1;
            end else begin
                fifo_full_cnt <= 0;
                fifo_overflow_flag <= 0;
            end
            
            // FIFO空状态计数
            if (fifo_empty) begin
                if (fifo_empty_cnt < 4'hF)
                    fifo_empty_cnt <= fifo_empty_cnt + 1;
                if (fifo_empty_cnt > 4'h8)  // 连续空状态超过8个时钟周期
                    fifo_underflow_flag <= 1;
            end else begin
                fifo_empty_cnt <= 0;
                fifo_underflow_flag <= 0;
            end
        end
    end

    // 时钟域交叉同步器 - 解决VSYNC信号跨时钟域传递的时序问题
    reg [2:0] vsync_sync_dma;
    reg [2:0] vsync_sync_video;
    wire vsync_dma_sync, vsync_video_sync;
    
    // 将VSYNC同步到DMA时钟域
    always @(posedge dma_clk or negedge rst_n) begin
        if (!rst_n)
            vsync_sync_dma <= 3'b0;
        else
            vsync_sync_dma <= {vsync_sync_dma[1:0], fb_vin_vsync};
    end
    assign vsync_dma_sync = vsync_sync_dma[2];
    
    // 将输出VSYNC同步到视频时钟域
    always @(posedge video_clk or negedge rst_n) begin
        if (!rst_n)
            vsync_sync_video <= 3'b0;
        else
            vsync_sync_video <= {vsync_sync_video[1:0], syn_off0_vs};
    end
    assign vsync_video_sync = vsync_sync_video[2];

    Video_Frame_Buffer_Top Video_Frame_Buffer_Top_inst
    ( 
        .I_rst_n              (init_calib_complete ),
        .I_dma_clk            (dma_clk          ),
    `ifdef USE_THREE_FRAME_BUFFER 
        .I_wr_halt            (1'd0             ), //1:halt,  0:no halt
        .I_rd_halt            (1'd0             ), //1:halt,  0:no halt
    `endif

        // video data input - 使用同步后的信号       
        .I_vin0_clk           (fb_vin_clk   ),
        .I_vin0_vs_n          (~vsync_dma_sync),//使用同步后的VSYNC信号
        .I_vin0_de            (fb_vin_de    ),
        .I_vin0_data          (fb_vin_data  ),
        .O_vin0_fifo_full     (fifo_full    ),  // 连接FIFO满信号

        // video data output - 使用同步后的信号            
        .I_vout0_clk          (video_clk        ),
        .I_vout0_vs_n         (~vsync_video_sync),//使用同步后的VSYNC信号
        .I_vout0_de           (out_de           ),
        .O_vout0_den          (off0_syn_de      ),  // 保持原有连接
        .O_vout0_data         (off0_syn_data    ),
        .O_vout0_fifo_empty   (fifo_empty   ),  // 连接FIFO空信号
        // ddr write request
        .I_cmd_ready          (cmd_ready          ),
        .O_cmd                (cmd                ),//0:write;  1:read
        .O_cmd_en             (cmd_en             ),
    //    .O_app_burst_number   (app_burst_number   ),
        .O_addr               (addr               ),//[ADDR_WIDTH-1:0]
        .I_wr_data_rdy        (wr_data_rdy        ),
        .O_wr_data_en         (wr_data_en         ),//
        .O_wr_data_end        (wr_data_end        ),//
        .O_wr_data            (wr_data            ),//[DATA_WIDTH-1:0]
        .O_wr_data_mask       (wr_data_mask       ),
        .I_rd_data_valid      (rd_data_valid      ),
        .I_rd_data_end        (rd_data_end        ),//unused 
        .I_rd_data            (rd_data            ),//[DATA_WIDTH-1:0]
        .I_init_calib_complete(init_calib_complete)
    ); 

    DDR3MI u_ddr3 
    (
        .clk                (clk                ),
        .memory_clk         (memory_clk         ),
        .pll_stop           (pll_stop           ),
        .pll_lock           (DDR_pll_lock       ),
        .rst_n              (rst_n              ),
    //    .app_burst_number   (app_burst_number   ),
        .cmd_ready          (cmd_ready          ),
        .cmd                (cmd                ),
        .cmd_en             (cmd_en             ),
        .addr               (addr               ),
        .wr_data_rdy        (wr_data_rdy        ),
        .wr_data            (wr_data            ),
        .wr_data_en         (wr_data_en         ),
        .wr_data_end        (wr_data_end        ),
        .wr_data_mask       (wr_data_mask       ),
        .rd_data            (rd_data            ),
        .rd_data_valid      (rd_data_valid      ),
        .rd_data_end        (rd_data_end        ),
        .sr_req             (1'b0               ),
        .ref_req            (1'b0               ),
        .sr_ack             (                   ),
        .ref_ack            (                   ),
        .init_calib_complete(init_calib_complete),
        .clk_out            (dma_clk            ),
        .burst              (1'b1               ),
        // mem interface
        .ddr_rst            (                 ),
        .O_ddr_addr         (ddr_addr         ),
        .O_ddr_ba           (ddr_bank         ),
        .O_ddr_cs_n         (ddr_cs           ),
        .O_ddr_ras_n        (ddr_ras          ),
        .O_ddr_cas_n        (ddr_cas          ),
        .O_ddr_we_n         (ddr_we           ),
        .O_ddr_clk          (ddr_ck           ),
        .O_ddr_clk_n        (ddr_ck_n         ),
        .O_ddr_cke          (ddr_cke          ),
        .O_ddr_odt          (ddr_odt          ),
        .O_ddr_reset_n      (ddr_reset_n      ),
        .O_ddr_dqm          (ddr_dm           ),
        .IO_ddr_dq          (ddr_dq           ),
        .IO_ddr_dqs         (ddr_dqs          ),
        .IO_ddr_dqs_n       (ddr_dqs_n        )
    );


    // DDR Output video Timing Align - 改进时序对齐
    //---------------------------------------------
    wire [4:0] lcd_r,lcd_b;
    wire [5:0] lcd_g;
    wire lcd_vs,lcd_de,lcd_hs,lcd_dclk;
    
    // 改进RGB数据映射，确保正确的RGB565颜色通道顺序
    // RGB565格式：[15:11]=R[4:0], [10:5]=G[5:0], [4:0]=B[4:0]
    assign lcd_r    = off0_syn_de ? off0_syn_data[15:11] : 5'h00;  // R通道：位[15:11]
    assign lcd_g    = off0_syn_de ? off0_syn_data[10:5]  : 6'h00;  // G通道：位[10:5]
    assign lcd_b    = off0_syn_de ? off0_syn_data[4:0]   : 5'h00;  // B通道：位[4:0]
    assign lcd_vs      			  = Pout_vs_dn[2];//使用更深的延迟对齐
    assign lcd_hs      			  = Pout_hs_dn[2];//使用更深的延迟对齐
    assign lcd_de      			  = Pout_de_dn[2];//使用更深的延迟对齐
    assign lcd_dclk    			  = video_clk;//video_clk_phs;

    reg  [2:0]  Pout_hs_dn;  // 增加延迟深度
    reg  [2:0]  Pout_vs_dn;  // 增加延迟深度
    reg  [2:0]  Pout_de_dn;  // 增加延迟深度

    always@(posedge video_clk or negedge rst_n)
    begin
        if(!rst_n)
            begin                          
                Pout_hs_dn  <= {3'b111};  // 更新初始值
                Pout_vs_dn  <= {3'b111};  // 更新初始值
                Pout_de_dn  <= {3'b000};  // 更新初始值
            end
        else 
            begin                          
                Pout_hs_dn  <= {Pout_hs_dn[1:0],syn_off0_hs};  // 3位移位寄存器
                Pout_vs_dn  <= {Pout_vs_dn[1:0],syn_off0_vs};  // 3位移位寄存器
                Pout_de_dn  <= {Pout_de_dn[1:0],out_de};       // 3位移位寄存器
            end
    end

    //==============================================================================
    //TMDS TX(HDMI4)
    wire serial_clk;
    wire hdmi4_rst_n;

    TMDS_PLL u_tmds_pll(
        .clkin     (clk              ),     //input clk 
        .clkout0   (serial_clk       ),     //output clk x5ni
        .clkout1   (video_clk        ),     //output clk x1
        .lock      (TMDS_DDR_pll_lock)      //output lock
        );

    assign hdmi4_rst_n = rst_n & TMDS_DDR_pll_lock;

    wire dvi0_rgb_clk;
    wire dvi0_rgb_vs ;
    wire dvi0_rgb_hs ;
    wire dvi0_rgb_de ;
    wire [7:0] dvi0_rgb_r  ;
    wire [7:0] dvi0_rgb_g  ;
    wire [7:0] dvi0_rgb_b  ;

    wire dvi1_rgb_clk;
    wire dvi1_rgb_vs ;
    wire dvi1_rgb_hs ;
    wire dvi1_rgb_de ;
    wire [7:0] dvi1_rgb_r  ;
    wire [7:0] dvi1_rgb_g  ;
    wire [7:0] dvi1_rgb_b  ;

generate if(DDR_BYPASS == "true")
begin
    // DVI直通：使用拼接后的视频流，而非TPG或DDR
    assign dvi0_rgb_clk = video_clk;
    assign dvi0_rgb_vs  = syn_off0_vs; // 使用主视频域的VS
    assign dvi0_rgb_hs  = syn_off0_hs; // 使用主视频域的HS
    assign dvi0_rgb_de  = out_de;      // 使用主视频域的DE
    assign dvi0_rgb_r   = {final_rgb565[15:11], 3'b0}; // 扩展至8位
    assign dvi0_rgb_g   = {final_rgb565[10:5],  2'b0}; // 扩展至8位
    assign dvi0_rgb_b   = {final_rgb565[4:0],   3'b0}; // 扩展至8位
end else begin
    assign dvi0_rgb_clk = lcd_dclk;
    assign dvi0_rgb_vs  = lcd_vs;
    assign dvi0_rgb_hs  = lcd_hs;
    assign dvi0_rgb_de  = lcd_de;
    // 从DDR帧缓冲读出的off0_syn_data映射到8位RGB，保证显示与帧缓冲一致
    assign dvi0_rgb_r   = {off0_syn_data[15:11], off0_syn_data[15:13]};
    assign dvi0_rgb_g   = {off0_syn_data[10:5],  off0_syn_data[10:9]};
    assign dvi0_rgb_b   = {off0_syn_data[4:0],   off0_syn_data[4:2]};
end
endgenerate

    DVI_TX_Top DVI_TX_Top_inst0
    (
        .I_rst_n       (hdmi4_rst_n   ),  //asynchronous reset, low active
        .I_serial_clk  (serial_clk    ),

        //RGB input
        .I_rgb_clk     (dvi0_rgb_clk  ),  //pixel clock
        .I_rgb_vs      (dvi0_rgb_vs   ), 
        .I_rgb_hs      (dvi0_rgb_hs   ),    
        .I_rgb_de      (dvi0_rgb_de   ), 
        .I_rgb_r       (dvi0_rgb_r    ), 
        .I_rgb_g       (dvi0_rgb_g    ),  
        .I_rgb_b       (dvi0_rgb_b    ),  

        .O_tmds_clk_p  (tmds_clk_p_0  ),
        .O_tmds_clk_n  (tmds_clk_n_0  ),
        .O_tmds_data_p (tmds_d_p_0    ),  //{r,g,b}
        .O_tmds_data_n (tmds_d_n_0    )
    );
    // Bluetooth servo control integration
    // Does not affect camera pipeline; runs on system clk/rst
    bt_servo_ctrl u_bt_servo_ctrl (
        .clk       (clk),
        .rst_n     (rst_n),
        .bt_state  (bt_state),
        .bt_rxd    (bt_rxd),
        .bt_txd    (bt_txd),
        .bt_en     (bt_en),
        .servo_pwm (servo_pwm)
    );

endmodule