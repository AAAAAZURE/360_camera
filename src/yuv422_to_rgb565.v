module yuv422_to_rgb565 #(
    parameter ORDER = "UYVY"  // UYVY 或 YUYV
) (
    input              pclk,
    input              rst,
    input              de_i,          // 每16bit有效（来自cmos_8_16bit）
    input  [15:0]      uyvy_i,        // 按字节顺序打包的16bit：高8位在前
    output reg         de_o,          // 每像素输出有效（两个像素连续输出）
    output reg [15:0]  rgb565_o
);

    // 状态：偶数/奇数16位字
    reg state; // 0: 第一个字(含U或Y0)，1: 第二个字(含V或Y1)
    reg [7:0] U, V, Y0, Y1;
    reg       emit_second; // 第二个像素待输出标记（下一拍输出）
    reg [15:0] rgb_buf;    // 第二个像素暂存

    // 有符号差值
    wire signed [9:0] Ud = {1'b0, U} - 10'sd128;
    wire signed [9:0] Vd = {1'b0, V} - 10'sd128;

    // 近似定点转换（系数×256）
    function [15:0] yuv_to_rgb565;
        input [7:0] Y;
        input signed [9:0] Ud_in;
        input signed [9:0] Vd_in;
        integer r,g,b;
        integer tmp;
        begin
            // R = Y + 1.402*Vd
            tmp = $signed({Y,8'd0}) + $signed(Vd_in) * 359; // 1.402*256≈359
            r = tmp >>> 8;
            // G = Y - 0.344*Ud - 0.714*Vd
            tmp = $signed({Y,8'd0}) - $signed(Ud_in)*88 - $signed(Vd_in)*183; // 0.344*256≈88, 0.714*256≈183
            g = tmp >>> 8;
            // B = Y + 1.772*Ud
            tmp = $signed({Y,8'd0}) + $signed(Ud_in) * 454; // 1.772*256≈454
            b = tmp >>> 8;

            // 限幅 0..255
            if (r < 0) r = 0; else if (r > 255) r = 255;
            if (g < 0) g = 0; else if (g > 255) g = 255;
            if (b < 0) b = 0; else if (b > 255) b = 255;

            // 压缩到RGB565
            yuv_to_rgb565 = {r[7:3], g[7:2], b[7:3]};
        end
    endfunction

    always @(posedge pclk or posedge rst) begin
        if (rst) begin
            state       <= 1'b0;
            de_o        <= 1'b0;
            emit_second <= 1'b0;
            rgb565_o    <= 16'h0000;
        end else begin
            // 默认无输出（除非我们要在下一拍发第二个像素）
            if (emit_second) begin
                de_o        <= 1'b1;
                rgb565_o    <= rgb_buf;
                emit_second <= 1'b0;
            end else begin
                de_o        <= 1'b0;
            end

            if (de_i) begin
                // 根据ORDER解释当前16位
                if (ORDER == "UYVY") begin
                    if (!state) begin
                        U  <= uyvy_i[15:8];
                        Y0 <= uyvy_i[7:0];
                        state <= 1'b1;
                    end else begin
                        V  <= uyvy_i[15:8];
                        Y1 <= uyvy_i[7:0];
                        state <= 1'b0;
                        // 计算两个像素并输出第一个，缓存第二个，下一拍再发
                        rgb565_o    <= yuv_to_rgb565(Y0, Ud, Vd);
                        rgb_buf     <= yuv_to_rgb565(Y1, Ud, Vd);
                        de_o        <= 1'b1;
                        emit_second <= 1'b1;
                    end
                end else begin // YUYV
                    if (!state) begin
                        Y0 <= uyvy_i[15:8];
                        U  <= uyvy_i[7:0];
                        state <= 1'b1;
                    end else begin
                        Y1 <= uyvy_i[15:8];
                        V  <= uyvy_i[7:0];
                        state <= 1'b0;
                        rgb565_o    <= yuv_to_rgb565(Y0, Ud, Vd);
                        rgb_buf     <= yuv_to_rgb565(Y1, Ud, Vd);
                        de_o        <= 1'b1;
                        emit_second <= 1'b1;
                    end
                end
            end
        end
    end

endmodule