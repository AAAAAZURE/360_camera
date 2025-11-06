// Simple UART RX, 8N1
module uart_rx #(
    parameter CLK_FREQ = 50_000_000,
    parameter BAUD     = 9600
) (
    input  wire clk,
    input  wire rst_n,
    input  wire rxd,
    output reg  rx_valid,
    output reg [7:0] rx_data
);
    localparam integer CLKS_PER_BIT = CLK_FREQ / BAUD; // 5208 for 50MHz@9600

    reg [15:0] clk_cnt;
    reg [3:0]  bit_idx;
    reg        rxd_sync0, rxd_sync1;
    reg        busy;
    reg [7:0]  data_reg;

    // synchronize
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rxd_sync0 <= 1'b1;
            rxd_sync1 <= 1'b1;
        end else begin
            rxd_sync0 <= rxd;
            rxd_sync1 <= rxd_sync0;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy     <= 1'b0;
            clk_cnt  <= 16'd0;
            bit_idx  <= 4'd0;
            data_reg <= 8'd0;
            rx_valid <= 1'b0;
            rx_data  <= 8'd0;
        end else begin
            rx_valid <= 1'b0;
            if (!busy) begin
                // wait for start bit (falling edge)
                if (rxd_sync1 == 1'b0) begin
                    busy    <= 1'b1;
                    clk_cnt <= CLKS_PER_BIT/2; // move to middle of start bit
                    bit_idx <= 4'd0;
                end
            end else begin
                if (clk_cnt == 0) begin
                    // sample
                    if (bit_idx == 0) begin
                        // still in start bit, next move to first data bit
                        clk_cnt <= CLKS_PER_BIT-1;
                        bit_idx <= 4'd1;
                    end else if (bit_idx >= 1 && bit_idx <= 8) begin
                        data_reg[bit_idx-1] <= rxd_sync1; // LSB first
                        clk_cnt <= CLKS_PER_BIT-1;
                        bit_idx <= bit_idx + 1'b1;
                    end else if (bit_idx == 9) begin
                        // stop bit
                        busy     <= 1'b0;
                        rx_valid <= 1'b1;
                        rx_data  <= data_reg;
                    end
                end else begin
                    clk_cnt <= clk_cnt - 1'b1;
                end
            end
        end
    end
endmodule