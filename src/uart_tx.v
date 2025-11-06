// Simple UART TX, 8N1
module uart_tx #(
    parameter CLK_FREQ = 50_000_000,
    parameter BAUD     = 9600
) (
    input  wire clk,
    input  wire rst_n,
    input  wire [7:0] tx_data,
    input  wire tx_start,
    output reg  tx_busy,
    output reg  txd
);
    localparam integer CLKS_PER_BIT = CLK_FREQ / BAUD; // 5208 for 50MHz@9600

    reg [15:0] clk_cnt;
    reg [3:0]  bit_idx;
    reg [7:0]  data_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            txd     <= 1'b1; // idle high
            tx_busy <= 1'b0;
            clk_cnt <= 16'd0;
            bit_idx <= 4'd0;
            data_reg<= 8'd0;
        end else begin
            if (!tx_busy) begin
                if (tx_start) begin
                    tx_busy <= 1'b1;
                    data_reg<= tx_data;
                    txd     <= 1'b0; // start bit
                    bit_idx <= 4'd0;
                    clk_cnt <= CLKS_PER_BIT-1;
                end
            end else begin
                if (clk_cnt == 0) begin
                    if (bit_idx < 8) begin
                        txd     <= data_reg[bit_idx];
                        bit_idx <= bit_idx + 1'b1;
                        clk_cnt <= CLKS_PER_BIT-1;
                    end else if (bit_idx == 8) begin
                        txd     <= 1'b1; // stop bit
                        bit_idx <= 4'd9;
                        clk_cnt <= CLKS_PER_BIT-1;
                    end else begin
                        tx_busy <= 1'b0;
                    end
                end else begin
                    clk_cnt <= clk_cnt - 1'b1;
                end
            end
        end
    end
endmodule