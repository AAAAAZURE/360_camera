// 50Hz servo PWM (standard hobby servo). Input pulse_us in microseconds.
module servo_pwm #(
    parameter CLK_FREQ = 50_000_000
) (
    input  wire clk,
    input  wire rst_n,
    input  wire [15:0] pulse_us,     // 500..2500 supported, typical 1000..2000
    output reg  pwm_out
);
    // 50Hz => 20ms period
    localparam integer PERIOD_US   = 20_000;
    localparam integer TICKS_PER_US= CLK_FREQ/1_000_000; // 50 at 50MHz
    localparam integer PERIOD_TICKS= PERIOD_US * TICKS_PER_US; // 1,000,000

    reg [31:0] tick_cnt;
    reg [31:0] high_ticks;

    // clamp pulse to safe range
    wire [15:0] pulse_us_clamped = (pulse_us < 16'd500) ? 16'd500 :
                                   (pulse_us > 16'd2500) ? 16'd2500 : pulse_us;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tick_cnt  <= 32'd0;
            pwm_out   <= 1'b0;
            high_ticks<= 32'd75_000; // ~1500us default
        end else begin
            high_ticks <= pulse_us_clamped * TICKS_PER_US;
            if (tick_cnt >= PERIOD_TICKS-1) begin
                tick_cnt <= 32'd0;
            end else begin
                tick_cnt <= tick_cnt + 1'b1;
            end
            pwm_out <= (tick_cnt < high_ticks);
        end
    end
endmodule