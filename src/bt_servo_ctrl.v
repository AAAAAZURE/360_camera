// Bluetooth HC-05 servo control module
// Integrates UART RX/TX parser and 50Hz PWM for 270° servo
// Commands: A0..270 set angle, P500..2500 set pulse_us, L/R/X shortcuts
module bt_servo_ctrl(
    input  wire clk,
    input  wire rst_n,
    input  wire bt_state,
    input  wire bt_rxd,
    output wire bt_txd,
    output wire bt_en,
    output wire servo_pwm
);
    // keep EN low for normal mode
    assign bt_en = 1'b0;

    // UART RX/TX
    wire rx_ready; wire [7:0] rx_data;
    wire tx_busy;  reg  tx_start; reg [7:0] tx_byte;
    // match uart_rx port names: rxd, rx_valid, rx_data
    uart_rx #(.CLK_FREQ(50_000_000), .BAUD(9600)) u_rx(
        .clk(clk), .rst_n(rst_n), .rxd(bt_rxd), .rx_valid(rx_ready), .rx_data(rx_data)
    );
    // match uart_tx port names: tx_data, tx_start, tx_busy, txd
    uart_tx #(.CLK_FREQ(50_000_000), .BAUD(9600)) u_tx(
        .clk(clk), .rst_n(rst_n), .tx_data(tx_byte), .tx_start(tx_start), .tx_busy(tx_busy), .txd(bt_txd)
    );

    // parser state
    reg [15:0] pulse_us;
    reg [1:0]  mode; // 0 idle, 1 reading P, 3 reading A
    reg [15:0] num_accum;
    reg [15:0] min_us;
    reg [15:0] max_us;
    // test sweep helper
    reg        sweep_en;
    reg        sweep_dir; // 0 up, 1 down
    reg [15:0] sweep_val;
    reg [23:0] slow_cnt;
    reg        invert; // PWM polarity toggle for hardware-level test

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pulse_us  <= 16'd1500;
            mode      <= 2'd0;
            num_accum <= 16'd0;
            min_us    <= 16'd500;
            max_us    <= 16'd2500;
            tx_start  <= 1'b0;
            tx_byte   <= 8'h00;
            invert    <= 1'b0;
        end else begin
            tx_start <= 1'b0;
            if (rx_ready) begin
                case (rx_data)
                    "P": begin mode <= 2'd1; num_accum <= 16'd0; end
                    "A": begin mode <= 2'd3; num_accum <= 16'd0; end
                    "L": begin pulse_us <= min_us;  mode <= 2'd0; tx_byte <= "O"; tx_start <= 1'b1; end
                    "R": begin pulse_us <= max_us;  mode <= 2'd0; tx_byte <= "O"; tx_start <= 1'b1; end
                    "X": begin // mid angle 135°
                        pulse_us <= min_us + (((16'd135 << 3) - 16'd135) + (16'd135 >> 2));
                        mode <= 2'd0; tx_byte <= "O"; tx_start <= 1'b1;
                    end
                    "T": begin // toggle test sweep
                        sweep_en <= ~sweep_en;
                        if (!sweep_en) begin
                            sweep_val <= min_us;
                            sweep_dir <= 1'b0;
                        end
                        tx_byte <= "O"; tx_start <= 1'b1; mode <= 2'd0;
                    end
                    "I": begin // toggle PWM polarity
                        invert  <= ~invert;
                        tx_byte <= "O"; tx_start <= 1'b1; mode <= 2'd0;
                    end
                    "\n","\r": begin
                        if (mode == 2'd1) begin
                            if (num_accum < 16'd500)      pulse_us <= 16'd500;
                            else if (num_accum > 16'd2500) pulse_us <= 16'd2500;
                            else                            pulse_us <= num_accum;
                        end else if (mode == 2'd3) begin
                            if (num_accum > 16'd270) begin
                                pulse_us <= max_us;
                            end else begin
                                // min_us + angle * ~7.25
                                pulse_us <= min_us + ((num_accum << 3) - num_accum) + (num_accum >> 2);
                            end
                        end
                        mode <= 2'd0; tx_byte <= "K"; tx_start <= 1'b1;
                    end
                    default: begin
                        if (mode != 2'd0) begin
                            if (rx_data >= "0" && rx_data <= "9") begin
                                num_accum <= num_accum*10 + (rx_data-"0");
                            end
                        end
                    end
                endcase
            end

            // test sweep: slow ramp across range when enabled
            if (sweep_en) begin
                slow_cnt <= slow_cnt + 1'b1;
                if (slow_cnt[18]) begin // ~5ms step @50MHz
                    slow_cnt <= 24'd0;
                    if (!sweep_dir) begin
                        if (sweep_val < max_us) sweep_val <= sweep_val + 1'b1; else sweep_dir <= 1'b1;
                    end else begin
                        if (sweep_val > min_us) sweep_val <= sweep_val - 1'b1; else sweep_dir <= 1'b0;
                    end
                    pulse_us <= sweep_val;
                end
            end else begin
                slow_cnt <= 24'd0;
            end
        end
    end

    // PWM output
    wire servo_raw;
    servo_pwm #(.CLK_FREQ(50_000_000)) u_servo (
        .clk(clk), .rst_n(rst_n), .pulse_us(pulse_us), .pwm_out(servo_raw)
    );
    assign servo_pwm = invert ? ~servo_raw : servo_raw;
endmodule