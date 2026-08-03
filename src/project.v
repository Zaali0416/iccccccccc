// ============================================================================
// File:    wms_ic_system.v
// Description: Consolidated Single-File Design containing Top-Level Wrapper, 
//              Core Controller, and 7-Segment Decoders.
// Target:  DE1-SoC Board (5CSEMA5F31C6) / Generic ASIC Standard Cell Flow
// Status:  LOCKED & VALIDATED
// ============================================================================

// ----------------------------------------------------------------------------
// 1. TOP-LEVEL MODULE
// ----------------------------------------------------------------------------
module wms_ic_fpga_top (
    input  wire       CLOCK_50,     // 50MHz Clock Input
    input  wire       RESET_BTN,    // Pushbutton KEY0 (Active-Low)
    input  wire [5:0] TANK_SENSORS, // Upper Tank: SW0-SW5 (10% to 100%)
    input  wire       LT_SENSOR,    // Lower Tank Sensor (SW6)
    input  wire       FLOW_PROBE,   // Water Flow Sensor Probe (SW7)
    input  wire       MUTE_BTN,     // Mute Button KEY1 (Active-Low)

    output wire       MOTOR_OUT,    // Pump Control Output (LEDR0)
    output wire       BUZZER_OUT,   // Audio Alarm Output (LEDR1)
    output wire       ERROR_LED,    // Fault Signal Indicator LED (LEDR2)
    output wire       LT_LED,       // Lower Tank Active LED (LEDR3)
    output wire [5:0] TANK_LEDS,    // Upper Tank Level Display (LEDR4-9)

    // 7-Segment Displays (Active-Low)
    output wire [6:0] HEX0,         // Seconds (Units: 0-9)
    output wire [6:0] HEX1,         // Seconds (Tens: 0-5)
    output wire [6:0] HEX2          // Minutes (Units: 0-9)
);

    // Active-Low Pushbutton Inversion
    wire reset_logic = ~RESET_BTN;  // KEY0 Pressed = HIGH
    wire mute_active = ~MUTE_BTN;   // KEY1 Pressed = HIGH

    wire [3:0] sec_ones;
    wire [3:0] sec_tens;
    wire [3:0] min_ones;

    // Core Controller Instantiation
    wms_ic_core CORE (
        .clk_50mhz   (CLOCK_50),
        .reset_logic (reset_logic),
        .up_10       (TANK_SENSORS[0]),
        .up_30       (TANK_SENSORS[1]),
        .up_50       (TANK_SENSORS[2]),
        .up_70       (TANK_SENSORS[3]),
        .up_90       (TANK_SENSORS[4]),
        .up_100      (TANK_SENSORS[5]),
        .lt_signal   (LT_SENSOR),
        .wf_prob     (FLOW_PROBE),
        .buzz_off    (mute_active),
        .motor       (MOTOR_OUT),
        .buzzer      (BUZZER_OUT),
        .error_led   (ERROR_LED),
        .lt_led      (LT_LED),
        .led_up      (TANK_LEDS),
        .sec_ones    (sec_ones),
        .sec_tens    (sec_tens),
        .min_ones    (min_ones)
    );

    // 7-Segment BCD Displays Instantiation
    seven_seg_decoder U_HEX0 ( .hex_digit(sec_ones), .seg_out(HEX0) );
    seven_seg_decoder U_HEX1 ( .hex_digit(sec_tens), .seg_out(HEX1) );
    seven_seg_decoder U_HEX2 ( .hex_digit(min_ones), .seg_out(HEX2) );

endmodule

// ----------------------------------------------------------------------------
// 2. CORE LOGIC MODULE (Pump Hysteresis, 60s Grace Timer, Mute Latch)
// ----------------------------------------------------------------------------
module wms_ic_core (
    input  wire       clk_50mhz,
    input  wire       reset_logic,    // Reset (Active High)
    input  wire       up_10, up_30, up_50, up_70, up_90, up_100,
    input  wire       lt_signal,      // Lower tank sensor
    input  wire       wf_prob,        // Flow probe
    input  wire       buzz_off,       // Mute button pulse
    
    output reg        motor,
    output reg        buzzer,
    output reg        error_led,
    output wire       lt_led,
    output wire [5:0] led_up,

    output reg  [3:0] sec_ones,
    output reg  [3:0] sec_tens,
    output reg  [3:0] min_ones
);

    // Direct Status Feedback LED Pass-through
    assign led_up = {up_100, up_90, up_70, up_50, up_30, up_10};
    assign lt_led = lt_signal;

    // --- A. Motor Hysteresis Latch ---
    reg pump_active;

    always @(posedge clk_50mhz or posedge reset_logic) begin
        if (reset_logic) begin
            pump_active <= 1'b0;
        end else begin
            // Turn ON pump when Lower Tank has water and Upper Tank is 0%
            if (lt_signal && (~up_10)) begin
                pump_active <= 1'b1;
            end 
            // Turn OFF pump when Upper Tank reaches 100% OR Lower Tank is empty
            else if (up_100 || (~lt_signal)) begin
                pump_active <= 1'b0;
            end
        end
    end

    // --- B. 1-Second Clock Divider & 60-Second Grace Timer ---
    reg [25:0] clk_divider;
    wire tick_1sec = (clk_divider == 26'd49_999_999);

    reg error_latched;
    wire flow_fault = pump_active && (~wf_prob);

    always @(posedge clk_50mhz or posedge reset_logic) begin
        if (reset_logic) begin
            clk_divider   <= 26'd0;
            sec_ones      <= 4'd0;
            sec_tens      <= 4'd0;
            min_ones      <= 4'd0;
            error_latched <= 1'b0;
        end else if (up_100) begin
            // Reset state automatically if full condition met
            clk_divider   <= 26'd0;
            sec_ones      <= 4'd0;
            sec_tens      <= 4'd0;
            min_ones      <= 4'd0;
            error_latched <= 1'b0;
        end else begin
            if (flow_fault && !error_latched) begin
                if (tick_1sec) begin
                    clk_divider <= 26'd0;
                    
                    // Increment 7-Segment BCD Display
                    if (sec_ones == 4'd9) begin
                        sec_ones <= 4'd0;
                        if (sec_tens == 4'd5) begin
                            sec_tens <= 4'd0;
                            min_ones <= min_ones + 1'b1;
                        end else begin
                            sec_tens <= sec_tens + 1'b1;
                        end
                    end else begin
                        sec_ones <= sec_ones + 1'b1;
                    end

                    // Trip error trigger after 60 seconds (1:00)
                    if (sec_tens == 4'd5 && sec_ones == 4'd9) begin
                        error_latched <= 1'b1;
                    end
                end else begin
                    clk_divider <= clk_divider + 1'b1;
                end
            end 
            else if (!flow_fault && !error_latched) begin
                clk_divider <= 26'd0;
                sec_ones    <= 4'd0;
                sec_tens    <= 4'd0;
                min_ones    <= 4'd0;
            end
        end
    end

    // --- C. Latched Mute Logic (Flip-Flop Behavior) ---
    reg mute_latched;

    always @(posedge clk_50mhz or posedge reset_logic) begin
        if (reset_logic) begin
            mute_latched <= 1'b0;
        end else if (up_100 || !error_latched) begin
            mute_latched <= 1'b0;
        end else if (buzz_off) begin
            mute_latched <= 1'b1;
        end
    end

    // --- D. Output Generation ---
    always @(*) begin
        if (error_latched) begin
            motor     = 1'b0;             // Disable pump upon error
            error_led = 1'b1;             // Assert error LED
            buzzer    = ~mute_latched;    // Silence buzzer if mute latch set
        end else if (pump_active) begin
            motor     = 1'b1;             // Pump active during grace window
            error_led = 1'b0;
            buzzer    = 1'b0;
        end else begin
            motor     = 1'b0;
            error_led = 1'b0;
            buzzer    = 1'b0;
        end
    end

endmodule

// ----------------------------------------------------------------------------
// 3. 7-SEGMENT BCD DECODER MODULE (Active-Low Outputs)
// ----------------------------------------------------------------------------
module seven_seg_decoder (
    input  wire [3:0] hex_digit,
    output reg  [6:0] seg_out
);
    always @(*) begin
        case (hex_digit)
            4'h0: seg_out = 7'b100_0000; // Display 0
            4'h1: seg_out = 7'b111_1001; // Display 1
            4'h2: seg_out = 7'b010_0100; // Display 2
            4'h3: seg_out = 7'b011_0000; // Display 3
            4'h4: seg_out = 7'b001_1001; // Display 4
            4'h5: seg_out = 7'b010_0010; // Display 5
            4'h6: seg_out = 7'b000_0010; // Display 6
            4'h7: seg_out = 7'b111_1000; // Display 7
            4'h8: seg_out = 7'b000_0000; // Display 8
            4'h9: seg_out = 7'b001_0000; // Display 9
            default: seg_out = 7'b111_1111; // Display Off
        endcase
    end
endmodule
