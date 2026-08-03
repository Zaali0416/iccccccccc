// =============================================================================
// File:        project.v
// Purpose:     Tiny Tapeout Top-Level Wrapper for WMS IC ASIC System
// Target:      Tiny Tapeout (SkyWater 130nm PDK / OpenLane Flow)
// =============================================================================

`default_nettype none

module tt_um_wms_system (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // Bidirectional pins (Input path)
    output wire [7:0] uio_out,  // Bidirectional pins (Output path)
    output wire [7:0] uio_oe,   // Bidirectional pins (Enable path: 1=Output, 0=Input)
    input  wire       ena,      // Always 1 when powered
    input  wire       clk,      // System clock input
    input  wire       rst_n     // Active-LOW system reset
);

    // -------------------------------------------------------------------------
    // 1. PIN MAPPING & INTERFACE CONVERSION
    // -------------------------------------------------------------------------
    
    // Convert Active-Low reset (rst_n) to Active-High for internal RESET_LOGIC
    wire reset_logic = ~rst_n;

    // Direct Input Assignments (ui_in)
    wire [5:0] tank_sensors = ui_in[5:0]; // {UP_100, UP_90, UP_70, UP_50, UP_30, UP_10}
    wire       lt_signal    = ui_in[6];   // Lower Tank Level Sensor
    wire       wf_probe     = ui_in[7];   // Water Flow Confirmation Probe

    // Bidirectional Input Assignment (uio_in)
    wire       buzz_off     = uio_in[0];  // Mute Alarm Button

    // Unused Bidirectional pins configuration (Set to Inputs)
    assign uio_oe  = 8'b0000_0000;
    assign uio_out = 8'b0000_0000;

    // Core Output Wires
    wire       motor;
    wire       error;
    wire       buzzer;
    wire [5:0] led_up;
    wire       led_lt;

    // -------------------------------------------------------------------------
    // 2. TOP-LEVEL ASIC MODULE INSTANTIATION
    // -------------------------------------------------------------------------
    wms_ic_gatelevel_top u_wms_asic_top (
        .CLK          (clk),
        .RESET_LOGIC  (reset_logic),
        .BUZZ_OFF     (buzz_off),
        .TANK_SENSORS (tank_sensors),
        .LT_SIGNAL    (lt_signal),
        .WF_PROBE     (wf_probe),
        .MOTOR        (motor),
        .ERROR        (error),
        .BUZZER       (buzzer),
        .LED_UP       (led_up),
        .LED_LT       (led_lt)
    );

    // -------------------------------------------------------------------------
    // 3. OUTPUT MAPPING (uo_out)
    // -------------------------------------------------------------------------
    assign uo_out[0] = motor;      // Pump Relay Driver
    assign uo_out[1] = error;      // Fault Indicator LED
    assign uo_out[2] = buzzer;     // Alarm Sounder Output
    assign uo_out[3] = led_lt;     // Lower Tank Level Active LED
    assign uo_out[4] = led_up[0];  // Upper Tank 10% Level LED
    assign uo_out[5] = led_up[2];  // Upper Tank 50% Level LED
    assign uo_out[6] = led_up[4];  // Upper Tank 90% Level LED
    assign uo_out[7] = led_up[5];  // Upper Tank 100% Level LED

endmodule


// =============================================================================
// 1. TOP-LEVEL ASIC GATE-LEVEL WRAPPER WITH ON-CHIP DEBOUNCERS
// =============================================================================
module wms_ic_gatelevel_top (
    input  wire       CLK,             // External clock input (50 MHz)
    input  wire       RESET_LOGIC,     // System reset button (Active-High)
    input  wire       BUZZ_OFF,        // Alarm mute button

    // Tank Sensors & Probes
    input  wire [5:0] TANK_SENSORS,    // {UP_100, UP_90, UP_70, UP_50, UP_30, UP_10}
    input  wire       LT_SIGNAL,       // Lower Tank Level Probe
    input  wire       WF_PROBE,        // Water Flow Confirmation Probe

    // System Outputs
    output wire       MOTOR,           // Pump Relay Drive Output
    output wire       ERROR,           // Fault Indicator Output
    output wire       BUZZER,          // Alarm Sounder Output
    output wire [5:0] LED_UP,          // 6-Level Upper Tank LED Outputs
    output wire       LED_LT           // Lower Tank Level LED Output
);

    // --- Clean Internal Signals after Debouncing / CDC Synchronization ---
    wire       wf_probe_clean;
    wire       lt_signal_clean;
    wire       buzz_off_clean;

    // Direct Lower Tank LED output drives directly from clean probe state
    assign LED_LT = lt_signal_clean;

    // --- On-Chip Digital Debouncers (Filter chatter & water sloshing) ---
    wms_debouncer u_deb_wf (
        .clk       (CLK),
        .reset     (RESET_LOGIC),
        .async_in  (WF_PROBE),
        .clean_out (wf_probe_clean)
    );

    wms_debouncer u_deb_lt (
        .clk       (CLK),
        .reset     (RESET_LOGIC),
        .async_in  (LT_SIGNAL),
        .clean_out (lt_signal_clean)
    );

    wms_debouncer u_deb_buzz (
        .clk       (CLK),
        .reset     (RESET_LOGIC),
        .async_in  (BUZZ_OFF),
        .clean_out (buzz_off_clean)
    );

    // --- Core Logic Engine Instantiation ---
    wms_ic_core u_wms_core (
        .CLK          (CLK),
        .RESET_LOGIC  (RESET_LOGIC),
        .BUZZ_OFF     (buzz_off_clean),
        .UP_100       (TANK_SENSORS[5]),
        .UP_90        (TANK_SENSORS[4]),
        .UP_70        (TANK_SENSORS[3]),
        .UP_50        (TANK_SENSORS[2]),
        .UP_30        (TANK_SENSORS[1]),
        .UP_10        (TANK_SENSORS[0]),
        .LT_SIGNAL    (lt_signal_clean),
        .WF_PROBE     (wf_probe_clean),
        .LED_UP_100   (LED_UP[5]),
        .LED_UP_90    (LED_UP[4]),
        .LED_UP_70    (LED_UP[3]),
        .LED_UP_50    (LED_UP[2]),
        .LED_UP_30    (LED_UP[1]),
        .LED_UP_10    (LED_UP[0]),
        .MOTOR        (MOTOR),
        .ERROR        (ERROR),
        .BUZZER       (BUZZER)
    );

endmodule


// =============================================================================
// 2. REUSABLE DIGITAL DEBOUNCER & CDC SYNCHRONIZER MODULE
// =============================================================================
module wms_debouncer #(
    parameter [19:0] DEBOUNCE_LIMIT = 20'd1_000_000 // Default: 20ms at 50 MHz
)(
    input  wire clk,
    input  wire reset,
    input  wire async_in,
    output reg  clean_out
);

    `ifdef SIMULATION
        // Fast single-cycle passthrough for top-level system simulation
        always @(posedge clk or posedge reset) begin
            if (reset) begin
                clean_out <= 1'b0;
            end else begin
                clean_out <= async_in;
            end
        end
    `else
        // Target Silicon / Hardware Verification Branch
        reg sync_0, sync_1;
        reg [19:0] count;

        always @(posedge clk or posedge reset) begin
            if (reset) begin
                sync_0    <= 1'b0;
                sync_1    <= 1'b0;
                count     <= 20'd0;
                clean_out <= 1'b0;
            end else begin
                // 2-Stage CDC Synchronizer (Prevents metastability)
                sync_0 <= async_in;
                sync_1 <= sync_0;

                // Filtering counter logic with abort-on-bounce reset
                if (sync_1 != clean_out) begin
                    if (count >= DEBOUNCE_LIMIT - 1'b1) begin
                        clean_out <= sync_1;
                        count     <= 20'd0;
                    end else begin
                        count <= count + 1'b1;
                    end
                end else begin
                    count <= 20'd0;
                end
            end
        end
    `endif

endmodule


// =============================================================================
// 3. CORE LOGIC ENGINE (Hysteresis, 60s Grace Timer & Latched Mute)
// =============================================================================
module wms_ic_core (
    input  wire CLK,
    input  wire RESET_LOGIC,
    input  wire UP_10,
    input  wire UP_30,
    input  wire UP_50,
    input  wire UP_70,
    input  wire UP_90,
    input  wire UP_100,
    input  wire LT_SIGNAL,
    input  wire WF_PROBE,
    input  wire BUZZ_OFF,

    output wire LED_UP_10,
    output wire LED_UP_30,
    output wire LED_UP_50,
    output wire LED_UP_70,
    output wire LED_UP_90,
    output wire LED_UP_100,

    output reg  MOTOR,
    output reg  ERROR,
    output reg  BUZZER
);

    // Direct LED passthrough
    assign LED_UP_10  = UP_10;
    assign LED_UP_30  = UP_30;
    assign LED_UP_50  = UP_50;
    assign LED_UP_70  = UP_70;
    assign LED_UP_90  = UP_90;
    assign LED_UP_100 = UP_100;

    // Shared "Reset Logic" clear term:
    wire fault_clear = WF_PROBE || (~LT_SIGNAL);

    // --- A. Motor Hysteresis Latch ---
    reg pump_active;

    always @(posedge CLK or posedge RESET_LOGIC) begin
        if (RESET_LOGIC) begin
            pump_active <= 1'b0;
        end else begin
            // Turn pump ON: Lower Tank has water AND Upper Tank is 0%
            if (LT_SIGNAL && (~UP_10)) begin
                pump_active <= 1'b1;
            end
            // Turn pump OFF: Upper Tank is 100% full OR Lower Tank empty
            else if (UP_100 || (~LT_SIGNAL)) begin
                pump_active <= 1'b0;
            end
        end
    end

    // --- B. Clock Divider & Grace Timer ---
    `ifdef SIMULATION
        // Fast tick for rapid simulation execution
        wire tick_1sec = 1'b1;
    `else
        reg [25:0] clk_divider;
        wire tick_1sec = (clk_divider == 26'd49_999_999);
    `endif

    reg [5:0] sec_counter;
    reg       error_latched;
    wire      flow_fault = pump_active && (~WF_PROBE);

    always @(posedge CLK or posedge RESET_LOGIC) begin
        if (RESET_LOGIC) begin
            `ifndef SIMULATION
                clk_divider <= 26'd0;
            `endif
            sec_counter   <= 6'd0;
            error_latched <= 1'b0;
        end else if (UP_100 || fault_clear) begin
            `ifndef SIMULATION
                clk_divider <= 26'd0;
            `endif
            sec_counter   <= 6'd0;
            error_latched <= 1'b0;
        end else begin
            if (flow_fault && !error_latched) begin
                if (tick_1sec) begin
                    `ifndef SIMULATION
                        clk_divider <= 26'd0;
                    `endif
                    if (sec_counter == 6'd59) begin
                        error_latched <= 1'b1;
                    end else begin
                        sec_counter <= sec_counter + 1'b1;
                    end
                end else begin
                    `ifndef SIMULATION
                        clk_divider <= clk_divider + 1'b1;
                    `endif
                end
            end
            else if (!flow_fault && !error_latched) begin
                `ifndef SIMULATION
                    clk_divider <= 26'd0;
                `endif
                sec_counter <= 6'd0;
            end
        end
    end

    // --- C. Latched Alarm Mute Logic ---
    reg mute_latched;

    always @(posedge CLK or posedge RESET_LOGIC) begin
        if (RESET_LOGIC) begin
            mute_latched <= 1'b0;
        end else if (UP_100 || fault_clear || !error_latched) begin
            mute_latched <= 1'b0;
        end else if (BUZZ_OFF) begin
            mute_latched <= 1'b1;
        end
    end

    // --- D. Output Combination Logic ---
    always @(*) begin
        if (error_latched) begin
            MOTOR  = 1'b0;
            ERROR  = 1'b1;
            BUZZER = ~mute_latched;
        end else if (pump_active) begin
            MOTOR  = 1'b1;
            ERROR  = 1'b0;
            BUZZER = 1'b0;
        end else begin
            MOTOR  = 1'b0;
            ERROR  = 1'b0;
            BUZZER = 1'b0;
        end
    end

endmodule
