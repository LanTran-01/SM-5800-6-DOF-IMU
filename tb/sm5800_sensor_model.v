//==============================================================================
// Module      : sm5800_sensor_model
// Project     : SM-5800 6-DOF Motion Sensor Module (IMU) AXI4-Lite Interface
// Standard    : Verilog-2001 (IEEE 1364-2001) — Simulation Behavioral Model
// Description : Behavioral emulator of the SM-5800 6-DOF IMU SDLC transmitter:
//               - Drives NRZ serial data synchronous to host RCV_CLK (shifts
//                 on falling edge of rcv_clk_i, matching Z8530 SCC behavior).
//               - Implements genuine bit-stuffing (zero insertion after 5 ones)
//                 and Z8530 SCC CRC-16-CCITT (preset 0xFFFF, inverted FCS).
//               - Supports idle 0x7E flags, shared flags, Commanded BIT (BIT_L),
//                 and fault/error injection for verification.
//==============================================================================

`timescale 1ns / 1ps
`default_nettype none

module sm5800_sensor_model (
    input  wire rcv_clk_i,   // 1.0 MHz Receive Clock from host FPGA
    input  wire bit_l_i,     // Active-low Commanded BIT input
    input  wire reset_l_i,   // Active-low Sensor Module Reset input
    output reg  ser_data_o   // NRZ SDLC Transmit Data to host FPGA
);

    // Internal CRC & bit-stuffing state for behavioral transmitter tasks
    reg [15:0] tx_crc;
    integer    tx_ones_cnt;

    initial begin
        ser_data_o  = 1'b1; // Idle mark
        tx_crc      = 16'hFFFF;
        tx_ones_cnt = 0;
    end

    //--------------------------------------------------------------------------
    // Helper Task: Shift one raw bit onto ser_data_o on falling edge of rcv_clk
    //--------------------------------------------------------------------------
    task send_raw_bit;
        input bit_val;
        begin
            @(negedge rcv_clk_i);
            #15; // Simulate realistic RS-422 + SCC clock-to-out delay (15 ns)
            ser_data_o = bit_val;
        end
    endtask

    //--------------------------------------------------------------------------
    // Helper Task: Send unstuffed SDLC Flag (0x7E = 01111110, LSB-first)
    //--------------------------------------------------------------------------
    task send_flag;
        integer k;
        reg [7:0] flag_byte;
        begin
            flag_byte   = 8'h7E;
            tx_ones_cnt = 0;
            for (k = 0; k < 8; k = k + 1) begin
                send_raw_bit(flag_byte[k]);
            end
        end
    endtask

    //--------------------------------------------------------------------------
    // Helper Task: Send N consecutive idle flags (0x7E)
    //--------------------------------------------------------------------------
    task send_idle_flags;
        input integer num_flags;
        integer n;
        begin
            for (n = 0; n < num_flags; n = n + 1) begin
                send_flag();
            end
        end
    endtask

    //--------------------------------------------------------------------------
    // Helper Task: Update CRC-16-CCITT (reflected poly 0x8408, LSB-first)
    //--------------------------------------------------------------------------
    task crc_update_bit;
        input bit_val;
        reg fb;
        begin
            fb     = tx_crc[0] ^ bit_val;
            tx_crc = {1'b0, tx_crc[15:1]} ^ (fb ? 16'h8408 : 16'h0000);
        end
    endtask

    //--------------------------------------------------------------------------
    // Helper Task: Send 1 data byte LSB-first with SDLC zero-insertion & CRC
    //--------------------------------------------------------------------------
    task send_stuffed_byte;
        input [7:0] data_byte;
        input       update_crc;
        integer k;
        reg b;
        begin
            for (k = 0; k < 8; k = k + 1) begin
                b = data_byte[k];
                if (update_crc) begin
                    crc_update_bit(b);
                end
                send_raw_bit(b);
                if (b == 1'b1) begin
                    tx_ones_cnt = tx_ones_cnt + 1;
                    if (tx_ones_cnt == 5) begin
                        send_raw_bit(1'b0); // Zero insertion (bit stuffing)
                        tx_ones_cnt = 0;
                    end
                end else begin
                    tx_ones_cnt = 0;
                end
            end
        end
    endtask

    //--------------------------------------------------------------------------
    // Helper Task: Send 16-bit Little-Endian word
    //--------------------------------------------------------------------------
    task send_u16_le;
        input [15:0] val;
        begin
            send_stuffed_byte(val[7:0],  1'b1);
            send_stuffed_byte(val[15:8], 1'b1);
        end
    endtask

    //--------------------------------------------------------------------------
    // Helper Task: Send 32-bit Little-Endian word
    //--------------------------------------------------------------------------
    task send_u32_le;
        input [31:0] val;
        begin
            send_stuffed_byte(val[7:0],   1'b1);
            send_stuffed_byte(val[15:8],  1'b1);
            send_stuffed_byte(val[23:16], 1'b1);
            send_stuffed_byte(val[31:24], 1'b1);
        end
    endtask

    //--------------------------------------------------------------------------
    // Helper Task: Transmit CRC-16-CCITT FCS (standard SDLC inverted by default)
    //--------------------------------------------------------------------------
    task send_fcs;
        input invert_fcs;
        input inject_crc_err;
        reg [15:0] fcs_word;
        begin
            fcs_word = invert_fcs ? (~tx_crc) : tx_crc;
            if (inject_crc_err) begin
                fcs_word = fcs_word ^ 16'h0001; // Flip 1 bit to force CRC failure
            end
            send_stuffed_byte(fcs_word[7:0],  1'b0);
            send_stuffed_byte(fcs_word[15:8], 1'b0);
        end
    endtask

    //--------------------------------------------------------------------------
    // Task: Transmit SM-5800 Message ID 1 (Motion Control Data — 20 bytes)
    //--------------------------------------------------------------------------
    task send_msg1;
        input [7:0]  addr;
        input [15:0] rate_x;
        input [15:0] rate_y;
        input [15:0] rate_z;
        input [15:0] acc_x;
        input [15:0] acc_y;
        input [15:0] acc_z;
        input [15:0] sw1;
        input [15:0] sw2;
        input        invert_fcs;
        input        inject_crc_err;
        input        skip_open_flag;  // 1: use previous frame's closing flag (shared flag)
        begin
            if (!skip_open_flag) begin
                send_flag();
            end
            tx_crc      = 16'hFFFF;
            tx_ones_cnt = 0;

            send_stuffed_byte(addr,  1'b1); // Module Addr (normally 0x0A)
            send_stuffed_byte(8'h01, 1'b1); // Module Msg ID 1
            send_u16_le(rate_x);
            send_u16_le(rate_y);
            send_u16_le(rate_z);
            send_u16_le(acc_x);
            send_u16_le(acc_y);
            send_u16_le(acc_z);
            send_u16_le(sw1);
            send_u16_le(sw2);
            send_fcs(invert_fcs, inject_crc_err);
            send_flag();                    // Closing flag
        end
    endtask

    //--------------------------------------------------------------------------
    // Task: Transmit SM-5800 Message ID 2 (Motion Control + Spatial — 44 bytes)
    //--------------------------------------------------------------------------
    task send_msg2;
        input [7:0]  addr;
        input [15:0] rate_x;
        input [15:0] rate_y;
        input [15:0] rate_z;
        input [15:0] acc_x;
        input [15:0] acc_y;
        input [15:0] acc_z;
        input [15:0] sw1;
        input [15:0] sw2;
        input [31:0] dang_x;
        input [31:0] dang_y;
        input [31:0] dang_z;
        input [31:0] dvel_x;
        input [31:0] dvel_y;
        input [31:0] dvel_z;
        input        invert_fcs;
        input        inject_crc_err;
        input        skip_open_flag;
        begin
            if (!skip_open_flag) begin
                send_flag();
            end
            tx_crc      = 16'hFFFF;
            tx_ones_cnt = 0;

            send_stuffed_byte(addr,  1'b1); // Module Addr (normally 0x0A)
            send_stuffed_byte(8'h02, 1'b1); // Module Msg ID 2
            send_u16_le(rate_x);
            send_u16_le(rate_y);
            send_u16_le(rate_z);
            send_u16_le(acc_x);
            send_u16_le(acc_y);
            send_u16_le(acc_z);
            send_u16_le(sw1);
            send_u16_le(sw2);
            send_u32_le(dang_x);
            send_u32_le(dang_y);
            send_u32_le(dang_z);
            send_u32_le(dvel_x);
            send_u32_le(dvel_y);
            send_u32_le(dvel_z);
            send_fcs(invert_fcs, inject_crc_err);
            send_flag();                    // Closing flag
        end
    endtask

    //--------------------------------------------------------------------------
    // Task: Transmit partial frame followed by SDLC Abort (>=7 consecutive '1's)
    //--------------------------------------------------------------------------
    task send_abort_frame;
        integer k;
        begin
            send_flag();
            tx_crc      = 16'hFFFF;
            tx_ones_cnt = 0;
            send_stuffed_byte(8'h0A, 1'b1);
            send_stuffed_byte(8'h01, 1'b1);
            send_stuffed_byte(8'h55, 1'b1);
            // Send 8 consecutive '1's (SDLC Abort sequence)
            for (k = 0; k < 8; k = k + 1) begin
                send_raw_bit(1'b1);
            end
        end
    endtask

    //--------------------------------------------------------------------------
    // Task: Hold SDLC line in idle mark ('1') for N bit periods
    //--------------------------------------------------------------------------
    task send_idle_ones;
        input integer num_bits;
        integer k;
        begin
            for (k = 0; k < num_bits; k = k + 1) begin
                send_raw_bit(1'b1);
            end
        end
    endtask

endmodule

`default_nettype wire
