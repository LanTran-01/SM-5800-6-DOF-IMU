//==============================================================================
// Module      : sm5800_sdlc_rx
// Project     : SM-5800 6-DOF Motion Sensor Module (IMU) AXI4-Lite Interface
// Standard    : Verilog-2001 (IEEE 1364-2001)
// Description : Bit-oriented SDLC receiver for SM-5800 point-to-point interface
//               (Sec. 5.0, Table 3):
//               - Flag detection (01111110 / 0x7E) using an 8-bit lookahead
//                 shift pipeline so closing-flag bits NEVER pollute data or CRC.
//               - Seamless support for continuous idle flags (0x7E 0x7E ...),
//                 7-bit shared-zero flags, and shared closing/opening flags.
//               - Zero-insertion removal (bit destuffing after five '1' bits).
//               - SDLC Abort / Idle detection (>=7 consecutive '1' bits).
//               - LSB-first octet assembly and octet-alignment verification.
//               - Integrated CRC-16-CCITT residue verification (0xF0B8 std
//                 SDLC/Z8530 inverted FCS, or 0x0000 non-inverted mode).
//               - Guarantees frame_end_o pulses strictly 1 clock cycle AFTER
//                 the final rx_byte_valid_o pulse of the frame.
//==============================================================================

`timescale 1ns / 1ps
`default_nettype none

module sm5800_sdlc_rx (
    input  wire        clk_i,               // System clock (s_axi_aclk)
    input  wire        rst_ni,              // Synchronous active-low reset
    input  wire        soft_rst_i,          // Synchronous soft reset pulse

    // Configuration
    input  wire        crc_mode_i,          // 0: SDLC std inverted FCS (0xF0B8), 1: raw (0x0000)

    // Serial NRZ bit stream from sm5800_clk_ctrl
    input  wire        rx_bit_valid_i,      // 1-cycle strobe per received bit
    input  wire        rx_bit_i,            // Received NRZ bit (LSB-first)

    // Deframed byte stream & frame events to sm5800_msg_parser
    output wire        rx_byte_valid_o,     // 1-cycle strobe per destuffed byte (incl. 2B CRC)
    output wire [7:0]  rx_byte_o,           // Destuffed byte (LSB-first assembled)
    output wire        frame_end_o,         // 1-cycle strobe after final byte of closed frame
    output wire        frame_abort_o,       // 1-cycle strobe if active frame aborted
    output wire        frame_crc_ok_o,      // CRC valid (latched, valid on frame_end_o)
    output wire        frame_align_err_o,   // Non-octet-aligned bit count (valid on frame_end_o)
    output wire [15:0] frame_crc_residue_o, // Final CRC residue (valid on frame_end_o)
    output wire        flag_detected_o,     // 1-cycle strobe on every 0x7E flag seen
    output wire        in_frame_o           // 1: synchronized to SDLC flag stream
);

    //--------------------------------------------------------------------------
    // SDLC Protocol Constants
    //--------------------------------------------------------------------------
    localparam [7:0]  P_SDLC_FLAG         = 8'h7E;   // 0111_1110
    localparam [6:0]  P_SDLC_ABORT_7ONES  = 7'b111_1111;
    localparam [15:0] P_CRC_RESIDUE_SDLC  = 16'hF0B8; // ISO 3309 / Z8530 inverted FCS
    localparam [15:0] P_CRC_RESIDUE_RAW   = 16'h0000; // Non-inverted FCS

    //--------------------------------------------------------------------------
    // Internal Registers
    //--------------------------------------------------------------------------
    reg [7:0]  raw_sr_r;           // 8-bit raw bit pipeline (MSB=newest, LSB=oldest)
    reg        in_frame_r;         // 0=HUNT for flag, 1=synchronized / in frame
    reg [3:0]  skip_cnt_r;         // Flushes 8 flag bits out of raw_sr_r after flag detect
    reg [2:0]  ones_cnt_r;         // Consecutive '1' counter for zero destuffing (0..5)
    reg [2:0]  bit_cnt_r;          // Destuffed bit index within current octet (0..7)
    reg [6:0]  byte_sr_r;          // LSB-first byte shift register (7 oldest bits;
                                  // 8th bit comes straight from out_bit_w)
    reg        has_bits_r;         // 1 once >=1 destuffed data bit received in current frame

    reg        rx_byte_valid_r;
    reg [7:0]  rx_byte_r;
    reg        frame_end_d1_r;     // 1-cycle pipeline delay after final byte strobe
    reg        frame_end_r;
    reg        frame_abort_r;
    reg        frame_crc_ok_r;
    reg        frame_align_err_r;
    reg [15:0] frame_crc_residue_r;
    reg        flag_detected_r;

    //--------------------------------------------------------------------------
    // Combinational Lookahead & Destuffing Classification
    //--------------------------------------------------------------------------
    wire        out_bit_w;
    wire [7:0]  next_raw_w;
    wire        is_flag_w;
    wire        is_abort_w;
    wire        process_out_bit_w;
    wire        is_stuffed_zero_w;
    wire        is_stuff_err_w;
    wire        is_data_bit_w;

    wire [15:0] crc_curr_w;
    wire [15:0] crc_next_w;
    wire        crc_init_w;
    wire        crc_bit_valid_w;

    wire [15:0] expected_residue_w;
    wire [15:0] final_crc_w;
    wire [2:0]  final_bit_cnt_w;
    wire        final_has_bits_w;
    wire [7:0]  next_byte_w;

    assign out_bit_w  = raw_sr_r[0];
    assign next_raw_w = {rx_bit_i, raw_sr_r[7:1]};
    assign is_flag_w  = (next_raw_w == P_SDLC_FLAG);
    assign is_abort_w = (next_raw_w[7:1] == P_SDLC_ABORT_7ONES);

    assign process_out_bit_w = in_frame_r && !is_abort_w && (skip_cnt_r == 4'd0);
    assign is_stuffed_zero_w = process_out_bit_w && (ones_cnt_r == 3'd5) && (out_bit_w == 1'b0);
    assign is_stuff_err_w    = process_out_bit_w && (ones_cnt_r == 3'd5) && (out_bit_w == 1'b1);
    assign is_data_bit_w     = process_out_bit_w && (ones_cnt_r != 3'd5);

    assign expected_residue_w = crc_mode_i ? P_CRC_RESIDUE_RAW : P_CRC_RESIDUE_SDLC;
    assign final_crc_w        = is_data_bit_w ? crc_next_w : crc_curr_w;
    assign final_bit_cnt_w    = is_data_bit_w ? (bit_cnt_r + 3'd1) : bit_cnt_r;
    assign final_has_bits_w   = has_bits_r || is_data_bit_w;
    assign next_byte_w        = {out_bit_w, byte_sr_r};

    // CRC control: reset to 0xFFFF on soft reset, flag, abort, stuff error, or hunt state
    assign crc_init_w      = soft_rst_i ||
                             (rx_bit_valid_i && (is_flag_w || is_abort_w || is_stuff_err_w || !in_frame_r));
    assign crc_bit_valid_w = rx_bit_valid_i && is_data_bit_w && !is_flag_w;

    //--------------------------------------------------------------------------
    // CRC-16-CCITT Submodule Instantiation
    //--------------------------------------------------------------------------
    sm5800_crc16_ccitt u_crc16 (
        .clk_i       (clk_i),
        .rst_ni      (rst_ni),
        .crc_init_i  (crc_init_w),
        .bit_valid_i (crc_bit_valid_w),
        .data_bit_i  (out_bit_w),
        .crc_o       (crc_curr_w),
        .crc_next_o  (crc_next_w)
    );

    //--------------------------------------------------------------------------
    // SDLC Deframer Sequential Logic
    //--------------------------------------------------------------------------
    always @(posedge clk_i) begin
        if (!rst_ni) begin
            raw_sr_r            <= 8'hFF;
            in_frame_r          <= 1'b0;
            skip_cnt_r          <= 4'd0;
            ones_cnt_r          <= 3'd0;
            bit_cnt_r           <= 3'd0;
            byte_sr_r           <= 7'h00;
            has_bits_r          <= 1'b0;
            rx_byte_valid_r     <= 1'b0;
            rx_byte_r           <= 8'h00;
            frame_end_d1_r      <= 1'b0;
            frame_end_r         <= 1'b0;
            frame_abort_r       <= 1'b0;
            frame_crc_ok_r      <= 1'b0;
            frame_align_err_r   <= 1'b0;
            frame_crc_residue_r <= 16'h0000;
            flag_detected_r     <= 1'b0;
        end else if (soft_rst_i) begin
            raw_sr_r            <= 8'hFF;
            in_frame_r          <= 1'b0;
            skip_cnt_r          <= 4'd0;
            ones_cnt_r          <= 3'd0;
            bit_cnt_r           <= 3'd0;
            byte_sr_r           <= 7'h00;
            has_bits_r          <= 1'b0;
            rx_byte_valid_r     <= 1'b0;
            frame_end_d1_r      <= 1'b0;
            frame_end_r         <= 1'b0;
            frame_abort_r       <= 1'b0;
            flag_detected_r     <= 1'b0;
        end else begin
            // Default 1-cycle output pulses
            rx_byte_valid_r <= 1'b0;
            frame_end_d1_r  <= 1'b0;
            frame_end_r     <= frame_end_d1_r;
            frame_abort_r   <= 1'b0;
            flag_detected_r <= 1'b0;

            if (rx_bit_valid_i) begin
                // Advance 8-bit raw bit pipeline
                raw_sr_r <= next_raw_w;

                if (!in_frame_r) begin
                    //----------------------------------------------------------
                    // HUNT State: wait for Opening Flag (0x7E)
                    //----------------------------------------------------------
                    if (is_flag_w) begin
                        in_frame_r      <= 1'b1;
                        skip_cnt_r      <= 4'd8; // Flush the 8 opening-flag bits
                        ones_cnt_r      <= 3'd0;
                        bit_cnt_r       <= 3'd0;
                        byte_sr_r       <= 7'h00;
                        has_bits_r      <= 1'b0;
                        flag_detected_r <= 1'b1;
                    end
                end else begin
                    //----------------------------------------------------------
                    // IN_FRAME State: process exiting bit and watch for Flag/Abort
                    //----------------------------------------------------------
                    if (is_abort_w) begin
                        // >=7 consecutive ones: abort active frame or enter idle hunt
                        if (has_bits_r) begin
                            frame_abort_r <= 1'b1;
                        end
                        in_frame_r <= 1'b0;
                        skip_cnt_r <= 4'd0;
                        ones_cnt_r <= 3'd0;
                        bit_cnt_r  <= 3'd0;
                        byte_sr_r  <= 7'h00;
                        has_bits_r <= 1'b0;
                    end else if (is_stuff_err_w) begin
                        // Defensive guard: six '1's without valid flag/abort lookahead
                        frame_abort_r <= 1'b1;
                        in_frame_r    <= 1'b0;
                        skip_cnt_r    <= 4'd0;
                        ones_cnt_r    <= 3'd0;
                        bit_cnt_r     <= 3'd0;
                        byte_sr_r     <= 7'h00;
                        has_bits_r    <= 1'b0;
                    end else begin
                        // Step 1: Emit completed byte if out_bit_w completes an octet
                        if (is_data_bit_w && (bit_cnt_r == 3'd7)) begin
                            rx_byte_valid_r <= 1'b1;
                            rx_byte_r       <= next_byte_w;
                        end

                        // Step 2: Check for Closing / Consecutive / Shared Flag
                        if (is_flag_w) begin
                            flag_detected_r <= 1'b1;

                            if (final_has_bits_w) begin
                                // Close frame (frame_end_o pulses 1 cycle after final byte)
                                frame_end_d1_r      <= 1'b1;
                                frame_crc_ok_r      <= (final_crc_w == expected_residue_w);
                                frame_align_err_r   <= (final_bit_cnt_w != 3'd0);
                                frame_crc_residue_r <= final_crc_w;
                            end

                            // Prepare immediately for next frame (supports shared flags & idle flags)
                            skip_cnt_r <= 4'd8;
                            ones_cnt_r <= 3'd0;
                            bit_cnt_r  <= 3'd0;
                            byte_sr_r  <= 7'h00;
                            has_bits_r <= 1'b0;
                        end else begin
                            // Normal bit progression within frame
                            if (skip_cnt_r != 4'd0) begin
                                skip_cnt_r <= skip_cnt_r - 4'd1;
                            end else if (is_stuffed_zero_w) begin
                                // Discard stuffed '0' bit and clear consecutive-ones count
                                ones_cnt_r <= 3'd0;
                            end else begin
                                // Valid destuffed data bit
                                ones_cnt_r <= out_bit_w ? (ones_cnt_r + 3'd1) : 3'd0;
                                byte_sr_r  <= next_byte_w[7:1];
                                bit_cnt_r  <= bit_cnt_r + 3'd1; // Wraps 7 -> 0 naturally
                                has_bits_r <= 1'b1;
                            end
                        end
                    end
                end
            end
        end
    end

    //--------------------------------------------------------------------------
    // Output Assignments
    //--------------------------------------------------------------------------
    assign rx_byte_valid_o     = rx_byte_valid_r;
    assign rx_byte_o           = rx_byte_r;
    assign frame_end_o         = frame_end_r;
    assign frame_abort_o       = frame_abort_r;
    assign frame_crc_ok_o      = frame_crc_ok_r;
    assign frame_align_err_o   = frame_align_err_r;
    assign frame_crc_residue_o = frame_crc_residue_r;
    assign flag_detected_o     = flag_detected_r;
    assign in_frame_o          = in_frame_r;

endmodule

`default_nettype wire
