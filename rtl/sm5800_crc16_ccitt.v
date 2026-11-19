//==============================================================================
// Module      : sm5800_crc16_ccitt
// Project     : SM-5800 6-DOF Motion Sensor Module (IMU) AXI4-Lite Interface
// Standard    : Verilog-2001 (IEEE 1364-2001)
// Description : Bit-serial CRC-16-CCITT calculator for SDLC/HDLC frames.
//               - Polynomial  : X^16 + X^12 + X^5 + 1
//                               (0x1021 normal, 0x8408 bit-reflected LSB-first)
//               - Preset      : 16'hFFFF (all ones, per SM-5800 Sec. 5.0)
//               - Bit Order   : LSB-first (matches SM-5800 wire ordering)
//               - Residues    : 16'hF0B8 for standard SDLC/Z8530 inverted FCS
//                               16'h0000 for non-inverted FCS
//==============================================================================

`timescale 1ns / 1ps
`default_nettype none

module sm5800_crc16_ccitt (
    input  wire        clk_i,       // System clock (s_axi_aclk)
    input  wire        rst_ni,      // Synchronous active-low reset
    input  wire        crc_init_i,  // Synchronous preset to 16'hFFFF
    input  wire        bit_valid_i, // Shift in data_bit_i when high (if !crc_init_i)
    input  wire        data_bit_i,  // Incoming destuffed bit (LSB-first)
    output wire [15:0] crc_o,       // Current registered CRC value
    output wire [15:0] crc_next_o   // Combinational next CRC value after data_bit_i
);

    // Bit-reflected CCITT polynomial: X^16 + X^12 + X^5 + 1 -> 16'h8408
    localparam [15:0] P_POLY_REFLECTED = 16'h8408;
    localparam [15:0] P_CRC_PRESET     = 16'hFFFF;

    reg  [15:0] crc_r;
    wire        feedback_w;
    wire [15:0] crc_shifted_w;

    // Bit-serial LSB-first reflected LFSR step
    assign feedback_w    = crc_r[0] ^ data_bit_i;
    assign crc_shifted_w = {1'b0, crc_r[15:1]};
    assign crc_next_o    = feedback_w ? (crc_shifted_w ^ P_POLY_REFLECTED) : crc_shifted_w;
    assign crc_o         = crc_r;

    always @(posedge clk_i) begin
        if (!rst_ni) begin
            crc_r <= P_CRC_PRESET;
        end else if (crc_init_i) begin
            crc_r <= P_CRC_PRESET;
        end else if (bit_valid_i) begin
            crc_r <= crc_next_o;
        end
    end

endmodule

`default_nettype wire
