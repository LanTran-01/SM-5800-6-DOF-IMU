# SM-5800 6-DOF Motion Sensor Module (IMU) — AXI4-Lite Verilog IP Core

IP Core viết bằng **Verilog-2001 (IEEE 1364-2001)**, tuân thủ chuẩn **AMBA AXI4-Lite (ARM IHI 0022E)**, thiết kế để giao tiếp và đọc toàn bộ dữ liệu từ cảm biến **SM-5800 6-DOF Motion Sensor Module** qua giao thức nối tiếp đồng bộ **SDLC RS-422 (1.0 MHz)**.

---

## 1. Kiến trúc Module

```text
                        +-----------------------------------------------------------+
                        |                    sm5800_imu_axi_top                     |
                        |                                                           |
  SM-5800 (RS-422/OC)   |  +------------------+   +----------------+   +---------+  |   AMBA AXI4-Lite
  <--- imu_rcv_clk_o ---+--| sm5800_clk_ctrl  |-->| sm5800_sdlc_rx |-->| sm5800_ |  |   (32-bit, Single Clk)
  ---> imu_ser_data_i --+--| - 1 MHz RCV_CLK  |   | - 0x7E Flag    |   | msg_    |  |   +------------------+
  <--- imu_bit_l_o -----+--| - 2-FF Sync      |   | - Zero Destuff |   | parser  |--+-->| sm5800_axi4_     |<==> s_axi_*
  <--- imu_reset_l_o ---+--| - >1ms RESET_L   |   | - Abort/Idle   |   | - Msg1/2|  |   | lite_if          |---> irq_o
                        |  | - 1 us Timestamp |   | - u_crc16      |   | - Watch |  |   | - CSR & W1C IRQ  |
                        |  +------------------+   +----------------+   |   dog   |  |   | - DATA_LOCK      |
                        |                                              +---------+  |   +------------------+
                        +-----------------------------------------------------------+
```

### Danh sách tệp nguồn

| Tệp | Chức năng |
|---|---|
| `rtl/sm5800_imu_axi_top.v` | Top-level IP wrapper (xuất thêm chân `msg1_valid_o`, `msg2_valid_o` và `irq_o`, tương thích Vivado IP Packager). |
| `rtl/sm5800_clk_ctrl.v` | Tạo xung `RCV_CLK` 1.0 MHz (±0% tại 100 MHz, duty 50%), đồng bộ hóa 2-FF (`ASYNC_REG`) cho `SER_DATA`, bộ đếm thời gian `1 µs`, điều khiển `RESET_L` (>1 ms theo Sec. 4.4) và `BIT_L` (Sec. 4.3.2). |
| `rtl/sm5800_crc16_ccitt.v` | Bộ tính CRC-16-CCITT bit-serial ($X^{16}+X^{12}+X^5+1$, đa thức phản xạ `0x8408`, preset `0xFFFF`, LSB-first). |
| `rtl/sm5800_sdlc_rx.v` | Bộ tách khung SDLC với pipeline lookahead 8-bit: tách Flag `0x7E` không làm bẩn CRC, hỗ trợ cờ liên tiếp & Shared Flag giữa 2 khung, bỏ bit chèn `0` (destuffing), phát hiện Abort/Idle (≥7 bit `1`), kiểm tra căn chỉnh octet và residue CRC (`0xF0B8` chuẩn Z8530 SCC hoặc `0x0000`). |
| `rtl/sm5800_msg_parser.v` | Giải mã gói SM-5800 (Addr `0x0A`, Msg ID 1 = 20B / 600 Hz, Msg ID 2 = 44B / 100 Hz), phân biệt rõ 2 chân đầu ra `msg1_valid_o` và `msg2_valid_o`, hỗ trợ `msg2_only_mode_i` (100 Hz) & `msg1_only_mode_i` (600 Hz); giám sát bộ đếm 4-bit, lỗi BIT và watchdog mất dữ liệu. |
| `rtl/sm5800_axi4_lite_if.v` | Giao diện Slave AMBA AXI4-Lite đầy đủ (nhận AW/W độc lập mọi thứ tự, hỗ trợ `WSTRB`, khóa snapshot `DATA_LOCK` chống xé dữ liệu, ngắt W1C 7 nguồn gồm `MSG1_READY` / `MSG2_READY` riêng biệt, điều khiển `RESET_L` / `BIT_L` từ PS). |
| `tb/sm5800_sensor_model.v` | Mô hình mô phỏng cảm biến SM-5800 (bit-stuffing, CRC-16-CCITT chuẩn Z8530, Shared Flag, tiêm lỗi). |
| `tb/tb_sm5800_imu_axi.v` | Testbench tự kiểm tra toàn diện 13 kịch bản (`[T1]`–`[T13]`, gồm kiểm tra tách chân `msg1_valid_o`/`msg2_valid_o`, chế độ `MSG1_ONLY_MODE` và chế độ 100 Hz `MSG2_ONLY_MODE`). |
| `driver/sm5800_regs.h` | Định nghĩa địa chỉ thanh ghi (gồm Motion Control + Spatial Data `0x40`–`0x54`), `_MASK` và `_SHIFT`. |
| `driver/sm5800_hal.h` / `.c` | Lớp HAL truy cập MMIO AXI4-Lite, barrier `DMB`/`DSB` và vùng găng cho ARM Cortex-A9 PS. |
| `driver/sm5800_drv.h` / `.c` | Driver C lớp PS tuân thủ MISRA C:2012 + BARR-C:2018 + DO-178A/B/C: ngắt 100 Hz bản tin `02`, hàm quy đổi 1 lệnh gọi `sm5800_read_msg2_eng()` / `sm5800_raw_to_eng_msg2()` ra SI dễ kiểm chứng, và điều khiển Reset/BIT từ PS → FPGA → IMU. |
| `driver/sm5800_ps_example.c` | Chương trình mẫu tích hợp GIC ISR 100 Hz, tự kiểm chứng quy đổi 1 g và phục hồi lỗi BIT/Reset từ PS. |
| `driver/DEVIATION_LOG.md` | Nhật ký ngoại lệ chuẩn *MISRA Compliance:2020* và cờ biên dịch DO-178C. |
| `docs/LLR_SM5800.md` | Đặc tả yêu cầu mức thấp (64 LLR cho RTL + HAL + Driver + App) truy vết 1-1 với `@req`. |
| `docs/SDD_SM5800.md` | Tài liệu thiết kế chi tiết phần cứng/phần mềm (sơ đồ Mermaid, WCSU, SEU, PDI CRC-32, ma trận truy vết). |
| `docs/TEST_REPORT_SM5800.md` | Báo cáo kiểm chứng & kiểm thử (SVR): kết quả static/bit-exact đã chạy, kế hoạch `[T1]`–`[T13]`, `TC-DRV-01`–`16`, HIL và bảng ký duyệt. |

---

## 2. Kết nối phần cứng với SM-5800 (Section 4.0 & 5.0)

| Cổng IP (FPGA) | Mạch giao tiếp ngoài bắt buộc | Chân đầu nối 30-pin SM-5800 | Ghi chú theo tài liệu SM-5800 |
|---|---|---|---|
| `imu_rcv_clk_o` | RS-422 Driver (`AM26C31` / tương đương) | Pin 28 (`RCV_CLK_H`), Pin 6 (`RCV_CLK_L`) | 1.0 MHz ± 2%, điện trở đầu cuối 182 Ω trong SM-5800. |
| `imu_ser_data_i` | RS-422 Receiver (`AM26C32` / tương đương) | Pin 4 (`SER_DATA_H`), Pin 24 (`SER_DATA_L`) | Gắn điện trở đầu cuối **180 Ω ± 5%** giữa `SER_DATA_H` và `SER_DATA_L` phía FPGA (Table 6). |
| `imu_bit_l_o` | Mạch Open-Collector / Open-Drain (kéo xuống `5V_RTN`) | Pin 12 (`BIT_L`) | Kéo lên 5 VDC nội bộ qua 20 kΩ. `0` = kích hoạt Commanded BIT, `1` = thả nổi (Hi-Z). |
| `imu_reset_l_o` | Mạch Open-Collector / Open-Drain (bắt buộc theo Sec. 4.4) | Pin 18 (`RESET_L`) | Kéo `0` > 1 ms để reset vi điều khiển SM-5800. Thả nổi (`1`) khi cấp nguồn để SM-5800 tự reset. |

> **Lưu ý phần cứng:** Không nối trực tiếp chân I/O 3.3 V của FPGA vào `BIT_L` hay `RESET_L` do các chân này có điện trở kéo lên **5 VDC**. Sử dụng MOSFET kênh N (chú ý đảo cực nếu cần) hoặc buffer open-drain chịu áp 5 V (`74LVC1G07`).

---

## 3. Bản đồ thanh ghi AXI4-Lite (Offset `0x00` – `0x6C`)

Mọi thanh ghi rộng 32-bit, căn chỉnh theo 4 byte (`C_S_AXI_ADDR_WIDTH >= 8`).

| Offset | Tên thanh ghi | Quyền | Giá trị Reset | Mô tả chi tiết |
|---|---|---|---|---|
| `0x00` | `CORE_ID` | RO | `0x5800_0100` | Mã định danh IP (`0x5800`) và phiên bản (`v1.0.0`). |
| `0x04` | `CTRL` | RW / W1P | `0x0000_0001` | `[0]` `SDLC_CLK_EN`: Bật xung `RCV_CLK` 1.0 MHz.<br>`[1]` `DATA_LOCK`: `1` = đóng băng snapshot toàn bộ thanh ghi dữ liệu & timestamp để đọc đồng nhất; `0` = chế độ trực tiếp.<br>`[2]` `CMD_BIT_EN`: `1` = kéo `BIT_L` xuống mức thấp (Commanded BIT từ PS → FPGA → IMU).<br>`[3]` `IMU_RESET_PULSE` (W1P): Ghi `1` để tạo xung `RESET_L` thấp tự động ≥ `C_RESET_PULSE_US` (mặc định 2.0 ms > 1.0 ms).<br>`[4]` `IMU_RESET_HOLD`: Giữ `RESET_L` thấp thủ công từ PS.<br>`[5]` `SAMPLE_EDGE`: `0` = lấy mẫu `SER_DATA` ở sườn lên `RCV_CLK` (mặc định SDLC); `1` = sườn xuống.<br>`[6]` `CRC_MODE`: `0` = chuẩn SDLC/Z8530 đảo FCS (residue `0xF0B8`); `1` = FCS không đảo (residue `0x0000`).<br>`[7]` `CRC_BYPASS`: Bỏ qua kiểm tra CRC (chỉ dùng gỡ lỗi).<br>`[8]` `CLEAR_STATS` (W1P): Xóa toàn bộ bộ đếm thống kê/lỗi.<br>`[9]` `SOFT_RESET` (W1P): Reset mềm khối SDLC RX, Parser và cờ ngắt.<br>`[10]` `MSG1_ONLY_MODE`: `1` = chỉ commit dữ liệu cho bản tin số 01.<br>`[11]` `MSG2_ONLY_MODE`: `1` = chế độ **100 Hz** chỉ commit toàn bộ 14 trường (Motion Control + Spatial) cho bản tin số 02 (Msg 01 không ghi đè dữ liệu Msg 02; watchdog giám sát Msg 02). |
| `0x08` | `STATUS` | RO | `0x0000_0000` | `[0]` `FLAGS_ALIVE`: Đang nhận cờ `0x7E`.<br>`[1]` `DATA_ACTIVE`: Đang nhận gói dữ liệu hợp lệ trong ngưỡng watchdog.<br>`[2]` `IMU_FAULT`: Có lỗi BIT trong `STATUS_WORD1[7:4]` hoặc `STATUS_WORD2[15:11]`.<br>`[3]` `MODULE_FAILED`: Cờ chốt `Status Word 1[4]`.<br>`[4]` `DATA_LOCKED`: Trạng thái khóa snapshot.<br>`[5]` `IMU_RESET_BUSY`: Xung reset tự động đang chạy.<br>`[6]` `CMD_BIT_ACTIVE`: Đang kích hoạt `BIT_L`.<br>`[7]` `SDLC_IN_FRAME`: Khối SDLC đã đồng bộ khung.<br>`[15:8]` `LAST_MSG_ID`: ID gói hợp lệ gần nhất (`0x01` / `0x02`).<br>`[19:16]` `LAST_SEQ_CNT`: Bộ đếm 4-bit gần nhất (`SW1[3:0]`).<br>`[23:20]` `LAST_ERR_CODE`: Mã lỗi gói bị loại gần nhất (`[20]`=CRC, `[21]`=Align, `[22]`=Addr, `[23]`=Len/ID).<br>`[24]` `MSG1_ONLY_ACTIVE`, `[25]` `MSG2_ONLY_ACTIVE`. |
| `0x0C` | `INT_STATUS` | RW1C | `0x0000_0000` | Ghi `1` để xóa: `[0]` `MSG1_READY` (chỉ Msg ID 01), `[1]` `MSG2_READY` (chỉ Msg ID 02 — 100 Hz), `[2]` `IMU_FAULT`, `[3]` `RX_ERROR`, `[4]` `SEQ_GAP`, `[5]` `LINK_TIMEOUT`, `[6]` `MC_ANY_READY`. |
| `0x10` | `INT_ENABLE` | RW | `0x0000_0000` | Mặt nạ cho phép ngắt `irq_o` ứng với `INT_STATUS[6:0]`. Driver PS bật `0x02` (`MSG2_READY`) để chỉ ngắt ở 100 Hz khi có bản tin 02. |
| `0x14` | `TIMEOUT_CFG` | RW | `5000` (`0x1388`) | Thời gian watchdog mất dữ liệu (µs). Mặc định 5.0 ms (~3 chu kỳ 600 Hz). Ghi `0` để tắt. |
| `0x18` | `MC_TIMESTAMP` | RO | `0` | Mốc thời gian 32-bit (1 µs/LSB) khi cập nhật Motion Control gần nhất. |
| `0x1C` | `SP_TIMESTAMP` | RO | `0` | Mốc thời gian 32-bit (1 µs/LSB) khi cập nhật Spatial Data gần nhất. |
| `0x20`–`0x28` | `ANG_RATE_X/Y/Z` | RO | `0` | Vận tốc góc X/Y/Z — 16-bit có dấu mở rộng thành 32-bit (`int32_t`). LSB = $2^{-20} \times 600$ rad/s. |
| `0x2C`–`0x34` | `LIN_ACC_X/Y/Z` | RO | `0` | Gia tốc tuyến tính X/Y/Z — 16-bit có dấu mở rộng thành 32-bit (`int32_t`). LSB = $2^{-14} \times 600$ ft/s². |
| `0x38` | `STATUS_WORD1` | RO | `0` | `[15:0]` Status Word 1 nguyên bản (`[3:0]` Counter, `[4]` Mod Fail, `[7:5]` Gyro C/B/A PLC Fail, `[15:8]` Accel Temp 1°C/LSB). |
| `0x3C` | `STATUS_WORD2` | RO | `0` | `[15:0]` Status Word 2 nguyên bản (`[7:0]` SW Ver, `[11]` Gyro, `[12]` Accel, `[13]` Other, `[14]` Mem, `[15]` Proc Fail). |
| `0x40`–`0x48` | `DELTA_ANG_X/Y/Z` | RO | `0` | Gia số góc X/Y/Z (100 Hz, Msg 2) — 32-bit có dấu (`int32_t`). LSB = $2^{-33}$ rad. |
| `0x4C`–`0x54` | `DELTA_VEL_X/Y/Z` | RO | `0` | Gia số vận tốc X/Y/Z (100 Hz, Msg 2) — 32-bit có dấu (`int32_t`). LSB = $2^{-27}$ ft/s. |
| `0x58` | `MSG1_RX_CNT` | RO | `0` | Tổng số khung Message ID 1 hợp lệ đã nhận. |
| `0x5C` | `MSG2_RX_CNT` | RO | `0` | Tổng số khung Message ID 2 hợp lệ đã nhận. |
| `0x60` | `CRC_ERR_CNT` | RO | `0` | Tổng số khung bị loại do sai CRC-16-CCITT. |
| `0x64` | `FRAME_ERR_CNT` | RO | `0` | `[15:0]` Số lỗi khung (Align/Addr/Len/ID); `[31:16]` Số lần SDLC Abort. |
| `0x68` | `SEQ_ERR_CNT` | RO | `0` | Số lần nhảy cóc bộ đếm 4-bit (`Status Word 1[3:0]`). |
| `0x6C` | `DEBUG_CRC` | RO | `0` | `[31:24]` Addr khung cuối (`0x0A`), `[21:16]` Độ dài byte (`20`/`44`), `[15:0]` CRC residue (`0xF0B8`). |

---

## 4. Driver C lớp PS (MISRA C:2012 + BARR-C:2018 + DO-178A/B/C)

Thư mục `driver/` cung cấp trọn bộ driver C cho vi xử lý PS (ARM Cortex-A / Zynq bare-metal) hoạt động ở chế độ **100 Hz (Bản tin số 02: Motion Control + Spatial Data)** theo ngắt `irq_o`:

1. **Khởi tạo (`sm5800_init`):** Kiểm tra PDI (`schema_version`, phạm vi tham số, `CRC-32`), xác thực `CORE_ID == 0x58000100`, bật `CTRL.MSG2_ONLY_MODE = 1` và chỉ bật ngắt `INT_ENABLE = MSG2_READY (0x02)`.
2. **Ngắt 100 Hz từ PL (`sm5800_irq_isr`):** Đọc & xóa W1C `INT_STATUS`, đồng bộ `DSB`, dựng cờ `is_msg2_pending` (O(1), không vòng lặp, không dấu phẩy động).
3. **Đọc & Quy đổi 1 lệnh gọi sang đơn vị kiểm chứng được:**
   - `sm5800_read_msg2_eng(p_drv, p_eng)`: Tự khóa `CTRL.SNAP_LOCK`, đọc toàn bộ 15 thanh ghi raw của bản tin 02, nhả khóa và trả về trực tiếp giá trị vật lý SI (`rad/s`, `m/s²`, `rad`, `m/s`, `°C`, cờ lỗi BIT, counter, timestamp `µs`).
   - `sm5800_raw_to_eng_msg2(p_raw, p_eng)`: Quy đổi trực tiếp từ cấu trúc raw (`sm5800_msg2_raw_t`) sang đơn vị vật lý (`sm5800_msg2_eng_t`) để người dùng dễ kiểm chứng (ví dụ `accel_z_lsb = 879` → `~9.81 m/s² ≈ 1 g`).
4. **Điều khiển Reset & BIT từ PS → FPGA → IMU:** `sm5800_trigger_reset()` (> 1 ms tự động), `sm5800_hold_reset()` / `sm5800_release_reset()`, `sm5800_start_cmd_bit()` / `sm5800_stop_cmd_bit()`, `sm5800_is_reset_busy()`, `sm5800_is_cmd_bit_active()`.

Xem ví dụ tích hợp hoàn chỉnh tại `driver/sm5800_ps_example.c` và nhật ký ngoại lệ + cờ biên dịch tại `driver/DEVIATION_LOG.md`.

---

## 5. Hướng dẫn chạy mô phỏng (Simulation)

### Icarus Verilog (`iverilog` + `vvp`)
```bash
iverilog -g2001 -Wall -o sim_sm5800.vvp \
    rtl/sm5800_crc16_ccitt.v \
    rtl/sm5800_clk_ctrl.v \
    rtl/sm5800_sdlc_rx.v \
    rtl/sm5800_msg_parser.v \
    rtl/sm5800_axi4_lite_if.v \
    rtl/sm5800_imu_axi_top.v \
    tb/sm5800_sensor_model.v \
    tb/tb_sm5800_imu_axi.v

vvp sim_sm5800.vvp
```

### Xilinx Vivado (`xvlog` / `xelab` / `xsim`)
```bash
xvlog rtl/sm5800_crc16_ccitt.v rtl/sm5800_clk_ctrl.v rtl/sm5800_sdlc_rx.v \
      rtl/sm5800_msg_parser.v rtl/sm5800_axi4_lite_if.v rtl/sm5800_imu_axi_top.v \
      tb/sm5800_sensor_model.v tb/tb_sm5800_imu_axi.v
xelab -debug typical tb_sm5800_imu_axi -s sim_sm5800
xsim sim_sm5800 -runall
```

### Verilator Lint Check
```bash
verilator --lint-only -Wall --top-module sm5800_imu_axi_top \
    rtl/sm5800_crc16_ccitt.v rtl/sm5800_clk_ctrl.v rtl/sm5800_sdlc_rx.v \
    rtl/sm5800_msg_parser.v rtl/sm5800_axi4_lite_if.v rtl/sm5800_imu_axi_top.v
```
