# Báo cáo rà soát & sửa lỗi — SM-5800 IMU (`Nguyenhai642101/IMU`) — Đợt 1

> **Bản lưu lịch sử.** Patch của đợt này đã được gộp vào `main` qua PR #1 ("Apply patch 1", `ad36059`). Danh sách "Việc còn lại" ở Phần 5 đã được cập nhật trong [REVIEW_REPORT_2.md](REVIEW_REPORT_2.md) (Phần 6).

| Mục | Giá trị |
|---|---|
| Ngày | 2026-11-20 |
| Bản gốc | commit `955a222` ("first commit") |
| Kết quả bàn giao | `IMU-review-fixes.patch` — 6 commit, áp bằng `git am` |
| Phạm vi | RTL Verilog, testbench, driver C, unit test, Makefile, script kiểm tra, tài liệu |

---

## 1. Tóm tắt

Tài liệu của dự án ghi phần lớn hạng mục là **PASS**, nhưng các test thực tế (mô phỏng RTL, unit test C) **chưa từng được chạy**.
Khi chạy thật, cả hai bộ test đều **lỗi**. Mình đã tìm ra **5 lỗi thật** (2 trong RTL, 1 trong driver, 1 trong test, 1 về cấu hình mặc định) cộng 7 cảnh báo lint, đã sửa toàn bộ, và thêm test để các lỗi này không quay lại.

> 🏠 Ẩn dụ: bản vẽ nhà rất chi tiết và có "giấy chứng nhận", nhưng chưa ai xây thử. Lần rà soát này là **xây thử** — tìm ra chỗ dột, sửa, rồi lắp thêm cảm biến báo dột cho lần sau.

**Đánh giá:** sau khi áp patch, dự án **sẵn sàng cho thử nghiệm phần cứng HIL**. Trước khi bay/triển khai thật vẫn cần làm các mục ở **Phần 5**.

---

## 2. Kết quả trước / sau

Tất cả đều được **chạy thật** trên: GCC 13.3, Icarus Verilog 12, Verilator 5.020, `arm-none-eabi-gcc` 13.2.1, PowerShell 7.4.6.
Cột "Sau" được chạy trên **bản clone mới từ GitHub + `git am` patch** (đúng như bạn sẽ làm).

| Lệnh | Bản gốc | Sau khi áp patch |
|---|---|---|
| `make verify-static` (script PowerShell, Gate 1) | ⚠️ Không chạy được ngoài Windows; sửa đường dẫn thì **PASS** | ✅ PASS (0 violations), chạy được cả Windows/Linux/macOS |
| `make lint-rtl` (Verilator `-Wall`) | ❌ 7 cảnh báo, exit ≠ 0 | ✅ 0 cảnh báo (kiểm tra ở 50/100/125/150/200 MHz) |
| `make sim` (testbench RTL) | ❌ **4 lỗi** / 167 assertion | ✅ **218/218** (thêm kịch bản T14, T15) |
| `make check-guards` (mới) | — không có | ✅ 9/9 (4 cấu hình hợp lệ được chấp nhận, 5 cấu hình sai bị chặn) |
| `make test-c` (unit test driver) | ❌ **1 lỗi** / 56 | ✅ **67/67** (thêm TC-DRV-17) |
| `make build-arm` (Cortex-A9, Gate 4) | ✅ 0 warning | ✅ 0 warning; stack lớn nhất 128 B ≤ 256 B; ISR: 0 lệnh VFP/NEON, 0 vòng lặp, `dsb` sau W1C, `dmb` trước khi thoát |

> ⚠️ Lưu ý quan trọng: **bản gốc cũng PASS Gate 1** dù có 5 lỗi thật. Script PowerShell chỉ kiểm tra hình thức (so khớp mẫu), không thay thế được mô phỏng, unit test hay công cụ MISRA chuyên dụng.

---

## 3. Các lỗi tìm thấy & cách sửa

### Lỗi 1 — RTL: "khung ma" làm báo lỗi giả (commit `6b9d545`)
- **Hiện tượng:** 4 assertion lỗi trong T3/T4/T7: cờ `RX_ERROR` bật sai, `CRC_ERR_CNT` = 6 thay vì 1.
- **Nguyên nhân:** bất kỳ khoảng trống ngắn nào giữa hai cờ `0x7E` (nhiễu, đường dây kẹt mức 0, hoặc < 7 bit mức 1) đều bị coi là **một khung lỗi**. Trên phần cứng thật, nhiễu giữa hai khung sẽ liên tục làm bật ngắt lỗi và tăng bộ đếm.
- **Sửa:** `rtl/sm5800_msg_parser.v` bỏ qua im lặng các khung < 32 bit (đúng chuẩn HDLC ISO 3309). Mô hình cảm biến trong testbench trả đường dây về mức 1 khi rảnh, giống chip Z8530 thật.
- **Test mới [T14]:** đường dây kẹt 0 giữa hai khung, khung cụt 3 byte, đúng ngưỡng 4 byte (vẫn phải báo lỗi), khung hợp lệ sau đó vẫn nhận bình thường. Chạy T14 với RTL cũ → 12 lỗi.

### Lỗi 2 — Test: vector delta-velocity sai 10 lần (commit `2190784`)
- **Hiện tượng:** `TC-DRV-08` lỗi.
- **Nguyên nhân:** test mong đợi 4.322.274 LSB → 0,0981 m/s, nhưng thực tế ra **0,0098 m/s**. Giá trị hex nạp vào (`0x0041F372` = 4.322.162) cũng không khớp nhãn. **Code driver đúng**; dữ liệu test sai. Bảng "PASS" trong `TEST_REPORT_SM5800.md` cũng ghi sai phép tính này.
- **Sửa:** 1 g × 10 ms = 0,0980665 m/s = **43.183.277 LSB (`0x0292ECAD`)**; siết sai số cho phép xuống 1e-5 m/s. Chương trình mẫu kiểm thêm delta-velocity. Sửa dòng "PASS" sai trong tài liệu.

### Lỗi 3 — RTL: watchdog không báo nếu cảm biến chết từ lúc bật nguồn (commit `0149d7d`)
- **Nguyên nhân:** watchdog chỉ bắt đầu đếm **sau** khung hợp lệ đầu tiên.
- **Sửa:** watchdog được kích hoạt ngay từ reset/soft-reset và sau mỗi khung hợp lệ; báo **một lần** rồi chờ (không gây bão ngắt).
- **Test mới [T15a–c]** — chạy trên RTL cũ: lỗi.

### Lỗi 4 — RTL: timeout mặc định 5 ms xung đột với chế độ 100 Hz (commit `0149d7d`)
- **Nguyên nhân:** ở chế độ `MSG2_ONLY`, bản tin đến mỗi 10 ms nhưng watchdog báo sau 5 ms → **mỗi khung một báo động giả** với ai dùng IP không qua driver.
- **Sửa:** mặc định 30 ms (khớp driver). Thêm **chốt lúc biên dịch**: `MSG2_ONLY` + timeout < 12 ms → Vivado/Verilator dừng với lỗi `SM5800_PARAM_ERROR_TIMEOUT_US_BELOW_12000_IN_MSG2_ONLY_MODE`.
- **Test mới [T15d]** (bản IP thứ hai dựng với tham số mặc định).

### Lỗi 5 — Driver: CPU không được báo khi cảm biến im lặng (commit `90c2ea3`)
- **Nguyên nhân:** `sm5800_init` chỉ bật ngắt `MSG2_READY`. Không có Msg 02 → không có ngắt → `LINK_TIMEOUT` của FPGA không bao giờ tới CPU.
- **Sửa:** `INT_ENABLE = MSG2_READY | IMU_FAULT | LINK_TIMEOUT (0x26)`. API mới `sm5800_take_irq_faults()` (đọc-và-xóa cờ lỗi trong vùng găng). Chương trình mẫu: `handle_link_faults()` phát xung `RESET_L` tối đa 3 lần liên tiếp rồi báo `SM5800_ERR_TIMEOUT`.
- **Test mới TC-DRV-17**; tài liệu truy vết: `LLR-SM5800-DRV-028`, `LLR-SM5800-APP-008`, SDD 3.3.19 / 4.7.

### Dọn dẹp & công cụ (commit `3c53fb1`, `c4db028`)
- 7 cảnh báo Verilator: bỏ flip-flop thừa (`byte_sr_r[0]`, `aw_addr_r[1:0]`), sửa độ rộng hằng số, ghi chú chân bỏ trống có chủ ý.
- Comment trong `clk_ctrl.v` ghi "đã kiểm tra ±2%" nhưng **không có kiểm tra nào** → thêm chốt thật: clock ±2%, bộ chia 16-bit, `RESET_L` > 1 ms, độ rộng bus AXI. Target mới `make check-guards` kiểm tra tự động.
- `verify_project.ps1` dùng đường dẫn `\` chỉ chạy trên Windows → đổi sang `/` (Windows vẫn chạy). Makefile tự chọn `pwsh`/`powershell`.

**Nguyên tắc đã áp dụng cho mọi lỗi:** viết test **trước**, chứng minh test lỗi trên code cũ, sửa, rồi chứng minh test đạt — và thử "đảo ngược sửa đổi" để chắc test thực sự bắt được lỗi.

---

## 4. Cách áp patch

```bash
git clone https://github.com/Nguyenhai642101/IMU.git
```
```bash
cd IMU
```
```bash
git checkout -b fix/review-steps
```
```bash
git am /đường/dẫn/IMU-review-fixes.patch
```
Kiểm tra lại (cần `iverilog`, `verilator`, `gcc`; tùy chọn `arm-none-eabi-gcc`, `pwsh`):
```bash
make lint-rtl check-guards sim test-c
```
```bash
make verify-static build-arm
```
Sau khi ổn, đẩy nhánh lên GitHub và mở Pull Request:
```bash
git push -u origin fix/review-steps
```

`git am` giữ nguyên 6 commit riêng lẻ (mỗi commit một vấn đề, có mô tả chi tiết) để dễ review từng phần.

---

## 5. Việc còn lại / khuyến nghị (chưa làm trong patch này)

| # | Mức độ | Nội dung | Gợi ý |
|---|---|---|---|
| 1 | **Cao** | **Thử nghiệm HIL** với cảm biến SM-5800 thật (mục 9 TEST_REPORT) — chưa có phần cứng. | Chạy 4 kịch bản HIL-01..04 đã mô tả trong tài liệu. |
| 2 | **Cao** | Repo **không có datasheet SM-5800** → hệ số scale (2⁻²⁰×600, 2⁻¹⁴×600 ft/s², …), bit Status Word, cạnh lấy mẫu chưa được đối chiếu độc lập. | Đối chiếu từng hằng số với datasheet. |
| 3 | **Cao** | Thời gian **power-up BIT** sau `RESET_L`: nếu > 30 ms, watchdog sẽ báo timeout trong lúc cảm biến đang khởi động bình thường → chương trình mẫu có thể reset liên tục 3 lần. | Chỉnh `TIMEOUT_CFG` hoặc thêm thời gian chờ sau reset theo datasheet. |
| 4 | Trung bình | Delta-angle/velocity được đổi sang `float` (24-bit mantissa) → **tích lũy sai số** nếu dùng để tích phân dẫn đường. | Tích phân bằng `int64_t` hoặc `double`. |
| 5 | Trung bình | `RESET_L`/`BIT_L` là ngõ ra push-pull, chưa phải open-collector thật. | Dùng IC OC ngoài (7407…) hoặc `IOBUF` 3 trạng thái. |
| 6 | Trung bình | Vùng găng chỉ tắt IRQ trên **1 lõi**; `read_locked_msg2_raw` ghi `CTRL` ngoài vùng găng. An toàn cho bare-metal 1 vòng lặp, **không** an toàn nếu chuyển sang RTOS đa task / SMP 2 lõi. | Thêm mutex/spinlock khi chuyển kiến trúc. |
| 7 | Thấp | Bộ kiểm tra số thứ tự 4-bit không đồng bộ lại sau Abort/timeout → có thể báo 1 lỗi `SEQ_GAP` giả sau khi mất kết nối lâu. | Xóa `has_prev_seq_r` khi `LINK_TIMEOUT`. |
| 8 | Thấp | Tài liệu nhắc "MC/DC coverage" nhưng **chưa đo** độ phủ. | `gcc --coverage` + `gcovr` cho line/branch; công cụ MC/DC chuyên dụng nếu cần DO-178C. |
| 9 | Thấp | Repo chưa có CI. | GitHub Actions chạy `make lint-rtl check-guards sim test-c` mỗi lần push. |

---

## 6. Phụ lục — danh sách commit

| Commit | Nội dung | File chính |
|---|---|---|
| `6b9d545` | fix(rtl): bỏ qua khung cụt < 32 bit (ISO 3309) + test T14 | `rtl/sm5800_msg_parser.v`, `tb/*` |
| `2190784` | fix(test): sửa vector delta-velocity 1 g (sai 10 lần) | `tests/test_sm5800_drv.c`, `driver/sm5800_ps_example.c`, `docs/TEST_REPORT_SM5800.md` |
| `0149d7d` | fix(rtl): watchdog phát hiện im lặng từ đầu; mặc định 30 ms + chốt tham số; test T15 | `rtl/sm5800_msg_parser.v`, `rtl/sm5800_imu_axi_top.v`, `rtl/sm5800_axi4_lite_if.v`, `tb/*` |
| `90c2ea3` | feat(driver): báo `LINK_TIMEOUT`/`IMU_FAULT` cho ứng dụng; TC-DRV-17 | `driver/sm5800_drv.[ch]`, `driver/sm5800_ps_example.c`, `tests/*`, `docs/LLR`, `docs/SDD` |
| `3c53fb1` | chore(rtl): 0 cảnh báo Verilator; chốt tham số clock/reset/AXI; `make check-guards` | `rtl/*`, `Makefile` |
| `c4db028` | chore(scripts): `verify_project.ps1` chạy đa nền tảng; ghi nhận build ARM | `scripts/verify_project.ps1`, `Makefile`, `docs/TEST_REPORT_SM5800.md` |
