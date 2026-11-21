# Báo cáo rà soát đợt 2 — SM-5800 IMU (`Nguyenhai642101/IMU`)

> Báo cáo đợt trước: [REVIEW_REPORT_1.md](REVIEW_REPORT_1.md).

| Mục | Giá trị |
|---|---|
| Ngày | 2026-11-21 |
| Phạm vi | Các thay đổi sau báo cáo đợt 1: PR #1 "Apply patch 1" (`ad36059`), commit "Fix PL-PS IRQ" (`52031fa`), và 3 patch sửa mới |
| Bàn giao | `IMU-fix-tc-drv-17.patch`, `IMU-t16-rtl-tests.patch`, `IMU-rate-plausibility.patch`, `IMU-review-reports.patch` (áp theo thứ tự này bằng `git am`) |
| Tổng thay đổi của 3 patch | 11 file, +426 / −32 dòng |

---

## 1. Tóm tắt

- **PR #1 "Apply patch 1"** khớp **100%** với patch đợt 1, không sai lệch dòng nào.
- **Commit "Fix PL-PS IRQ"** có nhiều cải tiến đúng hướng, trong đó **sửa được 3 lỗi tiềm ẩn do chính đợt 1 tạo ra**. Tuy nhiên:
  1. `make test-c` **lỗi 1 test** (`TC-DRV-17`), trong khi tài liệu ghi "PASS".
  2. **Cả 6 thay đổi RTL đều chưa có test bảo vệ**: đảo ngược bất kỳ thay đổi nào, mô phỏng vẫn báo 218/218 đạt.
  3. Kiểm tra "tốc độ quay hợp lệ" trong chương trình mẫu là **code chết**, kèm 3 lỗi phụ.
- **3 patch mới** sửa toàn bộ các điểm trên. Sau khi áp: **6/6 cổng kiểm tra đạt** trên một bản clone mới. Mọi thay đổi đều được chứng minh bằng **kiểm tra đột biến**: cố tình làm hỏng code thì test phải báo FAIL.

> 🔐 Ẩn dụ: commit `52031fa` lắp thêm nhiều ổ khóa tốt, nhưng chưa ai thử xem cửa có thực sự khóa không. Đợt này gắn cho mỗi ổ một chuông báo riêng: ai tháo ổ nào thì chuông của ổ đó reo.

---

## 2. Kết quả kiểm chứng trước / sau

Môi trường: GCC 13.3, Icarus Verilog 12.0, Verilator 5.020, `arm-none-eabi-gcc` 13.2.1, PowerShell 7.4.6.
Cột "Sau" được chạy trên **bản clone mới từ GitHub, rồi `git am` cả 3 patch**.

| Lệnh | `52031fa` (main hiện tại) | Sau 3 patch |
|---|---|---|
| `make verify-static` | ✅ PASS (0 violations) | ✅ PASS |
| `make lint-rtl` | ✅ 0 cảnh báo | ✅ 0 cảnh báo |
| `make check-guards` | ✅ 9/9 | ✅ **11/11** (thêm 2 trường hợp duty cycle) |
| `make sim` | ✅ 218/218, nhưng **không bảo vệ** thay đổi RTL mới | ✅ **255/255** (thêm `[T16]`) |
| `make test-c` (driver) | ❌ **90 đạt, 1 lỗi** | ✅ **92/92** |
| `make test-c` (chương trình mẫu) | — chưa có test | ✅ **9/9** (file test mới) |
| `make build-arm` | ✅ | ✅ stack lớn nhất 128 B ≤ 256 B |

---

## 3. Đánh giá commit "Fix PL-PS IRQ" (`52031fa`)

### 3.1 RTL: 6 thay đổi, đều đúng về ý tưởng

| # | Thay đổi | Đánh giá |
|---|---|---|
| R1 | `irq_o` tắt cùng chu kỳ với `BVALID` khi xóa W1C hoặc ghi `INT_ENABLE` | ✅ Giảm nguy cơ ngắt giả sau khi ISR thoát |
| R2 | `reset_busy_o` tính cả chu kỳ yêu cầu xung reset | ✅ Bịt khoảng hở 1 chu kỳ khi driver đọc `RST_BUSY` |
| R3 | Chỉ báo `IMU_FAULT` khi khung được commit | ✅ **Sửa lỗi của đợt 1**: ở chế độ 100 Hz, khi lỗi BIT kéo dài, Msg 01 từng ngắt CPU **600 lần/giây** |
| R4 | Watchdog tự kích hoạt lại khi nhả `RESET_L` / `BIT_L` | ✅ **Sửa lỗi của đợt 1**: timeout hết hạn trong lúc BIT thì watchdog tắt luôn, sau đó cảm biến câm không ai báo |
| R5 | `STATUS.LAST_SEQ` theo mọi khung hợp lệ (`prev_seq_cnt_r`) | ✅ Ở chế độ 100 Hz, giá trị không còn bị trễ |
| R6 | Tick 1 µs làm tròn lên; chốt kiểm tra duty cycle 50% ±2% | ✅ Bảo đảm `RESET_L` luôn > 1 ms, công thức duty đúng (bộ chia chẵn hoặc ≥ 25). ⚠️ Cái giá: với clock lẻ MHz, `timestamp_us` chạy chậm tới khoảng 2% |

### 3.2 Driver và chương trình mẫu

| Thay đổi | Đánh giá |
|---|---|
| ISR chỉ ghi lại bit lỗi thật (`DRV_INT_FAULT_MASK`) | ✅ **Sửa lỗi của đợt 1**: `MSG1_READY`/`MC_ANY` từng lọt vào danh sách lỗi |
| `sm5800_init` an toàn khi gọi lại (tắt IRQ, chuyển `UNINIT` trước) | ✅ |
| `read_msg2_raw` giữ vùng găng trong suốt lúc đọc | ✅ An toàn hơn khi đa task. ⚠️ Tắt ngắt toàn hệ thống khoảng vài µs (17 lần truy cập AXI) |
| Delta-angle / delta-velocity tính bằng `double` rồi làm tròn 1 lần | ✅ Bớt một lần làm tròn. Đầu ra vẫn là `float`, nên khuyến nghị về tích phân (mục 6) vẫn còn nguyên |
| Macro `SM5800_ASSERT` theo kiểu MISRA (`else`, `while (false)`) | ✅ |
| Chương trình mẫu: tự kiểm tra thêm trường, báo lỗi khi thu thiếu mẫu, kiểm tra BIT giữ đủ thời gian | ✅ |
| Chương trình mẫu: kiểm tra tốc độ quay ±2000°/s | ❌ Code chết. Xem mục 4.3 |

### 3.3 Test và script
- ✅ Test C mới: vector chuẩn CRC-32 (`0x6C6D6CC7`), các nhánh `NULL`, init lại với cờ khác (MC/DC), luồng phục hồi `TC-DRV-16`.
- ✅ `verify_project.ps1`: bỏ qua comment khối, phát hiện thân hàm chính xác hơn, kiểm tra thanh ghi và truy vết LLR **theo 2 chiều**.
- ❌ Testbench RTL chỉ được cập nhật phần comment, không có kịch bản mới.

---

## 4. Vấn đề phát hiện và cách xử lý

### 4.1 `TC-DRV-17` lỗi: sai thứ tự bước trong test, driver vẫn đúng
- **Hiện tượng:** `[FAIL] TC-DRV-17: LINK_TIMEOUT does not fake a Msg 02 sample`.
- **Nguyên nhân:** đoạn test mới gửi ngắt Msg 02 bình thường (`0x43`) nhưng **không đọc mẫu ra**. Kiểm tra "không có mẫu đang chờ" sau đó thấy mẫu cũ vẫn còn. Bảng `TEST_REPORT` ghi "92 assertion, PASS", nhưng code commit thực tế có 91 assertion và lỗi 1.
- **Sửa** (patch 1, 2 file): thêm bước đọc mẫu ra, giống ứng dụng thật. Cập nhật tài liệu.
- **Bằng chứng:** 92/92 đạt. Nếu ISR bị sửa sai (báo có mẫu khi chỉ có timeout), test vẫn bắt được.

### 4.2 Các thay đổi RTL chưa có test bảo vệ
- **Hiện tượng:** đảo ngược lần lượt R1–R6, mô phỏng vẫn báo `ALL TESTS PASSED (218)`.
- **Sửa** (patch 2, 8 file): kịch bản mới **`[T16]`**, cộng 3 bộ theo dõi tín hiệu cho các chi tiết thời gian mà đọc AXI không thấy được. `check-guards` thêm 25 MHz (chấp nhận) và 15 MHz (chặn, duty 46,7%).

| Thay đổi bị đảo ngược | Kiểm tra bắt được |
|---|---|
| R1 `irq_o` tắt muộn | `T16a IRQ low in same cycle as W1C BVALID` + `... INT_ENABLE BVALID` |
| R2 `reset_busy` thiếu chu kỳ yêu cầu | `T16b RESET_BUSY=1 in the pulse-request cycle` |
| R3 `IMU_FAULT` từ Msg 01 không được commit | `T16c` (đọc được `0x05`, đúng ra phải `0x01`) |
| R5 `LAST_SEQ` chỉ theo Msg 02 | `T16d LAST_SEQ == 4` (đọc được 3) |
| R4 watchdog không kích hoạt lại sau BIT | `T16e LINK_TIMEOUT re-armed after BIT_L release` |
| R6 tick làm tròn xuống | `T16f` (đo được 33 nhịp, đúng ra phải 34) |
| R6 bỏ chốt duty cycle | `check-guards`: 15 MHz NOT rejected |

### 4.3 Kiểm tra tốc độ quay trong chương trình mẫu: code chết và 3 lỗi phụ
| Lỗi | Hậu quả |
|---|---|
| Ngưỡng 2000°/s, nhưng full-scale 16-bit chỉ là 2¹⁵ × 2⁻²⁰ × 600 = **18,75 rad/s ≈ 1074°/s** | Kiểm tra không bao giờ kích hoạt |
| Chỉ kiểm tra trục X | Trục Y, Z bão hòa vẫn lọt qua |
| Ghi `g_last_eng` trước khi kiểm tra | Mẫu sai ghi đè mẫu tốt gần nhất |
| Gặp mẫu sai thì trả `ERR_PARAM` | Một mẫu sai làm dừng hẳn vòng lặp thu dữ liệu |

- **Sửa** (patch 3, 6 file): `is_rate_plausible()` kiểm tra 3 trục, cả hai chiều, ngưỡng **18,5 rad/s** (bão hòa). Mẫu sai thì đếm vào `g_rate_reject_count`, trả `ERR_NO_SAMPLE`, không ghi đè mẫu tốt; `main` bỏ qua và thu tiếp.
- **Test mới:** `tests/test_ps_example.c` là **unit test đầu tiên cho chương trình mẫu**. Nó `#include` file `.c` để gọi các hàm `static`, và đã được thêm vào `make test-c`.
  - `TC-APP-01`: hàm tự kiểm tra `verify_raw_to_eng_math()`, lần đầu được chạy thật.
  - `TC-APP-02`: mẫu bình thường, 18,4 rad/s ở biên trong, +Z / −Y / −X ở mức bão hòa, giữ nguyên mẫu tốt, đếm số lần loại.

| Bản lỗi có chủ đích | Kết quả |
|---|---|
| Quay lại ngưỡng 2000°/s | ❌ 5 FAIL |
| Chỉ kiểm tra trục X | ❌ 4 FAIL |
| Lưu mẫu trước khi kiểm tra | ❌ 2 FAIL |

### 4.4 Ghi nhận, chưa sửa (mức thấp)
- `read_msg2_raw` tắt ngắt khoảng vài µs. Ở 100 Hz thì chấp nhận được; nên ghi vào tài liệu WCET.
- Vẫn có thể đọc trùng một mẫu 2 lần nếu khung mới đến đúng lúc đang đọc. Ứng dụng có thể lọc bằng cách so `timestamp_us` với lần trước.
- Tick 1 µs làm tròn lên khiến `timestamp_us` chậm khoảng 2% trên clock lẻ MHz (ví dụ 33,333 MHz). Với clock nguyên MHz (100 MHz mặc định) thì chính xác.

---

## 5. Cách áp patch

```bash
cd IMU
```
```bash
git pull
```
```bash
git am IMU-fix-tc-drv-17.patch
```
```bash
git am IMU-t16-rtl-tests.patch
```
```bash
git am IMU-rate-plausibility.patch
```
```bash
git am IMU-review-reports.patch
```
```bash
make lint-rtl check-guards sim test-c
```
```bash
git push
```

| Patch | Commit (trên nhánh của người rà soát) | File | Dòng |
|---|---|---|---|
| 1 | `fix(test): TC-DRV-17 consumes the nominal Msg 02 sample before timeout case` | 2 | +9 / −2 |
| 2 | `test(rtl): [T16] protects every RTL change of 52031fa; duty-cycle guard test` | 8 | +209 / −20 |
| 3 | `fix(app): real 3-axis gyro saturation check; first unit tests for ps_example` | 6 | +209 / −11 |
| 4 | `docs: add review reports (round 1 & 2) under docs/reviews/` | Chỉ tài liệu: 2 báo cáo + link trong `README.md` | — |

Mã hash commit sẽ khác sau khi bạn `git am`, vì người commit thay đổi; nội dung thì giữ nguyên.

---

## 6. Việc còn lại (cập nhật)

| # | Mức độ | Nội dung | Gợi ý |
|---|---|---|---|
| 1 | **Cao** | Thử nghiệm **HIL** với cảm biến SM-5800 thật | Chạy HIL-01..04 trong `TEST_REPORT` |
| 2 | **Cao** | **Datasheet SM-5800** chưa có trong repo: hệ số scale, bit Status Word, cạnh lấy mẫu chưa được đối chiếu | Đối chiếu từng hằng số |
| 3 | **Cao** | **Dải đo định mức của gyro**: `APP_RATE_LIMIT_RAD_S = 18.5` hiện chỉ là ngưỡng bão hòa | Thay bằng dải đo định mức (ví dụ ±1000°/s = 17,45 rad/s) |
| 4 | **Cao** | **Thời gian power-up BIT** sau `RESET_L`: nếu > 30 ms, watchdog có thể báo nhầm | Chỉnh `TIMEOUT_CFG` hoặc chờ thêm sau reset |
| 5 | Trung bình | Tích phân delta-angle/velocity bằng `float` sẽ tích lũy sai số | Tích phân bằng `int64_t`/`double` |
| 6 | Trung bình | `RESET_L`/`BIT_L` là ngõ ra push-pull | IC open-collector ngoài hoặc `IOBUF` |
| 7 | Thấp | Chưa đo độ phủ MC/DC | `gcc --coverage` + `gcovr`; công cụ MC/DC chuyên dụng nếu cần DO-178C |
| 8 | Thấp | Repo chưa có CI | GitHub Actions chạy `make lint-rtl check-guards sim test-c` mỗi lần push |

---

## 7. Phụ lục: tiến trình số liệu kiểm thử

| Mốc | Mô phỏng RTL | Unit test C | Lint | Chốt tham số |
|---|---|---|---|---|
| Bản gốc `955a222` | ❌ 163/167 | ❌ 55/56 | ❌ 7 cảnh báo | — |
| Sau đợt 1 (`ad36059`) | ✅ 218/218 | ✅ 67/67 | ✅ 0 | ✅ 9/9 |
| `52031fa` | ✅ 218/218 (thiếu bảo vệ) | ❌ 90/91 | ✅ 0 | ✅ 9/9 |
| **Sau đợt 2 (3 patch)** | ✅ **255/255** | ✅ **92/92 + 9/9** | ✅ 0 | ✅ **11/11** |
