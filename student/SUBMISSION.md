# Báo cáo bài nộp — Day 23 Sensor Fusion Lab

> Điền file này rồi commit. Cách nộp: [hướng dẫn nộp](../SUBMISSION.md).

## Thông tin học viên

- Họ tên: Van Quoc Dung
- MSSV: 2A202602505
- Email: vvstdung89@gmail.com
- Link repo (fork): https://github.com/vvstdung89/K4-L3L4-DAY23-VanQuocDung-2A202602505-SensorFusion
- Commit hash nộp (`git rev-parse HEAD`): commit cuối trên `main`, nộp kèm link trên LMS. Code E–H và artifacts chấm điểm được tạo ở commit `204774ada6e931061a4144e7cde80df310d4d114` (CP5); các commit sau chỉ sửa báo cáo.

## Tóm tắt kết quả

- `fusion_mode` (bắt buộc `compare`), `frames`, `segment`, `seed`: `compare`, `[0, 198]` (199 frame), `training_segment-1005081002024129653_5313_150_5333_150_with_camera_labels.tfrecord`, `seed = 0`
- `detection.precision`, `detection.recall`, `detection.tp/fp/fn`: precision = 0.9701, recall = 0.7004, tp = 519, fp = 16, fn = 222
- `tracking.lidar.rmse`, `matches`, `sum_sq_err`, `ghost_track_frames`, `missed_gt_frames`, `mean_confirmed_tracks`: rmse = 0.1503 m, matches = 502, sum_sq_err = 11.3436 m², ghost_track_frames = 0, missed_gt_frames = 239, mean_confirmed_tracks = 2.5226
- `tracking.fused.rmse`, `matches`, `sum_sq_err`, `ghost_track_frames`, `missed_gt_frames`, `mean_confirmed_tracks`: rmse = 0.1359 m, matches = 502, sum_sq_err = 9.2668 m², ghost_track_frames = 0, missed_gt_frames = 239, mean_confirmed_tracks = 2.5226
- Giải thích khác biệt hai mode, đọc RMSE cùng số ghép và ghost/miss: xem bảng và phân tích dưới đây.

| Mode | RMSE (m) | matches | ghost | miss | precision_track | coverage (`det_tp` = 519) |
|---|---|---|---|---|---|---|
| LiDAR | 0.1503 | 502 | 0 | 239 | 502/502 = 1.000 | 502/519 = 0.967 |
| Fused | 0.1359 | 502 | 0 | 239 | 502/502 = 1.000 | 502/519 = 0.967 |

`rmse_fused − rmse_lidar = −0.0145 m` (≤ 0.05 m): camera **không** làm tracking xấu đi.

**Phân tích.**

- **Số đếm giống hệt, chỉ RMSE khác.** Trong `grade_run.log`, với **cả 199 frame**, các trường
  `det_tp, det_fp, det_fn, valid_gt, confirmed, matches, ghosts, misses` của hai mode trùng nhau;
  chỉ `sum_sq_err` khác. Ví dụ frame 100 (dòng 101 là lidar, dòng 300 là fused): cả hai có
  `confirmed = matches = 3`, `misses = 0`, nhưng `sum_sq_err` là 0.0601 (lidar) và 0.0488 (fused).
  Đây đúng là hệ quả của thiết kế: camera chỉ EKF update, không tạo, xác nhận hay xoá track,
  nên tập confirmed tracks và việc ghép GT không đổi; chỉ vị trí ước lượng thay đổi.
- **Fused chính xác hơn một chút.** Tổng `sum_sq_err` giảm từ 11.34 xuống 9.27 m² (−18%) trên
  cùng 502 cặp, nên RMSE giảm 1.4 cm. Theo frame, fused có `sum_sq_err` thấp hơn ở 137/199 frame
  và cao hơn ở 58/199 frame, nên cải thiện là xu hướng chung chứ không phải đều ở mọi frame.
  Camera bổ sung đo góc (u, v) với σ = 5 px, giúp tinh chỉnh vị trí ngang và độ cao. Trên trục
  độ sâu, camera gần như không có thông tin.
- **Ghost = 0.** Mọi confirmed track đều ghép được GT (precision_track = 1.0). 16 detection FP không
  tạo ghost vì track mới cần ≥ 4 hit LiDAR liên tiếp mới vượt `confirmed_threshold`, còn FP
  thường không lặp lại đủ.
- **239 miss = 222 + 22 − 5** (đối chiếu `misses` với `det_fn` từng frame trong log lidar).
  - **222** là xe detector bỏ sót (`det_fn`).
  - **+22** là xe đã được detect nhưng chưa có confirmed track vì track còn initialized/tentative:
    frame 0–3 (2 xe × 4 frame = 8, track confirmed từ frame 4) và 14 frame-miss ở frame 44–47,
    60–61, 63, 85, 88, 92–93, 95–97 khi xe mới xuất hiện hoặc track được tạo lại. Đây là chi phí
    của cơ chế xác nhận chống ghost.
  - **−5** là frame mà detector bỏ sót xe nhưng track confirmed vẫn ghép được GT nhờ EKF predict
    (frame 50, 55, 56, 58, 188). Tracking bù được một phần lỗi detection.
- **Giới hạn.** Đo camera trong lab là tâm hộp 2D ground-truth FRONT cộng nhiễu theo `seed`,
  không phải camera detector. Kết quả fused tốt hơn chỉ cho thấy EKF kết hợp đúng một đo 2D có
  nhiễu nhỏ và không có bias. Nó **không** chứng minh một hệ perception có camera thật sẽ tốt hơn,
  vì detector thật có miss, FP, bias và lệch calibration.

Chạy từ root repo:

```bash
fusion-run-lab --config student/config/paths.yaml --fusion compare --seed 0
```

`rmse = sqrt(sum_sq_err/matches)` trên vị trí 3D của confirmed tracks ghép
một-một với GT xe trong cửa sổ BEV, gate XY **2.0 m**; `null` nếu không có cặp.
Camera dùng tâm hộp 2D ground-truth FRONT có nhiễu seeded, **không** dùng camera
detector. Kết quả này không đo hiệu quả một perception system độc lập với GT.

`grade_run.log` là JSONL, mỗi `(mode,frame)` đúng một record với các trường:
`mode`, `frame`, `det_tp`, `det_fp`, `det_fn`, `valid_gt`, `confirmed`, `matches`,
`sum_sq_err`, `ghosts`, `misses`. Đảm bảo `matches+ghosts==confirmed` và
`matches+misses==valid_gt`; tổng/trung bình record phải khớp `metrics.json`.
File per-mode `metrics_lidar.json`, `metrics_fused.json`, `grade_run_lidar.log`,
`grade_run_fused.log` được giữ để đối chiếu.

Đã tự kiểm tra: 398 record (199 frame × 2 mode), không trùng hoặc thiếu; 0 record vi phạm hai
bất biến trên; detection giống nhau ở hai mode; `fusion_lab.evaluation.validate_metrics_records`
chấp nhận cả `metrics.json` và hai cặp file per-mode.

## Giải thích ngắn (Parts E–H — tự viết)

1. **Khác biệt đo lidar 3D và camera 2D trong EKF (`z`, `R`)?**
   - **LiDAR:** `z = (x, y, z)ᵀ` là tâm hộp 3D, đơn vị mét, 3×1. `R = diag(0.1², 0.1², 0.1²)` m².
     `h(x)` tuyến tính (đổi hệ toạ độ), nên `H` là ma trận hằng 3×6 `[R_rot | 0]`. Đo cả ba
     trục vị trí, kể cả độ sâu.
   - **Camera:** `z = (u, v)ᵀ` là pixel, 2×1. `R = diag(5², 5²)` px², xem
     `camera_fusion.py:90`. `h(x)` là phép chiếu pinhole phi tuyến `u = c_i − f_i·y_s/x_s`,
     `v = c_j − f_j·z_s/x_s` (`camera_fusion.py:74-75`). Vì vậy `H` là Jacobian 2×6, phụ thuộc
     trạng thái và tính lại mỗi lần update (platform cung cấp `get_H`).
   - **Hệ quả:** camera chỉ cho hướng (góc), không cho độ sâu. `S = HPHᵀ + R` của camera có đơn vị
     px², và một sai số 1 m ở xa tạo ít pixel hơn ở gần.
   - Cùng một hàm `ekf_update` (`kalman.py:111-129`) xử lý cả hai: nó lấy `H` và `h(x)` từ
     `meas.sensor`, rồi tính `K = P Hᵀ S⁻¹`, `x⁺ = x + Kγ`, `P⁺ = (I − KH)P`.
2. **Vì sao cần gating Mahalanobis trước khi gán?**
   - `d² = γᵀ S⁻¹ γ` (`association.py:28-41`) chuẩn hoá innovation theo độ bất định tổng
     `S = HPHᵀ + R` của cả track lẫn sensor. Track mới có `P` lớn (σ vận tốc 50 m/s) nên chấp nhận
     đo xa hơn; track đã hội tụ thì chặt hơn. Khoảng cách Euclid bỏ qua `P` và không so sánh được
     giữa mét (lidar) và pixel (camera).
   - Với γ Gauss, `d²` tuân theo χ² với số bậc tự do bằng `dim_meas`. Cổng
     `d² < chi2.ppf(0.995, dim)` (`association.py:54`), tức 12.84 cho lidar và 10.60 cho camera,
     loại cặp mà xác suất là đo đúng của track < 0.5%.
   - Ô bị loại mang cost `inf`, nên greedy (`pick_next_pair`) không bao giờ ghép nó. Nhờ vậy một
     FP hoặc một xe khác không kéo lệch track, và đo không ghép có thể sinh track mới.
   - Cặp ngoài FOV bị loại **trước** khi tính Mahalanobis (`association.py:75`), để không chiếu điểm
     sau camera.
3. **Pipeline là track-then-fuse hay fuse-then-track? Chỉ ra trên log `fusion-run-lab`.**
   - **Track-then-fuse.** Có một danh sách track. Mỗi frame, `platform/fusion_lab/scripts/run_lab.py`
     làm theo thứ tự:
     - dòng 199–201: `KF.predict(track)` một lần cho mọi track;
     - dòng 202: `associate_and_update(manager, observations, KF, lidar_sensor)` (AssocL), xong
       gọi `manage_tracks`;
     - dòng 209: `associate_and_update(..., camera_sensor)` (AssocC), không predict lại.
   - Không có bước gộp raw point cloud với ảnh trước detection.
   - **Trên log:**
     - Detection (`det_tp/fp/fn`) giống nhau ở hai mode vì detector chỉ dùng LiDAR.
     - `confirmed/matches/ghosts/misses` trùng nhau ở cả 199 frame vì lifecycle chỉ do lượt LiDAR
       quyết định. Track đầu tiên thành confirmed ở frame 4 trong **cả hai** mode.
     - Chỉ `sum_sq_err` khác (ví dụ frame 4: 0.0113 lidar so với 0.0320 fused; frame 100: 0.0601
       so với 0.0488). Camera chỉ tinh chỉnh trạng thái của track **đã tồn tại**. Nếu là
       fuse-then-track, camera sẽ ảnh hưởng tới việc sinh/xác nhận track và các số đếm sẽ khác.
4. **Nếu camera lệch calibration, triệu chứng gì trên innovation/residual?**
   (Phân tích, chưa chạy thí nghiệm, vì bonus calibration không làm.)
   - Extrinsic sai làm `h(x)` lệch **có hệ thống**. Innovation camera `γ = z − h(x)` không còn
     trung bình 0 mà có bias ổn định:
     - lệch yaw: chủ yếu `u` lệch cùng dấu;
     - lệch pitch: chủ yếu `v`;
     - lệch tịnh tiến: bias phụ thuộc khoảng cách, lớn ở xe gần.
   - `d²` tăng đều, phân bố `d²` không còn khớp χ²(2). Innovation của LiDAR trên cùng track cũng
     lệch theo, vì camera kéo `x` sai và lidar kéo lại, nên track dao động qua lại.
   - **Lệch nhỏ:** cặp vẫn qua cổng nhưng kéo lệch vị trí, RMSE fused tăng và vượt lidar.
   - **Lệch lớn:** `d²` > 10.6, cặp bị cổng loại, track không nhận update camera. Fused tiến về
     lidar-only, đây là lỗi "im lặng" nhưng an toàn.
   - Cổng không phát hiện được bias nhỏ, vì nó chỉ xét từng cặp, không xét trung bình residual
     theo thời gian.
5. **Vì sao `associate_and_update(..., sensor)` cần sensor tường minh ở frame rỗng? Giải thích vì
   sao lidar quyết định score/init/delete còn camera chỉ EKF update.**
   - Khi `meas_list` rỗng thì không có `meas.sensor` để biết đây là lượt lidar hay camera. Nhưng
     lượt **lidar** rỗng vẫn phải chạy `manage_tracks` để trừ score cho track trong FOV và xoá
     track hết hạn. Nếu bỏ qua, track ma sẽ sống mãi khi detector không thấy gì. Vì vậy hàm luôn
     gọi `manager.manage_tracks(unassigned_tracks, unassigned_meas, sensor)` ở cuối
     (`association.py:140`), kể cả với danh sách rỗng.
   - **Lidar quyết định tồn tại** vì nó cho vị trí 3D đầy đủ, từ đó khởi tạo được `x` và `P`, và có
     FOV ±90° bao trùm toàn bộ cửa sổ BEV của detector (`lim_x = [0, 50]`, `lim_y = [−25, 25]` m).
     Camera chỉ có 2D (không có độ sâu, không khởi tạo được track 3D),
     chỉ thấy phía trước (FOV hẹp). Nếu camera được trừ score, mọi track ngoài FOV camera hoặc
     không có nhãn FRONT sẽ bị xoá oan.
   - Trong lab, camera còn là đo mô phỏng từ nhãn, nên cho nó quyết định tồn tại sẽ làm rò rỉ
     ground truth vào lifecycle.
   - Trong code: `handle_updated_track` chỉ ghi hit khi `sensor.name == "lidar"`, và
     `manage_tracks` return ngay với lượt camera.
6. **Nêu điều kiện xác nhận, giữ confirmed sau miss, và điều kiện xóa track.**
   (`track_management.py:44-90`, tham số từ `get_tracking_params()`.)
   - **Khởi tạo:** từ đo lidar chưa ghép. `score = 1/window = 1/6`, `state = "initialized"`,
     vận tốc 0 với σ = (50, 50, 5).
   - **Score:** mỗi hit lidar cộng 1/6 (tối đa 1). Mỗi miss **trong FOV lidar** trừ 1/6.
   - **Xác nhận:** khi `score > confirmed_threshold = 0.8` (so sánh chặt). Từ 1/6 cần 4 hit liên
     tiếp, tức 5/6 ≈ 0.833, nên track sinh ở frame 0 thành confirmed ở frame 4, khớp log. Hit mà
     chưa đủ điểm thì `state = "tentative"`.
   - **Giữ confirmed sau miss:** track confirmed không bị hạ trạng thái. Sau một miss từ 1.0 còn
     5/6 ≈ 0.833 ≥ 0.6 nên vẫn sống.
   - **Xoá** (OR, `track_management.py:85-90`):
     - `P[0,0]` hoặc `P[1,1]` > `max_P = 9` m², tức độ bất định ngang quá lớn, bất kể score;
     - track confirmed có `score < delete_threshold = 0.6`, tức 3 miss liên tiếp từ 1.0;
     - track chưa confirmed có `score ≤ 0`.
   - Camera không gọi các hàm này.

## Bonus (không bắt buộc)

Liệt kê phần bonus đã làm, file bằng chứng trong `student/bonus/` và kết quả chính
(xem [RUBRIC.md](../RUBRIC.md) mục 2). Không làm thì ghi "Không".

- Không

## Khai báo sử dụng AI (bắt buộc)

Ghi rõ, kể cả khi không dùng ("Không dùng AI"). Xem [RULES.md](../RULES.md) mục 2.

- Công cụ đã dùng (ChatGPT, Copilot, Claude, …): Claude Code (mô hình Claude Opus 5.5 của Anthropic)
- Dùng cho phần nào (hàm, câu hỏi, debug):
  - đọc repo và các trang lab trên VLearn, lập kế hoạch làm bài;
  - cài môi trường, giải nén và kiểm tra dữ liệu;
  - viết code Part E–H: `kalman.py`, `camera_fusion.py`, `association.py`, `track_management.py`;
  - debug lỗi `float()` trên `np.matrix` 1×1 trong `mahalanobis_distance`;
  - chạy `fusion-run-lab`, viết script kiểm tra artifacts, soạn nháp báo cáo này.
- Cách bạn đã kiểm tra lại (pytest, chạy Waymo, đối chiếu công thức):
  - `pytest student/tests -q` cho 128 passed, 0 failed, 0 xfailed;
  - đối chiếu công thức với `docs/HUONG_DAN_KY_THUAT.md`, các TODO trong stub và
    `platform/fusion_lab/tracking_params.py`;
  - chạy `fusion-run-lab --fusion compare --seed 0` trên frame 0–198;
  - kiểm tra artifacts bằng `fusion_lab.evaluation.validate_metrics_records` (398 record, bất
    biến đúng, detection giống nhau ở hai mode);
  - mọi số liệu trong báo cáo được lấy trực tiếp từ `metrics.json` và `grade_run.log`.

## Checklist nộp

- [x] **Part E–H** trong `workspace/` đã implement; `pytest student/tests -q` không còn `failed`/`xfailed`
- [x] Part A–D: không bắt buộc sửa (hoặc ghi chú nếu bạn đã sửa)
- [x] Lần chạy chấm điểm: `--fusion compare --seed 0`, `frame_start: 0`, `frame_end: 198`
- [x] Đã commit `student/artifacts/metrics*.json` và `student/artifacts/grade_run*.log` (không sửa tay)
- [x] Đã điền đủ file này, gồm khai báo AI
- [x] Không commit dữ liệu Waymo, weights, `paths.yaml`, API key
- [x] `python tools/check_submission.py` báo `KẾT QUẢ: SẴN SÀNG NỘP`
- [ ] Đã push và nộp link repo + commit hash trên LMS ([hướng dẫn nộp](../SUBMISSION.md))
