# Báo cáo bài nộp — Day 23 Sensor Fusion Lab

> Điền file này rồi commit. Cách nộp: [hướng dẫn nộp](../SUBMISSION.md).

## Thông tin học viên

- Họ tên: Văn Quốc Dũng
- MSSV: 2A202602505
- Email: [vvstdung89@gmail.com](mailto:vvstdung89@gmail.com)
- Link repo (fork): [https://github.com/vvstdung89/K4-L3L4-DAY23-VanQuocDung-2A202602505-SensorFusion](https://github.com/vvstdung89/K4-L3L4-DAY23-VanQuocDung-2A202602505-SensorFusion)
- Commit hash nộp (`git rev-parse HEAD`): `73179147e60e1dd69fd10b02ef551465a47b5646` 

## Tóm tắt kết quả

- `fusion_mode` (bắt buộc `compare`), `frames`, `segment`, `seed`: `compare`, `[0, 198]` (199 frame), `training_segment-1005081002024129653_5313_150_5333_150_with_camera_labels.tfrecord`, `seed = 0`
- `detection.precision`, `detection.recall`, `detection.tp/fp/fn`: precision = 0.9701, recall = 0.7004, tp = 519, fp = 16, fn = 222
- `tracking.lidar.rmse`, `matches`, `sum_sq_err`, `ghost_track_frames`, `missed_gt_frames`, `mean_confirmed_tracks`: rmse = 0.1503 m, matches = 502, sum_sq_err = 11.3436 m², ghost_track_frames = 0, missed_gt_frames = 239, mean_confirmed_tracks = 2.5226
- `tracking.fused.rmse`, `matches`, `sum_sq_err`, `ghost_track_frames`, `missed_gt_frames`, `mean_confirmed_tracks`: rmse = 0.1359 m, matches = 502, sum_sq_err = 9.2668 m², ghost_track_frames = 0, missed_gt_frames = 239, mean_confirmed_tracks = 2.5226
- Giải thích khác biệt hai mode, đọc RMSE cùng số ghép và ghost/miss: xem bảng và phân tích dưới đây.


| Mode  | RMSE (m) | matches | ghost | miss | precision_track | coverage (`det_tp` = 519) |
| ----- | -------- | ------- | ----- | ---- | --------------- | ------------------------- |
| LiDAR | 0.1503   | 502     | 0     | 239  | 502/502 = 1.000 | 502/519 = 0.967           |
| Fused | 0.1359   | 502     | 0     | 239  | 502/502 = 1.000 | 502/519 = 0.967           |


`rmse_fused − rmse_lidar = −0.0145 m` (≤ 0.05 m): camera **không** làm tracking xấu đi.

**Phân tích.**

- **Số đếm giống hệt, chỉ RMSE khác.** Trong `grade_run.log`, với **cả 199 frame**, các trường
`det_tp, det_fp, det_fn, valid_gt, confirmed, matches, ghosts, misses` của hai mode trùng nhau;
chỉ `sum_sq_err` khác. Ví dụ frame 100 (dòng 101 là lidar, dòng 300 là fused): cả hai có
`confirmed = matches = 3`, `misses = 0`, nhưng `sum_sq_err` là 0.0601 (lidar) và 0.0488 (fused).
Đây là hệ quả của thiết kế: camera chỉ EKF update, không **trực tiếp** tạo, xác nhận hay xoá track.
Camera vẫn ảnh hưởng gián tiếp, vì `x`, `P` sau update camera quyết định cổng LiDAR ở frame sau
(bonus: lệch yaw +5° cho 499 match / 1 ghost / 242 miss thay vì 502 / 0 / 239). Trong lần chạy
chấm điểm, ảnh hưởng gián tiếp này không đổi số đếm nào; chỉ vị trí ước lượng thay đổi.
- **Fused chính xác hơn một chút.** Tổng `sum_sq_err` giảm từ 11.34 xuống 9.27 m² (−18%) trên
cùng 502 cặp, nên RMSE giảm 1.4 cm. Theo frame, fused có `sum_sq_err` thấp hơn ở 137/199 frame
và cao hơn ở 58/199 frame, nên cải thiện là xu hướng chung chứ không phải đều ở mọi frame.
Camera bổ sung đo góc (u, v) với σ = 5 px. Tách RMSE theo trục trên 502 cặp (bonus,
`student/bonus/fig3_track_error_and_axis_rmse.png`):
  - x (dọc): 0.108 → 0.093 m;
  - y (ngang): 0.079 → 0.062 m, cải thiện rõ nhất;
  - z (cao): 0.068 → 0.077 m, **xấu đi**.

  x cũng giảm dù camera không đo trực tiếp độ sâu. Một giải thích hợp lý (chưa kiểm chứng
  riêng): với xe lệch tâm, `u` phụ thuộc `y/x` nên ràng buộc một tổ hợp của x và y, và vận tốc
  ước lượng tốt hơn giúp bước predict chính xác hơn. z xấu đi vì đo camera có bias theo chiều
  dọc (xem Giới hạn).
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
không phải camera detector.
  - Nhiễu thêm vào chỉ có σ = 0.5 px (`run_lab.py`, `rng.normal(0, 0.5, 2)`), trong khi `R`
  giả định σ = 5 px.
  - Dù vậy đo này **có bias**: với calibration đúng, innovation camera trung bình là
  γu ≈ −9.5 px và γv ≈ +8.5 px (bonus). Tâm hộp 2D là tâm phần nhìn thấy, có thể bị cắt ở mép
  ảnh như T10 ở frame 116, nên không trùng với hình chiếu tâm hộp 3D. Lệch thời gian
  camera/LiDAR cũng có thể góp phần; chưa tách được hai nguyên nhân này.
  - Bias γv giải thích vì sao z xấu đi. Bias γu không phải lỗi calibration: thí nghiệm yaw
  +0.25° đưa γu về ≈ 0 nhưng RMSE y lại tăng từ 0.062 lên 0.075 m.
  - Vì vậy kết quả fused tốt hơn **không** chứng minh một hệ perception có camera thật sẽ tốt
  hơn, vì detector thật có miss, FP, bias và lệch calibration.

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
  nghĩa là giữ lại 99.5% đo đúng của track: một đo đúng chỉ có 0.5% khả năng có `d²` vượt
  ngưỡng và bị loại nhầm.
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
    quyết định (camera chỉ ảnh hưởng gián tiếp qua trạng thái EKF, và ở lần chạy này không làm
    đổi số đếm nào). Track đầu tiên thành confirmed ở frame 4 trong **cả hai** mode.
    - Chỉ `sum_sq_err` khác (ví dụ frame 4: 0.0113 lidar so với 0.0320 fused; frame 100: 0.0601
    so với 0.0488). Camera chỉ tinh chỉnh trạng thái của track **đã tồn tại**. Nếu là
    fuse-then-track, camera sẽ ảnh hưởng tới việc sinh/xác nhận track và các số đếm sẽ khác.
4. **Nếu camera lệch calibration, triệu chứng gì trên innovation/residual?**
Đã đo trong bonus: bảng ở mục Bonus, chi tiết ở `student/bonus/calibration_table.md`.)
  - **Innovation có bias tuyến tính theo góc lệch (khi lệch nhỏ).** Extrinsic sai làm `h(x)`
  lệch có hệ thống, nên innovation camera `γ = z − h(x)` dịch trung bình:
    - lệch yaw từ −0.5° đến +1° chỉ dịch `u`: +36.0 đến +36.7 px/°, trong khi γv gần như không
    đổi (+8.2 đến +8.8 px);
    - lệch pitch chỉ dịch `v` ở mọi mức (+0.5° đến +2°): −35.5 đến −36.4 px/°;
    - lý thuyết f·π/180 = 2083·π/180 ≈ 36.4 px/°, khớp với đo đạc trong vùng này.
    - Ở yaw −1°, ±2°, +5°, độ dốc giảm còn 32.6, 22.5–26.0 và 15.9 px/° (yaw +5° cho γu = +69.9 px
    thay vì ≈ +172 px theo lý thuyết), và γv lệch khỏi +8.5 px (−1°: +5.8; ±2°: ≈ −8; +5°:
    −16.3). Có thể do khi lệch ngang lớn, đo gần nhất theo d² không còn là hộp của chính xe đó
    mà là hộp của xe bên cạnh; giả thuyết này chưa kiểm chứng riêng.
  - Lệch tịnh tiến (bias phụ thuộc khoảng cách) **không đo**.
  - `**d²` tăng theo góc lệch.** Median `d²` của cặp gần nhất là 1.6 khi calibration đúng,
  2.0–9.1 ở ±0.5°, 8.3–23.4 ở ±1°, và 42–67 ở ±2°.
  - **Lệch nhỏ (yaw +0.25°, yaw +0.5°, pitch +0.5°): cổng không bắt được.** 98–99% cặp vẫn qua
  cổng nhưng kéo lệch vị trí theo đúng trục:
    - yaw +0.5°: RMSE y 0.062 → 0.142 m, RMSE 0.194 m, xấu hơn LiDAR-only (0.150) 0.043 m, sát
    ngưỡng nhất quán 0.05 m của rubric (ngưỡng này so fused với LiDAR-only);
    - pitch +0.5°: RMSE z 0.077 → 0.139 m.
  - **Lệch trung bình (yaw −0.5°, yaw +1°, pitch +1°): tệ nhất.** Cổng loại 38–56% cặp. Các cặp
  còn lọt qua đều lệch cùng một hướng, nên RMSE là 0.211, 0.256 và 0.214 m, tức xấu hơn
  LiDAR-only +0.061, +0.106 và +0.063 m: **vượt** ngưỡng nhất quán 0.05 m.
  - **Lệch lớn (yaw −1°, ±2°, +5°, pitch +2°): cổng loại 97–100% cặp với track confirmed, nhưng
  không hoàn toàn an toàn.**
    - yaw +2° và +5° gần về LiDAR-only (0.155 và 0.154 m);
    - yaw −1° (0.173 m), yaw −2° (0.176 m) và pitch +2° (0.212 m) vẫn xấu hơn rõ; riêng pitch +2°
    xấu hơn LiDAR-only +0.062 m, vượt ngưỡng nhất quán;
    - lý do: vài update vẫn lọt qua, và 35–62% trong số đó (yaw −1°: 7/20, yaw −2°: 7/19,
    pitch +2°: 8/13) rơi vào track mới chưa confirmed, vì `P` lớn làm cổng rất rộng. Những update này làm sai trạng thái
    ban đầu, và sai số còn kéo dài về sau.
  - **Bất đối xứng.** Do đo vốn có bias γu ≈ −9.5 px, yaw −0.5° cộng dồn bias nên 38% cặp bị
  loại, còn yaw +0.5° bù trừ bias nên chỉ 2% bị loại.
  - **Kết luận.** Cổng χ² chỉ xét từng cặp nên không phát hiện được bias nhỏ. Cần theo dõi
  trung bình innovation theo thời gian (khác 0 là dấu hiệu lệch calibration) thay vì chỉ
  dựa vào cổng.
5. **Vì sao `associate_and_update(..., sensor)` cần sensor tường minh ở frame rỗng? Giải thích vì
ao lidar quyết định score/init/delete còn camera chỉ EKF update.**
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
`track_management.py:44-90`, tham số từ `get_tracking_params()`.)
  - **Khởi tạo:** từ đo lidar chưa ghép. `score = 1/window = 1/6`, `state = "initialized"`,
   vận tốc 0 với σ = (50, 50, 5).
  - **Score:** mỗi hit lidar cộng 1/6 (tối đa 1). Mỗi miss **trong FOV lidar** trừ 1/6.
  - **Xác nhận:** khi `score > confirmed_threshold = 0.8` (so sánh chặt). Từ 1/6 cần 4 hit liên
  tiếp, tức 5/6 ≈ 0.833, nên track sinh ở frame 0 thành confirmed ở frame 4, khớp log. Hit mà
  chưa đủ điểm thì `state = "tentative"`.
  - **Giữ confirmed sau miss:** track confirmed không bị hạ trạng thái. Sau một miss từ 1.0 còn
  5/6 ≈ 0.833 ≥ 0.6 nên vẫn sống.
  - **Xoá** (OR, `track_management.py:85-90`):
    - `P[0,0]` hoặc `P[1,1]` &gt; `max_P = 9` m², tức độ bất định ngang quá lớn, bất kể score;
    - track confirmed có `score < delete_threshold = 0.6`, tức 3 miss liên tiếp từ 1.0;
    - track chưa confirmed có `score ≤ 0`.
  - Camera không gọi các hàm này.

## Bonus (không bắt buộc)

Liệt kê phần bonus đã làm, file bằng chứng trong `student/bonus/` và kết quả chính
(xem [RUBRIC.md](../RUBRIC.md) mục 2). Không làm thì ghi "Không".

**Đã làm:** bonus 2 (trực quan hoá, +3) và bonus 3 (phân tích calibration, +4).
**Không làm:** bonus 1 (CVAT).

**Cách tạo bằng chứng.** Chạy từ root repo:

```bash
python student/bonus/bonus_study.py --config student/config/paths.yaml
```

- Script đi một lượt qua frame 0–198 (seed 0). Detector chạy một lần mỗi frame, sau đó 13 biến
thể tracker chạy song song trên cùng detection: LiDAR-only, fused, và fused với extrinsic
camera bị xoay.
- Script chỉ import helper của platform: không sửa `platform/`, không ghi vào
`student/artifacts/`. Đã kiểm tra checksum 6 file artifacts trước và sau khi chạy: không đổi.
- **Kiểm tra tái lập:** biến thể `lidar` và `fused` cho kết quả **trùng khớp**
`metrics_lidar.json` và `metrics_fused.json` (cùng matches, ghosts, misses, `sum_sq_err`).
Vì vậy mọi khác biệt ở các biến thể còn lại chỉ đến từ lệch calibration.
- Kết quả: `student/bonus/calibration_results.json` (đầy đủ) và
`student/bonus/calibration_table.md`.

### Bonus 2 — Trực quan hoá tác dụng của camera update

- `student/bonus/fig1_bev_frame116.png`: BEV frame 116, gồm GT (hộp xám), detection LiDAR,
track LiDAR-only (□) và track fused (△), sai số 3D từng track, FOV camera FRONT.
  - T1: 0.11 → 0.07 m; T0: 0.12 → 0.09 m.
  - T10: 0.19 → 0.28 m, xấu đi. T10 vừa vào FOV, nằm sát mép ảnh, hộp 2D bị cắt nên tâm hộp lệch.
- `student/bonus/fig2_camera_update_frame116.png`: ảnh camera FRONT frame 116, cùng crop phóng
to quanh từng track. Mỗi crop có đo `z` (+ xanh lá), `h(x⁻)` trước update (đỏ) và `h(x⁺)`
sau update (xanh lam). Camera update kéo hình chiếu track về phía đo:
  
  | Track | |z − h(x)| trước → sau | d²                   |
  | ----- | ---------------------- | -------------------- |
  | T1    | 12.0 → 3.8 px          | 1.84                 |
  | T0    | 20.2 → 4.5 px          | 3.63                 |
  | T10   | 39.2 → 6.0 px          | 9.13 (sát cổng 10.6) |
  
- `student/bonus/fig3_track_error_and_axis_rmse.png`: sai số 3D theo thời gian của hai track
và RMSE theo trục.
  - **T10, camera giúp nhiều nhất** (101 frame): từ frame 98 đến 115 track chưa nhận update
  camera nào (xe ở ngoài hoặc sát mép FOV camera), nên hai đường **trùng nhau**. Lần update
  camera đầu tiên ở frame 116 lại làm sai số **tăng** (0.19 → 0.28 m, hộp 2D bị cắt ở mép ảnh,
  xem fig1). Sau vài frame, đường fused xuống dưới LiDAR-only và thấp hơn trong phần lớn thời
  gian còn lại.
  Trung bình 0.188 → 0.116 m; RMSE y 0.105 → 0.043, z 0.105 → 0.061 m. Đây là bằng chứng
  trực tiếp tác dụng của camera update trên một track.
  - **T0, camera làm xấu nhiều nhất** (xe màu trắng phía trước, 195 frame): trung bình
  0.141 → 0.163 m. x cải thiện (0.127 → 0.098) nhưng z xấu đi (0.043 → 0.102), do bias dọc
  của tâm hộp 2D.
  - **Toàn bộ 502 cặp:** x 0.108 → 0.093, y 0.079 → 0.062, z 0.068 → 0.077 m.

### Bonus 3 — Phân tích calibration: làm lệch extrinsic camera

**Cách làm.** Tracker giả định extrinsic camera bị xoay thêm một góc yaw (quanh trục đứng)
hoặc pitch (quanh trục ngang) trong hệ camera. Đo camera vẫn lấy từ ảnh thật, nên tracker
"tin" một calibration sai. Có 8 mức yaw và 3 mức pitch.

**Cách đo.**

- **Tỉ lệ update** = số update camera trên track confirmed / số lượt track confirmed nằm trong
FOV camera.
- **γ, d² "nearest"** tính trên cặp đo gần nhất (theo d²) của mỗi track confirmed trong FOV,
**trước** cổng, để thấy cả những cặp bị cổng loại.


| Lệch extrinsic          | RMSE (m) | ΔRMSE so với fused (m) | RMSE y (m) | RMSE z (m) | tỉ lệ update camera | mean γu (px) | mean γv (px) | median d² | % cặp d² ≥ 10.6 |
| ----------------------- | -------- | ---------------------- | ---------- | ---------- | ------------------- | ------------ | ------------ | --------- | --------------- |
| LiDAR-only (tham chiếu) | 0.150    | –                      | 0.079      | 0.068      | –                   | –            | –            | –         | –               |
| 0 (calibration đúng)    | 0.136    | +0.000                 | 0.062      | 0.077      | 99%                 | −9.5         | +8.5         | 1.6       | 1%              |
| yaw −2°                 | 0.176    | +0.040                 | 0.102      | 0.094      | 2%                  | −61.6        | −8.5         | 66.6      | 98%             |
| yaw −1°                 | 0.173    | +0.037                 | 0.113      | 0.073      | 3%                  | −42.1        | +5.8         | 23.4      | 97%             |
| yaw −0.5°               | 0.211    | +0.075                 | 0.169      | 0.074      | 62%                 | −27.8        | +8.8         | 9.1       | 38%             |
| yaw +0.25°              | 0.146    | +0.011                 | 0.075      | 0.076      | 99%                 | −0.4         | +8.4         | 0.9       | 1%              |
| yaw +0.5°               | 0.194    | +0.058                 | 0.142      | 0.076      | 98%                 | +8.5         | +8.3         | 2.0       | 2%              |
| yaw +1°                 | 0.256    | +0.120                 | 0.210      | 0.083      | 59%                 | +27.0        | +8.2         | 8.3       | 41%             |
| yaw +2°                 | 0.155    | +0.019                 | 0.079      | 0.078      | 2%                  | +35.5        | −8.2         | 42.3      | 98%             |
| yaw +5°                 | 0.154    | +0.018                 | 0.081      | 0.074      | 0%                  | +69.9        | −16.3        | 68.0      | 100%            |
| pitch +0.5°             | 0.182    | +0.046                 | 0.064      | 0.139      | 99%                 | −9.3         | −9.3         | 2.9       | 1%              |
| pitch +1°               | 0.214    | +0.078                 | 0.086      | 0.167      | 44%                 | −9.2         | −27.5        | 11.4      | 56%             |
| pitch +2°               | 0.212    | +0.076                 | 0.079      | 0.164      | 1%                  | −8.9         | −64.3        | 51.0      | 99%             |


Matches, ghosts và misses giữ nguyên 502 / 0 / 239 ở mọi biến thể, trừ yaw +5° (499 / 1 / 242).
Camera không quyết định vòng đời track, nên lệch calibration gần như chỉ thể hiện qua RMSE. Hình `student/bonus/fig4_calibration_sweep.png` vẽ
RMSE, tỉ lệ update và median d² theo góc lệch.

Cột ΔRMSE so với fused có calibration đúng. Ngưỡng nhất quán của rubric lại so fused với
**LiDAR-only** (≤ 0.05 m): chỉ yaw −0.5° (+0.061), yaw +1° (+0.106), pitch +1° (+0.063) và
pitch +2° (+0.062) vượt ngưỡng; yaw +0.5° (+0.043) vẫn trong ngưỡng.

**Nhận xét.**

1. **Triệu chứng trên innovation.** Khi lệch nhỏ (yaw −0.5° đến +1°, pitch tới +2°), mean γ dịch
 tuyến tính theo góc lệch, khoảng 36 px/°, khớp với f·π/180 = 36.4 px/° (f = 2083 px). Trong
 vùng này yaw chỉ dịch `u`, pitch chỉ dịch `v`, nên nhìn trung bình innovation theo từng trục
 là chẩn đoán được loại lệch. Ở yaw −1°, ±2°, +5°, chỉ số "nearest" bão hoà (15.9–32.6 px/°)
 và γv đổi dấu, có thể vì cặp gần nhất không còn là hộp của chính xe đó (chưa kiểm chứng).
2. **Gating chặn hay không chặn.** Cổng `d² < 10.6` xét **từng cặp** nên:
  - **không chặn** lệch ≤ 0.5° theo hướng bù bias: median d² chỉ 0.9–2.9, nhưng RMSE đã tăng
   tới +0.058 m so với fused đúng calibration (yaw +0.5°: 0.194 m, xấu hơn LiDAR-only 0.043 m,
   sát ngưỡng nhất quán 0.05 m);
  - **chặn một phần** ở mức lệch trung bình (38–56% cặp bị loại), và đây là vùng tệ nhất,
  vì các cặp lọt qua đều lệch cùng một hướng (yaw +1°: +0.106 m so với LiDAR-only, vượt
  ngưỡng nhất quán);
  - **chặn gần hết** ở mức lệch lớn (97–100%). Nhưng track mới có `P` lớn làm cổng rộng, nên vài
  update lệch vẫn lọt qua (35–62% số update còn lại rơi vào track chưa confirmed) và RMSE
  không trở về LiDAR-only (pitch +2°: 0.212 m).
3. **Bias γu ≈ −9.5 px không phải lỗi calibration.** Lệch yaw +0.25° đưa γu về ≈ −0.4 px nhưng
 RMSE lại tăng từ 0.136 lên 0.146 m (y: 0.062 → 0.075 m). Bias này nằm ở định nghĩa đo (tâm
 hộp 2D khác hình chiếu tâm 3D), không phải ở extrinsic. Vì vậy không nên "hiệu chỉnh"
 calibration chỉ để triệt tiêu trung bình innovation.
4. **Thực tế.** Lệch 0.5° là rất nhỏ với extrinsic thật, nhưng đã đủ làm fused xấu hơn
 LiDAR-only. Một hệ thật cần giám sát trung bình innovation và cổng chặt hơn cho track mới.

**Bonus 1 (CVAT): không làm.** Cần CVAT server hoặc tài khoản để import và chụp màn hình, và
`export_cvat.export_tracks_json` của platform chỉ là stub, không xuất định dạng CVAT import được.

## Khai báo sử dụng AI (bắt buộc)

Ghi rõ, kể cả khi không dùng ("Không dùng AI"). Xem [RULES.md](../RULES.md) mục 2.

- Công cụ đã dùng (ChatGPT, Copilot, Claude, …): Claude Code (mô hình Claude Opus 5.5 của Anthropic)
- Dùng cho phần nào (hàm, câu hỏi, debug):
  - đọc repo và các trang lab trên VLearn, lập kế hoạch làm bài;
  - cài môi trường, giải nén và kiểm tra dữ liệu;
  - viết code Part E–H: `kalman.py`, `camera_fusion.py`, `association.py`, `track_management.py`;
  - debug lỗi `float()` trên `np.matrix` 1×1 trong `mahalanobis_distance`;
  - chạy `fusion-run-lab`, viết script kiểm tra artifacts, soạn nháp báo cáo này;
  - bonus: viết `student/bonus/bonus_study.py` (trực quan hoá + quét lệch calibration), chạy
  script và soạn nháp mục Bonus;
  - rà soát lại toàn bài trước khi nộp: sửa các chỗ báo cáo không khớp số liệu (ngưỡng nhất
  quán so với LiDAR-only, khoảng tuyến tính của độ dịch innovation, mô tả T10 ở frame 116).
- Cách bạn đã kiểm tra lại (pytest, chạy Waymo, đối chiếu công thức):
  - `pytest student/tests -q` cho 128 passed, 0 failed, 0 xfailed;
  - đối chiếu công thức với `docs/HUONG_DAN_KY_THUAT.md`, các TODO trong stub và
  `platform/fusion_lab/tracking_params.py`;
  - chạy `fusion-run-lab --fusion compare --seed 0` trên frame 0–198;
  - kiểm tra artifacts bằng `fusion_lab.evaluation.validate_metrics_records` (398 record, bất
  biến đúng, detection giống nhau ở hai mode);
  - mọi số liệu trong báo cáo được lấy trực tiếp từ `metrics.json` và `grade_run.log`;
  - bonus: biến thể `lidar`/`fused` của script tái lập đúng artifacts chấm điểm; độ dịch
  innovation đo được (≈ 36 px/°) khớp với lý thuyết f·π/180; checksum artifacts không đổi sau
  khi chạy; bảng bonus được sinh trực tiếp từ `calibration_results.json`.

## Checklist nộp

- [x] **Part E–H** trong `workspace/` đã implement; `pytest student/tests -q` không còn `failed`/`xfailed`
- [x] Part A–D: không bắt buộc sửa (hoặc ghi chú nếu bạn đã sửa)
- [x] Lần chạy chấm điểm: `--fusion compare --seed 0`, `frame_start: 0`, `frame_end: 198`
- [x] Đã commit `student/artifacts/metrics*.json` và `student/artifacts/grade_run*.log` (không sửa tay)
- [x] Đã điền đủ file này, gồm khai báo AI
- [x] Không commit dữ liệu Waymo, weights, `paths.yaml`, API key
- [x] `python tools/check_submission.py` báo `KẾT QUẢ: SẴN SÀNG NỘP`
- [x] Đã push và nộp link repo + commit hash trên LMS ([hướng dẫn nộp](../SUBMISSION.md))

