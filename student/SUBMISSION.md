# Báo cáo bài nộp — Day 23 Sensor Fusion Lab

> Điền file này rồi commit. Cách nộp: [hướng dẫn nộp](../SUBMISSION.md).

## Thông tin học viên

- Họ tên: Nguyen Van Uoc
- MSSV: 2A202602445
- Email: nuoc2694@gmail.com
- Link repo (fork): https://github.com/nvuoc/K4-Track4-Day23-NguyenVanUoc-2A202602445-Sensor-Fusion-Student
- Commit hash nộp (`git rev-parse HEAD`): HEAD

## Tóm tắt kết quả

- `fusion_mode` (bắt buộc `compare`), `frames`, `segment`, `seed`: `compare`, `[0, 198]`, `training_segment-1005081002024129653_5313_150_5333_150_with_camera_labels.tfrecord`, `0`
- `detection.precision`, `detection.recall`, `detection.tp/fp/fn`: `0.9701`, `0.7004`, `519 / 16 / 222`
- `tracking.lidar.rmse`, `matches`, `sum_sq_err`, `ghost_track_frames`, `missed_gt_frames`, `mean_confirmed_tracks`: `0.1503 m` (0.150323), `502`, `11.3436 m²`, `0`, `239`, `2.5226`
- `tracking.fused.rmse`, `matches`, `sum_sq_err`, `ghost_track_frames`, `missed_gt_frames`, `mean_confirmed_tracks`: `0.1359 m` (0.135867), `502`, `9.2668 m²`, `0`, `239`, `2.5226`
- Giải thích khác biệt hai mode, đọc RMSE cùng số ghép và ghost/miss:
  1. **Số lượng track và gán ghép (matches, ghosts, misses)**: Cả hai chế độ đều đạt cùng số cặp ghép `matches = 502`, số track ảo `ghost_track_frames = 0`, số nhãn bỏ sót `missed_gt_frames = 239`, và số track confirmed trung bình `mean_confirmed_tracks = 2.5226`. Điều này hoàn toàn khớp với thiết kế kiến trúc **track-then-fuse**: chỉ có duy nhất cảm biến LiDAR quyết định việc khởi tạo track mới, tính điểm hit/miss để xác nhận hay xóa track. Camera FRONT không can thiệp vào vòng đời track, do đó số lượng track tồn tại và danh tính track ở cả hai mode là giống hệt nhau.
  2. **Độ chính xác vị trí (RMSE và sum_sq_err)**: Ở chế độ Fused, RMSE giảm từ `0.1503 m` xuống `0.1359 m`, tổng bình phương sai số `sum_sq_err` giảm từ `11.3436 m²` xuống `9.2668 m²`. Độ chênh lệch `rmse_fused - rmse_lidar = -0.0145 m <= 0.05 m`, thỏa mãn xuất sắc tiêu chí nhất quán (không làm tracking xấu đi). Đo lường 2D pixel từ camera FRONT đã cung cấp thêm thông tin phương vị, giúp EKF hiệu chỉnh lại vị trí trong không gian trạng thái và thu hẹp độ bất định covariance.
  3. **Chất lượng tracking tổng thể**: Cả hai chế độ đều đạt `precision_track = 502 / (502 + 0) = 1.0` (vượt ngưỡng 0.75), `coverage = 502 / 519 = 0.967` (vượt ngưỡng 0.70), và RMSE <= 0.45 m. Hệ số chất lượng đạt mức tối đa q = 1.0.
  4. **Giới hạn thực nghiệm**: Đo camera được lấy từ tâm ground-truth 2D camera FRONT cộng thêm nhiễu Gaussian ngẫu nhiên (`seed=0`), không chạy mô hình camera detector thực tế. Do đó, kết quả Fused cải thiện phản ánh sự hoạt động chính xác của bộ lọc EKF đa cảm biến chứ không chứng minh hiệu năng của một perception detector độc lập.

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

## Giải thích ngắn (Parts E–H — tự viết)

1. Khác biệt đo lidar 3D và camera 2D trong EKF (`z`, `R`)?
   - **Đo LiDAR 3D**: Đo trực tiếp tọa độ không gian 3D `(x, y, z)` theo đơn vị mét trong hệ tọa độ cảm biến/xe ($z \in \mathbb{R}^3$). Ma trận đo lường $H$ tuyến tính chiếu thẳng từ 6D state sang 3D position ($H = [I_{3\times 3}, 0_{3\times 3}]$). Covariance đo $R \in \mathbb{R}^{3\times 3}$ độc lập theo mét bình phương: $R = \text{diag}(\sigma_{\text{lidar\_x}}^2, \sigma_{\text{lidar\_y}}^2, \sigma_{\text{lidar\_z}}^2) = \text{diag}(0.1^2, 0.1^2, 0.1^2) = \text{diag}(0.01, 0.01, 0.01)\text{ m}^2$.
   - **Đo Camera 2D**: Đo tọa độ pixel ảnh 2D `(u, v)` trên mặt phẳng cảm biến ảnh ($z \in \mathbb{R}^2$). Hàm đo $h(x)$ phi tuyến tính theo mô hình camera pinhole ($u = c_i - f_i \frac{y_s}{x_s}, v = c_j - f_j \frac{z_s}{x_s}$ với $p_s = R_{\text{ext}} p + t_{\text{ext}}$), do đó EKF phải xấp xỉ tuyến tính bằng ma trận Jacobian $H = \frac{\partial h}{\partial x} \in \mathbb{R}^{2\times 6}$ đạo hàm theo quy tắc chuỗi. Covariance đo $R \in \mathbb{R}^{2\times 2}$ theo pixel bình phương: $R = \text{diag}(\sigma_{\text{cam\_i}}^2, \sigma_{\text{cam\_j}}^2) = \text{diag}(5.0^2, 5.0^2) = \text{diag}(25, 25)\text{ px}^2$.

2. Vì sao cần gating Mahalanobis trước khi gán?
   - Khoảng cách Euclid thông thường chỉ đo khoảng cách hình học đơn thuần mà bỏ qua độ bất định của cả track lẫn cảm biến (thể hiện qua covariance innovation $S = H P H^T + R$). Khoảng cách Mahalanobis bình phương $d^2 = \gamma^T S^{-1} \gamma$ chuẩn hóa phần dư innovation $\gamma = z - h(x)$ theo ma trận hiệp phương sai $S$. Nhờ đó, một đo lường nằm xa nhưng có phương sai lớn vẫn có thể có khoảng cách Mahalanobis hợp lý, và ngược lại.
   - Cổng chi-square gate ($\chi^2$) dựa trên phân phối $\chi^2$ với bậc tự do bằng chiều đo (3 cho LiDAR, 2 cho camera) và ngưỡng tin cậy (ví dụ $99.5\%$). Gating giúp loại bỏ sớm các đo lường ngoại lai (outliers/clutter) quá xa so với độ tin cậy của track trước khi chạy thuật toán gán Greedy. Điều này ngăn chặn việc gán nhầm làm sai lệch trạng thái và gây phân kỳ bộ lọc EKF, đồng thời giảm chi phí tính toán cho ma trận cost.

3. Pipeline là track-then-fuse hay fuse-then-track? Chỉ ra trên log `fusion-run-lab`.
   - Pipeline là **track-then-fuse**: Hệ thống duy trì một danh sách track tập trung duy nhất (`TrackManager.track_list`). Trong mỗi frame, bộ lọc EKF chỉ thực hiện dự báo (`KF.predict`) **một lần duy nhất**, sau đó thực hiện cập nhật nối tiếp theo từng modality: đầu tiên gán và update LiDAR (`AssocL`), sau đó gán và update camera FRONT (`AssocC`) để tinh chỉnh trạng thái cho cùng track đó. Đây không phải là fuse-then-track vì không gộp dữ liệu cảm biến thô (point cloud + camera image) trước khi phát hiện và theo dõi.
   - Trên file `run_lab.py` (dòng 199–209) và trong file log `grade_run.log`, ta thấy rõ: Trong mỗi frame Waymo, vòng lặp chạy `KF.predict(track)` một lần cho mọi track, kế đến gọi `assoc.associate_and_update(manager, observations, KF, lidar_sensor)`, rồi gọi tiếp `assoc.associate_and_update(manager, observations, KF, camera_sensor)`. Toàn bộ quá trình diễn ra trên cùng một đối tượng track, số lượng track confirmed và các ID track không bị nhân đôi.

4. Nếu camera lệch calibration, triệu chứng gì trên innovation/residual?
   - Khi camera bị lệch calibration (lệch extrinsic xoay/tịnh tiến hoặc lệch intrinsic tiêu cự/tâm ảnh), hàm chiếu pinhole $h(x)$ sẽ dự báo điểm pixel bị lệch có hệ thống so với vị trí pixel thực tế $z$.
   - **Triệu chứng cụ thể**:
     1. Innovation/residual $\gamma = z - h(x)$ sẽ có kỳ vọng khác không ($\mathbb{E}[\gamma] \neq 0$, xuất hiện sai số hệ thống / systematic bias) thay vì là nhiễu trắng chuẩn Gaussian có kỳ vọng bằng 0.
     2. Khoảng cách Mahalanobis $d^2 = \gamma^T S^{-1} \gamma$ tăng cao vượt ngưỡng cổng $\chi^2$, dẫn tới các đo camera hợp lệ bị từ chối hàng loạt (out-of-gate), làm giảm tỷ lệ matches của camera.
     3. Nếu một số đo bị lệch vẫn lọt qua cổng gating và được cập nhật vào EKF, Kalman gain $K$ sẽ kéo trạng thái vị trí 3D của track lệch theo hướng sai số calibration, làm tăng sai số vị trí (tăng RMSE) và thổi phồng ma trận covariance $P$, thậm chí có thể khiến $P[0, 0]$ hoặc $P[1, 1]$ vượt `max_P` làm track bị xóa nhầm.

5. Vì sao `associate_and_update(..., sensor)` cần sensor tường minh ở frame rỗng?
   Giải thích vì sao lidar quyết định score/init/delete còn camera chỉ EKF update.
   - Hàm `associate_and_update` luôn kết thúc bằng việc gọi `manager.manage_tracks(unassigned_tracks, unassigned_meas, sensor)`. Ở frame rỗng (không có measurement nào), `manage_tracks` vẫn phải chạy để cập nhật vòng đời track:
     + Nếu sensor là **LiDAR**: các track nằm trong FOV của LiDAR mà không có đo (miss) bắt buộc phải bị trừ điểm score ($-\frac{1}{\text{window}}$), và các track có điểm quá thấp hoặc $P$ vượt `max_P` phải bị xóa. Nếu không truyền `sensor` tường minh, manager không biết đây là frame rỗng của LiDAR để trừ điểm track.
     + Nếu sensor là **Camera**: `manage_tracks` kiểm tra `if sensor.name != "lidar": return`, đảm bảo frame rỗng của camera không làm trừ điểm score hay xóa bất kỳ track nào.
   - **Lý do LiDAR quyết định score/init/delete còn Camera chỉ EKF update**:
     LiDAR cung cấp trực tiếp vị trí không gian 3D và kích thước hình học bao quanh vật thể với độ tin cậy cao trên toàn bộ vùng quan sát BEV 360 độ. Ngược lại, camera trong bài lab chỉ có góc nhìn hẹp phía trước (FRONT) và đo lường 2D pixel không có thông tin chiều sâu trực tiếp (độ sâu được mô phỏng). Nếu cho phép camera can thiệp vào vòng đời track, các xe nằm ngoài tầm nhìn FRONT của camera sẽ bị trừ điểm và xóa nhầm, hoặc các phát hiện 2D có thể khởi tạo track 3D không chính xác. Do đó, LiDAR giữ vai trò quyết định vòng đời track, còn camera chỉ đóng vai trò tinh chỉnh trạng thái EKF khi xe nằm trong tầm nhìn.

6. Nêu điều kiện xác nhận, giữ confirmed sau miss, và điều kiện xóa track.
   - **Điều kiện xác nhận track (`confirmed`)**: Khi track mới khởi tạo (`score = 1/window = 1/6 \approx 0.1667`, state `"initialized"`), mỗi lần nhận được đo LiDAR phù hợp (hit), điểm score được cộng thêm $+1/\text{window}$ (tối đa 1.0). Khi `score > confirmed_threshold` (ngưỡng 0.8, tức cần ít nhất 5 hit liên tiếp), track được xác nhận và chuyển trạng thái thành `"confirmed"`.
   - **Giữ confirmed sau miss**: Khi một track đã ở trạng thái `"confirmed"` gặp một miss trong FOV LiDAR, điểm score chỉ bị trừ $-1/\text{window}$ (ví dụ từ 1.0 xuống $5/6 \approx 0.8333$). Do $0.8333 \ge \text{delete\_threshold} = 0.6$, track vẫn duy trì trạng thái `"confirmed"`, không bị hạ cấp về `"tentative"` hay bị xóa.
   - **Điều kiện xóa track (`should_delete_track`)**: Track bị xóa khi thỏa mãn bất kỳ điều kiện nào sau đây:
     1. Độ bất định phương ngang vượt ngưỡng: $P[0, 0] > \text{max\_P}$ hoặc $P[1, 1] > \text{max\_P}$ (với $\text{max\_P} = 3.0^2 = 9.0\text{ m}^2$).
     2. Track đã `"confirmed"` nhưng bị miss liên tiếp khiến điểm tụt xuống dưới ngưỡng xóa: $\text{score} < \text{delete\_threshold}$ (với $\text{delete\_threshold} = 0.6$).
     3. Track chưa confirmed (`"tentative"` hoặc `"initialized"`) có điểm $\text{score} \le 0$.

## Bonus (không bắt buộc)

- Không

## Khai báo sử dụng AI (bắt buộc)

- Công cụ đã dùng (ChatGPT, Copilot, Claude, …): Antigravity AI Assistant (Google DeepMind)
- Dùng cho phần nào (hàm, câu hỏi, debug): Hỗ trợ phân tích cấu trúc mã nguồn, đối chiếu công thức toán học EKF và các hàm trong Part E-H (`kalman.py`, `association.py`, `camera_fusion.py`, `track_management.py`), gỡ lỗi mã hóa UTF-8 trên Windows, và hỗ trợ soạn thảo báo cáo giải thích kỹ thuật.
- Cách bạn đã kiểm tra lại (pytest, chạy Waymo, đối chiếu công thức): Tự chạy kiểm tra toàn bộ 128 bài test bằng pytest (`pytest student/tests`), chạy pipeline Waymo hoàn chỉnh ở chế độ so sánh `compare` trên 199 frame (`fusion-run-lab --config student/config/paths.yaml --fusion compare --seed 0`), kiểm tra đối chiếu tính nhất quán giữa `metrics.json` và `grade_run.log`, và chạy script kiểm tra bài nộp `tools/check_submission.py`.

## Checklist nộp

- [x] **Part E–H** trong `workspace/` đã implement; `pytest student/tests -q` không còn `failed`/`xfailed`
- [x] Part A–D: không bắt buộc sửa (hoặc ghi chú nếu bạn đã sửa)
- [x] Lần chạy chấm điểm: `--fusion compare --seed 0`, `frame_start: 0`, `frame_end: 198`
- [x] Đã commit `student/artifacts/metrics*.json` và `student/artifacts/grade_run*.log` (không sửa tay)
- [x] Đã điền đủ file này, gồm khai báo AI
- [x] Không commit dữ liệu Waymo, weights, `paths.yaml`, API key
- [x] `python tools/check_submission.py` báo `KẾT QUẢ: SẴN SÀNG NỘP`
- [x] Đã push và nộp link repo + commit hash trên LMS ([hướng dẫn nộp](../SUBMISSION.md))
