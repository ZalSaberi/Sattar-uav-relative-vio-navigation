# گزارش جامع تحلیل ریشه‌ای خطای VIO/MSCKF در دیتاست MH_05

**پروژه:** UAV Relative VIO Navigation / MSCKF-based Visual-Inertial Odometry
**دیتاست مرجع:** EuRoC MH_05_difficult
**Offset اجرا:** 10s
**تاریخ گزارش:** 2026-06-11
**وضعیت گزارش:** تحلیل ریشه‌ای تا پایان Phase 6I؛ قبل از اعمال fix نهایی geometry gate

---

## 1. خلاصه مدیریتی

هدف این گزارش مستندسازی کامل مسیر دیباگ علمی خطای drift و jump در خروجی VIO/MSCKF پروژه است. در ابتدا چند فرضیه مطرح بود: ممکن بود مشکل از GUI و live dashboard باشد، ممکن بود sync یا اجرای زنده باعث بهم‌ریختگی شود، ممکن بود feature tracker outlier بدهد، ممکن بود covariance update در MSCKF ناپایدار باشد، یا ممکن بود observation noise بد تنظیم شده باشد.

برای جلوگیری از fixهای شانسی، مسیر بررسی مرحله‌ای انجام شد. هر مرحله فقط یک فرضیه را تست کرد. نتیجه‌ی نهایی تا این نقطه این است:

**Root cause اصلی drift اولیه در MH_05، ورود featureهای هندسی ضعیف به updateهای MSCKF است؛ مخصوصاً featureهایی که در مسیر `prune_cam_state_buffer` وارد update می‌شوند.**

این featureها از نظر residual و chi-square gating قابل قبول به نظر می‌رسند، اما از نظر هندسی برای ساختن update قابل اعتماد مناسب نیستند. مشخصات معمول آن‌ها این است:

```text
used_len پایین، معمولاً 2
baseline بسیار کوچک
parallax تقریباً صفر
depth نسبتاً زیاد
residual کوچک
accepted شدن توسط gating
```

این ترکیب باعث می‌شود MSCKF residual ظاهراً کوچک را به correction بزرگ در position تبدیل کند. بنابراین مشکل اصلی این نیست که featureها حتماً outlier تصویری واضح هستند؛ بلکه featureها از نظر geometry برای triangulation و MSCKF update ضعیف‌اند.

در بازه‌ی بد `5-10s`، که با drift اولیه ارتباط مستقیم دارد، featureهای پذیرفته‌شده این وضعیت را داشتند:

```text
accepted_features:        1570
used_len_median:          2.0
depth0_median_median:     10.961904 m
baseline_median:          0.021971 m
parallax_deg_median:      0.014845 deg
weak_parallax_pct:        99.235669 %
far_depth_pct:            67.834395 %
tiny_baseline_pct:        41.847134 %
```

در timestampهای کلیدی هم همین الگو دیده شد:

```text
6.75s:
accepted_features = 33
prune_count = 33
used_len_median = 2
parallax_deg_median = 0.012396 deg
far_depth_pct = 69.696970 %

8.55s:
accepted_features = 33
prune_count = 33
used_len_median = 2
parallax_deg_median = 0.020274 deg
tiny_baseline_pct = 100 %
far_depth_pct = 66.666667 %
```

بنابراین fix بعدی نباید guard کور روی بزرگی update باشد. fix درست باید **feature geometry gate** باشد؛ یعنی قبل از ورود feature به updateهای MSCKF، مخصوصاً در `prune_cam_state_buffer`، کیفیت هندسی feature از نظر parallax، baseline، depth و تعداد observation بررسی شود.

---

## 2. مسئله اولیه چه بود؟

در اجرای دیتاست `MH_05_difficult`، مسیر تخمینی VIO نسبت به ground truth drift و jump داشت. در نگاه اول ممکن بود تصور شود که مشکل فقط از نمایش زنده trajectory یا GUI سنگین است، اما لاگ‌های MSCKF نشان دادند که updateهای بزرگ واقعاً داخل فیلتر رخ می‌دهند.

بنابراین مسئله اصلی این بود:

```text
آیا trajectory فقط در GUI بد نمایش داده می‌شود؟
یا خود state فیلتر واقعاً updateهای بد دریافت می‌کند؟
```

با اجرای headless و بررسی فایل‌های خروجی مشخص شد که حتی بدون GUI نیز خطا باقی می‌ماند. بنابراین GUI علت اصلی نیست. از این نقطه به بعد تمرکز از UI و dashboard به خود pipeline الگوریتمی منتقل شد.

---

## 3. مسیر علمی دیباگ

برای اینکه بررسی‌ها قابل اعتماد باشند، مسیر به phaseهای کوچک تقسیم شد. هر phase یک فرضیه مشخص را تست کرد.

```text
Phase 6A: ساخت لاگ‌های تشخیصی برای image pipeline و MSCKF
Phase 6B: تست Joseph covariance update
Phase 6C: تست temporal RANSAC در feature tracker
Phase 6D: تمیز کردن output directory و اضافه شدن --output-dir
Phase 6F: بررسی conditioning / rank / gain در measurement_update
Phase 6G: تست row-space SVD برای حذف جهت‌های rank-deficient
Phase 6H: تست global observation noise scaling
Phase 6I: لاگ‌گیری feature geometry و تحلیل parallax/depth/baseline
```

تا این نقطه، Phase 6A، Phase 6F و Phase 6I مهم‌ترین phaseهای تشخیصی بودند. Phaseهای Joseph، RANSAC، SVD و observation noise scaling به عنوان fix نهایی پذیرفته نشدند، اما هرکدام کمک کردند یک فرضیه اشتباه یا ناقص را رد کنیم.

---

## 4. فایل‌ها و لاگ‌های اصلی استفاده‌شده

در این بررسی از چند دسته فایل استفاده شد.

### 4.1. فایل‌های trajectory evaluation

```text
mh05_se3_eval.txt
mh05_sim3_eval.txt
mh05_time_error_summary.txt
mh05_joseph_se3_eval.txt
mh05_ransac_se3_eval.txt
mh05_rowspace_svd_se3_eval.txt
```

این فایل‌ها برای بررسی ATE، RPE، SE3 alignment و Sim3 alignment استفاده شدند.

### 4.2. لاگ‌های Phase 6A

```text
image_pipeline.csv
msckf_frame.csv
msckf_update.csv
msckf_gating.csv
phase6a_runtime_log_summary.txt
```

این فایل‌ها نشان دادند که updateهای بزرگ در خود MSCKF رخ می‌دهند.

### 4.3. لاگ‌های Phase 6F

```text
msckf_conditioning.csv
phase6f_conditioning_summary.txt
```

این فایل‌ها برای بررسی rank، condition number، Kalman gain norm و نسبت delta به residual استفاده شدند.

### 4.4. لاگ‌های Phase 6G

```text
phase6g_conditioning_targets.txt
mh05_rowspace_svd_se3_eval.txt
```

این فایل‌ها نشان دادند که SVD عددی، correctionهای اصلی را کم نمی‌کند.

### 4.5. لاگ‌های Phase 6I

```text
feature_geometry.csv
phase6i_feature_geometry_summary.txt
phase6i_target_geometry_summary.csv
phase6i_interval_geometry_summary.csv
```

این فایل‌ها root cause اصلی را نشان دادند: featureهایی که وارد updateهای بد می‌شوند، هندسه ضعیف دارند.

---

## 5. نتیجه baseline و بررسی scale

ابتدا trajectory baseline روی MH_05 با SE3 و Sim3 بررسی شد.

نتیجه SE3 baseline:

```text
ATE RMSE:   0.365167 m
ATE mean:   0.338036 m
ATE max:    0.699268 m
RPE RMSE:   0.141016 m
```

نتیجه Sim3 baseline:

```text
Sim3 scale: 0.972082771
ATE RMSE:  0.303685 m
RPE RMSE:  0.135497 m
```

Sim3 بهتر شد، اما مشکل را کامل حذف نکرد. بنابراین scale error وجود دارد، ولی root cause اصلی نیست. اگر مشکل فقط scale بود، Sim3 باید بخش اصلی خطا را حذف می‌کرد. اما همچنان errorهای مهم باقی ماندند.

نتیجه این مرحله:

```text
مشکل فقط scale drift نیست.
```

---

## 6. بررسی GUI و headless run

برای رد کردن فرضیه GUI، یک run بدون dashboard و بدون live GUI گرفته شد. نتیجه نشان داد که خطا همچنان باقی است. بنابراین GUI علت اصلی drift نیست.

این نکته مهم است، چون در پروژه تغییرات زیادی روی dashboard، live trajectory و viewer انجام شده بود. اما شواهد نشان دادند که drift مورد بحث داخل MSCKF و مسیر feature update ساخته می‌شود، نه در UI.

نتیجه این مرحله:

```text
GUI مسیر نمایش است، نه root cause اصلی drift.
```

---

## 7. Phase 6A: لاگ‌های اولیه MSCKF

در Phase 6A لاگ‌های زیر اضافه شدند:

```text
image_pipeline.csv
msckf_frame.csv
msckf_update.csv
msckf_gating.csv
```

تعداد ردیف‌ها:

```text
image_pipeline: 2072
msckf_frame:    2052
msckf_update:   2478
msckf_gating:   67448
```

این لاگ‌ها نشان دادند که بعضی timestampها updateهای position بزرگ دارند. نمونه‌های مهم:

```text
59.30s: delta_position_norm = 0.359675 m
34.05s: delta_position_norm = 0.251107 m
28.45s: delta_position_norm = 0.226195 m
8.55s:  delta_position_norm = 0.215499 m
6.75s:  delta_position_norm = 0.209279 m
9.60s:  delta_position_norm = 0.175382 m
11.15s: delta_position_norm = 0.171611 m
```

در بازه `5-15s`، مقدار `delta_position_max` به حدود `0.215499m` رسید. این عدد برای یک update در MSCKF بزرگ است، مخصوصاً وقتی residual آن update کوچک باشد.

نکته مهم این بود که gating فعلی این updateها را معمولاً قبول می‌کرد. پس مشکل این نبود که residualها همیشه خیلی بزرگ‌اند و gating کار نمی‌کند. بلکه بعضی residualها کوچک بودند، اما update بد تولید می‌کردند.

---

## 8. Phase 6B: تست Joseph covariance update

در Phase 6B covariance update به فرم Joseph تغییر داده شد. از نظر ریاضی، Joseph form برای پایداری عددی covariance مناسب‌تر است، چون ساختار covariance را بهتر حفظ می‌کند.

نتیجه trajectory با Joseph:

```text
ATE RMSE:   0.411364 m
ATE mean:   0.374503 m
ATE max:    0.746588 m
RPE RMSE:   0.138620 m
```

این نتیجه از baseline بدتر بود. البته بعضی spikeهای موضعی کمی بهتر شدند، اما trajectory کلی MH_05 بهتر نشد.

نتیجه Phase 6B:

```text
Joseph update از نظر عددی درست‌تر است، اما root cause اصلی drift اولیه نیست.
```

---

## 9. Phase 6C: تست Temporal RANSAC

در Phase 6C، fake RANSAC قبلی در feature tracker با RANSAC واقعی بین frameهای متوالی cam0 جایگزین شد. هدف این بود که اگر drift از feature outlierهای تصویری ساده می‌آید، RANSAC آن‌ها را حذف کند.

نتیجه trajectory با RANSAC:

```text
ATE RMSE:   0.532088 m
ATE mean:   0.455886 m
ATE max:    1.470857 m
RPE RMSE:   0.140417 m
```

این نتیجه از baseline خیلی بدتر شد. همچنین در timestampهای خراب، RANSAC reject قابل توجهی نداشت.

نتیجه Phase 6C:

```text
مشکل اصلی outlier ساده‌ی 2D temporal نیست.
```

این نتیجه بعداً با Phase 6I کاملاً منطقی شد: featureهای مشکل‌ساز ممکن است در تصویر درست track شده باشند، اما از نظر هندسی برای update سه‌بعدی ضعیف باشند.

---

## 10. Phase 6D: تمیز کردن output directory

قبل از Phase 6D، خروجی trajectoryها ممکن بود در مسیرهای مشترک ذخیره شوند و نتیجه‌ها با هم قاطی شوند. برای همین `--output-dir` اضافه شد.

این تغییر الگوریتمی نبود، اما برای reproducibility ضروری بود. بعد از این مرحله هر phase خروجی جداگانه داشت:

```text
results/phase6a_diagnostics/...
results/phase6b_joseph/...
results/phase6c_ransac/...
results/phase6g_rowspace_svd/...
results/phase6i_feature_geometry/...
```

نتیجه Phase 6D:

```text
زیرساخت خروجی‌ها قابل اعتماد شد و مقایسه phaseها تمیزتر شد.
```

---

## 11. Phase 6F: بررسی conditioning داخل measurement_update

بعد از رد شدن Joseph و RANSAC، سؤال اصلی این شد:

```text
آیا updateهای بزرگ به خاطر residual بزرگ هستند؟
یا residual کوچک است ولی gain/covariance/geometry باعث correction بزرگ می‌شود؟
```

برای پاسخ، در `measurement_update` این موارد لاگ شدند:

```text
H_thin_rank
H_thin_cond
S_cond
K_norm
r_norm
r_thin_norm
delta_position_norm
delta_pos_over_r
```

نتایج Phase 6F بسیار مهم بودند.

در `6.75s`:

```text
context:              prune_cam_state_buffer
H_rows:               25
H_thin_rank:          8
H_thin_cond:          8.559831e+30
r_norm:               0.046896
delta_position_norm:  0.209333
delta_pos_over_r:     4.463740
K_norm:               12.821932
```

در `8.55s`:

```text
context:              prune_cam_state_buffer
H_rows:               30
H_thin_rank:          8
H_thin_cond:          9.987828e+34
r_norm:               0.036864
delta_position_norm:  0.215263
delta_pos_over_r:     5.839388
K_norm:               29.973478
```

در `9.60s`:

```text
context:              remove_lost_features
H_rows:               9
H_thin_rank:          6
H_thin_cond:          2.815180e+16
r_norm:               0.017753
delta_position_norm:  0.175829
delta_pos_over_r:     9.903986
K_norm:               31.909105
```

این یعنی در چند لحظه مهم، residual کوچک است اما correction بزرگ تولید می‌شود. بنابراین مشکل فقط residual بزرگ نیست. MSCKF در بعضی updateها، به علت geometry ضعیف یا gain بالا، residual کوچک را به جابه‌جایی بزرگ در state تبدیل می‌کند.

نتیجه Phase 6F:

```text
مسئله به measurement_update، conditioning، Kalman gain و geometry featureها مرتبط است.
```

---

## 12. Phase 6G: تست row-space SVD

بعد از Phase 6F، یک fix آزمایشی منطقی این بود که اگر `H_thin` rank-deficient است، قبل از ساختن `S` و `K`، جهت‌های ضعیف و وابسته با SVD حذف شوند.

نتیجه trajectory با row-space SVD:

```text
ATE RMSE:   0.362887 m
ATE mean:   0.330572 m
ATE max:    0.693240 m
RPE RMSE:   0.143043 m
```

این نسبت به baseline فقط کمی بهتر بود؛ حدود 0.002m در ATE RMSE. اما RPE بدتر شد. بنابراین Phase 6G fix واقعی محسوب نشد.

بررسی targetها نشان داد:

```text
6.75s:
H_thin_rows after SVD = 8
delta_position_norm = 0.209336

8.55s:
H_thin_rows after SVD = 8
delta_position_norm = 0.215012

9.60s:
H_thin_rows after SVD = 6
delta_position_norm = 0.176241
```

یعنی SVD واقعاً row-space را فشرده کرد، اما correctionهای بد اصلی تقریباً باقی ماندند.

نتیجه Phase 6G:

```text
مشکل فقط rank-deficiency عددی نیست. اطلاعات باقی‌مانده همچنان از نظر فیزیکی و هندسی ضعیف است.
```

---

## 13. Phase 6H: تست global observation noise scaling

فرضیه بعدی این بود که شاید فیلتر به همه‌ی visual measurementها بیش از حد اعتماد می‌کند. برای تست این فرضیه، یک env variable اضافه شد:

```text
MSCKF_OBS_NOISE_SCALE
```

با scale=4، variance اندازه‌گیری visual چهار برابر شد. یعنی فیلتر باید کمتر به visual update اعتماد کند.

نتیجه از نظر نمودار trajectory بدتر شد، مخصوصاً در ابتدای مسیر. بنابراین فرضیه‌ی «همه‌ی visual measurementها بیش از حد قوی‌اند» رد شد.

تفسیر:

```text
visual updateها برای کنترل IMU propagation و bias لازم‌اند.
مشکل همه‌ی visual measurementها نیستند.
مشکل یک subset خاص از featureهای هندسی ضعیف است.
```

پس global noise scaling هم fix نهایی نیست.

---

## 14. بررسی کد feature initialization و MSCKF update path

برای فهمیدن اینکه featureهای ضعیف چطور وارد MSCKF می‌شوند، این فایل‌ها بررسی شدند:

```text
src/feature/base_feature.py
src/feature/feature_position_initializer.py
src/feature/feature_motion_checker.py
src/feature/feature_depth_estimator.py
src/msckf.py
```

در `BaseFeature` فقط اطلاعات پایه نگهداری می‌شود:

```text
id
observations
position
is_initialized
optimization_config
```

هیچ فیلدی برای کیفیت هندسی feature وجود ندارد؛ مثل parallax، baseline، depth confidence یا disparity quality.

در `initialize_position` شرط‌های اصلی این‌ها هستند:

```text
measurement finite باشد
initial depth مثبت باشد
solution finite باشد
feature جلوی همه cameraها باشد
```

این‌ها لازم هستند، اما کافی نیستند. در این تابع شرط مستقیمی برای موارد زیر دیده نشد:

```text
حداقل parallax
حداقل baseline مؤثر
حداکثر depth قابل اعتماد
حداقل disparity
کیفیت نهایی triangulation
```

در `FeatureMotionChecker` فقط orthogonal translation بین اولین و آخرین observation بررسی می‌شود. این خوب است، اما تضمین نمی‌کند که observationهایی که در update prune استفاده می‌شوند geometry کافی دارند.

در `remove_lost_features` و `prune_cam_state_buffer` مسیر کلی این است:

```text
feature_jacobian
      ↓
gating_test
      ↓
measurement_update
```

چیزی که کم داریم:

```text
feature_jacobian
      ↓
feature_geometry_gate
      ↓
gating_test
      ↓
measurement_update
```

---

## 15. Phase 6I: لاگ‌گیری feature geometry

برای اثبات یا رد فرضیه feature geometry، قبل از gating برای هر feature اطلاعات هندسی لاگ شد:

```text
feature_id
track_len
used_len
is_initialized
position_norm
depth0_min
depth0_median
depth0_max
all_baseline
used_baseline
all_parallax_deg
used_parallax_deg
stereo_disparity_min
stereo_disparity_median
stereo_disparity_max
gate_accepted
context
r_norm
```

فایل اصلی:

```text
results/phase6i_feature_geometry/mh05/runtime_logs/feature_geometry.csv
```

حجم فایل حدود 20MB بود و ستون‌ها درست ساخته شدند. همین فایل نشان داد که featureهایی که gating قبول می‌کند، در ابتدای مسیر اغلب geometry ضعیف دارند.

---

## 16. نتیجه عددی Phase 6I در timestampهای مشکوک

### 16.1. timestamp = 4.75s

```text
accepted_features:        35
remove_lost_count:        3
prune_count:              32
track_len_median:         7
used_len_median:          2
depth0_median_median:     10.935186 m
baseline_median:          0.019359 m
parallax_deg_median:      0.020626 deg
weak_parallax_pct:        97.142857 %
far_depth_pct:            65.714286 %
tiny_baseline_pct:        91.428571 %
```

تقریباً تمام featureهای پذیرفته‌شده parallax ضعیف دارند و بیشترشان از prune آمده‌اند.

### 16.2. timestamp = 6.75s

```text
accepted_features:        33
remove_lost_count:        0
prune_count:              33
track_len_median:         9
used_len_median:          2
depth0_median_median:     10.962239 m
depth0_median_max:        37.107303 m
baseline_median:          0.024648 m
parallax_deg_median:      0.012396 deg
weak_parallax_pct:        100.000000 %
far_depth_pct:            69.696970 %
```

این timestamp یکی از updateهای بزرگ اولیه بود. همه accepted featureها از `prune_cam_state_buffer` آمده‌اند و parallax median فقط 0.012 درجه است.

### 16.3. timestamp = 8.55s

```text
accepted_features:        33
remove_lost_count:        0
prune_count:              33
track_len_median:         10
used_len_median:          2
depth0_median_median:     10.947808 m
depth0_median_max:        37.125300 m
baseline_median:          0.018072 m
parallax_deg_median:      0.020274 deg
weak_parallax_pct:        100.000000 %
far_depth_pct:            66.666667 %
tiny_baseline_pct:        100.000000 %
```

این timestamp یکی از قوی‌ترین شواهد است: همه accepted featureها از prune آمده‌اند، همه weak parallax هستند و همه tiny baseline دارند.

### 16.4. timestamp = 11.15s

```text
accepted_features:        61
remove_lost_count:        15
prune_count:              46
track_len_median:         3
used_len_median:          2
depth0_median_median:     8.158469 m
baseline_median:          0.016777 m
parallax_deg_median:      0.061517 deg
weak_parallax_pct:        77.049180 %
far_depth_pct:            31.147541 %
tiny_baseline_pct:        75.409836 %
```

در این لحظه هم سهم prune زیاد است و used_len median هنوز 2 است.

---

## 17. مقایسه بازه‌های زمانی

### 17.1. بازه 0-5s

```text
accepted_features:        1500
track_len_median:         8
used_len_median:          2
depth0_median_median:     10.950890 m
baseline_median:          0.018511 m
parallax_deg_median:      0.025717 deg
weak_parallax_pct:        94.200000 %
far_depth_pct:            63.066667 %
tiny_baseline_pct:        72.400000 %
```

در همان ابتدای مسیر، featureها با geometry ضعیف وارد update می‌شوند.

### 17.2. بازه 5-10s

```text
accepted_features:        1570
track_len_median:         9
used_len_median:          2
depth0_median_median:     10.961904 m
baseline_median:          0.021971 m
parallax_deg_median:      0.014845 deg
weak_parallax_pct:        99.235669 %
far_depth_pct:            67.834395 %
tiny_baseline_pct:        41.847134 %
```

این بازه با updateهای 6.75 و 8.55 مرتبط است. تقریباً تمام featureهای پذیرفته‌شده weak parallax هستند.

### 17.3. بازه 30-40s

```text
accepted_features:        6456
track_len_median:         7
used_len_median:          2
depth0_median_median:     6.783130 m
baseline_median:          0.058828 m
parallax_deg_median:      0.386969 deg
weak_parallax_pct:        61.648079 %
far_depth_pct:            9.696406 %
tiny_baseline_pct:        0.000000 %
```

در این بازه هندسه بهتر است: baseline بیشتر، depth کمتر و parallax بیشتر است.

### 17.4. بازه 90-95s reference

```text
accepted_features:        4017
track_len_median:         8
used_len_median:          2
depth0_median_median:     7.429177 m
baseline_median:          0.041745 m
parallax_deg_median:      0.166914 deg
weak_parallax_pct:        93.552402 %
far_depth_pct:            30.395818 %
tiny_baseline_pct:        0.000000 %
```

اگرچه weak parallax هنوز زیاد است، tiny baseline تقریباً صفر است و baseline median بهتر از اول مسیر است. بنابراین صرف weak parallax کافی نیست؛ ترکیب `used_len=2`، parallax خیلی کم، baseline خیلی کم و depth بالا مشکل‌ساز است.

---

## 18. چرا gating فعلی کافی نیست؟

gating فعلی بر اساس residual و chi-square کار می‌کند. یعنی سؤال آن این است:

```text
آیا residual با covariance فعلی سازگار است؟
```

اما featureهای weak geometry می‌توانند residual کوچک داشته باشند. یک feature دور با parallax تقریباً صفر ممکن است projection residual کمی داشته باشد، اما عمق و triangulation آن قابل اعتماد نباشد.

به همین دلیل در 6.75s و 8.55s، residual کوچک بود ولی correction بزرگ شد:

```text
6.75s:
r_norm ≈ 0.0469
delta_position_norm ≈ 0.209m

8.55s:
r_norm ≈ 0.0369
delta_position_norm ≈ 0.215m
```

پس residual gating به تنهایی کافی نیست. قبل از gating یا همراه آن باید feature geometry بررسی شود.

---

## 19. چرا RANSAC مشکل را حل نکرد؟

RANSAC در Phase 6C روی geometry دوبعدی temporal کار می‌کرد. اما featureهای مشکل‌ساز لزوماً match اشتباه نیستند. آن‌ها ممکن است در تصویر درست track شده باشند، اما از نظر triangulation و MSCKF update ضعیف باشند.

تفاوت مهم:

```text
RANSAC می‌پرسد:
آیا match دوبعدی با حرکت frameها سازگار است؟

Geometry gate باید بپرسد:
آیا feature برای update سه‌بعدی و MSCKF اطلاعات قابل اعتماد دارد؟
```

این دو سؤال یکی نیستند. به همین دلیل RANSAC مشکل را حل نکرد و حتی trajectory را بدتر کرد.

---

## 20. چرا Joseph مشکل را حل نکرد؟

Joseph covariance update می‌تواند covariance را عددی‌تر و پایدارتر کند، اما featureهای هندسی ضعیف را حذف نمی‌کند. اگر measurement از نظر geometry ضعیف باشد، Joseph نمی‌تواند بفهمد آن feature parallax کافی ندارد.

پس Joseph ممکن است بعضی spikeهای covariance-related را کم کند، اما root cause هندسی را حل نمی‌کند.

---

## 21. چرا SVD مشکل را کامل حل نکرد؟

SVD در Phase 6G جهت‌های rank-deficient را حذف کرد. اما در 6.75s و 8.55s، با وجود کاهش `H_thin_rows`، مقدار `delta_position_norm` تقریباً همان باقی ماند.

این یعنی اطلاعات باقی‌مانده هنوز از نظر فیزیکی و هندسی مشکل داشت. SVD فقط وابستگی عددی را حذف می‌کند، نه اینکه تشخیص دهد feature با baseline دو سانتی‌متر و parallax 0.01 درجه برای update قابل اعتماد نیست.

---

## 22. چرا global observation noise scaling مشکل را بدتر کرد؟

در Phase 6H، observation noise همه visual measurementها با scale=4 افزایش داده شد. اگر مشکل این بود که فیلتر به همه visual measurementها بیش از حد اعتماد می‌کند، باید نتیجه بهتر می‌شد. اما نتیجه بدتر شد.

این یعنی visual measurementها در کل برای کنترل drift IMU لازم‌اند. مشکل همه measurementها نیستند؛ مشکل subset خاصی از featureهای weak geometry است.

پس fix درست نباید global noise scale باشد. باید selective gate یا adaptive weighting باشد.

---

## 23. جمع‌بندی فنی root cause

زنجیره علت و معلول به شکل زیر است:

```text
در ابتدای MH_05، حرکت مؤثر دوربین برای بعضی featureها کم است.
      ↓
featureها با used_len=2 و baseline خیلی کوچک وارد prune می‌شوند.
      ↓
parallax آن‌ها نزدیک صفر است و عمقشان نسبتاً زیاد است.
      ↓
triangulation / feature Jacobian از نظر هندسی ضعیف می‌شود.
      ↓
residual ممکن است کوچک بماند، بنابراین chi-square gating آن را قبول می‌کند.
      ↓
Kalman gain / ill-conditioned update همان residual کوچک را به correction بزرگ تبدیل می‌کند.
      ↓
MSCKF position update در 6.75s، 8.55s و 9.60s jump می‌سازد.
      ↓
trajectory اولیه دچار drift و offset می‌شود.
```

بنابراین root cause اصلی:

```text
ورود featureهای کم‌پارالاکس، دور، با baseline بسیار کوچک و used_len پایین به updateهای MSCKF، مخصوصاً در prune_cam_state_buffer.
```

---

## 24. وضعیت branchها تا این نقطه

### branchهای قابل نگه‌داری

```text
phase6d-output-dir-cleanup
```

این branch باید نگه داشته شود، چون `--output-dir` را اضافه کرد و reproducibility را بهتر کرد.

```text
phase6a-vio-diagnostics
phase6f-msckf-conditioning-diagnostics
phase6i-feature-geometry-diagnostics
```

این branchها diagnostic ارزشمند دارند و برای تحلیل‌های بعدی مفیدند.

### branchهایی که fix نهایی نیستند

```text
phase6b-msckf-joseph-update
phase6c-feature-ransac
phase6g-msckf-rowspace-svd-fix
phase6h-msckf-observation-noise-sweep
```

این‌ها به عنوان آزمایش علمی مفید بودند، اما نباید فعلاً به عنوان fix نهایی merge شوند.

---

## 25. پیشنهاد fix بعدی: Phase 6J

Phase 6J باید یک geometry gate هدفمند باشد. پیشنهاد فعلی این است که ابتدا فقط روی `prune_cam_state_buffer` اعمال شود، چون شواهد نشان می‌دهد drift اولیه عمدتاً از featureهای prune با `used_len=2` و parallax بسیار کم می‌آید.

### 25.1. اصل fix

قبل از اینکه feature وارد gating و سپس measurement_update شود، معیارهای هندسی بررسی شوند:

```text
used_len
used_baseline
used_parallax_deg
depth0_median
stereo_disparity_median
```

اگر feature هندسه ضعیف داشت، از update prune حذف شود.

### 25.2. چرا فقط prune؟

چون در timestampهای 6.75s و 8.55s همه accepted featureها از prune آمده‌اند:

```text
6.75s: prune_count = 33 از 33
8.55s: prune_count = 33 از 33
```

بنابراین gate را ابتدا فقط روی prune اعمال می‌کنیم تا ریسک حذف featureهای مفید در remove_lost_features کمتر شود.

### 25.3. پیشنهاد معیارهای اولیه برای تست

Thresholdهای دقیق باید با sweep کوچک تعیین شوند، اما بر اساس Phase 6I می‌توان یک gate اولیه آزمایشی تعریف کرد:

```text
reject feature in prune if:
    used_len <= 2
    AND used_parallax_deg < 0.05 deg
    AND depth0_median > 8 m
```

یا یک نسخه محافظه‌کارتر:

```text
reject feature in prune if:
    used_len <= 2
    AND used_baseline < 0.02 m
    AND used_parallax_deg < 0.1 deg
    AND depth0_median > 8 m
```

این thresholdها نباید بدون تست نهایی شوند. بهتر است با env flag قابل خاموش و روشن کردن باشند:

```text
MSCKF_PRUNE_GEOMETRY_GATE=1
MSCKF_PRUNE_MIN_PARALLAX_DEG=0.05
MSCKF_PRUNE_MIN_BASELINE=0.02
MSCKF_PRUNE_MAX_DEPTH_FOR_LOW_PARALLAX=8.0
```

### 25.4. معیار پذیرش Phase 6J

Phase 6J فقط وقتی fix قابل قبول است که این شرایط را داشته باشد:

```text
ATE RMSE کمتر از baseline 0.365167 شود.
max error کمتر یا برابر baseline شود.
spikeهای 6.75s و 8.55s کاهش پیدا کنند.
RPE RMSE بدتر از baseline نشود یا بدتر شدن آن ناچیز باشد.
feature count و update count به شکل غیرعادی collapse نکند.
```

---

## 26. ریسک‌های Phase 6J

geometry gate اگر بیش از حد سخت‌گیر باشد، ممکن است featureهای زیادی را حذف کند و فیلتر را بیشتر به IMU propagation وابسته کند. این همان مشکلی است که در Phase 6H با کاهش وزن global measurement دیدیم. بنابراین gate باید selective باشد، نه aggressive.

ریسک‌های اصلی:

```text
حذف بیش از حد featureهای visual
افزایش IMU drift در ابتدای مسیر
کاهش update frequency
بدتر شدن RPE
خراب شدن دیتاست‌های دیگر با motion متفاوت
```

برای کنترل این ریسک‌ها، Phase 6J باید با env flag و diagnostics اجرا شود. همچنین باید تعداد featureهای rejected توسط geometry gate لاگ شود.

---

## 27. پیشنهاد لاگ‌های Phase 6J

در Phase 6J باید علاوه بر اعمال gate، این موارد لاگ شوند:

```text
geometry_gate_enabled
geometry_gate_checked
geometry_gate_rejected
reject_reason
used_len
used_baseline
used_parallax_deg
depth0_median
stereo_disparity_median
context
feature_id
```

همچنین در summary باید برای timestampهای 4.75، 6.75، 8.55 و 11.15 تعداد featureهای حذف‌شده گزارش شود.

---

## 28. نتیجه نهایی تا پایان Phase 6I

تا اینجا چند فرضیه رد شد:

```text
GUI علت اصلی نیست.
Scale تنها علت اصلی نیست.
RANSAC temporal مشکل را حل نمی‌کند.
Joseph covariance update به تنهایی کافی نیست.
Row-space SVD به تنهایی کافی نیست.
Global observation noise scaling راه‌حل نیست.
```

و یک فرضیه با شواهد قوی تأیید شد:

```text
featureهای هندسی ضعیف، مخصوصاً در prune_cam_state_buffer، عامل اصلی updateهای مخرب اولیه هستند.
```

بنابراین مسیر تمیز بعدی:

```text
Phase 6J: targeted prune feature geometry gate
```

این fix باید به شکل کنترل‌شده، قابل خاموش/روشن، با لاگ کامل و فقط روی prune شروع شود. بعد از تست روی MH_05، باید روی دیتاست‌های دیگر هم ارزیابی شود تا مطمئن شویم fix بیش از حد مخصوص MH_05 نشده است.

---

## 29. وضعیت تصمیم‌ها

| مورد بررسی                   |                          نتیجه | وضعیت                  |
| ---------------------------- | -----------------------------: | ---------------------- |
| GUI / dashboard              |                  علت اصلی نیست | کنار گذاشته شد         |
| SE3 baseline                 |           ATE RMSE = 0.365167m | مرجع مقایسه            |
| Sim3 alignment               |               scale = 0.972083 | scale تنها علت نیست    |
| Joseph update                |           ATE RMSE = 0.411364m | رد به عنوان fix نهایی  |
| Temporal RANSAC              |           ATE RMSE = 0.532088m | رد                     |
| Conditioning diagnostics     | high gain / rank issue دیده شد | تشخیصی مهم             |
| Row-space SVD                |           ATE RMSE = 0.362887m | بهبود ناچیز، کافی نیست |
| Observation noise scale=4    |                        بدتر شد | رد                     |
| Feature geometry diagnostics |         root cause را نشان داد | مسیر اصلی fix          |

---

## 30. یادداشت مهم برای ادامه پروژه

از اینجا به بعد نباید چند fix همزمان زده شود. مسیر باید مرحله‌ای بماند:

```text
1. فقط geometry gate روی prune
2. تست MH_05
3. بررسی spikeهای 4.75، 6.75، 8.55، 11.15
4. بررسی ATE/RPE
5. بررسی تعداد featureهای حذف‌شده
6. اگر خوب بود، تست روی دیتاست دیگر
7. اگر بد بود، thresholdها را با sweep کوچک تنظیم کنیم
```

این پروژه الان از حالت حدس خارج شده و وارد فاز root-cause-driven fix شده است. مهم‌ترین دستاورد تا اینجا این است که دیگر نمی‌دانیم فقط «یک drift عجیب» داریم؛ اکنون می‌دانیم drift اولیه با چه نوع featureهایی و از کدام مسیر update ساخته می‌شود.

**نتیجه نهایی گزارش:**
MSCKF در MH_05 featureهایی را از مسیر `prune_cam_state_buffer` وارد update می‌کند که residual کوچک دارند اما هندسه triangulation بسیار ضعیف دارند. این featureها باعث updateهای high-gain و correctionهای بزرگ position می‌شوند. fix بعدی باید یک geometry gate هدفمند برای prune باشد، نه RANSAC، نه global noise tuning، نه guard کور روی delta.
