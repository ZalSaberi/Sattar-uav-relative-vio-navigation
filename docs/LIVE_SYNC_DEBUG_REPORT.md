# Live Camera Preview and Live 3D Trajectory Sync Debug Report

تاریخ تهیه: 2026-06-12
شاخه کاری: `phase9c-live-preview-sync`
فایل‌های اصلی درگیر:

```text
tools/evaluation_dashboard.py
src/msckf.py
```

## 1. هدف دیباگ

هدف این دیباگ این بود که هنگام اجرای VIO از داخل dashboard، سه بخش زیر با یکدیگر هماهنگ باشند:

```text
1. تصویر زنده cam0
2. خروجی console مربوط به پردازش VIO
3. live 3D trajectory
```

رفتار مطلوب این است که هر pose جدید VIO با timestamp خودش وارد dashboard شود. همان timestamp باید برای انتخاب نزدیک‌ترین frame تصویر استفاده شود، و همان position باید به live trajectory اضافه شود.

## 2. علائم مشکل

در تست‌های مختلف چند رفتار نادرست دیده شد:

```text
تصویر دوربین با سرعت timer جلو می‌رفت ولی trajectory عقب می‌ماند.
Trajectory چند متر جلو می‌رفت ولی تصویر هنوز frameهای ابتدایی dataset را نشان می‌داد.
بعد از تمام شدن تصاویر، trajectory همچنان ادامه پیدا می‌کرد.
گاهی frame روی 1/2033 ثابت بود ولی pose count به حدود 204 رسیده بود.
```

نمونه وضعیت مشکل‌دار:

```text
Live 3D Trajectory | samples 204
Frame 1 / 2033
FPS cam0 / pose 204
```

این وضعیت نشان می‌داد که trajectory از stdout داده می‌گیرد، اما camera frame selector با همان pose sync نشده است.

## 3. مسیرهای بررسی‌شده

### 3.1 free-running preview timer

در ابتدا camera preview با timer مستقل dashboard جلو می‌رفت. این روش برای preview ساده قابل قبول است، اما برای run زنده VIO غلط است. چون سرعت پردازش VIO ثابت نیست و تصویر نباید مستقل از poseهای VIO جلو برود.

نتیجه:

```text
در حالت اجرای VIO، timer آزاد نباید منبع اصلی حرکت camera preview باشد.
```

### 3.2 متوقف کردن preview در زمان اجرای process

در یک مرحله، preview هنگام اجرای QProcess متوقف شد. این باعث شد تصویر دیگر از trajectory جلو نزند، اما مشکل جدید ایجاد کرد: تصویر می‌توانست ثابت بماند چون جایگزین timestamp-driven برای update تصویر وجود نداشت.

نتیجه:

```text
فقط متوقف کردن timer کافی نیست. باید تصویر با timestamp pose جلو برود.
```

### 3.3 استفاده از pose_count به عنوان frame index

یک فرض اشتباه این بود که pose count می‌تواند معادل frame index باشد. این فرض غلط است، چون نرخ انتشار pose با نرخ frameهای camera یکی نیست و initialization نیز می‌تواند offset ایجاد کند.

نتیجه:

```text
pose_count نباید برای انتخاب frame استفاده شود.
```

### 3.4 مسیر EXE و packaging

در میانه کار، مسیر تبدیل پروژه به EXE بررسی شد. این مسیر به sync ربط مستقیم نداشت و باعث پیچیدگی اضافه شد. GUI ممکن بود باز شود، اما `Run Selected` در حالت packaged کامل نبود، چون worker اصلی VIO جداگانه package نشده بود.

نتیجه:

```text
packaging فعلاً متوقف شد و تمرکز به اجرای سورس dashboard برگشت.
```

## 4. تشخیص اصلی

مشکل اصلی این بود که image preview و trajectory از دو منبع زمانی متفاوت جلو می‌رفتند. برای حل درست، باید یک event واحد از backend به dashboard برسد که هم timestamp داشته باشد و هم position.

فرمت event انتخاب‌شده:

```text
VIO_POSE timestamp=... position=[x y z]
```

این event باید از `src/msckf.py` چاپ شود و در `tools/evaluation_dashboard.py` parse شود.

## 5. تغییرات اعمال‌شده

### 5.1 اضافه شدن خروجی `VIO_POSE`

در `src/msckf.py` خروجی جدیدی اضافه شد:

```text
VIO_POSE timestamp=... position=[...]
```

این کار باعث شد dashboard به جای جمع کردن timestamp و position از چند خط جداگانه، یک event کامل و atomic داشته باشد.

### 5.2 اضافه شدن parser در dashboard

در `tools/evaluation_dashboard.py` parser زیر اضافه شد:

```python
r'\bVIO_POSE\b.*?timestamp=([^\s]+).*?position=\[([^\]]+)\]'
```

وقتی این parser یک pose را پیدا کند، باید دو کار انجام شود:

```text
1. update تصویر با همان timestamp
2. append کردن position به live trajectory
```

### 5.3 کنترل free-running preview

تابع `_advance_preview()` طوری تغییر کرد که در حالت عادی تصویر را جلو نبرد. free-run فقط برای debug و با متغیر محیطی فعال می‌شود:

```bash
DASHBOARD_FREE_RUN_PREVIEW=1 python tools/evaluation_dashboard.py ...
```

در اجرای عادی، تصویر باید فقط با eventهای VIO جلو برود.

### 5.4 انتخاب frame بر اساس timestamp

تابع `_update_camera_preview_for_timestamp()` مسئول شد که نزدیک‌ترین frame تصویر را به timestamp خروجی VIO پیدا کند.

برای datasetهای EuRoC، منبع قابل اعتماد timestamp تصویر، نام فایل تصویر است:

```text
mav0/cam0/data/1403636616713555574.png
```

این عدد timestamp نانوثانیه‌ای است و باید به ثانیه تبدیل شود.

### 5.5 اصلاح `timestamp_to_seconds`

یکی از خطاهای مهم، تبدیل اشتباه timestamp بود. timestampهای EuRoC ممکن است nanosecond باشند:

```text
1403636616713555574
```

اما خروجی live VIO ممکن است از قبل second باشد:

```text
1403636616.713555574
```

اگر timestamp ثانیه‌ای دوباره تقسیم بر `1e9` شود، تبدیل به عددی حدود `1.4` می‌شود. در این حالت dashboard فکر می‌کند pose قبل از اولین فریم camera است و camera preview روی frame 1 باقی می‌ماند.

منطق درست برای `timestamp_to_seconds` این است:

```python
if abs_value >= 1.0e17:
    return value * 1.0e-9

if abs_value >= 1.0e14:
    return value * 1.0e-6

if abs_value >= 1.0e11:
    return value * 1.0e-3

return value
```

بنابراین timestamp مطلق ثانیه‌ای EuRoC که حدود `1.4e9` است، بدون تغییر باقی می‌ماند.

## 6. شواهد فعلی در کد

در آخرین بررسی، این موارد در فایل‌ها دیده شدند:

```text
def timestamp_to_seconds(value)
r'\bVIO_POSE\b.*?timestamp=([^\s]+).*?position=\[([^\]]+)\]'
VIO_POSE timestamp=...
```

این یعنی تغییرات اصلی داخل کد وجود دارند. اما وجود کد کافی نیست؛ باید runtime ثابت کند که frame واقعاً جلو می‌رود.

## 7. روش تست runtime

اجرای تست:

```bash
cd "/j/Sattar Run"
source .venv/Scripts/activate

DASHBOARD_SYNC_DEBUG=1 python tools/evaluation_dashboard.py \
  --datasets-root ./datasets \
  --results-root ./results \
  --warmup-skip-seconds 20
```

بعد از زدن `Run Selected` باید در Console Preview خط‌هایی شبیه این دیده شود:

```text
[SYNC] target=1403636616.713556 range=[1403636606.713556, 1403636758.713556] frame=204/2033 pose=204
```

معنای فیلدها:

```text
target: timestamp خروجی VIO pose
range: بازه timestampهای تصویر cam0
frame: frame انتخاب‌شده برای نمایش
pose: تعداد poseهای اضافه‌شده به live trajectory
```

## 8. تفسیر خروجی‌های ممکن

اگر frame از 1 جلو رفت، sync به احتمال زیاد درست شده است.

اگر `[SYNC]` وجود داشت ولی frame روی 1 ماند، باید `target` و `range` بررسی شوند. اگر `target` عددی مثل `1.4036` بود، یعنی timestamp دوباره scale شده است. اگر `target` از `range` کوچک‌تر بود، احتمالاً offset ثابت بین timestampهای VIO و camera وجود دارد.

اگر اصلاً `[SYNC]` نیامد، یعنی یکی از این موارد رخ داده است:

```text
VIO_POSE از stdout چاپ نمی‌شود.
parser داخل _read_process_output اجرا نمی‌شود.
_update_camera_preview_for_timestamp قبل از رسیدن به debug return می‌زند.
process output توسط dashboard خوانده نمی‌شود.
```

در این حالت باید debug عمیق‌تر اضافه شود، نه patch حدسی.

## 9. نتیجه دیباگ

مشکل sync تصویر و trajectory از جنس UI timing و timestamp mapping است، نه الزاماً از جنس MSCKF math. این مسئله باید جدا از drift اولیه VIO بررسی شود.

نتیجه‌های فنی این فاز:

```text
برای live sync باید از timestamp استفاده شود، نه pose_count.
camera frame باید با نزدیک‌ترین timestamp تصویر انتخاب شود.
timestamp فایل‌های EuRoC منبع قابل اعتماد برای زمان تصویر است.
timer آزاد preview نباید در run زنده فعال باشد.
VIO باید event اتمیک timestamp + position چاپ کند.
timestamp ثانیه‌ای نباید دوباره تقسیم بر 1e9 شود.
packaging فعلاً از مسیر debug حذف شده است.
```

## 10. وضعیت نهایی مورد انتظار

پس از تکمیل fix، dashboard باید این رفتار را داشته باشد:

```text
Run Selected شروع می‌شود.
VIO_POSEها از stdout خوانده می‌شوند.
هر VIO_POSE یک frame نزدیک از cam0 را انتخاب می‌کند.
همان VIO_POSE یک نقطه جدید به live trajectory اضافه می‌کند.
تصویر و trajectory با هم جلو می‌روند.
بعد از پایان پردازش، تصویر و trajectory از هم جدا نمی‌شوند.
```

## 11. قدم بعدی

قدم بعدی فقط تست runtime با `DASHBOARD_SYNC_DEBUG=1` است. اگر frame جلو رفت، تغییرات باید commit شوند. اگر frame جلو نرفت، چند خط `[SYNC]` باید بررسی شود و patch بعدی فقط بر اساس همان داده نوشته شود.
