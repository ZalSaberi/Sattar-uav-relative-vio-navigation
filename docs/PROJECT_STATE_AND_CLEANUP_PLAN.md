# Project State and Root Cleanup Plan

تاریخ تهیه: 2026-06-12
شاخه کاری فعلی: `phase9c-live-preview-sync`
مسیر فعلی پروژه: `J:\Sattar Run`

## 1. هدف این سند

این سند برای ثبت وضعیت فعلی پروژه، جمع‌بندی کارهای انجام‌شده، مشخص کردن مسیر درست ادامه کار، و تمیزسازی ریشه پروژه نوشته شده است. در این مرحله هدف اصلی پروژه ساخت EXE یا بسته نرم‌افزاری نیست. هدف فعلی این است که dashboard از سورس اجرا شود و نمایش تصویر دوربین، خروجی پردازش، و live 3D trajectory با یکدیگر هماهنگ باشند.

## 2. وضعیت کلی پروژه

پروژه مربوط به ناوبری نسبی پهپاد بدون GPS با روش VIO/MSCKF است. ساختار کلی پروژه شامل pipeline اصلی VIO، ماژول‌های MSCKF، ابزارهای ارزیابی trajectory، dashboard گرافیکی، نمایش live trajectory و گزارش‌های تحلیلی است.

در فازهای قبلی ابزارهای ارزیابی مسیر، محاسبه معیارهای ATE/RPE، dashboard گرافیکی، نمایش سه‌بعدی trajectory، و قابلیت warm-up skip اضافه شدند. سپس تمرکز به سمت مشاهده زنده پردازش رفت؛ یعنی هنگام اجرای VIO از داخل dashboard، تصویر دوربین و مسیر تخمینی باید هم‌زمان جلو بروند.

## 3. شاخه‌ها و وضعیت Git

شاخه اصلی فعلی برای کار روی sync زنده:

```text
phase9c-live-preview-sync
```

شاخه‌ای که برای بسته‌بندی EXE ساخته شد ولی فعلاً کنار گذاشته شده:

```text
release/windows-dashboard-package
```

آخرین وضعیت مشاهده‌شده قبل از cleanup:

```text
branch: phase9c-live-preview-sync
modified: tools/evaluation_dashboard.py
untracked: docs/PHASE6_ROOT_CAUSE_ANALYSIS_REPORT.md
```

فایل `tools/evaluation_dashboard.py` شامل تغییرات مربوط به live sync است و قبل از هر تمیزکاری باید commit یا stash شود. فایل `docs/PHASE6_ROOT_CAUSE_ANALYSIS_REPORT.md` هنوز untracked است و باید بعداً تصمیم گرفته شود که وارد مستندات رسمی پروژه شود یا فقط به‌عنوان یادداشت محلی باقی بماند.

## 4. کارهای انجام‌شده تا این مرحله

در مرحله evaluation، ابزارهایی برای بررسی trajectory و مقایسه مسیر تخمینی با ground truth اضافه شد. سپس dashboard برای اجرای راحت‌تر evaluation، مشاهده نتایج، و بررسی مسیرها توسعه پیدا کرد.

در مرحله warm-up skip، قابلیت حذف چند ثانیه ابتدایی از evaluation اضافه شد. این کار به معنی اصلاح الگوریتم VIO نیست، بلکه فقط اجازه می‌دهد اثر initialization ابتدای مسیر جداگانه تحلیل شود.

در مرحله live visualization، تلاش شد dashboard در لحظه تصویر دوربین، console output و trajectory سه‌بعدی را نشان دهد. این بخش چند بار تغییر کرد، چون ابتدا تصویر و trajectory با clockهای متفاوت جلو می‌رفتند. نهایتاً تصمیم فنی این شد که sync باید بر اساس timestamp باشد، نه بر اساس frame counter یا pose count.

در مرحله packaging، تلاش شد dashboard به EXE تبدیل شود. این مسیر فعلاً متوقف شد، چون باز شدن GUI به‌تنهایی کافی نبود. برای کار کردن `Run Selected` در حالت EXE باید worker جداگانه VIO هم package شود و dashboard در حالت frozen به جای `python main.py` آن worker را اجرا کند. چون هدف فعلی sync است، packaging فعلاً از مسیر خارج شد.

## 5. فایل‌ها و پوشه‌هایی که باید نگه داشته شوند

این موارد بخشی از پروژه هستند و نباید در cleanup حذف شوند:

```text
.git/
.venv/                  تا وقتی در همین مسیر کار می‌کنیم
src/
tools/
configs/
docs/
main.py
requirements.txt
README.md یا فایل‌های اصلی پروژه
results/                اگر خروجی‌های ارزیابی مهم داخل آن است
datasets/               اگر datasetها داخل همین پروژه نگهداری می‌شوند
```

اگر پروژه قرار است تمیزتر شود، بهتر است `datasets/` و بعضی خروجی‌های سنگین `results/` در مسیر جداگانه‌ای خارج از root پروژه نگهداری شوند. اما حذف مستقیم آن‌ها توصیه نمی‌شود.

## 6. فایل‌ها و پوشه‌هایی که معمولاً لازم نیستند

این موارد خروجی build، cache، یا اثر آزمایش EXE هستند و معمولاً می‌توانند حذف شوند:

```text
build/
dist/
dist_installer/
packaging/                          اگر فقط برای آزمایش EXE ساخته شده
UAVAirvisionDashboard.spec
UAVAirvisionVIO.spec
tools/dashboard_release_entry.py     اگر فقط برای آزمایش EXE ساخته شده
__pycache__/
.pytest_cache/
.mypy_cache/
.ruff_cache/
*.pyc
*.pyo
*.tmp
```

این موارد نباید روی سورس اصلی پروژه اثر بگذارند، چون یا cache هستند یا خروجی تولیدی.

## 7. روش cleanup پیشنهادی

قبل از حذف، باید dry-run گرفته شود:

```bash
git status -sb
git clean -ndX
```

بعد فقط موارد generated حذف شوند:

```bash
rm -rf build
rm -rf dist
rm -rf dist_installer
rm -rf packaging

rm -f UAVAirvisionDashboard.spec
rm -f UAVAirvisionVIO.spec
rm -f tools/dashboard_release_entry.py

find . -type d -name "__pycache__" -prune -exec rm -rf {} +
find . -type d -name ".pytest_cache" -prune -exec rm -rf {} +
find . -type d -name ".mypy_cache" -prune -exec rm -rf {} +
find . -type d -name ".ruff_cache" -prune -exec rm -rf {} +

find . -type f -name "*.pyc" -delete
find . -type f -name "*.pyo" -delete
find . -type f -name "*.tmp" -delete
```

بعد از cleanup باید وضعیت Git بررسی شود:

```bash
git status -sb
git status --ignored -sb
```

## 8. انتقال پروژه به مسیر جدید

برای انتقال پروژه به یک پوشه دیگر، نباید cut/paste مستقیم انجام شود. روش امن این است که اول copy کامل گرفته شود، مسیر جدید تست شود، و بعد مسیر قدیمی به backup تغییر نام داده شود.

در PowerShell:

```powershell
$SRC = "J:\Sattar Run"
$DST = "D:\Projects\Sattar Run"

New-Item -ItemType Directory -Force -Path $DST | Out-Null

robocopy $SRC $DST /E /COPY:DAT /DCOPY:DAT /R:2 /W:2 /XD ".venv" "build" "dist" "dist_installer" "__pycache__" ".pytest_cache" /XF "*.pyc"
```

در مسیر جدید باید `.venv` تازه ساخته شود:

```bash
cd "/d/Projects/Sattar Run"

python -m venv .venv
source .venv/Scripts/activate

python -m pip install -U pip
python -m pip install -r requirements.txt
python -m pip install PyQt5 pyqtgraph PyOpenGL matplotlib reportlab pillow
```

سپس پروژه باید تست شود:

```bash
python -m py_compile tools/evaluation_dashboard.py src/msckf.py

DASHBOARD_SYNC_DEBUG=1 python tools/evaluation_dashboard.py \
  --datasets-root ./datasets \
  --results-root ./results \
  --warmup-skip-seconds 20
```

## 9. نتیجه عملی

فعلاً مسیر درست پروژه این است:

```text
اجرای سورس از داخل virtual environment
تمرکز روی sync تصویر و trajectory
عدم ادامه packaging تا پایدار شدن GUI
پاک‌سازی فایل‌های generated و cache
ثبت وضعیت پروژه در docs
```

بعد از اینکه live sync با چشم و با لاگ `[SYNC]` تأیید شد، تغییرات باید commit و push شوند. تا قبل از آن نباید مسیر EXE یا packaging دوباره باز شود.
