# Native IQS5xx / TPS43 driver

Use an I²C node with compatible `azoteq,iqs5xx-ptp`, `rdy-gpios`,
`rst-gpios`, and the board's address. `CONFIG_ZMK_TRACKPAD_IQS5XX` defaults
on for that node when PTP/I²C are enabled. Five contact slots are required.
The driver reads one 42-byte snapshot starting at 0x000F, identifies contacts by
hardware slot (not coordinate ordering), and never generates mouse buttons or
gestures. Active frames count as keyboard activity on both roles.

Set `x-resolution`/`y-resolution` (TPS43 defaults 2048/1792);
`switch-xy` swaps them in software. `invert-x`/`invert-y` reverse native
sensor axes before swapping. Match the resulting ranges on both halves and
measure the active physical surface for the host dimensions. Factory sensing,
palm and low-power settings are retained, while firmware gestures are disabled.

A dedicated preemptible workqueue isolates IRQ sampling and 50 ms active polling
from keyboard/USB system-workqueue stalls. Its stack defaults to 1536 bytes
(`CONFIG_ZMK_IQS5XX_WORKQUEUE_STACK_SIZE`). Sensor scan rates are unchanged.
Palms, resets, malformed samples and I²C failures mark previous contacts unconfident
before lifting; configuration retries without blocking key processing.
Window termination is checked, and controller resets trigger retuning; ordinary
communication recovery retains calibration.
Peripheral backpressure refreshes held observations so an offline lift is cached,
while pending contact/button/confidence changes and lifts/cancellations remain
barriers before slot reuse. This also preserves births during heartbeat contention.
Basic touchpad operation is confirmed on Toucan. Calibration, other controller
revisions and power behavior still need hardware checks.

## Power management

Like the vendor firmware's enabled deep-sleep management, Zephyr device PM
suspends the sensor for ZMK deep sleep and resumes it if sleep is aborted.
Sampling/IRQs stop before the suspend command; live contacts are cancelled
before lifting. Backpressure or I²C errors veto suspension rather than silently
leaving a live contact or an unsuspended sensor. Resume uses I²C wake, restores
configuration and waits for fresh RDY data instead of replaying retained samples.
A failed resume stays quiescent until the PM caller retries (a cold boot resets
and initializes the sensor normally).

Ordinary ZMK idle retains automatic low-power scanning, including the vendor's
640 ms LP2 interval. The vendor's default-off `idle-sleep` option and runtime PM
are not implemented. Full suspend has no touch-to-wake; keyboard wake/reset is
unchanged. Battery current and wake latency require physical validation.

The implementation and decoder are `iqs5xx.c` and `iqs5xx.h`; the binding is
[`azoteq,iqs5xx-ptp.yaml`](../../dts/bindings/input/azoteq,iqs5xx-ptp.yaml). They are built by this repository's
Zephyr module and depend only on the fork's public `zmk/ptp.h` producer API.
The driver's `CMakeLists.txt` and `Kconfig` live alongside the implementation;
the root CMake file includes this directory, and `zephyr/module.yml` points
directly to its Kconfig.

## Checks

From the repository root after setup:

```sh
cd .build
ZEPHYR_TOOLCHAIN_VARIANT=host uv run west build ../tests/iqs5xx \
  -d build/iqs5xx-tests -b native_sim/native/64
build/iqs5xx-tests/zephyr/zephyr.exe
```

The tests cover contact decoding, IRQ scheduling, split forwarding, fault recovery
and device-PM suspend/resume using emulated I²C/GPIO. They also check sampling
during a stalled system workqueue and retained births under backpressure.
The `no-pm.conf` variant also checks operation with device PM disabled.
They default to the pinned `.build/zmk` checkout. For development, add
`-DZMK_SOURCE="$PWD/../zmk"` after `--` in the build command.

The fork's generic core/split suites remain in `zmk/app/tests/ptp`.
These tests do not measure radio throughput, power consumption or host sleep/wake
behavior.
