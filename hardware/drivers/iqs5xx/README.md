# Native IQS5xx / TPS43 driver

Use an I²C node with compatible `azoteq,iqs5xx-ptp`, `rdy-gpios`,
`rst-gpios`, and the board's address. `CONFIG_ZMK_TRACKPAD_IQS5XX` defaults
on for that node when PTP/I²C are enabled. Five contact slots are required.
The driver reads one 42-byte snapshot starting at 0x000F, identifies contacts by
hardware slot (not coordinate ordering), and never generates mouse buttons or
gestures. Active frames count as keyboard activity on both roles.

Set `x-resolution`/`y-resolution` (TPS43 defaults 2048/1792, matching the
IQS572's 8×256 and 7×256 inter-electrode interpolation spans);
`switch-xy` swaps them in software. `invert-x`/`invert-y` reverse native
sensor axes before swapping. Match the resulting ranges on both halves and
measure the active physical surface for the host dimensions.
`filter-settings` defaults to `0x0B`: the vendor-recommended
combination of 2-sample MAV and dynamic XY IIR, plus the ALP count filter.
IIR coefficients retain their stored defaults.
Factory sensing, palm and low-power settings are retained; gestures are disabled.

A dedicated preemptible workqueue isolates RDY-synchronized sampling from
keyboard/USB system-workqueue stalls. Its stack defaults to 1536 bytes
(`CONFIG_ZMK_IQS5XX_WORKQUEUE_STACK_SIZE`). RDY's rising edge captures monotonic
uptime ticks; every contact in that window retains that scan-completion timestamp
in wrapping 100 µs units through output retries, split forwarding and HID. The
13 ms active/idle-touch intervals are sensor targets, not synthesized timestamps.
A 50 ms active watchdog checks RDY's level without forcing sample reads; a missed
edge falls back to acquisition time because the original completion time is unknown.
Setup/recovery/PM can force I²C communication (including the documented wake NACK
retry), but failed sample reads are not retried against a different window.

Acquisition reads/closes each window independently of transport admission. A
four-entry output queue on the system workqueue preserves contact transitions and
coalesces adjacent motion. Output stalls cannot hold the hardware window open.
Overflow discards incomplete history, cancels admitted/attempted lifetimes with
unconfident contacts and a lift, then recovers from buffered fresh observations.
Palms, resets, malformed samples and I²C failures also cancel before lifting;
only a controller reset reruns configuration and requests tuning. An ordinary
read failure resumes at the next RDY window. Failed termination retries only the
close command every 50 ms and discards the retained sample, without rewriting
settings or requesting calibration. Setup failures retry after 250 ms. Event-mode stationary contacts do not manufacture fresh
scans: RDY-level checks alone cannot diagnose a sensor that silently stops
responding without raising RDY. Split heartbeats are transport leases, not scans.
Basic touchpad operation is confirmed on Toucan. Calibration, other controller
revisions and power behavior still need hardware checks.

## Diagnostics

`iqs5xx_get_diagnostics()` in `iqs5xx.h` returns cumulative sample, read/close
failure, malformed snapshot, reset, ATI-error/report-rate-miss, timestamp fallback,
and acquisition coalescing/overflow counts. It also returns the latest status
bytes and last/maximum observed gaps between consecutive active snapshots.
Event-mode stationary holds can produce long gaps; these are not measurements
of internal scan frequency. ATI-error/report-rate flags are counted per observed
sample, not per unique incident. They never trigger a host-driven retune loop.

With logging enabled, faults are logged on occurrences 1, 2, 4, 8, … and a
cumulative summary is emitted at most every five seconds of successful sampling.
Normal firmware does not enable logging. See the
[checks and hardware diagnostic procedure](../../../tests/iqs5xx/README.md).

## Power management

Like the vendor firmware's enabled deep-sleep management, Zephyr device PM
suspends the sensor for ZMK deep sleep and resumes it if sleep is aborted.
Sampling/IRQs stop before the suspend command; live contacts are cancelled
before lifting. Backpressure or I²C errors veto suspension rather than silently
leaving a live contact or an unsuspended sensor. Resume uses I²C wake, retains
configuration and waits for fresh RDY data instead of replaying retained samples.
The ASIC automatically reseeds on wake; the driver does not request extra tuning.
A failed resume stays quiescent until the PM caller retries (a cold boot resets
and initializes the sensor normally).

Ordinary ZMK idle retains automatic low-power scanning, including the vendor's
640 ms LP2 interval. The vendor's default-off `idle-sleep` option and runtime PM
are not implemented. Full suspend has no touch-to-wake; keyboard wake/reset is
unchanged. Battery current and wake latency require physical validation.

The implementation and decoder are `iqs5xx.c` and `iqs5xx.h`; the binding is
[`azoteq,iqs5xx-ptp.yaml`](../../dts/bindings/input/azoteq,iqs5xx-ptp.yaml). They are built by this repository's
Zephyr module and use the fork's public `zmk/ptp.h` producer API and bounded
`zmk/ptp/queue.h` queue helper.
The driver's `CMakeLists.txt` and `Kconfig` live alongside the implementation;
the root CMake file includes this directory, and `zephyr/module.yml` points
directly to its Kconfig.
