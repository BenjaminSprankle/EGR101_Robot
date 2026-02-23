"""SPIKE Prime crane controller with latched manual directions + optional auto shuttle.

Manual (latched) controls:
- Button 1 toggles extend direction:
    0 = extend outward, 1 = retract inward.
- Button 2 toggles lift direction:
    0 = down/drop (gravity claw opens), 1 = up/lift (gravity claw closes).
- Motors run continuously in the selected direction (unless safety interlock is active
  or a direction-specific motion time limit has been reached).

Safety interlock:
- If BOTH buttons are pressed, all motors are stopped immediately.
- If BOTH are held for LONG_PRESS_SECONDS, AUTO mode toggles once and is consumed
  until both buttons are released.

Auto mode:
- Base shuttles between 0° (pick) and +90° (drop) with DWELL_SECONDS at each end.
- Extend/Lift continue from their latched direction states during dwell and rotation,
  except while BOTH buttons are held (safety interlock stop) and subject to limits.
"""

from spike import ForceSensor, Motor, PrimeHub
import time

# =========================
# Hardware mapping (edit for your build)
# =========================
BASE_ROTATE_PORT = "A"
EXTEND_PORT = "B"
LIFT_PORT = "C"
BUTTON1_PORT = "D"  # Force Sensor for extend direction toggle
BUTTON2_PORT = "E"  # Force Sensor for lift direction toggle

# =========================
# Tuning constants
# =========================
EXTEND_SPEED = 45
LIFT_UP_SPEED = 20
LIFT_DOWN_SPEED = 30
ROTATE_SPEED_MAX = 35
ROTATE_SPEED_MIN = 10
HEADING_TOLERANCE = 2.0
SLOWDOWN_BAND = 25.0
FORCE_THRESHOLD = 20
LONG_PRESS_SECONDS = 2.0
LOOP_DT_MS = 30
DWELL_SECONDS = 15.0

# Time-based motion limits (edit as needed)
EXTEND_OUT_MAX_MS = 5000   # max time allowed to run outward
EXTEND_IN_MAX_MS = 5000    # max time allowed to run inward
LIFT_DOWN_MAX_MS = 5000    # max time allowed to run down
LIFT_UP_MAX_MS = 5000      # max time allowed to run up

# =========================
# Devices
# =========================
hub = PrimeHub()
base_motor = Motor(BASE_ROTATE_PORT)
extend_motor = Motor(EXTEND_PORT)
lift_motor = Motor(LIFT_PORT)
button1_sensor = ForceSensor(BUTTON1_PORT)
button2_sensor = ForceSensor(BUTTON2_PORT)

# Fallback heading reference
base_motor.set_degrees_counted(0)

use_gyro = True
try:
    hub.motion_sensor.reset_yaw_angle()
except Exception:
    use_gyro = False

# =========================
# Global state
# =========================
# Latched motor direction states (CORE RULE)
extend_dir_state = 0  # 0=outward, 1=inward
lift_dir_state = 1    # 0=down/drop, 1=up/lift

# Button history for rising-edge detection
prev_btn1 = False
prev_btn2 = False

# Dual-button interlock / long press tracking
both_pressed_start_ms = None
long_press_consumed = False

# Auto state
auto_enabled = False
auto_state = "IDLE"  # IDLE, GO_PICK, DWELL_PICK, GO_DROP, DWELL_DROP
auto_dwell_started_ms = 0

# Extend per-direction elapsed runtime accounting
extend_out_elapsed_ms = 0
extend_in_elapsed_ms = 0
extend_last_update_ms = None

# Lift per-direction elapsed runtime accounting
lift_down_elapsed_ms = 0
lift_up_elapsed_ms = 0
lift_last_update_ms = None


def read_buttons():
    """Read button pressed state from button-type sensors.

    Force Sensor is treated as pressed when force > FORCE_THRESHOLD.
    """

    def is_pressed(sensor):
        if hasattr(sensor, "get_force"):
            try:
                return sensor.get_force() > FORCE_THRESHOLD
            except Exception:
                return False
        if hasattr(sensor, "is_pressed"):
            try:
                return sensor.is_pressed()
            except Exception:
                return False
        return False

    return is_pressed(button1_sensor), is_pressed(button2_sensor)


def update_extend_direction_on_press(btn1, both_pressed):
    """Toggle extend direction on Button 1 rising edge.

    Ignored while BOTH buttons are held (safety / long-press context).
    Resets elapsed timer for the NEW direction so it can run.
    """
    global prev_btn1, extend_dir_state
    global extend_out_elapsed_ms, extend_in_elapsed_ms, extend_last_update_ms

    rising_edge = btn1 and (not prev_btn1)
    if rising_edge and (not both_pressed):
        extend_dir_state = 1 - extend_dir_state

        # Reset timer for NEW direction.
        if extend_dir_state == 0:
            extend_out_elapsed_ms = 0
        else:
            extend_in_elapsed_ms = 0

        # Reset dt reference to avoid stale dt jump.
        extend_last_update_ms = None

    prev_btn1 = btn1


def update_lift_direction_on_press(btn2, both_pressed):
    """Toggle lift direction on Button 2 rising edge.

    Ignored while BOTH buttons are held (safety / long-press context).
    Resets elapsed timer for the NEW direction so it can run.
    """
    global prev_btn2, lift_dir_state
    global lift_down_elapsed_ms, lift_up_elapsed_ms, lift_last_update_ms

    rising_edge = btn2 and (not prev_btn2)
    if rising_edge and (not both_pressed):
        lift_dir_state = 1 - lift_dir_state

        # Reset timer for NEW direction.
        if lift_dir_state == 0:
            lift_down_elapsed_ms = 0
        else:
            lift_up_elapsed_ms = 0

        # Reset dt reference to avoid stale dt jump.
        lift_last_update_ms = None

    prev_btn2 = btn2


def check_long_press_toggle(btn1, btn2, now_ms):
    """Handle simultaneous-button safety + long-press AUTO toggle.

    Returns True when BOTH are currently pressed (interlock active), else False.
    """
    global both_pressed_start_ms, long_press_consumed
    global auto_enabled, auto_state, auto_dwell_started_ms
    global extend_last_update_ms, lift_last_update_ms

    both_pressed = btn1 and btn2

    if both_pressed:
        # Immediate safety interlock on every loop while both are pressed.
        base_motor.stop()
        extend_motor.stop()
        lift_motor.stop()

        # Do not accumulate timers while interlock is active.
        extend_last_update_ms = None
        lift_last_update_ms = None

        if both_pressed_start_ms is None:
            both_pressed_start_ms = now_ms
            long_press_consumed = False

        held_ms = time.ticks_diff(now_ms, both_pressed_start_ms)
        if (not long_press_consumed) and (held_ms >= int(LONG_PRESS_SECONDS * 1000)):
            auto_enabled = not auto_enabled
            long_press_consumed = True

            if auto_enabled:
                auto_state = "GO_PICK"  # AUTO must start by going to pick heading first.
            else:
                auto_state = "IDLE"
                base_motor.stop()
                auto_dwell_started_ms = 0
    else:
        # Require full release before next long-press toggle.
        both_pressed_start_ms = None
        long_press_consumed = False

    return both_pressed


def service_extend_from_direction(now_ms):
    """Run extend motor from latched direction, constrained by per-direction time limits."""
    global extend_out_elapsed_ms, extend_in_elapsed_ms, extend_last_update_ms

    if extend_last_update_ms is None:
        extend_last_update_ms = now_ms

    dt_ms = time.ticks_diff(now_ms, extend_last_update_ms)
    if dt_ms < 0:
        dt_ms = 0

    if extend_dir_state == 0:
        # outward direction
        if extend_out_elapsed_ms >= EXTEND_OUT_MAX_MS:
            extend_motor.stop()
            extend_last_update_ms = now_ms
            return
        extend_out_elapsed_ms += dt_ms
        if extend_out_elapsed_ms >= EXTEND_OUT_MAX_MS:
            extend_motor.stop()
        else:
            extend_motor.start(abs(EXTEND_SPEED))
    else:
        # inward direction
        if extend_in_elapsed_ms >= EXTEND_IN_MAX_MS:
            extend_motor.stop()
            extend_last_update_ms = now_ms
            return
        extend_in_elapsed_ms += dt_ms
        if extend_in_elapsed_ms >= EXTEND_IN_MAX_MS:
            extend_motor.stop()
        else:
            extend_motor.start(-abs(EXTEND_SPEED))

    extend_last_update_ms = now_ms


def service_lift_from_direction(now_ms):
    """Run lift motor from latched direction, constrained by per-direction time limits.

    Gravity claw behavior:
    - DOWN/drop tends to open claw by gravity.
    - UP/lift tends to close claw by gravity.
    """
    global lift_down_elapsed_ms, lift_up_elapsed_ms, lift_last_update_ms

    if lift_last_update_ms is None:
        lift_last_update_ms = now_ms

    dt_ms = time.ticks_diff(now_ms, lift_last_update_ms)
    if dt_ms < 0:
        dt_ms = 0

    if lift_dir_state == 0:
        # down direction
        if lift_down_elapsed_ms >= LIFT_DOWN_MAX_MS:
            lift_motor.stop()
            lift_last_update_ms = now_ms
            return
        lift_down_elapsed_ms += dt_ms
        if lift_down_elapsed_ms >= LIFT_DOWN_MAX_MS:
            lift_motor.stop()
        else:
            lift_motor.start(-abs(LIFT_DOWN_SPEED))
    else:
        # up direction
        if lift_up_elapsed_ms >= LIFT_UP_MAX_MS:
            lift_motor.stop()
            lift_last_update_ms = now_ms
            return
        lift_up_elapsed_ms += dt_ms
        if lift_up_elapsed_ms >= LIFT_UP_MAX_MS:
            lift_motor.stop()
        else:
            lift_motor.start(abs(LIFT_UP_SPEED))

    lift_last_update_ms = now_ms


def _current_heading_deg():
    """Read heading from gyro yaw if available, else base motor degrees."""
    if use_gyro:
        try:
            return float(hub.motion_sensor.get_yaw_angle())
        except Exception:
            pass
    return float(base_motor.get_degrees_counted())


def _normalize_error(target, current):
    """Return shortest signed angular error in [-180, 180]."""
    err = target - current
    while err > 180:
        err -= 360
    while err < -180:
        err += 360
    return err


def rotate_to_heading(target_heading_deg):
    """Non-blocking rotate toward target heading. Returns True when at target."""
    current = _current_heading_deg()
    error = _normalize_error(target_heading_deg, current)
    mag = abs(error)

    if mag <= HEADING_TOLERANCE:
        base_motor.stop()
        return True

    if mag >= SLOWDOWN_BAND:
        speed = ROTATE_SPEED_MAX
    else:
        band = SLOWDOWN_BAND if SLOWDOWN_BAND > 0 else 1.0
        speed = ROTATE_SPEED_MIN + ((ROTATE_SPEED_MAX - ROTATE_SPEED_MIN) * (mag / band))

    base_motor.start(int(speed if error > 0 else -speed))
    return False


def run_auto_state_machine(now_ms):
    """Non-blocking automatic shuttle for base motor only."""
    global auto_state, auto_dwell_started_ms

    if not auto_enabled:
        return

    if auto_state == "GO_PICK":
        if rotate_to_heading(0.0):
            auto_state = "DWELL_PICK"
            auto_dwell_started_ms = now_ms

    elif auto_state == "DWELL_PICK":
        base_motor.stop()
        if time.ticks_diff(now_ms, auto_dwell_started_ms) >= int(DWELL_SECONDS * 1000):
            auto_state = "GO_DROP"

    elif auto_state == "GO_DROP":
        if rotate_to_heading(90.0):
            auto_state = "DWELL_DROP"
            auto_dwell_started_ms = now_ms

    elif auto_state == "DWELL_DROP":
        base_motor.stop()
        if time.ticks_diff(now_ms, auto_dwell_started_ms) >= int(DWELL_SECONDS * 1000):
            auto_state = "GO_PICK"

    else:
        auto_state = "GO_PICK"


def main():
    """Fast control loop (20-50 ms). Default: MANUAL ONLY (auto off)."""
    global auto_enabled, auto_state

    auto_enabled = False
    auto_state = "IDLE"

    while True:
        now_ms = time.ticks_ms()

        btn1, btn2 = read_buttons()
        both_pressed = check_long_press_toggle(btn1, btn2, now_ms)

        # Rising-edge direction updates (suppressed during dual-button hold).
        update_extend_direction_on_press(btn1, both_pressed)
        update_lift_direction_on_press(btn2, both_pressed)

        # Continuous extend/lift behavior unless safety interlock is active.
        if not both_pressed:
            service_extend_from_direction(now_ms)
            service_lift_from_direction(now_ms)
            run_auto_state_machine(now_ms)  # auto controls base only

        time.sleep_ms(LOOP_DT_MS)


main()
