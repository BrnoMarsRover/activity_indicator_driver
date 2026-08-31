"""LED mast driver: shows who has control of the rover, with smooth transitions and a slow
settled pulse so the mast reads clearly from a distance.

State priority, highest first:
    autonomy            blue    - set via `autonomy_active` (Bool) or the mode service
    external control    red     - the teleop app is the multiplexer's active source
    local controller    yellow  - USB/BT gamepad present (gamepad_state != Disconnected)
    default             white   - nothing has control

Rendering runs in its own thread, not a ROS timer: a NeoPixel show() for 114 pixels over
I2C takes tens of milliseconds and must not block the executor. Colours are written with
auto_write disabled and one explicit show() per frame -- the previous code left auto_write
on, so party mode issued 114 I2C transactions per frame instead of one.

`set_activity_mode` still takes the original uint8 modes and now overrides the automatic
mapping until called with MODE_AUTO (255), which hands control back to the state machine.
"""

import math
import random
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from activity_indicator_msgs.srv import ActivityMode

import board
import busio
from rainbowio import colorwheel
from adafruit_seesaw import seesaw, neopixel

MODE_OFF = 0
MODE_DISCONNECTED = 1          # kept for compatibility: the "default" state
MODE_IDLE = 2
MODE_TELEOP = 3
MODE_AUTONOMOUS = 4
MODE_PARTY = 5
MODE_AUTO = 255                # clears a service override, back to automatic mapping

def _lerp(a, b, t):
    return a + (b - a) * t


def _ease(t):
    """Smoothstep: no visible kick at the start or overshoot at the end of a transition."""
    return t * t * (3.0 - 2.0 * t)


class ActivityIndicatorDriver(Node):
    def __init__(self):
        super().__init__('activity_indicator_driver')

        self.declare_parameter('color_default', [255, 255, 255])     # white
        self.declare_parameter('color_controller', [255, 190, 0])    # yellow
        self.declare_parameter('color_external', [255, 0, 0])        # red
        self.declare_parameter('color_autonomy', [0, 60, 255])       # blue
        self.declare_parameter('color_idle', [0, 255, 0])            # green (legacy MODE_IDLE)
        self.declare_parameter('brightness', 0.5)
        self.declare_parameter('num_pixels', 114)
        self.declare_parameter('neo_pin', 15)
        self.declare_parameter('seesaw_addr', 0x60)
        # Frames per second for the render thread. 114 pixels over I2C is the limit here, so
        # this is deliberately modest; raise it only if show() keeps up.
        self.declare_parameter('fps', 15.0)
        self.declare_parameter('transition_time', 0.6)
        # Settled pulse: brightness breathes between pulse_min and 1.0 of the state colour.
        self.declare_parameter('pulse_period', 3.0)
        self.declare_parameter('pulse_min', 0.35)
        self.declare_parameter('blank_on_exit', True)
        self.declare_parameter('active_source_topic', '/freya_1/chassis/active_source')
        self.declare_parameter('gamepad_state_topic', '/freya_1/chassis/gamepad_state')
        self.declare_parameter('autonomy_topic', '/freya_1/chassis/autonomy_active')
        # Token the twist multiplexer publishes for the operator-station source.
        self.declare_parameter('external_source_name', 'external_source')
        self.declare_parameter('gamepad_disconnected_state', 'Disconnected')

        gp = self.get_parameter
        self.colors = {
            MODE_DISCONNECTED: tuple(gp('color_default').value),
            MODE_IDLE: tuple(gp('color_idle').value),
            MODE_TELEOP: tuple(gp('color_external').value),
            MODE_AUTONOMOUS: tuple(gp('color_autonomy').value),
        }
        self.color_controller = tuple(gp('color_controller').value)
        self.fps = max(1.0, float(gp('fps').value))
        self.transition_time = max(0.0, float(gp('transition_time').value))
        self.pulse_period = max(0.1, float(gp('pulse_period').value))
        self.pulse_min = min(max(float(gp('pulse_min').value), 0.0), 1.0)
        self.blank_on_exit = bool(gp('blank_on_exit').value)
        self.external_source_name = gp('external_source_name').value
        self.gamepad_disconnected_state = gp('gamepad_disconnected_state').value

        self.num_pixels = int(gp('num_pixels').value)
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.ss = seesaw.Seesaw(self.i2c, addr=int(gp('seesaw_addr').value))
        self.pixels = neopixel.NeoPixel(
            self.ss, int(gp('neo_pin').value), self.num_pixels,
            brightness=float(gp('brightness').value), auto_write=False)

        # --- state -------------------------------------------------------------
        self._lock = threading.Lock()
        self._external_active = False
        self._controller_present = False
        self._autonomy_active = False
        self._override = None                  # service-forced mode, or None
        self._party = False
        self._displayed = (0.0, 0.0, 0.0)      # colour currently on the strip
        self._from = (0.0, 0.0, 0.0)
        self._to = self._resolve_color()
        self._transition_start = time.monotonic()
        self._running = threading.Event()
        self._running.set()

        self.srv = self.create_service(
            ActivityMode, '/freya_1/chassis/set_activity_mode', self.set_mode_callback)
        self.create_subscription(String, gp('active_source_topic').value,
                                self.active_source_callback, 1)
        self.create_subscription(String, gp('gamepad_state_topic').value,
                                self.gamepad_state_callback, 1)
        self.create_subscription(Bool, gp('autonomy_topic').value,
                                self.autonomy_callback, 1)

        self._render_thread = threading.Thread(target=self._render_loop, daemon=True)
        self._render_thread.start()
        self.get_logger().info(
            'Activity indicator started: default white / controller yellow / external red / '
            f'autonomy blue, {self.fps:.0f} fps, {self.transition_time:.2f}s fades, '
            f'{self.pulse_period:.1f}s pulse')

    # ------------------------------------------------------------------ state

    def _resolve_color(self):
        """Colour for the current state, honouring a service override."""
        mode = self._override
        if mode is None:
            if self._autonomy_active:
                return self.colors[MODE_AUTONOMOUS]
            if self._external_active:
                return self.colors[MODE_TELEOP]
            if self._controller_present:
                return self.color_controller
            return self.colors[MODE_DISCONNECTED]
        if mode == MODE_OFF:
            return (0, 0, 0)
        return self.colors.get(mode, self.colors[MODE_DISCONNECTED])

    def _retarget(self, reason):
        """Start a fade towards whatever the state now implies."""
        with self._lock:
            new_target = self._resolve_color()
            if new_target == self._to:
                return
            self._from = self._displayed
            self._to = new_target
            self._transition_start = time.monotonic()
        self.get_logger().info(f'LED mast -> RGB{new_target} ({reason})')

    # -------------------------------------------------------------- callbacks

    def active_source_callback(self, msg):
        active = (msg.data == self.external_source_name)
        if active != self._external_active:
            self._external_active = active
            self._retarget(f'active_source={msg.data}')

    def gamepad_state_callback(self, msg):
        present = (msg.data != self.gamepad_disconnected_state)
        if present != self._controller_present:
            self._controller_present = present
            self._retarget(f'gamepad_state={msg.data}')

    def autonomy_callback(self, msg):
        if bool(msg.data) != self._autonomy_active:
            self._autonomy_active = bool(msg.data)
            self._retarget(f'autonomy={self._autonomy_active}')

    def set_mode_callback(self, request, response):
        mode = int(request.mode)
        if mode == MODE_AUTO:
            self._override = None
            self._party = False
            self._retarget('override cleared, automatic mapping')
        elif mode == MODE_PARTY:
            self._override = MODE_PARTY
            self._party = True
            self.get_logger().info('LED mast -> party mode')
        else:
            self._override = mode
            self._party = False
            self._retarget(f'override mode={mode}')
        response.success = True
        return response

    # ------------------------------------------------------------------ render

    def _render_loop(self):
        period = 1.0 / self.fps
        party_offset = 0
        while self._running.is_set():
            started = time.monotonic()
            try:
                if self._party:
                    party_offset = self._render_party(party_offset)
                else:
                    self._render_state(started)
            except Exception as exc:                     # never let the strip kill the thread
                self.get_logger().warn(f'LED render error: {exc}')
                time.sleep(0.5)
            slack = period - (time.monotonic() - started)
            if slack > 0:
                time.sleep(slack)

    def _render_state(self, now):
        with self._lock:
            frm, to, start = self._from, self._to, self._transition_start
        elapsed = now - start

        if self.transition_time > 0.0 and elapsed < self.transition_time:
            t = _ease(elapsed / self.transition_time)
            rgb = tuple(_lerp(frm[i], to[i], t) for i in range(3))
            scale = 1.0                                  # no pulse mid-fade
        else:
            rgb = tuple(float(c) for c in to)
            # Settled: breathe, phase measured from the end of the fade so every state change
            # lands on full brightness and then dims -- the eye catches the onset.
            settled_for = elapsed - self.transition_time
            phase = (settled_for / self.pulse_period) * 2.0 * math.pi
            scale = self.pulse_min + (1.0 - self.pulse_min) * (0.5 + 0.5 * math.cos(phase))

        with self._lock:
            self._displayed = rgb
        out = tuple(int(max(0, min(255, round(c * scale)))) for c in rgb)
        self.pixels.fill(out)
        self.pixels.show()

    def _render_party(self, color_offset):
        flash_chance = 0.15
        for i in range(self.num_pixels):
            if random.random() < flash_chance:
                self.pixels[i] = (random.randint(180, 255),) * 3
            else:
                c = colorwheel((i * 256 // self.num_pixels + color_offset + i * 7) % 256)
                self.pixels[i] = ((c >> 16) & 0xFF, (c >> 8) & 0xFF, c & 0xFF)
        self.pixels.show()
        return (color_offset + 12) % 256

    def shutdown(self):
        self._running.clear()
        if self._render_thread.is_alive():
            self._render_thread.join(timeout=2.0)
        if self.blank_on_exit:
            try:
                self.pixels.fill((0, 0, 0))
                self.pixels.show()
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = ActivityIndicatorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
