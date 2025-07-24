import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from activity_indicator_msgs.srv import ActivityMode
import random
import time
import board
import busio
import threading
from rainbowio import colorwheel
from adafruit_seesaw import seesaw, neopixel

MODE_OFF = 0
MODE_IDLE = 1
MODE_TELEOP = 2
MODE_AUTONOMOUS = 3
MODE_PARTY = 4

COLOR_MAP = {
    MODE_IDLE: (255, 150, 0),       # Yellow
    MODE_TELEOP: (255, 0, 0),       # Red
    MODE_AUTONOMOUS: (0, 0, 255),   # Blue
}

class ActivityIndicatorDriver(Node):
    def __init__(self):
        super().__init__('activity_indicator_driver')
        self.srv = self.create_service(ActivityMode, 'set_activity_mode', self.set_mode_callback)

        # LED setup
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.ss = seesaw.Seesaw(self.i2c, addr=0x60)
        self.neo_pin = 15
        self.num_pixels = 114
        self.pixels = neopixel.NeoPixel(self.ss, self.neo_pin, self.num_pixels, brightness=1.0)

        # Control variables
        self.mode = MODE_OFF
        self.party_thread = None
        self.party_active = threading.Event()

        self.set_static_color()
        self.get_logger().info("Activity Indicator Node Started")

    def set_mode_callback(self, request, response):
        self.get_logger().info(f"Setting mode to {request.mode}")
        self.mode = request.mode

        if self.party_thread and self.party_thread.is_alive():
            self.party_active.clear()
            self.party_thread.join()

        if self.mode == MODE_PARTY:
            self.party_active.set()
            self.party_thread = threading.Thread(target=self.run_party_mode, daemon=True)
            self.party_thread.start()
        else:
            self.party_active.clear()
            self.set_static_color()

        response.success = True
        return response

    def set_static_color(self):
        if self.mode == MODE_OFF:
            self.pixels.fill((0, 0, 0))
        else:
            color = COLOR_MAP.get(self.mode, (0, 0, 0))
            self.pixels.fill(color)

    def run_party_mode(self):
        import random
        color_offset = 0 
        pixel_flash_chance = 0.15  # 15% of pixels flash white
        strobe_every = 5           # every N frames, flash all pixels
        frame = 0

        while self.party_active.is_set():
            frame += 1
            global_flash = (frame % strobe_every == 0)

            for i in range(self.num_pixels):
                if global_flash:
                    self.pixels[i] = (255, 255, 255)  # strobe white
                elif random.random() < pixel_flash_chance:
                    self.pixels[i] = (
                        random.randint(180, 255),
                        random.randint(180, 255),
                        random.randint(180, 255)
                    )  # glitter flash
                else:
                    rc_index = (i * 256 // self.num_pixels + color_offset + i * 7) % 256
                    color_int = colorwheel(rc_index)
                    r = (color_int >> 16) & 0xFF
                    g = (color_int >> 8) & 0xFF
                    b = color_int & 0xFF
                    self.pixels[i] = (r, g, b)

            self.pixels.show()
            color_offset = (color_offset + 12) % 256  # faster color rotation
            #time.sleep(0.015)  # ~66 FPS

def main(args=None):
    rclpy.init(args=args)
    node = ActivityIndicatorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
