"""
MicroPython entry point for the Pico side of the TetrisKart demo.

Responsibilities (see `game_overview.md`):
  - Sample sensors (MPU6050 tilt, rotary encoder, buttons, potentiometer) every 10 ms.
  - Stream the readings to the PC via USB serial as JSON.
  - Listen for feedback packets (score, level, events) and update the OLED + 8x8 matrix.
  - Trigger simple LED/buzzer patterns for LINE_CLEARED / LEVEL_UP / GAME_OVER events.

This file ties together the four hardware helper modules in the repo:
    `imu.py`, `vector3d.py`, `ssd1306.py`, `max7219.py`
"""

import math
import sys
import ujson
import uselect
import utime
from machine import ADC, I2C, Pin, SPI, PWM

from imu import MPU6050
from max7219 import Matrix8x8
from ssd1306 import SSD1306_I2C


class USBBridge:
    """Non-blocking stdin/stdout wrapper for USB CDC channel."""

    def __init__(self):
        self.poll = uselect.poll()
        self.poll.register(sys.stdin, uselect.POLLIN)

    def send(self, message: dict) -> None:
        sys.stdout.write(ujson.dumps(message) + "\n")
        try:
            sys.stdout.flush()
        except AttributeError:
            # MicroPython may not have flush on stdout
            pass

    def receive(self) -> dict | None:
        if not self.poll.poll(0):
            return None
        raw = sys.stdin.readline()
        if not raw:
            return None
        try:
            return ujson.loads(raw)
        except ValueError:
            return None


class RotaryEncoder:
    """Basic two-pin quadrature decoder with delta reporting."""

    # Expanded sequence table for better compatibility
    # Format: (old_state << 2) | new_state -> direction
    _SEQ = {
        # Clockwise transitions (A leads B)
        0b0001: 1,   # 00 -> 01
        0b0111: 1,   # 01 -> 11
        0b1110: 1,   # 11 -> 10
        0b1000: 1,   # 10 -> 00
        # Counter-clockwise transitions (B leads A)
        0b0010: -1,  # 00 -> 10
        0b1011: -1,  # 10 -> 11
        0b1101: -1,  # 11 -> 01
        0b0100: -1,  # 01 -> 00
        # Additional valid transitions (some encoders skip states)
        0b0011: 1,   # 00 -> 11 (fast CW)
        0b1100: -1,  # 11 -> 00 (fast CCW)
        0b0110: 0,   # 01 -> 10 (invalid, ignore)
        0b1001: 0,   # 10 -> 01 (invalid, ignore)
    }

    def __init__(self, pin_a: int, pin_b: int):
        self.pin_a = Pin(pin_a, Pin.IN, Pin.PULL_UP)
        self.pin_b = Pin(pin_b, Pin.IN, Pin.PULL_UP)
        # Initialize state from current pin values
        self.state = (self.pin_a.value() << 1) | self.pin_b.value()
        self.position = 0
        self._delta = 0
        self._last_poll_time = utime.ticks_ms()
        self._debounce_time = 2  # 2ms debounce

    def poll(self):
        """
        Poll the encoder pins and update state. Should be called frequently
        (e.g., every few milliseconds) to catch all transitions.
        """
        now = utime.ticks_ms()
        # Debounce: only process if enough time has passed
        if utime.ticks_diff(now, self._last_poll_time) < self._debounce_time:
            return
        
        self._last_poll_time = now
        value = (self.pin_a.value() << 1) | self.pin_b.value()
        
        # Only process if state changed
        if value != self.state:
            frame = (self.state << 2) | value
            delta = self._SEQ.get(frame & 0b1111, 0)
            if delta != 0:
                self.position += delta
                self._delta += delta
                self.state = value
            elif value in (0b00, 0b01, 0b10, 0b11):
                # Valid state but no recognized transition - update state anyway
                # This handles cases where we might have missed an intermediate state
                self.state = value

    def read(self) -> int:
        """
        Return accumulated delta since last read and reset it.
        Make sure to call poll() frequently for accurate readings.
        """
        result = self._delta
        self._delta = 0
        return result


class MusicManager:
    """Manages different background music tracks."""
    
    # Menu/Pause menu music - Nokia Ringtone
    # Tempo: 180 BPM, wholenote = 1333.33 ms
    MENU_MUSIC = [
        (659, 166),   # E5, 8
        (587, 166),   # D5, 8
        (740, 333),   # FS4, 4
        (831, 333),   # GS4, 4
        (554, 166),   # CS5, 8
        (494, 166),   # B4, 8
        (294, 333),   # D4, 4
        (330, 333),   # E4, 4
        (494, 166),   # B4, 8
        (440, 166),   # A4, 8
        (277, 333),   # CS4, 4
        (330, 333),   # E4, 4
        (440, 666),   # A4, 2
        (0, 200),     # Rest
    ]
    
    # Game music - Star Wars Main Theme
    # Tempo: 108 BPM, wholenote = 2222.22 ms
    GAME_MUSIC = [
        (466, 277),   # AS4, 8
        (466, 277),   # AS4, 8
        (466, 277),   # AS4, 8
        (698, 1111),  # F5, 2
        (1047, 1111), # C6, 2
        (932, 277),   # AS5, 8
        (880, 277),   # A5, 8
        (784, 277),   # G5, 8
        (1397, 1111), # F6, 2
        (1047, 555),  # C6, 4
        (932, 277),   # AS5, 8
        (880, 277),   # A5, 8
        (784, 277),   # G5, 8
        (1397, 1111), # F6, 2
        (1047, 555),  # C6, 4
        (932, 277),   # AS5, 8
        (880, 277),   # A5, 8
        (932, 277),   # AS5, 8
        (784, 1111),  # G5, 2
        (523, 277),   # C5, 8
        (523, 277),   # C5, 8
        (523, 277),   # C5, 8
        (698, 1111),  # F5, 2
        (1047, 1111), # C6, 2
        (932, 277),   # AS5, 8
        (880, 277),   # A5, 8
        (784, 277),   # G5, 8
        (1397, 1111), # F6, 2
        (1047, 555),  # C6, 4
        (932, 277),   # AS5, 8
        (880, 277),   # A5, 8
        (784, 277),   # G5, 8
        (1397, 1111), # F6, 2
        (1047, 555),  # C6, 4
        (932, 277),   # AS5, 8
        (880, 277),   # A5, 8
        (932, 277),   # AS5, 8
        (784, 1111),  # G5, 2
        (523, 416),   # C5, -8 (dotted 8th = 277*1.5)
        (523, 138),   # C5, 16
        (587, 833),   # D5, -4 (dotted quarter = 555*1.5)
        (587, 277),   # D5, 8
        (932, 277),   # AS5, 8
        (880, 277),   # A5, 8
        (784, 277),   # G5, 8
        (698, 277),   # F5, 8
        (698, 277),   # F5, 8
        (784, 277),   # G5, 8
        (880, 277),   # A5, 8
        (784, 555),   # G5, 4
        (587, 277),   # D5, 8
        (659, 555),   # E5, 4
        (523, 416),   # C5, -8 (dotted 8th = 277*1.5)
        (523, 138),   # C5, 16
        (587, 833),   # D5, -4 (dotted quarter = 555*1.5)
        (587, 277),   # D5, 8
        (932, 277),   # AS5, 8
        (880, 277),   # A5, 8
        (784, 277),   # G5, 8
        (698, 277),   # F5, 8
        (1047, 416),  # C6, -8 (dotted 8th = 277*1.5)
        (784, 138),   # G5, 16
        (784, 2222),  # G5, 2
        (0, 277),     # REST, 8
        (523, 277),   # C5, 8
        (587, 1111),  # D5, -4 (dotted)
        (587, 277),   # D5, 8
        (932, 277),   # AS5, 8
        (880, 277),   # A5, 8
        (784, 277),   # G5, 8
        (698, 277),   # F5, 8
        (698, 277),   # F5, 8
        (784, 277),   # G5, 8
        (880, 277),   # A5, 8
        (784, 555),   # G5, 4
        (587, 277),   # D5, 8
        (659, 555),   # E5, 4
        (1047, 416),  # C6, -8 (dotted 8th = 277*1.5)
        (1047, 138),  # C6, 16
        (1397, 555),  # F6, 4
        (1245, 277),  # DS6, 8
        (1109, 555),  # CS6, 4
        (1047, 277),  # C6, 8
        (932, 555),   # AS5, 4
        (831, 277),   # GS5, 8
        (784, 555),   # G5, 4
        (698, 277),   # F5, 8
        (1047, 2222), # C6, 1
        (0, 200),     # Rest
    ]
    
    # Scoring music - celebratory, short
    SCORING_MUSIC = [
        (523, 100),  # C5
        (659, 100),  # E5
        (784, 100),  # G5
        (1047, 200), # C6
        (784, 100),  # G5
        (1047, 300), # C6
        (0, 100),    # Rest
    ]
    
    def __init__(self, buzzer: PWM):
        self.buzzer = buzzer
        self.current_track = None
        self.current_notes = []
        self.note_index = 0
        self.note_start = 0
        self.enabled = True
        self.volume = 1000  # Background music volume
        self.loop_count = 0  # Track how many times current track has looped
        self.max_loops = 2  # Maximum number of loops for menu and game music
    
    def play_track(self, track_name: str):
        """Play a specific music track."""
        if track_name == "menu":
            self.current_notes = self.MENU_MUSIC
        elif track_name == "game":
            self.current_notes = self.GAME_MUSIC
        elif track_name == "scoring":
            self.current_notes = self.SCORING_MUSIC
        else:
            return
        
        self.current_track = track_name
        self.note_index = 0
        self.note_start = 0
        self.loop_count = 0  # Reset loop count when starting new track
    
    def stop(self):
        """Stop current music."""
        self.buzzer.duty_u16(0)
        self.current_track = None
        self.note_index = 0
        self.note_start = 0
        self.loop_count = 0  # Reset loop count when stopping
    
    def set_enabled(self, enabled: bool):
        """Enable or disable music."""
        old_enabled = self.enabled
        self.enabled = enabled
        if not enabled:
            self.stop()
        elif old_enabled != enabled:
            # Sound status changed (unmuted), reset loop count
            self.loop_count = 0
            if self.current_track:
                self.note_index = 0
                self.note_start = 0
    
    def update(self):
        """Update music playback. Call this frequently."""
        # If disabled, ensure buzzer is off
        if not self.enabled:
            self.buzzer.duty_u16(0)
            return
        
        if not self.current_track or not self.current_notes:
            self.buzzer.duty_u16(0)
            return
        
        now = utime.ticks_ms()
        if self.note_start == 0:
            self.note_start = now
        
        if self.note_index < len(self.current_notes):
            freq, duration = self.current_notes[self.note_index]
            elapsed = utime.ticks_diff(now, self.note_start)
            
            if elapsed >= duration:
                self.note_index += 1
                self.note_start = now
                elapsed = 0
                
                # Check if we've reached the end of the track
                if self.note_index >= len(self.current_notes):
                    # Track finished, check if we should loop
                    if self.current_track in ("menu", "game"):
                        self.loop_count += 1
                        if self.loop_count >= self.max_loops:
                            # Stop after max loops
                            self.stop()
                            return
                        else:
                            # Loop again
                            self.note_index = 0
                    else:
                        # Scoring music doesn't loop
                        self.stop()
                        return
            
            if freq > 0:
                self.buzzer.freq(freq)
                self.buzzer.duty_u16(self.volume)
            else:
                self.buzzer.duty_u16(0)  # Rest note


class EffectsEngine:
    def __init__(self, led_pins: list = [28, 27, 26, 25], buzzer_pin: int = 9):
        # Initialize 4 LEDs on pins 28, 27, 26, 25
        self.leds = [Pin(pin, Pin.OUT) for pin in led_pins]
        self.buzzer = PWM(Pin(buzzer_pin))
        self.buzzer.duty_u16(0)
        self.active_until = 0
        self.music = MusicManager(self.buzzer)
        self.sound_enabled = True  # Global sound enable/disable flag

    def trigger(self, frequency: int, duration_ms: int):
        """Trigger a one-shot sound effect (temporarily interrupts music)."""
        # Only play sound if sound is enabled
        if not self.sound_enabled:
            return
        self.buzzer.freq(frequency)
        self.buzzer.duty_u16(2000)
        # Turn on all LEDs
        for led in self.leds:
            led.value(1)
        self.active_until = utime.ticks_add(utime.ticks_ms(), duration_ms)

    def update(self):
        # Handle one-shot effects
        if self.active_until and utime.ticks_diff(self.active_until, utime.ticks_ms()) <= 0:
            self.buzzer.duty_u16(0)
            # Turn off all LEDs
            for led in self.leds:
                led.value(0)
            self.active_until = 0
        
        # Update music (only if no one-shot effect is playing)
        # Also ensure buzzer is off if sound is disabled
        if not self.active_until:
            if not self.sound_enabled:
                self.buzzer.duty_u16(0)  # Keep buzzer off when sound is disabled
            else:
                self.music.update()


class DisplayManager:
    def __init__(self, i2c: I2C, spi: SPI, matrix_cs_pin: int = 5, oled_addr: int = None):
        # Try to find OLED address automatically if not provided
        if oled_addr is None:
            devices = i2c.scan()
            # Common SSD1306 addresses: 0x3C (7-bit) or 0x78 (8-bit, which is 0x3C << 1)
            # Note: I2C scan returns 7-bit addresses, so 0x78 in scan means 0x3C
            if 60 in devices:  # 0x3C (7-bit)
                oled_addr = 0x3C
            elif 120 in devices:  # 0x78 in scan = 0x3C in 7-bit (some modules report this way)
                oled_addr = 0x3C  # Use 7-bit address
            elif 61 in devices:  # 0x3D
                oled_addr = 0x3D
            else:
                oled_addr = 0x3C  # Default fallback
        
        try:
            self.oled = SSD1306_I2C(128, 64, i2c, addr=oled_addr)
            self.oled_available = True
            print("OLED initialized at address 0x{:02x}".format(oled_addr))
        except Exception as e:
            self.oled = None
            self.oled_available = False
            print("OLED initialization failed:", e)
        
        self.matrix = Matrix8x8(spi, Pin(matrix_cs_pin, Pin.OUT), num=1, orientation=0)
        self.matrix.brightness(2)
        # Clear matrix on startup
        self.matrix.fill(0)
        self.matrix.show()
        self.score = -1
        self.level = -1

    def update_scoreboard(self, score: int, level: int, lines: int):
        if score == self.score and level == self.level:
            return
        self.score = score
        self.level = level
        self._draw_oled(score, level, lines)
        self._draw_matrix(score, level)

    def _draw_oled(self, score: int, level: int, lines: int):
        if not self.oled_available or self.oled is None:
            return
        try:
            self.oled.fill(0)
            self.oled.text("TetrisKart", 0, 0, 1)
            self.oled.text("Score: {}".format(score), 0, 16, 1)
            self.oled.text("Level: {}".format(level), 0, 32, 1)
            self.oled.text("Lines: {}".format(lines), 0, 48, 1)
            self.oled.show()
        except Exception:
            pass  # Ignore OLED errors during runtime

    def _draw_matrix(self, score: int, level: int):
        self.matrix.fill(0)
        digits = "{:02d}".format(score % 100)
        self.matrix.text(digits, 0, 0, 1)
        for i in range(min(level, 8)):
            self.matrix.pixel(i, 7, 1)
        self.matrix.show()
    
    def clear_matrix(self):
        """Turn off all pixels on the 8x8 matrix."""
        self.matrix.fill(0)
        self.matrix.show()
    
    def show_exit_message(self):
        """Clear OLED and display 'Trifaze' and 'Dentist Tetris' message."""
        if not self.oled_available or self.oled is None:
            return
        try:
            self.oled.fill(0)  # Clear the screen
            self.oled.text("Trifaze", 0, 0, 1)
            self.oled.text("Dentist Tetris", 0, 10, 1)
            self.oled.show()
        except Exception:
            pass  # Ignore OLED errors


class SensorSuite:
    """
    Hardware input configuration:
    
    I2C Bus (for MPU6050 IMU and SSD1306 OLED):
      - SCL: GPIO 13
      - SDA: GPIO 12
    
    SPI Bus (for MAX7219 8x8 matrix):
      - CLK (Clock): GPIO 6 → SPI SCK
      - DIN (Data In): GPIO 7 → SPI MOSI
      - CS (Chip Select): GPIO 5 → SPI CS
      - GND: Ground
      - VCC: 3.3V or 5V (check module specs)
    
    Digital Inputs:
      - Button A: GPIO 10 (PULL_DOWN, active HIGH) -> Pause/Menu
      - Button B: GPIO 11 (PULL_DOWN, active HIGH) -> Rotate piece
      - Encoder A: GPIO 14 (PULL_UP) -> Quadrature input
      - Encoder B: GPIO 15 (PULL_UP) -> Quadrature input
    
    Analog Input:
      - Potentiometer: ADC0 (GPIO 26) -> Speed control (0-65535)
      WARNING: GPIO 26 is also used for LED. Potentiometer should be moved to a different ADC pin
      (e.g., ADC1 on GPIO 27, but that's also an LED pin, or use GPIO 29/ADC3 if available)
    
    Outputs:
      - LEDs: GPIO 28, 27, 26, 25 -> Visual feedback (4 LEDs)
      - Buzzer: GPIO 9 (PWM) -> Audio feedback
    """
    
    def __init__(self):
        self.i2c = I2C(0, scl=Pin(13), sda=Pin(12), freq=400000)
        self.spi = SPI(0, baudrate=10_000_000, sck=Pin(6), mosi=Pin(7)) # MAX7219
        
        # Wait for I2C devices to stabilize
        utime.sleep_ms(100)
        
        # Scan I2C bus to diagnose connection issues
        devices = self.i2c.scan()
        print("I2C scan found devices at addresses:", [hex(d) for d in devices])
        
        # MPU6050 should be at 0x68 (104) or 0x69 (105)
        # SSD1306 should be at 0x3C (60), 0x3D (61), or 0x78 (120)
        if 104 not in devices and 105 not in devices:
            print("WARNING: MPU6050 not found! Expected 0x68 or 0x69")
        if 60 not in devices and 61 not in devices and 120 not in devices:
            print("WARNING: SSD1306 OLED not found! Expected 0x3C, 0x3D, or 0x78")
        
        self.imu = MPU6050(self.i2c)
        self.button_a = Pin(10, Pin.IN, Pin.PULL_DOWN)
        self.button_b = Pin(11, Pin.IN, Pin.PULL_DOWN)
        self.encoder = RotaryEncoder(14, 15)
        self.pot = ADC(26)
        
        # Pass detected OLED address to DisplayManager
        oled_addr = None
        if 60 in devices:
            oled_addr = 0x3C
        elif 120 in devices:
            oled_addr = 0x78
        elif 61 in devices:
            oled_addr = 0x3D
        
        self.display = DisplayManager(self.i2c, self.spi, oled_addr=oled_addr)
        self.effects = EffectsEngine()
        
        # Calibration offset for tilt X (will be set during calibration)
        self.tilt_x_offset = 0.0

    def calibrate_tilt_x(self, num_samples: int = 50, delay_ms: int = 20):
        """
        Calibrate tilt X by taking multiple samples when device is at rest.
        The average offset will be subtracted from future tilt X readings.
        """
        print("Calibrating tilt X... Please keep device still.")
        samples = []
        for i in range(num_samples):
            ax, ay, az = self.imu.accel.xyz
            roll = math.degrees(math.atan2(ay, max(1e-3, az)))
            samples.append(roll)
            utime.sleep_ms(delay_ms)
            if (i + 1) % 10 == 0:
                print(f"Calibration progress: {i + 1}/{num_samples}")
        
        self.tilt_x_offset = sum(samples) / len(samples)
        print(f"Tilt X calibration complete. Offset: {self.tilt_x_offset:.2f} degrees")
        return self.tilt_x_offset

    def sample(self) -> dict:
        """
        Sample all sensors and return JSON-ready dict.
        
        Tilt calculation:
        - roll (rotation around X-axis) -> tilt.x (left/right for Tetris movement)
        - pitch (rotation around Y-axis) -> tilt.y (forward/back, not used in game)
        
        Note: If your device orientation differs, swap pitch/roll or adjust the
        atan2 arguments accordingly.
        """
        ax, ay, az = self.imu.accel.xyz
        # Roll: rotation around X-axis (left/right tilt) -> used for horizontal movement
        roll = math.degrees(math.atan2(ay, max(1e-3, az)))
        # Apply calibration offset to tilt X
        roll_calibrated = roll - self.tilt_x_offset
        # Clamp tilt X to -30 to 30 degrees
        roll_calibrated = max(-30.0, min(30.0, roll_calibrated))
        # Pitch: rotation around Y-axis (forward/back tilt) -> not used in Tetris
        pitch = math.degrees(math.atan2(ax, max(1e-3, math.sqrt(ay * ay + az * az))))
        payload = {
            "ts": utime.ticks_ms(),
            "tilt": {"x": roll_calibrated, "y": pitch},  # roll for horizontal movement
            "buttons": {"a": int(self.button_a.value()), "b": int(self.button_b.value())},
            "encoder": {"delta": self.encoder.read(), "position": self.encoder.position},
            "pot": self.pot.read_u16(),
        }
        return payload

    def handle_feedback(self, packet: dict):
        # Handle commands from PC
        if "command" in packet:
            cmd = packet["command"]
            if cmd == "calibrate":
                print("Calibration requested from menu...")
                self.calibrate_tilt_x(num_samples=50, delay_ms=20)
            elif cmd == "music_menu":
                self.effects.music.play_track("menu")
                # Reset loop count when returning to main menu
                self.effects.music.loop_count = 0
                print("Playing menu music")
            elif cmd == "music_game":
                self.effects.music.play_track("game")
                print("Playing game music")
            elif cmd == "music_scoring":
                self.effects.music.play_track("scoring")
                print("Playing scoring music")
            elif cmd == "music_stop":
                self.effects.music.stop()
                print("Music stopped")
            elif cmd == "music_mute":
                self.effects.music.set_enabled(False)
                print("Music muted")
            elif cmd == "music_unmute":
                self.effects.music.set_enabled(True)
                print("Music unmuted")
            elif cmd == "sound_on":
                # Enable all sound (music and effects)
                self.effects.music.set_enabled(True)
                self.effects.sound_enabled = True
                # Reset loop count when sound status changes
                self.effects.music.loop_count = 0
                if self.effects.music.current_track:
                    self.effects.music.note_index = 0
                    self.effects.music.note_start = 0
                print("Sound enabled")
            elif cmd == "sound_off":
                # Disable all sound - mute buzzer completely
                self.effects.music.set_enabled(False)
                self.effects.music.stop()
                self.effects.buzzer.duty_u16(0)  # Ensure buzzer is off
                self.effects.sound_enabled = False
                # Reset loop count when sound status changes
                self.effects.music.loop_count = 0
                print("Sound disabled - buzzer muted")
            elif cmd == "exit":
                print("Exit command received. Resetting for new game...")
                # Stop all effects
                self.effects.music.stop()
                self.effects.buzzer.duty_u16(0)
                # Turn off 8x8 matrix
                self.display.clear_matrix()
                # Clear OLED and show Trifaze
                self.display.show_exit_message()
                # Reset calibration
                self.tilt_x_offset = 0.0
                # Reset sound state
                self.effects.sound_enabled = True
                self.effects.music.enabled = True
                self.effects.music.loop_count = 0
                # Don't exit - just reset and continue
                print("System reset. Ready for new game.")
                return False  # Don't exit, just reset
        
        # Handle game feedback
        score = int(packet.get("score", self.display.score))
        level = int(packet.get("level", self.display.level))
        lines = int(packet.get("lines", 0))
        self.display.update_scoreboard(score, level, lines)
        # Only trigger sound effects if sound is enabled
        if self.effects.sound_enabled:
            for event in packet.get("events", []):
                if event == "LINE_CLEARED":
                    self.effects.trigger(1400, 120)
                elif event == "LEVEL_UP":
                    self.effects.trigger(800, 200)
                elif event == "GAME_OVER":
                    self.effects.trigger(200, 400)

    def update_effects(self):
        self.effects.update()


def main():
    bridge = USBBridge()
    sensors = SensorSuite()
    
    # Initial calibration - will be zero until user calibrates from menu
    print("System ready. Waiting for calibration from menu...")
    
    next_tick = utime.ticks_ms()
    interval = 10  # Reduced to 10ms for better responsiveness
    encoder_poll_tick = utime.ticks_ms()
    encoder_poll_interval = 1  # Poll encoder every 1ms for maximum responsiveness
    encoder_debug_counter = 0
    while True:
        now = utime.ticks_ms()
        
        # Poll encoder frequently to catch all transitions
        if utime.ticks_diff(now, encoder_poll_tick) >= 0:
            encoder_poll_tick = utime.ticks_add(encoder_poll_tick, encoder_poll_interval)
            sensors.encoder.poll()
            
            # Debug: print encoder state every 100 polls (roughly every 200ms)
            encoder_debug_counter += 1
            if encoder_debug_counter >= 100:
                encoder_debug_counter = 0
                delta = sensors.encoder._delta
                if delta != 0:
                    print(f"Encoder delta: {delta}, position: {sensors.encoder.position}")
        
        # Sample all sensors at the main interval
        if utime.ticks_diff(now, next_tick) >= 0:
            next_tick = utime.ticks_add(next_tick, interval)
            try:
                bridge.send(sensors.sample())
            except OSError:
                pass
        if msg := bridge.receive():
            # Handle feedback (exit command now resets instead of exiting)
            sensors.handle_feedback(msg)
        sensors.update_effects()
        # No sleep - run as fast as possible for minimal delay


if __name__ == "__main__":
    main()

