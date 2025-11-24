"""
PC-side Tetris display and gameplay loop.

This module reproduces the flow described in `game_overview.md`:
  - Read Pico sensor packets over USB serial (tilt, buttons, encoder, pot).
  - Convert them into Tetris controls (move, rotate, speed).
  - Run the gameplay logic, render using pygame, and send score/level/events
    back to the Pico so it can update OLED + LED matrix.

The file is intentionally self-contained and can fall back to keyboard/mouse
input when no Pico is connected (`--mock-input` flag). To run:

    python display.py --port COM5          # real Pico serial port
    python display.py --mock-input         # keyboard fallback for dev
"""

from __future__ import annotations

import argparse
import json
import math
import random
import sys
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

try:
    import serial  # type: ignore
except ImportError:  # pragma: no cover - serial might be unavailable on dev PCs
    serial = None

import pygame


BOARD_WIDTH = 10
BOARD_HEIGHT = 22  # last two rows are hidden spawn rows
VISIBLE_HEIGHT = 20
TILE_SIZE = 28
SIDE_PANEL = 200
SCREEN_WIDTH = BOARD_WIDTH * TILE_SIZE + SIDE_PANEL
SCREEN_HEIGHT = VISIBLE_HEIGHT * TILE_SIZE

MIN_DROP_DELAY = 0.05
MAX_DROP_DELAY = 0.70
LEVEL_RULES = [
    {"level": 1, "score": 0, "max_delay": 0.60},
    {"level": 2, "score": 200, "max_delay": 0.50},
    {"level": 3, "score": 600, "max_delay": 0.40},
    {"level": 4, "score": 1200, "max_delay": 0.30},
    {"level": 5, "score": 2000, "max_delay": 0.20},
]


TETROMINOES: Dict[str, List[List[Tuple[int, int]]]] = {
    "I": [
        [(0, 1), (1, 1), (2, 1), (3, 1)],
        [(2, 0), (2, 1), (2, 2), (2, 3)],
    ],
    "O": [
        [(1, 0), (2, 0), (1, 1), (2, 1)],
    ],
    "T": [
        [(1, 0), (0, 1), (1, 1), (2, 1)],
        [(1, 0), (1, 1), (2, 1), (1, 2)],
        [(0, 1), (1, 1), (2, 1), (1, 2)],
        [(1, 0), (0, 1), (1, 1), (1, 2)],
    ],
    "S": [
        [(1, 0), (2, 0), (0, 1), (1, 1)],
        [(1, 0), (1, 1), (2, 1), (2, 2)],
    ],
    "Z": [
        [(0, 0), (1, 0), (1, 1), (2, 1)],
        [(2, 0), (1, 1), (2, 1), (1, 2)],
    ],
    "J": [
        [(0, 0), (0, 1), (1, 1), (2, 1)],
        [(1, 0), (2, 0), (1, 1), (1, 2)],
        [(0, 1), (1, 1), (2, 1), (2, 2)],
        [(1, 0), (1, 1), (0, 2), (1, 2)],
    ],
    "L": [
        [(2, 0), (0, 1), (1, 1), (2, 1)],
        [(1, 0), (1, 1), (1, 2), (2, 2)],
        [(0, 1), (1, 1), (2, 1), (0, 2)],
        [(0, 0), (1, 0), (1, 1), (1, 2)],
    ],
}

COLORS = {
    0: (10, 10, 10),
    "I": (0, 240, 255),
    "O": (255, 204, 0),
    "T": (173, 75, 255),
    "S": (48, 221, 110),
    "Z": (255, 70, 70),
    "J": (50, 100, 255),
    "L": (255, 176, 66),
    "ghost": (80, 80, 80),
}


def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(max_value, value))


def lerp(value: int, src_min: int, src_max: int, dst_min: float, dst_max: float) -> float:
    if src_max == src_min:
        return dst_max
    normalized = (value - src_min) / (src_max - src_min)
    return dst_min + normalized * (dst_max - dst_min)


@dataclass
class InputSample:
    """
    Input data structure from Pico hardware:
    
    - tilt_x: Roll angle in degrees (left/right tilt) -> controls horizontal movement
      Threshold: ±8.0 degrees to trigger movement
    - tilt_y: Pitch angle in degrees (forward/back tilt) -> currently unused
    - button_a: Pause/Menu button (edge-triggered)
    - button_b: Rotate piece button (edge-triggered on press)
    - encoder_delta: Rotary encoder change since last read -> adjusts speed modifier
    - pot_value: Potentiometer reading (0-65535) -> controls drop speed
      Maps to drop_delay: 0.70s (slow) to 0.05s (fast)
    - encoder_click: Encoder button press (uses button_a in menu mode)
    """
    timestamp: float = 0.0
    tilt_x: float = 0.0
    tilt_y: float = 0.0
    button_a: bool = False
    button_b: bool = False
    encoder_delta: int = 0
    encoder_click: bool = False
    pot_value: int = 32768
    raw: Dict = field(default_factory=dict)


class PicoBridge:
    """Bi-directional JSON stream over USB serial."""

    def __init__(self, port: str, baud: int = 115200):
        if serial is None:
            raise RuntimeError("pyserial is not installed. Install via `pip install pyserial`.")
        try:
            self.ser = serial.Serial(port, baudrate=baud, timeout=0.1, write_timeout=0.1)
            print(f"Connected to Pico on {port} at {baud} baud")
        except serial.SerialException as e:
            raise RuntimeError(f"Failed to open serial port {port}: {e}")
        self.buffer = bytearray()

    def read_inputs(self) -> Optional[InputSample]:
        try:
            if not self.ser.in_waiting:
                return None
            data = self.ser.read(self.ser.in_waiting)
            self.buffer.extend(data)
            if b"\n" not in self.buffer:
                return None
            line, _, rest = self.buffer.partition(b"\n")
            self.buffer = bytearray(rest)
            try:
                message = json.loads(line.decode().strip())
            except (ValueError, UnicodeDecodeError):
                return None
            return self._to_sample(message)
        except (serial.SerialException, OSError) as e:
            print(f"Serial read error: {e}")
            return None

    def send_feedback(self, payload: Dict) -> None:
        try:
            encoded = json.dumps(payload).encode() + b"\n"
            self.ser.write(encoded)
        except (serial.SerialException, OSError) as e:
            print(f"Serial write error: {e}")

    @staticmethod
    def _to_sample(message: Dict) -> InputSample:
        tilt = message.get("tilt", {})
        buttons = message.get("buttons", {})
        encoder = message.get("encoder", {})
        encoder_delta = int(encoder.get("delta", 0))
        
        # Removed debug print for performance
        
        return InputSample(
            timestamp=message.get("ts", time.time()),
            tilt_x=float(tilt.get("x", 0.0)),
            tilt_y=float(tilt.get("y", 0.0)),
            button_a=bool(buttons.get("a", 0)),
            button_b=bool(buttons.get("b", 0)),
            encoder_delta=encoder_delta,
            encoder_click=bool(buttons.get("a", 0)),  # Button A used as encoder click in menu
            pot_value=int(message.get("pot", 32768)),
            raw=message,
        )


class MockBridge:
    """Keyboard controlled fallback for development."""

    def __init__(self):
        self.last_sample = InputSample(timestamp=time.time())

    def read_inputs(self) -> InputSample:
        pygame.event.pump()
        keys = pygame.key.get_pressed()
        tilt_x = 0.0
        if keys[pygame.K_LEFT]:
            tilt_x = -20.0
        elif keys[pygame.K_RIGHT]:
            tilt_x = 20.0
        button_b = keys[pygame.K_UP]
        button_a = keys[pygame.K_SPACE]
        encoder_delta = (1 if keys[pygame.K_PAGEUP] else 0) - (1 if keys[pygame.K_PAGEDOWN] else 0)
        pot_value = 5000 if keys[pygame.K_DOWN] else 45000
        self.last_sample = InputSample(
            timestamp=time.time(),
            tilt_x=tilt_x,
            tilt_y=0.0,
            button_a=button_a,
            button_b=button_b,
            encoder_delta=encoder_delta,
            encoder_click=button_a,
            pot_value=pot_value,
        )
        return self.last_sample

    def send_feedback(self, payload: Dict) -> None:
        # Dev mode: just log to stdout
        sys.stdout.write(f"[MOCK FEEDBACK] {payload}\n")
        sys.stdout.flush()


class Piece:
    def __init__(self, shape: str):
        self.shape = shape
        self.rotation = 0
        self.x = 3
        self.y = -2

    @property
    def offsets(self) -> Sequence[Tuple[int, int]]:
        rotations = TETROMINOES[self.shape]
        return rotations[self.rotation % len(rotations)]

    def cells(self, dx: int = 0, dy: int = 0, rotation_delta: int = 0) -> List[Tuple[int, int]]:
        rot = (self.rotation + rotation_delta) % len(TETROMINOES[self.shape])
        offsets = TETROMINOES[self.shape][rot]
        return [(self.x + dx + cx, self.y + dy + cy) for (cx, cy) in offsets]


class PauseMenu:
    """Pause menu that appears when game is paused."""
    
    def __init__(self):
        self.selected_option = 0
        self.menu_items = ["Resume", "Main Menu", "Mute"]
        self.muted = False
        self.font_title = None
        self.font_menu = None
        self.font_small = None
        self._fonts_initialized = False
        self._last_encoder_position = None
    
    def _init_fonts(self):
        """Lazy initialization of fonts after pygame is initialized."""
        if not self._fonts_initialized:
            self.font_title = pygame.font.SysFont("consolas", 36, bold=True)
            self.font_menu = pygame.font.SysFont("consolas", 24)
            self.font_small = pygame.font.SysFont("consolas", 18)
            self._fonts_initialized = True
    
    def update(self, inputs: InputSample, prev_inputs: InputSample):
        """Update pause menu state. Returns command dict or None."""
        self._init_fonts()
        
        # Get encoder position from raw data
        current_position = None
        if hasattr(inputs, 'raw') and inputs.raw:
            encoder_data = inputs.raw.get('encoder', {})
            current_position = encoder_data.get('position')
        
        # Initialize position on first run
        if self._last_encoder_position is None:
            self._last_encoder_position = current_position if current_position is not None else 0
            return None
        
        # Handle encoder navigation - map encoder steps to menu options
        if current_position is not None:
            # Normalize encoder position to positive range (0-30)
            normalized_pos = abs(current_position) % 31
            
            # Map encoder position to menu option based on number of items
            # Reduced step ranges for faster navigation
            if len(self.menu_items) == 2:
                # 2 options: 0-10 = option 1, 11-20 = option 2
                if normalized_pos <= 10:
                    new_selection = 0
                else:
                    new_selection = 1
            elif len(self.menu_items) == 3:
                # 3 options: 0-7 = option 1, 8-15 = option 2, 16-23 = option 3
                if normalized_pos <= 7:
                    new_selection = 0
                elif normalized_pos <= 15:
                    new_selection = 1
                else:
                    new_selection = 2
            else:
                # For other numbers of options, use equal distribution
                steps_per_option = 20 // len(self.menu_items)
                new_selection = min(normalized_pos // (steps_per_option + 1), len(self.menu_items) - 1)
            
            # Update selection if it changed
            if new_selection != self.selected_option:
                self.selected_option = new_selection
            
            # Update last known position
            self._last_encoder_position = current_position
        else:
            # If position is None, keep last known position (encoder might not be sending data yet)
            pass
        
        # Handle button A press (selection)
        button_a_pressed = inputs.button_a or inputs.encoder_click
        button_a_prev = prev_inputs.button_a or (hasattr(prev_inputs, 'encoder_click') and prev_inputs.encoder_click)
        
        if button_a_pressed and not button_a_prev:
            if self.selected_option == 0:  # Resume
                return {"action": "resume"}
            elif self.selected_option == 1:  # Main Menu
                return {"action": "main_menu"}
            elif self.selected_option == 2:  # Mute
                self.muted = not self.muted
                return {"command": "music_mute" if self.muted else "music_unmute"}
        
        return None
    
    def draw(self, screen: pygame.Surface):
        """Draw the pause menu overlay."""
        self._init_fonts()
        
        # Semi-transparent overlay
        overlay = pygame.Surface((SCREEN_WIDTH, SCREEN_HEIGHT))
        overlay.set_alpha(200)
        overlay.fill((0, 0, 0))
        screen.blit(overlay, (0, 0))
        
        # Title
        title = self.font_title.render("PAUSED", True, (255, 100, 100))
        title_rect = title.get_rect(center=(SCREEN_WIDTH // 2, 150))
        screen.blit(title, title_rect)
        
        # Menu items
        y_start = SCREEN_HEIGHT // 2 - 40
        for i, item in enumerate(self.menu_items):
            color = (255, 255, 100) if i == self.selected_option else (200, 200, 200)
            prefix = "> " if i == self.selected_option else "  "
            
            if item == "Mute":
                value = "ON" if self.muted else "OFF"
                text = f"{prefix}{item}: {value}"
            else:
                text = f"{prefix}{item}"
            
            item_surface = self.font_menu.render(text, True, color)
            item_rect = item_surface.get_rect(center=(SCREEN_WIDTH // 2, y_start + i * 50))
            screen.blit(item_surface, item_rect)
        
        # Hint
        hint = self.font_small.render("Encoder: Navigate | Button A: Select", True, (150, 150, 150))
        hint_rect = hint.get_rect(center=(SCREEN_WIDTH // 2, SCREEN_HEIGHT - 50))
        screen.blit(hint, hint_rect)


class TetrisGame:
    def __init__(self):
        self.board = [[0 for _ in range(BOARD_WIDTH)] for _ in range(BOARD_HEIGHT)]
        self.active = self._next_piece()
        self.next_piece = self._next_piece()
        self.held_piece: Optional[Piece] = None
        self.hold_used = False
        self.score = 0
        self.lines_cleared = 0
        self.level = 1
        self.drop_delay = MAX_DROP_DELAY
        self.drop_timer = 0.0
        self.move_hold_timer = 0.0
        self.move_direction = 0
        self.paused = False
        self.pause_menu = PauseMenu()
        self.just_dropped = False
        self.speed_modifier = 1.0
        self.events: List[str] = []
        self.prev_inputs = InputSample()

    def _next_piece(self) -> Piece:
        return Piece(random.choice(list(TETROMINOES.keys())))

    def reset(self):
        self.__init__()

    def update(self, inputs: InputSample, dt: float) -> List[str]:
        self.events.clear()
        if inputs.button_a and not self.prev_inputs.button_a:
            self.paused = not self.paused
        if self.paused:
            self.prev_inputs = inputs
            return self.events

        self._handle_speed(inputs)
        self._handle_horizontal(inputs, dt)
        self._handle_rotation(inputs)
        self._gravity_step(dt)
        self.prev_inputs = inputs
        return self.events

    def _handle_speed(self, inputs: InputSample):
        base_delay = lerp(inputs.pot_value, 0, 65535, MIN_DROP_DELAY, MAX_DROP_DELAY)
        target_delay = min(base_delay, self._max_delay_for_level())
        # Encoder is now only used for menu navigation, not speed control
        # Speed is controlled only by potentiometer
        self.drop_delay = clamp(target_delay * self.speed_modifier, MIN_DROP_DELAY / 2, MAX_DROP_DELAY)

    def _handle_horizontal(self, inputs: InputSample, dt: float):
        # Map tilt X from -30 to +30 degrees directly to X position
        # Board width: 10 positions (0-9)
        # Tilt range: -30 to +30 (60 units total)
        # Mapping: every 6 degrees = 1 position
        # -30 to -24 = position 0
        # -24 to -18 = position 1
        # ...
        # 24 to 30 = position 9
        tilt = inputs.tilt_x
        tilt = max(-30.0, min(30.0, tilt))  # Clamp to range
        
        # Calculate target position: (tilt + 30) / 6
        # This maps -30 -> 0, 0 -> 5, +30 -> 10 (clamp to 9)
        target_x = int((tilt + 30.0) / 6.0)
        target_x = max(0, min(BOARD_WIDTH - 1, target_x))  # Clamp to valid range (0-9)
        
        # Get the actual leftmost cell position of the current piece
        cells = self.active.cells()
        if not cells:
            return
        min_cell_x = min(cx for cx, cy in cells)
        
        # Calculate how much we need to move to align leftmost cell with target
        # If target_x is 0, we want the leftmost cell to be at x=0
        # If target_x is 9, we want the leftmost cell to be at x=9
        current_leftmost = min_cell_x
        move_needed = target_x - current_leftmost
        
        # Move piece towards target position with rate limiting to prevent shaking
        if move_needed != 0:
            # Rate limiting: only move every 0.05 seconds
            self.move_hold_timer += dt
            if self.move_hold_timer >= 0.05:
                self.move_hold_timer = 0
                if move_needed > 0:
                    # Need to move right
                    self._try_move(dx=1)
                elif move_needed < 0:
                    # Need to move left
                    self._try_move(dx=-1)
        else:
            # Already at target, reset timer
            self.move_hold_timer = 0

    def _handle_rotation(self, inputs: InputSample):
        if inputs.button_b and not self.prev_inputs.button_b:
            self._try_rotate()

    def _gravity_step(self, dt: float):
        self.drop_timer += dt
        if self.drop_timer < self.drop_delay:
            return
        self.drop_timer = 0
        if not self._try_move(dy=1):
            self._lock_piece()

    def _max_delay_for_level(self) -> float:
        eligible = LEVEL_RULES[0]
        for rule in LEVEL_RULES:
            if self.score >= rule["score"]:
                eligible = rule
        self.level = eligible["level"]
        return eligible["max_delay"]

    def _try_move(self, dx: int = 0, dy: int = 0) -> bool:
        for cx, cy in self.active.cells(dx, dy):
            if cx < 0 or cx >= BOARD_WIDTH or cy >= BOARD_HEIGHT:
                return False
            if cy >= 0 and self.board[cy][cx]:
                return False
        self.active.x += dx
        self.active.y += dy
        return True

    def _try_rotate(self):
        for kick in (0, -1, 1, -2, 2):
            cells = self.active.cells(dx=kick, rotation_delta=1)
            if self._cells_valid(cells):
                self.active.x += kick
                self.active.rotation = (self.active.rotation + 1) % len(TETROMINOES[self.active.shape])
                return

    def _cells_valid(self, cells: Sequence[Tuple[int, int]]) -> bool:
        for cx, cy in cells:
            if cx < 0 or cx >= BOARD_WIDTH or cy >= BOARD_HEIGHT:
                return False
            if cy >= 0 and self.board[cy][cx]:
                return False
        return True

    def _lock_piece(self):
        # Lock the active piece to the board
        for cx, cy in self.active.cells():
            if cy < 0:
                self.events.append("GAME_OVER")
                self.reset()
                return
            # Ensure coordinates are valid
            if 0 <= cy < BOARD_HEIGHT and 0 <= cx < BOARD_WIDTH:
                self.board[cy][cx] = self.active.shape
        
        # Clear filled lines
        cleared = self._clear_lines()
        if cleared > 0:
            self.lines_cleared += cleared
            # Calculate score: [0, 10, 20, 30, 40] points per line cleared, multiplied by level
            gained = [0, 10, 20, 30, 40][min(cleared, 4)] * self.level
            self.score += gained
            self.events.append("LINE_CLEARED")
            print(f"[GAME] Locked piece. Cleared {cleared} lines. Gained {gained} points. Total score: {self.score}")
            
            # Check for level up
            if self._max_delay_for_level() < self.drop_delay:
                self.events.append("LEVEL_UP")
                print(f"[GAME] Level up! New level: {self.level}")
        
        # Spawn next piece
        self.active = self.next_piece
        self.next_piece = self._next_piece()
        self.hold_used = False

    def _clear_lines(self) -> int:
        """Find and remove completely filled lines. Returns number of lines cleared."""
        # Validate board structure first
        if not self.board or len(self.board) != BOARD_HEIGHT:
            print(f"[ERROR] Invalid board! Height: {len(self.board) if self.board else 0}, expected: {BOARD_HEIGHT}")
            return 0
        
        # Find rows that are NOT completely filled (have at least one empty cell)
        keep = []
        for i, row in enumerate(self.board):
            # Validate row
            if not row or len(row) != BOARD_WIDTH:
                print(f"[ERROR] Invalid row {i}! Width: {len(row) if row else 0}")
                keep.append([0 for _ in range(BOARD_WIDTH)])
                continue
            
            # Check if row has any empty cell (cell == 0)
            # If yes, keep the row. If no, row is full and will be removed.
            if any(cell == 0 for cell in row):
                keep.append(row)
        
        # Calculate how many lines were cleared
        cleared = BOARD_HEIGHT - len(keep)
        
        # Add empty rows at the top to maintain board height
        while len(keep) < BOARD_HEIGHT:
            keep.insert(0, [0 for _ in range(BOARD_WIDTH)])
        
        # Update board
        self.board = keep
        
        # Debug output
        if cleared > 0:
            print(f"[GAME] Cleared {cleared} line(s)! Board now has {len(self.board)} rows")
        
        return cleared

    def ghost_cells(self) -> List[Tuple[int, int]]:
        drop = 0
        while self._cells_valid(self.active.cells(dy=drop + 1)):
            drop += 1
        return self.active.cells(dy=drop)


class Menu:
    """Main menu system with options."""
    
    def __init__(self):
        self.state = "main"  # "main", "options"
        self.selected_option = 0
        self.main_menu_items = ["Start Game", "Options", "Exit"]
        self.options_menu_items = ["Sound", "Calibration"]
        self.sound_enabled = True
        self.font_title = None
        self.font_menu = None
        self.font_small = None
        self.game_started = False
        self.should_exit = False
        self._fonts_initialized = False
        self._last_encoder_position = None  # Track last known encoder position
    
    def _init_fonts(self):
        """Lazy initialization of fonts after pygame is initialized."""
        if not self._fonts_initialized:
            self.font_title = pygame.font.SysFont("consolas", 36, bold=True)
            self.font_menu = pygame.font.SysFont("consolas", 24)
            self.font_small = pygame.font.SysFont("consolas", 18)
            self._fonts_initialized = True
    
    def update(self, inputs: InputSample, prev_inputs: InputSample):
        """Update menu state based on encoder input. Returns command dict or None."""
        # Get encoder position from raw data
        current_position = None
        if hasattr(inputs, 'raw') and inputs.raw:
            encoder_data = inputs.raw.get('encoder', {})
            current_position = encoder_data.get('position')
        
        # Initialize position on first run
        if self._last_encoder_position is None:
            self._last_encoder_position = current_position if current_position is not None else 0
            return None
        
        # Handle encoder navigation - map encoder steps to menu options
        if current_position is not None:
            # Normalize encoder position to positive range (0-30)
            normalized_pos = abs(current_position) % 31
            
            # Determine which menu items to use
            if self.state == "main":
                menu_items = self.main_menu_items
            elif self.state == "options":
                menu_items = self.options_menu_items
            else:
                menu_items = []
            
            if len(menu_items) > 0:
                # Calculate steps per option (30 total steps divided by number of options)
                steps_per_option = 30 // len(menu_items)
                
                # Map encoder position to menu option
                if len(menu_items) == 2:
                    # 2 options: 0-10 = option 1, 11-20 = option 2
                    if normalized_pos <= 10:
                        new_selection = 0
                    else:
                        new_selection = 1
                elif len(menu_items) == 3:
                    # 3 options: 0-7 = option 1, 8-15 = option 2, 16-23 = option 3
                    if normalized_pos <= 7:
                        new_selection = 0
                    elif normalized_pos <= 15:
                        new_selection = 1
                    else:
                        new_selection = 2
                else:
                    # For other numbers of options, use equal distribution
                    steps_per_option = 20 // len(menu_items)
                    new_selection = min(normalized_pos // (steps_per_option + 1), len(menu_items) - 1)
                
                # Update selection if it changed
                if new_selection != self.selected_option:
                    self.selected_option = new_selection
                
                # Update last known position
                self._last_encoder_position = current_position
        else:
            # If position is None, keep last known position (encoder might not be sending data yet)
            pass
        
        # Handle button A press (selection)
        button_a_pressed = inputs.button_a or inputs.encoder_click
        button_a_prev = prev_inputs.button_a or (hasattr(prev_inputs, 'encoder_click') and prev_inputs.encoder_click)
        
        if button_a_pressed and not button_a_prev:
            if self.state == "main":
                if self.selected_option == 0:  # Start Game
                    self.game_started = True
                    return None
                elif self.selected_option == 1:  # Options
                    self.state = "options"
                    self.selected_option = 0
                elif self.selected_option == 2:  # Exit
                    self.should_exit = True
                    return {"command": "exit"}
            elif self.state == "options":
                if self.selected_option == 0:  # Sound
                    self.sound_enabled = not self.sound_enabled
                    return {"command": "sound_on" if self.sound_enabled else "sound_off"}
                elif self.selected_option == 1:  # Calibration
                    return {"command": "calibrate"}
        
        # Handle button B press (back)
        if inputs.button_b and not prev_inputs.button_b:
            if self.state == "options":
                self.state = "main"
                self.selected_option = 1  # Keep Options selected when going back
        
        return None
    
    def draw(self, screen: pygame.Surface, inputs: InputSample = None):
        """Draw the menu."""
        self._init_fonts()
        screen.fill((15, 15, 30))
        
        # Debug info at bottom (only in development)
        if inputs:
            encoder_pos = 0
            if hasattr(inputs, 'raw') and inputs.raw:
                encoder_pos = inputs.raw.get('encoder', {}).get('position', 0)
            last_pos = self._last_encoder_position if self._last_encoder_position is not None else 0
            pos_change = encoder_pos - last_pos if self._last_encoder_position is not None else 0
            debug_text = self.font_small.render(
                f"Encoder Pos: {encoder_pos} | Change: {pos_change:+d} | Selected: {self.selected_option} | A:{int(inputs.button_a)} B:{int(inputs.button_b)}",
                True, (255, 200, 100)  # More visible color
            )
            screen.blit(debug_text, (10, SCREEN_HEIGHT - 25))
        
        if self.state == "main":
            # Main menu
            title = self.font_title.render("Dentist Tetris", True, (255, 255, 255))
            title_rect = title.get_rect(center=(SCREEN_WIDTH // 2, 120))
            screen.blit(title, title_rect)
            
            y_start = SCREEN_HEIGHT // 2 - 30
            for i, item in enumerate(self.main_menu_items):
                color = (255, 255, 100) if i == self.selected_option else (200, 200, 200)
                prefix = "> " if i == self.selected_option else "  "
                text = f"{prefix}{item}"
                
                item_surface = self.font_menu.render(text, True, color)
                item_rect = item_surface.get_rect(center=(SCREEN_WIDTH // 2, y_start + i * 60))
                screen.blit(item_surface, item_rect)
            
            hint = self.font_small.render("Encoder: Navigate | Button A: Select | Button B: Back", True, (150, 150, 150))
            hint_rect = hint.get_rect(center=(SCREEN_WIDTH // 2, SCREEN_HEIGHT - 50))
            screen.blit(hint, hint_rect)
        
        elif self.state == "options":
            # Options menu
            title = self.font_title.render("Options", True, (255, 255, 255))
            title_rect = title.get_rect(center=(SCREEN_WIDTH // 2, 80))
            screen.blit(title, title_rect)
            
            y_start = SCREEN_HEIGHT // 2 - 50
            for i, item in enumerate(self.options_menu_items):
                color = (255, 255, 100) if i == self.selected_option else (200, 200, 200)
                prefix = "> " if i == self.selected_option else "  "
                
                if item == "Sound":
                    value = "ON" if self.sound_enabled else "OFF"
                    text = f"{prefix}{item}: {value}"
                else:
                    text = f"{prefix}{item}"
                
                item_surface = self.font_menu.render(text, True, color)
                item_rect = item_surface.get_rect(center=(SCREEN_WIDTH // 2, y_start + i * 60))
                screen.blit(item_surface, item_rect)
            
            hint = self.font_small.render("Button B: Back to Main Menu", True, (150, 150, 150))
            hint_rect = hint.get_rect(center=(SCREEN_WIDTH // 2, SCREEN_HEIGHT - 50))
            screen.blit(hint, hint_rect)


class GameDisplay:
    def __init__(self):
        pygame.init()
        # Set display to fullscreen
        self.screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
        # Get actual screen dimensions
        self.screen_width, self.screen_height = self.screen.get_size()
        # Update global screen dimensions for menu rendering
        global SCREEN_WIDTH, SCREEN_HEIGHT
        SCREEN_WIDTH = self.screen_width
        SCREEN_HEIGHT = self.screen_height
        pygame.display.set_caption("Dentist Tetris – Hardware-in-the-Loop")
        self.font_small = pygame.font.SysFont("consolas", 18)
        self.font_large = pygame.font.SysFont("consolas", 28, bold=True)
        self.font_pause = pygame.font.SysFont("consolas", 48, bold=True)

    def draw(self, game: TetrisGame, inputs: InputSample):
        self.screen.fill((15, 15, 30))
        self._draw_board(game)
        self._draw_piece(game.active)
        self._draw_ghost(game)
        self._draw_side_panel(game, inputs)
        
        # Draw pause menu if game is paused
        if game.paused:
            self._draw_pause_overlay(game)
        
        pygame.display.flip()  # Use flip() instead of update() for fullscreen performance

    def _draw_board(self, game: TetrisGame):
        for y in range(VISIBLE_HEIGHT):
            for x in range(BOARD_WIDTH):
                cell = game.board[y + (BOARD_HEIGHT - VISIBLE_HEIGHT)][x]
                rect = pygame.Rect(x * TILE_SIZE, y * TILE_SIZE, TILE_SIZE, TILE_SIZE)
                pygame.draw.rect(self.screen, COLORS[0], rect, 1)
                if cell:
                    pygame.draw.rect(self.screen, COLORS[cell], rect.inflate(-2, -2))

    def _draw_piece(self, piece: Piece):
        for x, y in piece.cells():
            if y < BOARD_HEIGHT - VISIBLE_HEIGHT:
                continue
            screen_y = y - (BOARD_HEIGHT - VISIBLE_HEIGHT)
            rect = pygame.Rect(x * TILE_SIZE, screen_y * TILE_SIZE, TILE_SIZE, TILE_SIZE)
            pygame.draw.rect(self.screen, COLORS[piece.shape], rect.inflate(-2, -2))

    def _draw_ghost(self, game: TetrisGame):
        for x, y in game.ghost_cells():
            if y < BOARD_HEIGHT - VISIBLE_HEIGHT:
                continue
            screen_y = y - (BOARD_HEIGHT - VISIBLE_HEIGHT)
            rect = pygame.Rect(x * TILE_SIZE, screen_y * TILE_SIZE, TILE_SIZE, TILE_SIZE)
            pygame.draw.rect(self.screen, COLORS["ghost"], rect, 1)

    def _draw_side_panel(self, game: TetrisGame, inputs: InputSample):
        x0 = BOARD_WIDTH * TILE_SIZE + 10
        y = 20
        self._blit_text(f"Score: {game.score}", self.font_large, (x0, y))
        y += 40
        self._blit_text(f"Level: {game.level}", self.font_large, (x0, y))
        y += 40
        self._blit_text(f"Lines: {game.lines_cleared}", self.font_small, (x0, y))
        y += 40
        self._blit_text("Next:", self.font_small, (x0, y))
        self._draw_next_piece(game.next_piece, x0, y + 25)
        y += 120
        self._blit_text("Tilt X: {:.1f}".format(inputs.tilt_x), self.font_small, (x0, y))
        y += 22
        self._blit_text(f"Buttons A/B: {int(inputs.button_a)}/{int(inputs.button_b)}", self.font_small, (x0, y))
        y += 22
        self._blit_text(f"Encoder Δ: {inputs.encoder_delta}", self.font_small, (x0, y))
        y += 22
        normalized = inputs.pot_value / 65535
        self._blit_text(f"Pot: {normalized:.2f}", self.font_small, (x0, y))

    def _draw_next_piece(self, piece: Piece, x0: int, y0: int):
        for x, y in piece.cells():
            rect = pygame.Rect(x0 + x * 18, y0 + y * 18, 16, 16)
            pygame.draw.rect(self.screen, COLORS[piece.shape], rect)

    def _blit_text(self, text: str, font: pygame.font.Font, pos: Tuple[int, int]):
        surface = font.render(text, True, (230, 230, 230))
        self.screen.blit(surface, pos)
    
    def _draw_pause_overlay(self, game: TetrisGame):
        """Draw the pause menu overlay."""
        game.pause_menu.draw(self.screen)


def feedback_packet(game: TetrisGame) -> Dict:
    return {
        "score": game.score,
        "level": game.level,
        "events": game.events[:],
        "lines": game.lines_cleared,
        "drop_delay": game.drop_delay,
    }


def run_game(args: argparse.Namespace):
    # Initialize pygame first
    pygame.init()
    
    clock = pygame.time.Clock()
    try:
        bridge = MockBridge() if args.mock_input else PicoBridge(args.port)
    except Exception as e:
        print(f"Failed to initialize bridge: {e}")
        print("Falling back to mock input mode...")
        bridge = MockBridge()
    
    menu = Menu()
    game = TetrisGame()
    display = GameDisplay()
    last_feedback = 0.0
    prev_inputs = InputSample()

    running = True
    in_menu = True
    
    # Start menu music
    try:
        bridge.send_feedback({"command": "music_menu"})
    except:
        pass
    
    while running:
        dt = clock.tick(240) / 1000.0  # Increased to 240 FPS for maximum responsiveness
        
        # Process events efficiently - only check for QUIT
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                break

        try:
            # Try to read new input sample - read multiple times per frame if available
            new_sample = None
            for _ in range(3):  # Try reading up to 3 times per frame
                sample = bridge.read_inputs()
                if sample is not None:
                    new_sample = sample
                else:
                    break
            
            # Always use new sample if available, otherwise reuse previous
            if new_sample is not None:
                sample = new_sample
            else:
                # Reuse previous sample for faster processing (but need to preserve for edge detection)
                sample = prev_inputs
            
            if in_menu:
                # Menu mode
                command = menu.update(sample, prev_inputs)
                if command:
                    bridge.send_feedback(command)
                    # Check if exit was requested
                    if menu.should_exit:
                        print("Exiting game...")
                        running = False
                        break
                
                menu.draw(display.screen, sample)
                pygame.display.flip()
                
                if menu.game_started:
                    in_menu = False
                    print("Starting game...")
                    bridge.send_feedback({"command": "music_game"})  # Start game music
            else:
                # Game mode
                # Handle pause menu if paused
                if game.paused:
                    pause_result = game.pause_menu.update(sample, prev_inputs)
                    if pause_result:
                        if "action" in pause_result:
                            if pause_result["action"] == "resume":
                                game.paused = False
                                bridge.send_feedback({"command": "music_game"})
                            elif pause_result["action"] == "main_menu":
                                # Return to main menu - reset everything to beginning
                                in_menu = True
                                game.reset()
                                menu.game_started = False  # Reset menu state
                                menu.state = "main"  # Go back to main menu
                                menu.selected_option = 0  # Reset selection
                                game.pause_menu._last_encoder_position = None  # Reset encoder tracking
                                bridge.send_feedback({"command": "music_menu"})
                        elif "command" in pause_result:
                            bridge.send_feedback(pause_result)
                else:
                    # Normal game update
                    events = game.update(sample, dt)
                    
                    # Send music commands for scoring events
                    for event in events:
                        if event == "LINE_CLEARED":
                            bridge.send_feedback({"command": "music_scoring"})
                
                display.draw(game, sample)

                # Send feedback more frequently for better responsiveness
                send_now = time.time() - last_feedback > 0.02 or events if not game.paused else False
                if send_now:
                    bridge.send_feedback(feedback_packet(game))
                    last_feedback = time.time()
            
            prev_inputs = sample  # Update prev_inputs at the end
        except KeyboardInterrupt:
            running = False
        except Exception as e:
            print(f"Game loop error: {e}")
            # Continue running to avoid crash

    pygame.quit()


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="PC display for Dentist Tetris.")
    parser.add_argument("--port", type=str, default="COM5", help="Serial port of the Pico (ignored in mock mode).")
    parser.add_argument("--mock-input", action="store_true", help="Use keyboard to simulate Pico inputs.")
    return parser.parse_args(argv)


if __name__ == "__main__":
    run_game(parse_args())

