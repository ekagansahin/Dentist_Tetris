# TetrisKart v2 - Hardware-in-the-Loop Tetris Game

## Overview

This project implements a Tetris game that uses a Raspberry Pi Pico microcontroller to read hardware sensors (IMU, buttons, rotary encoder, potentiometer) and control the game, while the PC handles the game logic and display rendering.
Created by Trifaze-ME461 METU

## Architecture

The system consists of two main components:

1. **Pico Side (`main.py`)**: Runs on Raspberry Pi Pico (MicroPython)
   - Samples sensors every 10ms
   - Streams sensor data to PC via USB serial
   - Receives game feedback and updates displays (OLED + 8x8 LED matrix)
   - Handles audio/visual effects

2. **PC Side (`display.py`)**: Runs on PC (Python)
   - Reads sensor data from Pico via serial
   - Runs Tetris game logic
   - Renders game using Pygame
   - Sends game state back to Pico

## How `main.py` Works

### Core Components

#### 1. **USBBridge Class**
- Handles bidirectional JSON communication over USB serial
- Uses `uselect` for non-blocking input polling
- `send()`: Serializes Python dicts to JSON and sends to PC
- `receive()`: Reads JSON lines from PC and parses them

#### 2. **RotaryEncoder Class**
- Implements quadrature decoding for rotary encoder
- Uses state machine with transition table to detect rotation direction
- Polls encoder pins every 1ms for maximum responsiveness
- Reports delta (change) since last read

#### 3. **MusicManager Class**
- Manages background music playback using PWM buzzer
- Supports three tracks:
  - Menu music (Nokia ringtone)
  - Game music (Star Wars theme)
  - Scoring music (short celebratory tune)
- Handles looping and volume control
- Can be muted/unmuted

#### 4. **EffectsEngine Class**
- Manages visual effects (4 LEDs) and audio effects (buzzer)
- Triggers one-shot effects for game events:
  - LINE_CLEARED: 1400 Hz, 120ms
  - LEVEL_UP: 800 Hz, 200ms
  - GAME_OVER: 200 Hz, 400ms
- Coordinates with MusicManager to avoid conflicts

#### 5. **DisplayManager Class**
- Manages two displays:
  - **SSD1306 OLED** (128x64): Shows score, level, lines cleared
  - **MAX7219 8x8 Matrix**: Shows last 2 digits of score and level indicator
- Auto-detects OLED I2C address
- Updates displays only when score/level changes

#### 6. **SensorSuite Class**
- Main hardware interface class
- Initializes all sensors and displays:
  - **MPU6050 IMU**: I2C bus (GPIO 12/13)
  - **Buttons**: GPIO 10 (Button A), GPIO 11 (Button B)
  - **Rotary Encoder**: GPIO 14/15 (quadrature inputs)
  - **Potentiometer**: ADC0 (GPIO 26)
  - **LEDs**: GPIO 25, 26, 27, 28
  - **Buzzer**: GPIO 9 (PWM)
- **Calibration**: Measures tilt offset when device is at rest
- **Sampling**: Collects all sensor readings into JSON payload

#### 7. **Main Loop**
- Runs at high frequency (no sleep, minimal delay)
- Polls encoder every 1ms
- Samples all sensors every 10ms
- Sends sensor data to PC
- Receives and processes feedback from PC
- Updates effects (music, LEDs) continuously

### Sensor Data Format

Each sensor sample is sent as JSON:
```json
{
  "ts": 12345,
  "tilt": {"x": 5.2, "y": -1.3},
  "buttons": {"a": 0, "b": 1},
  "encoder": {"delta": 2, "position": 45},
  "pot": 32768
}
```

### Feedback Commands

The Pico receives commands from PC:
- `{"command": "calibrate"}`: Start tilt calibration
- `{"command": "music_menu"}`: Play menu music
- `{"command": "music_game"}`: Play game music
- `{"command": "music_stop"}`: Stop music
- `{"command": "sound_on"}` / `{"command": "sound_off"}`: Enable/disable sound
- `{"command": "exit"}`: Reset system for new game

Game feedback includes:
- `{"score": 100, "level": 2, "lines": 5, "events": ["LINE_CLEARED"]}`

## How `display.py` Works

### Core Components

#### 1. **PicoBridge Class**
- Manages serial communication with Pico
- Reads JSON lines from serial port
- Converts raw sensor data to `InputSample` objects
- Sends feedback packets back to Pico

#### 2. **MockBridge Class**
- Fallback for development without hardware
- Maps keyboard keys to sensor inputs:
  - Arrow keys → tilt
  - Space → Button A
  - Up arrow → Button B
  - Page Up/Down → encoder
  - Down arrow → potentiometer

#### 3. **InputSample Dataclass**
- Structured representation of sensor data
- Includes timestamp, tilt angles, button states, encoder delta, pot value

#### 4. **TetrisGame Class**
- Core game logic implementation
- **Board**: 10x22 grid (last 2 rows hidden)
- **Pieces**: Standard 7 tetrominoes (I, O, T, S, Z, J, L)
- **Controls**:
  - **Tilt X**: Maps -30° to +30° to board positions 0-9
  - **Button B**: Rotate piece
  - **Button A**: Pause/unpause
  - **Potentiometer**: Controls drop speed (0.05s to 0.70s)
- **Scoring**: 0/10/20/30/40 points per line × level
- **Levels**: 5 levels with increasing speed
- **Gravity**: Automatic piece dropping based on drop_delay

#### 5. **Menu System**
- **Main Menu**: Start Game, Options, Exit
- **Options Menu**: Sound toggle, Calibration
- **Pause Menu**: Resume, Main Menu, Mute
- Navigation via rotary encoder
- Selection via Button A

#### 6. **GameDisplay Class**
- Pygame-based rendering
- Fullscreen display
- Renders:
  - Game board with pieces
  - Ghost piece (preview of landing position)
  - Side panel with score, level, next piece
  - Sensor debug info
- Handles pause menu overlay

#### 7. **Main Game Loop**
- Runs at 240 FPS for maximum responsiveness
- Reads sensor data from Pico (up to 3 times per frame)
- Updates game state or menu based on current mode
- Renders display
- Sends feedback to Pico every 20ms or on events

### Control Mapping

| Hardware Input | Game Function |
|----------------|---------------|
| Tilt X (-30° to +30°) | Horizontal piece position (0-9) |
| Button A | Pause/Menu selection |
| Button B | Rotate piece / Back in menu |
| Rotary Encoder | Menu navigation |
| Potentiometer | Drop speed (0.05s - 0.70s) |

### Game Flow

1. **Startup**: Menu music plays, main menu displayed
2. **Menu Navigation**: Use encoder to select, Button A to confirm
3. **Game Start**: Game music plays, Tetris game begins
4. **Gameplay**: 
   - Tilt device to move piece horizontally
   - Button B to rotate
   - Potentiometer adjusts drop speed
   - Button A pauses
5. **Pause Menu**: Navigate with encoder, select with Button A
6. **Game Events**: Line clears, level ups trigger sound/visual effects
7. **Game Over**: Piece locks above visible area → reset board

## Communication Protocol

### Pico → PC (Sensor Data)
- Format: JSON line per sample
- Frequency: ~100 Hz (every 10ms)
- Fields: timestamp, tilt, buttons, encoder, pot

### PC → Pico (Feedback)
- Format: JSON line per update
- Frequency: ~50 Hz (every 20ms) or on events
- Fields: score, level, lines, events, commands

## Dependencies

### Pico Side (MicroPython)
- Built-in: `machine`, `ujson`, `uselect`, `utime`
- Custom modules: `imu.py`, `vector3d.py`, `ssd1306.py`, `max7219.py`

### PC Side (Python)
- `pygame`: Game rendering and input
- `pyserial`: Serial communication with Pico
- Standard library: `json`, `argparse`, `dataclasses`, `time`, `math`, `random`

## Running the System

### On Pico:
1. Upload `main.py`, `imu.py`, `vector3d.py`, `ssd1306.py`, `max7219.py` to Pico
2. Run `main.py` (it will wait for PC connection)

### On PC:
```bash
# With hardware
python display.py --port COM5  # Windows
python display.py --port /dev/ttyACM0  # Linux

# Without hardware (keyboard mode)
python display.py --mock-input
```

## Hardware Setup

See `pinout.txt` for detailed pin connections.

