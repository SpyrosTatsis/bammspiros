"""
variable_rgb_bridge.py
Pi Pico 2 — MicroPython

For boards with:
  - Variable voltage on R, G, B cathode lines (color encoded as voltage)
  - Stable voltage on + anode rail (just power, ignore it)

This script reads the RGB voltages, maps them to 0-255,
classifies the color, and drives a Neopixel blade with
matching color and effects.

Wiring
------
Board R pad  -->  GP26  (ADC0)
Board G pad  -->  GP27  (ADC1)
Board B pad  -->  GP28  (ADC2)
Neopixel     <--  GP0
Common GND between Pico and board

CALIBRATION
-----------
Run debug_adc.py first and note the 0-255 values for each
color profile. Adjust CHANNEL_MAX and DEAD_ZONE below to match
what you observed.
"""

import machine
import neopixel
import time
import math
import random

# ════════════════════════════════════════════════════════════════════════════════
#  CALIBRATION — adjust these from your debug_adc.py readings
# ════════════════════════════════════════════════════════════════════════════════

# The maximum 0-255 value you observed on any channel at full brightness.
# If your board only goes up to e.g. 180 on a full-on channel, set this to 180.
# This lets us remap that to 255 so colors are vivid on the Neopixel.
CHANNEL_MAX = 255

# Readings below this are treated as zero (noise floor / channel off).
# Raise this if you get spurious color detection when the saber is off.
DEAD_ZONE = 12

# Minimum total brightness (sum of R+G+B after noise floor) to count as blade on.
# Raise if the blade triggers when saber is off.
ON_THRESHOLD = 30

# Consecutive near-zero frames before calling the blade off.
EXTINGUISH_FRAMES = 6

# Frames to skip after ignition before reacting to color changes.
STABLE_FRAMES = 10

# ════════════════════════════════════════════════════════════════════════════════
#  BLADE CONFIG
# ════════════════════════════════════════════════════════════════════════════════

BLADE_LEDS  = 144
BLADE_PIN   = 0
BRIGHTNESS  = 0.9
POLL_MS     = 16

# If True, the Neopixel blade mirrors the exact color read from the board.
# If False, the board color is snapped to the nearest named color below.
EXACT_COLOR_MATCH = True

# Named colors used when EXACT_COLOR_MATCH = False, or for clash/idle labeling.
# You can add your own entries here.
NAMED_COLORS = {
    "red"     : (255,   0,   0),
    "green"   : (  0, 255,   0),
    "blue"    : (  0,   0, 255),
    "cyan"    : (  0, 255, 255),
    "magenta" : (255,   0, 255),
    "yellow"  : (255, 200,   0),
    "orange"  : (255,  80,   0),
    "purple"  : (128,   0, 255),
    "lime"    : ( 80, 255,   0),
    "white"   : (255, 255, 255),
    "ice"     : (180, 220, 255),
}

# ── Effect styles ─────────────────────────────────────────────────────────────
IGNITE_STYLE    = "scroll"    # scroll | flicker | pulse | instant
EXTINGUISH_STYLE = "scroll"   # scroll | retract | pulse | instant
IDLE_EFFECT     = "shimmer"   # stable | shimmer | breathe | flicker

CLASH_COLOR     = (255, 255, 255)
CLASH_FLASHES   = 3
CLASH_FLASH_MS  = 40
IGNITE_MS       = 600
EXTINGUISH_MS   = 500

# ════════════════════════════════════════════════════════════════════════════════
#  HARDWARE
# ════════════════════════════════════════════════════════════════════════════════

blade = neopixel.NeoPixel(machine.Pin(BLADE_PIN), BLADE_LEDS)
adc_r = machine.ADC(26)
adc_g = machine.ADC(27)
adc_b = machine.ADC(28)

# ════════════════════════════════════════════════════════════════════════════════
#  ADC READING
# ════════════════════════════════════════════════════════════════════════════════

def read_raw():
    """
    Sample each channel 8 times and average for stability.
    Returns (r, g, b) in 0-255 range.
    Apply dead zone — values below DEAD_ZONE are treated as 0.
    Then remap 0-CHANNEL_MAX to 0-255 so we use full Neopixel range.
    """
    def sample(adc):
        raw = sum(adc.read_u16() for _ in range(8)) // 8 >> 8
        if raw < DEAD_ZONE:
            return 0
        if CHANNEL_MAX < 255:
            raw = min(255, int(raw * 255 / CHANNEL_MAX))
        return raw

    return sample(adc_r), sample(adc_g), sample(adc_b)

def blade_is_on(r, g, b):
    return (r + g + b) > ON_THRESHOLD

# ════════════════════════════════════════════════════════════════════════════════
#  COLOR CLASSIFICATION
# ════════════════════════════════════════════════════════════════════════════════

def classify_color(r, g, b):
    """
    Find the closest named color by Euclidean distance in RGB space.
    Returns (name, (r, g, b)).
    """
    best_name  = "white"
    best_dist  = float("inf")
    for name, (nr, ng, nb) in NAMED_COLORS.items():
        dist = math.sqrt((r - nr)**2 + (g - ng)**2 + (b - nb)**2)
        if dist < best_dist:
            best_dist  = dist
            best_name  = name
    return best_name, NAMED_COLORS[best_name]

def smooth_color(prev, curr, alpha=0.25):
    """
    Exponential moving average between previous and current color.
    Prevents flickering from small ADC noise between frames.
    alpha=1.0 = instant response, alpha=0.1 = very smooth/slow.
    """
    return (
        int(prev[0] + (curr[0] - prev[0]) * alpha),
        int(prev[1] + (curr[1] - prev[1]) * alpha),
        int(prev[2] + (curr[2] - prev[2]) * alpha),
    )

# ════════════════════════════════════════════════════════════════════════════════
#  COLOR HELPERS
# ════════════════════════════════════════════════════════════════════════════════

def dim(color, factor):
    return (
        int(color[0] * factor),
        int(color[1] * factor),
        int(color[2] * factor),
    )

def fill_all(color):
    for i in range(BLADE_LEDS):
        blade[i] = color
    blade.write()

def fill_off():
    fill_all((0, 0, 0))

def fill_blade(color):
    c = dim(color, BRIGHTNESS)
    for i in range(BLADE_LEDS):
        blade[i] = c
    blade.write()

# ════════════════════════════════════════════════════════════════════════════════
#  IGNITION EFFECTS
# ════════════════════════════════════════════════════════════════════════════════

def ignite_scroll(color):
    c = dim(color, BRIGHTNESS)
    delay = (IGNITE_MS / BLADE_LEDS) / 1000
    for i in range(BLADE_LEDS):
        blade[i] = c
        blade.write()
        time.sleep(delay)

def ignite_flicker(color):
    c = dim(color, BRIGHTNESS)
    end = time.ticks_add(time.ticks_ms(), IGNITE_MS - 150)
    while time.ticks_diff(end, time.ticks_ms()) > 0:
        target = random.randint(int(BLADE_LEDS * 0.3), BLADE_LEDS)
        factor = random.uniform(0.3, 1.0)
        fc = dim(c, factor)
        for i in range(target):
            blade[i] = fc
        for i in range(target, BLADE_LEDS):
            blade[i] = (0, 0, 0)
        blade.write()
        time.sleep_ms(random.randint(20, 60))
    # settle
    for step in range(20):
        t = (step / 19) ** 2
        for i in range(BLADE_LEDS):
            blade[i] = dim(c, t)
        blade.write()
        time.sleep_ms(8)

def ignite_pulse(color):
    steps = 40
    for step in range(steps):
        t = (step / (steps - 1)) ** 2
        for i in range(BLADE_LEDS):
            blade[i] = dim(color, t * BRIGHTNESS)
        blade.write()
        time.sleep_ms(IGNITE_MS // steps)

def do_ignite(color):
    if IGNITE_STYLE == "scroll":
        ignite_scroll(color)
    elif IGNITE_STYLE == "flicker":
        ignite_flicker(color)
    elif IGNITE_STYLE == "pulse":
        ignite_pulse(color)
    else:
        fill_blade(color)

# ════════════════════════════════════════════════════════════════════════════════
#  EXTINGUISH EFFECTS
# ════════════════════════════════════════════════════════════════════════════════

def extinguish_scroll():
    delay = (EXTINGUISH_MS / BLADE_LEDS) / 1000
    for i in range(BLADE_LEDS):
        blade[i] = (0, 0, 0)
        blade.write()
        time.sleep(delay)

def extinguish_retract():
    delay = (EXTINGUISH_MS / BLADE_LEDS) / 1000
    for i in range(BLADE_LEDS - 1, -1, -1):
        blade[i] = (0, 0, 0)
        blade.write()
        time.sleep(delay)

def extinguish_pulse(color):
    steps = 40
    for step in range(steps):
        t = (1.0 - step / (steps - 1)) ** 2
        for i in range(BLADE_LEDS):
            blade[i] = dim(color, t * BRIGHTNESS)
        blade.write()
        time.sleep_ms(EXTINGUISH_MS // steps)
    fill_off()

def do_extinguish(color):
    if EXTINGUISH_STYLE == "scroll":
        extinguish_scroll()
    elif EXTINGUISH_STYLE == "retract":
        extinguish_retract()
    elif EXTINGUISH_STYLE == "pulse":
        extinguish_pulse(color)
    else:
        fill_off()

# ════════════════════════════════════════════════════════════════════════════════
#  IDLE EFFECTS
# ════════════════════════════════════════════════════════════════════════════════

_idle_frame = 0

def do_idle(color):
    global _idle_frame
    _idle_frame += 1
    c = dim(color, BRIGHTNESS)

    if IDLE_EFFECT == "shimmer":
        if _idle_frame % 3 != 0:
            return                          
        for i in range(BLADE_LEDS):
            v = random.uniform(0.88, 1.0)
            blade[i] = dim(c, v)
        blade.write()

    elif IDLE_EFFECT == "breathe":
        t = (_idle_frame * POLL_MS) / 3000
        factor = 0.55 + 0.45 * math.sin(t * 2 * math.pi)
        fill_all(dim(color, factor * BRIGHTNESS))

    elif IDLE_EFFECT == "flicker":
        for i in range(BLADE_LEDS):
            noise = random.uniform(0.7, 1.0)
            tip_falloff = 1.0 - (i / BLADE_LEDS) * 0.3
            blade[i] = dim(c, noise * tip_falloff)
        blade.write()

    # stable — do nothing, blade already filled correctly

# ════════════════════════════════════════════════════════════════════════════════
#  CLASH
# ════════════════════════════════════════════════════════════════════════════════

def do_clash(current_color):
    cc = dim(CLASH_COLOR, BRIGHTNESS)
    bc = dim(current_color, BRIGHTNESS)
    for _ in range(CLASH_FLASHES):
        fill_all(cc)
        time.sleep_ms(CLASH_FLASH_MS)
        fill_all(bc)
        time.sleep_ms(CLASH_FLASH_MS)
    fill_blade(current_color)

# ════════════════════════════════════════════════════════════════════════════════
#  COLOR CHANGE DETECTION
# ════════════════════════════════════════════════════════════════════════════════

def color_changed(prev, curr, threshold=15):
    """
    Returns True if the color has shifted enough to be a real change
    rather than ADC noise. threshold is the minimum distance in RGB space.
    """
    dist = math.sqrt(
        (prev[0] - curr[0])**2 +
        (prev[1] - curr[1])**2 +
        (prev[2] - curr[2])**2
    )
    return dist > threshold

# ════════════════════════════════════════════════════════════════════════════════
#  STATE MACHINE
# ════════════════════════════════════════════════════════════════════════════════

STATE_OFF = 0
STATE_ON  = 1

state         = STATE_OFF
zero_count    = 0
stable_count  = 0
current_color = (0, 0, 0)   # smoothed display color
display_color = (0, 0, 0)   # what is actually on the blade


def process(r, g, b):
    global state, zero_count, stable_count, current_color, display_color

    on = blade_is_on(r, g, b)

    if state == STATE_OFF:
        if on:
            # Read actual color and classify
            if EXACT_COLOR_MATCH:
                target = (r, g, b)
            else:
                _, target = classify_color(r, g, b)

            current_color = target
            display_color = target
            do_ignite(target)
            fill_blade(target)
            state        = STATE_ON
            zero_count   = 0
            stable_count = 0
            name, _ = classify_color(r, g, b)
            print(f"Ignite — detected: {name} raw=({r},{g},{b})")       
    elif state == STATE_ON:
        if stable_count < STABLE_FRAMES:
            stable_count += 1
            return

        if not on:
            zero_count += 1
            if zero_count >= EXTINGUISH_FRAMES:
                do_extinguish(display_color)
                state      = STATE_OFF
                zero_count = 0
                print("Extinguish")
        else:
            zero_count = 0

            # Smooth the incoming color to reduce noise
            raw_color     = (r, g, b)
            current_color = smooth_color(current_color, raw_color, alpha=0.2)

            # Only update blade if color has shifted meaningfully
            if color_changed(display_color, current_color, threshold=15):
                display_color = current_color
                fill_blade(display_color)
                name, _ = classify_color(*display_color)
                print(f"Color change — {name} ({display_color[0]},{display_color[1]},{display_color[2]})")
            else:
                do_idle(display_color)

# ════════════════════════════════════════════════════════════════════════════════
#  MAIN
# ════════════════════════════════════════════════════════════════════════════════

fill_off()
print("Variable RGB bridge ready")
print(f"Mode        : {'exact color match' if EXACT_COLOR_MATCH else 'snap to named color'}")
print(f"Ignite      : {IGNITE_STYLE}")
print(f"Extinguish  : {EXTINGUISH_STYLE}")
print(f"Idle        : {IDLE_EFFECT}")
print(f"Dead zone   : {DEAD_ZONE}  |  On threshold : {ON_THRESHOLD}")
print("Waiting for signal on GP26/27/28...")

while True:
    r, g, b = read_raw()
    process(r, g, b)
    time.sleep_ms(POLL_MS)
