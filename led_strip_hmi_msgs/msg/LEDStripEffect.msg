# LEDStripEffect.msg
# ------------------------------------------------------------------------------
# An “appearance” descriptor for a detection on the virtual strip.
# You can extend it later (e.g. add color, brightness, etc.)

# --- Shape enum & params ---
uint8 SHAPE_GAUSSIAN=0
uint8 SHAPE_RECTANGLE=1
uint8 SHAPE_TRIANGLE=2
# …add more shapes as needed
uint8 shape

# shape_param is interpreted per-shape:
#  - GAUSSIAN: variance
#  - RECTANGLE: half-width
#  - TRIANGLE: half-base
float32 shape_param

# --- Effect enum & params ---
uint8 EFFECT_NONE=0
uint8 EFFECT_WAVE=1
uint8 EFFECT_PULSE=2
# …add more effects as needed
uint8 effect

# frequency in Hz for wave/pulse
float32 frequency