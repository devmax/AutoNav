# header
Header      header

# ----------------- raw 10d filter state ----------------------------
float32     x
float32     y
float32     z

float32     Lx
float32     Ly
float32     Lz

float32     dx
float32     dy
float32     dz

float32     roll
float32     pitch
float32     yaw
float32     dyaw

# ----------------- propagated from drone messages: -----------------
# 0: Unknown, 1: Init, 2: Landed, 3: Flying, 4: Hovering, 5: Test
# 6: Taking off, 7: Goto Fix Point, 8: Landing, 9: Looping
# Note: 3,7 seems to discriminate type of flying (isFly = 3 | 7)
uint32       droneState
float32      batteryPercent    # 0 means no battery, 100 means full battery
