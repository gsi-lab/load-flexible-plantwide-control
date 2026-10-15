"""Models, controller tunings and recorded variables of the two control systems."""

# PI tunings of Table 2 of the paper (Kp: controller gain, Ti: integral time in s).
# The loops below are the same in both control systems.
_common = [
    {"model": "XC2", "unit": "fraction", "Kp": 72.4, "Ti": 240},        # level, first flash tank
    {"model": "XC1", "unit": "fraction", "Kp": 83.0, "Ti": 240},        # level, second flash tank
    {"model": "XC3", "unit": "mole fraction", "Kp": 39.3, "Ti": 172},   # ammonia purity
    {"model": "XC9", "unit": "kg/s", "Kp": 31.5, "Ti": 6.7},            # nitrogen feed
    {"model": "XC13", "unit": "K", "Kp": 23.9, "Ti": 360},              # inlet temperature, third bed
    {"model": "XC14", "unit": "K", "Kp": 12.3, "Ti": 44},               # inlet temperature, second bed
    {"model": "XC15", "unit": "K", "Kp": 133, "Ti": 40},                # inlet temperature, first bed
    {"model": "XC10", "unit": "kPa", "Kp": 0.11, "Ti": 7},              # first-stage pressure
    {"model": "XC11", "unit": "kPa", "Kp": 1.15, "Ti": 82},             # second-stage pressure
    {"model": "XC12", "unit": "kg/s", "Kp": 0.44, "Ti": 3.5},           # hydrogen feed
    {"model": "XC16", "unit": "kPa", "Kp": 1.5, "Ti": 266},             # loop pressure
    {"model": "XC4", "unit": "m3/s", "Kp": 0.97, "Ti": 6.5},            # anti-surge, first stage
    {"model": "XC5", "unit": "m3/s", "Kp": 1, "Ti": 5.6},               # anti-surge, second stage
    {"model": "XC6", "unit": "m3/s", "Kp": 1.2, "Ti": 6.0},             # anti-surge, third stage
]

# Outlet temperatures of the three reactor beds and argon fraction at the reactor inlet
_recorded = [
    {"model": "S42", "variable": "T", "unit": "K"},
    {"model": "S47", "variable": "T", "unit": "K"},
    {"model": "S73", "variable": "T", "unit": "K"},
    {"model": "S1", "variable": "z[ARGON]", "unit": "mole fraction"},
]

CONTROL_SYSTEMS = {
    "basic": {
        "model": "FlexibleHB_Basic",
        "controllers": _common + [
            {"model": "XC8", "unit": "K", "Kp": 0.028, "Ti": 9.9},      # steam temperature
        ],
        "recorded": _recorded,
    },
    "advanced": {
        "model": "FlexibleHB_Advanced",
        "controllers": _common + [
            {"model": "XC8", "unit": "kg/s", "Kp": 1.14, "Ti": 4.4},    # water flow (inner loop)
            {"model": "XC17", "unit": "K", "Kp": 0.56, "Ti": 26.2},     # steam temperature (outer loop)
            {"model": "XC18", "unit": "", "Kp": 1.9, "Ti": 1244},       # H2/N2 ratio at the reactor inlet
        ],
        "recorded": _recorded,
    },
}
