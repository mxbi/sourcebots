# This file is vendored in from the robot-api library to make this library work when robot-api is not installed.
# This file should track https://raw.githubusercontent.com/sourcebots/robot-api/master/robot/game_specific.py

WALL = set(range(0, 28))  # 0 - 27

# Currently for SB2018
COLUMN_N = set(range(28, 32))
COLUMN_E = set(range(32, 36))
COLUMN_S = set(range(36, 40))
COLUMN_W = set(range(40, 44))
COLUMN_FACING_N = set(range(28, 44, 4))
COLUMN_FACING_E = set(range(29, 44, 4))
COLUMN_FACING_S = set(range(30, 44, 4))
COLUMN_FACING_W = set(range(31, 44, 4))

# Individual Column faces.
COLUMN_N_FACING_N = (COLUMN_N & COLUMN_FACING_N).pop()
COLUMN_N_FACING_S = (COLUMN_N & COLUMN_FACING_S).pop()
COLUMN_N_FACING_E = (COLUMN_N & COLUMN_FACING_E).pop()
COLUMN_N_FACING_W = (COLUMN_N & COLUMN_FACING_W).pop()

COLUMN_S_FACING_N = (COLUMN_S & COLUMN_FACING_N).pop()
COLUMN_S_FACING_S = (COLUMN_S & COLUMN_FACING_S).pop()
COLUMN_S_FACING_E = (COLUMN_S & COLUMN_FACING_E).pop()
COLUMN_S_FACING_W = (COLUMN_S & COLUMN_FACING_W).pop()

COLUMN_E_FACING_N = (COLUMN_E & COLUMN_FACING_N).pop()
COLUMN_E_FACING_S = (COLUMN_E & COLUMN_FACING_S).pop()
COLUMN_E_FACING_E = (COLUMN_E & COLUMN_FACING_E).pop()
COLUMN_E_FACING_W = (COLUMN_E & COLUMN_FACING_W).pop()

COLUMN_W_FACING_N = (COLUMN_W & COLUMN_FACING_N).pop()
COLUMN_W_FACING_S = (COLUMN_W & COLUMN_FACING_S).pop()
COLUMN_W_FACING_E = (COLUMN_W & COLUMN_FACING_E).pop()
COLUMN_W_FACING_W = (COLUMN_W & COLUMN_FACING_W).pop()

COLUMN = (COLUMN_N | COLUMN_E | COLUMN_S | COLUMN_W)

TOKEN = set(range(44, 64))

TOKEN_ZONE_0 = set(range(44, 49))
TOKEN_ZONE_1 = set(range(49, 54))
TOKEN_ZONE_2 = set(range(54, 59))
TOKEN_ZONE_3 = set(range(59, 64))

# The following constants are used to define the marker sizes

MARKER_SIZES = {}
MARKER_SIZES.update({m: (0.25, 0.25) for m in (WALL | COLUMN)})
MARKER_SIZES.update({m: (0.1, 0.1) for m in TOKEN})

# Size the vision system will assume a marker is if it's not in MARKER_SIZES
MARKER_SIZE_DEFAULT = (0.25, 0.25)
