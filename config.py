DEBUG_MODE = True  # Ustaw na False w produkcji
LOG_LEVEL = 'INFO'  # DEBUG, INFO, WARNING, ERROR

# Kolory debugowe
DEBUG_COLORS = {
    'ERROR': (0, 0, 255),    # Czerwony
    'WARNING': (0, 255, 255), # Żółty
    'INFO': (0, 255, 0),      # Zielony
    'DEBUG': (255, 255, 255)  # Biały
}

# Kolory
LASER_AREA_COLOR = (0, 255, 0)  # Zielony dla obszaru
LASER_POINT_COLOR = (0, 0, 255)  # Czerwony dla lasera
CAT_POINT_COLOR = (0, 255, 0)    # Zielony dla kota
LINE_COLOR = (255, 0, 0)         # Niebieski dla linii
HISTORY_COLOR = (0, 165, 255)    # Pomarańczowy dla historii

# Ustawienia
HISTORY_LENGTH = 30
REACTION_DELAY = 0.2
BASE_SPEED = 15
MIN_CALIBRATION_POINTS = 3

MIN_CAT_AREA = 1000  # Minimalna powierzchnia wykrywania kota (w pikselach)