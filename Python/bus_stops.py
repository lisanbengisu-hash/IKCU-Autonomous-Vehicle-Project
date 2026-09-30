from dataclasses import dataclass
from pathlib import Path
import json

import carla


# ============================================================
# DURAK SINIFI
# ============================================================

@dataclass
class BusStop:
    name: str
    location: carla.Location


# ============================================================
# JSON DOSYASI
# ============================================================

BASE_DIR = Path(__file__).resolve().parent

BUS_STOP_FILE = (
    BASE_DIR / "bus_stop_coordinates.json"
)


# ============================================================
# DURAK DEPOSU
# ============================================================

BUS_STOPS = {}


# ============================================================
# DURAK EKLE
# ============================================================

def add_bus_stop(
    name,
    x,
    y,
    z,
):
    BUS_STOPS[name] = BusStop(
        name=name,
        location=carla.Location(
            x=float(x),
            y=float(y),
            z=float(z),
        ),
    )


# ============================================================
# JSON'DAN DURAKLARI YÜKLE
# ============================================================

def load_bus_stops():

    BUS_STOPS.clear()

    if not BUS_STOP_FILE.exists():

        print(
            "UYARI: bus_stop_coordinates.json bulunamadı."
        )

        print(
            "Aranan dosya:",
            BUS_STOP_FILE,
        )

        return BUS_STOPS

    try:

        with BUS_STOP_FILE.open(
            "r",
            encoding="utf-8",
        ) as file:

            data = json.load(file)

    except json.JSONDecodeError as error:

        raise RuntimeError(
            "Durak JSON dosyası okunamadı: "
            f"{error}"
        )

    for name, coordinates in data.items():

        try:

            add_bus_stop(
                name=name,
                x=coordinates["x"],
                y=coordinates["y"],
                z=coordinates["z"],
            )

        except KeyError:

            print(
                f"UYARI: {name} durağında "
                "x/y/z bilgisi eksik."
            )

    print(
        f"{len(BUS_STOPS)} durak yüklendi."
    )

    return BUS_STOPS


# ============================================================
# DURAK BUL
# ============================================================

def get_bus_stop(name):

    return BUS_STOPS.get(
        name
    )


# ============================================================
# DURAK LİSTESİ
# ============================================================

def list_bus_stops():

    return list(
        BUS_STOPS.values()
    )


# ============================================================
# DURAK İSİMLERİ
# ============================================================

def get_bus_stop_names():

    return list(
        BUS_STOPS.keys()
    )


# ============================================================
# PROGRAM BAŞLARKEN OTOMATİK YÜKLE
# ============================================================

load_bus_stops()
