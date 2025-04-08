import evdev
from evdev import InputDevice, categorize, ecodes, list_devices
import os
import time

def find_controller():
    print("Recherche d'une manette Xbox connectée...")
    devices = [InputDevice(path) for path in list_devices()]
    for device in devices:
        if 'Xbox' in device.name or 'Controller' in device.name:
            print(f"✅ Manette trouvée : {device.name} ({device.path})")
            return device
    print("❌ Aucune manette trouvée.")
    return None

# Fonction utilitaire pour afficher une barre horizontale
def render_bar(value, min_val=-32768, max_val=32767, width=20):
    norm = (value - min_val) / (max_val - min_val)
    pos = int(norm * width)
    bar = "[" + " " * width + "]"
    if 0 <= pos < width:
        bar = "[" + " " * pos + "|" + " " * (width - pos - 1) + "]"
    return bar

# État des touches et sticks
state = {
    "ABS_X": 0,
    "ABS_Y": 0,
    "ABS_RX": 0,
    "ABS_RY": 0,
    "ABS_Z": 0,
    "ABS_RZ": 0,
    "BTN_SOUTH": 0,  # A
    "BTN_EAST": 0,   # B
    "BTN_NORTH": 0,  # X
    "BTN_WEST": 0,   # Y
    "BTN_TL": 0,     # LB
    "BTN_TR": 0,     # RB
    "BTN_THUMBL": 0,
    "BTN_THUMBR": 0,
    "BTN_START": 0,
    "BTN_SELECT": 0,
    "ABS_HAT0X": 0,
    "ABS_HAT0Y": 0,
}
def print_cli_state():
    os.system("clear")
    print("🎮 Entrées manette Bluetooth (via evdev)\n"
          "====================================\n")
    print("=== État des boutons et axes ===")
    print("====================================\n")
    for key, val in state.items():
        print(f"{key}: {val}")
    print("\n====================================\n")

def print_graphic_state():
    os.system("clear")
    print("🎮 Entrées manette Bluetooth (via evdev)\n")

    for key, val in state.items():
        if "ABS" in key:
            if "X" in key or "Y" in key:
                print(f"{key}: {render_bar(val)}")
            else:
                print(f"{key}: {val}")
        elif "BTN" in key:
            print(f"{key}: {'✅' if val else '  '}")

    print("=== Joystick Gauche ===")
    print(f"X: {render_bar(state['ABS_X'])}")
    print(f"Y: {render_bar(state['ABS_Y'])}")

    print("\n=== Joystick Droit ===")
    print(f"X: {render_bar(state['ABS_RX'])}")
    print(f"Y: {render_bar(state['ABS_RY'])}")

    print("\n=== Gâchettes ===")
    print(f"LT: {render_bar(state['ABS_Z'], 0, 255)}")
    print(f"RT: {render_bar(state['ABS_RZ'], 0, 255)}")

    print("\n=== Croix Directionnelle (D-Pad) ===")
    print(f"Horizontale: {state['ABS_HAT0X']}, Verticale: {state['ABS_HAT0Y']}")

    print("\n=== Boutons ===")
    for key in ["BTN_SOUTH", "BTN_EAST", "BTN_NORTH", "BTN_WEST",
                "BTN_TL", "BTN_TR", "BTN_THUMBL", "BTN_THUMBR",
                "BTN_START", "BTN_SELECT"]:
        if state[key]:
            print(f"{key}: ✅", end="  ")
    print()

def main():
    dev = find_controller()
    if not dev:
        return

    dev.grab()  # Prend le contrôle exclusif du device

    state = {}

    try:
        for event in dev.read_loop():
            if event.type == ecodes.EV_KEY or event.type == ecodes.EV_ABS:
                name = ecodes.bytype[event.type][event.code]
                state[name] = event.value
                print(state)
                # Affichage console "graphique"
                #print_cli_state()
                time.sleep(0.01)

    except KeyboardInterrupt:
        dev.ungrab()
        print("\nFermeture propre.")

if __name__ == "__main__":
    main()
    