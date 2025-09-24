from vedo import *
import time as t
from time import time

import serial
import serial.tools.list_ports
import numpy as np
from scipy.spatial.transform import Rotation as R

# --- funkcja znajdowania portu ESP32 ---
def find_esp32_port():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        desc = port.description.lower()
        if ("usb" in desc or "silicon" in desc or "ch340" in desc or "cp210" in desc):
            print(f"Znaleziono potencjalny port ESP32: {port.device} ({port.description})")
            return port.device
    return None

settings.use_parallel_projection = True

# --- wczytanie modelu ---
base_mesh = Mesh("Gimbal_1.obj").rotate_x(90).rotate_z(90).color("#ffc800").scale(0.1)
joint1_mesh = Mesh("Gimbal_2.obj").rotate_x(90).rotate_z(90).color("#ff5500").scale(0.1)
joint2_mesh = Mesh("Gimbal_3.obj").rotate_x(90).rotate_z(90).color("#00a2ff").scale(0.1)
joint3_mesh = Mesh("Gimbal_4.obj").rotate_x(90).rotate_z(90).color("#12b000").scale(0.1)

point_joint2 = vector(0, 0, -26.1972)
point_joint3 = vector(0, 11.7556, -26.2303)

# --- tworzenie hierarchii ---
joint3 = Assembly([joint3_mesh])
joint3.origin(point_joint3)

joint2 = Assembly([joint2_mesh, joint3])
joint2.origin(point_joint2)

joint1 = Assembly([joint1_mesh, joint2])
base = Assembly([base_mesh, joint1])

# --- zmienne globalne ---
quat_base = [1.0, 0.0, 0.0, 0.0]   # [w, x, y, z]
last_quat = [1.0, 0.0, 0.0, 0.0]   # poprzedni obrót
angles = [0.0, 0.0, 0.0]          # yaw, pitch, roll
last_angles = [0.0, 0.0, 0.0]      # yaw, pitch, roll

# --- konwersja kwaternion -> macierz 4x4 ---
def quat_to_matrix4(q):
    r = R.from_quat([q[1], q[2], q[3], q[0]])
    mat3 = r.as_matrix()
    mat4 = np.eye(4)
    mat4[:3, :3] = mat3
    return mat4

# --- funkcja: z kwaternionu wyciąga kąty kompensujące dla gimbala ---
def quat_to_gimbal_angles(q):
    r_base_motion = R.from_quat([q[1], q[2], q[3], q[0]])
    r_compensating = r_base_motion.inv()
    yaw, pitch, roll = r_compensating.as_euler("ZYX", degrees=True)
    return yaw, pitch, roll

# --- funkcja usuwająca stary obrót i nakładająca nowy ---
def init_rotation():
    global last_quat, quat_base, angles, last_angles

    # usuwanie poprzedniego obrotu
    inv_last = R.from_quat([last_quat[1], last_quat[2], last_quat[3], last_quat[0]]).inv()
    mat_inv = np.eye(4)
    mat_inv[:3, :3] = inv_last.as_matrix()
    base.apply_transform(mat_inv)

    yaw, pitch, roll = quat_to_gimbal_angles(quat_base)
    print(f"Yaw: {yaw:.3f}, Pitch: {pitch:.3f}, Roll: {roll:.3f}")

    joint1.rotate_z(-last_angles[0])
    joint2.rotate_y(-last_angles[1])
    joint3.rotate_x(-last_angles[2])

    joint1.rotate_z(angles[0])
    joint2.rotate_y(angles[1])
    joint3.rotate_x(angles[2])

    last_angles = angles.copy()

    # nakładanie nowego obrotu
    mat_new = quat_to_matrix4(quat_base)
    base.apply_transform(mat_new)

    # zapamiętanie ostatniego kwaternionu
    last_quat = quat_base.copy()

# --- odczyt danych z ESP32 ---
def read_serial():
    global quat_base, angles
    latest = None
    while ser.in_waiting > 0:
        latest = ser.readline().decode(errors='ignore').strip()

    if latest:
        print("ESP32 (ostatnie):", latest)
        vals = latest.split(',')
        if len(vals) >= 4:
            qw = float(vals[0])
            qx = float(vals[1])
            qy = float(vals[2])
            qz = float(vals[3])
            yaw = float(vals[4])
            pitch = float(vals[5])
            roll = float(vals[6])
            quat_base = [qw, qx, qy, qz]
            angles = [yaw, pitch, roll]

# --- pętla animacji ---
def loop_func(event):
    global quat_base
    read_serial()
    init_rotation()
    txt.text(f"time: {event.time - t0:.2f} sec\nquat: {quat_base}")
    plt.render()

# --- tekst na ekranie ---
txt = Text2D(bg='yellow', font="Calco")
t0 = time()

# --- czekamy na ESP32 ---
while find_esp32_port() is None:
    print("Nie znaleziono ESP32.")
    t.sleep(0.1)

t.sleep(1)  # dodatkowa sekunda na stabilizację
port = find_esp32_port()
print(f"Łączenie z {port}")
ser = serial.Serial(port, 115200, timeout=0.01)

# --- scena ---
plt = Plotter(bg='white', axes=1, offscreen=False)
plt.add_callback("timer", loop_func)
plt.timer_callback("start")

# definicja kamery
camera_1 = {
    'pos': (0, 200, -20),         # pozycja kamery (x, y, z)
    'focal_point': (0, -90, 0),    
    'viewup': (0, 0, 1)            # oś Z jest górą
}

plt.show(base, txt, camera=camera_1)
plt.close()
