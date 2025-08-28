from vedo import *
import time as t
from time import time

import serial
import serial.tools.list_ports

def find_esp32_port():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        desc = port.description.lower()
        if ("usb" in desc or "silicon" in desc or "ch340" in desc or "cp210" in desc):
            print(f"Znaleziono potencjalny port ESP32: {port.device} ({port.description})")
            return port.device
    return None

settings.use_parallel_projection = True

# --- Wczytanie modelu ---

base_mesh = Mesh("Gimbal_1.obj").color("#ffc800").rotate_y(-90).scale(0.1)
# c1 = base.clone().c("violet").alpha(0.5)
# v = vector(0,0,0)
# p = vector(0,0,0)
# c1.rotate(90, axis=(v - p), point=p)
# l = Line(v, p).lw(3).c('red')

joint1_mesh = Mesh("Gimbal_2.obj").color("#ff5500").rotate_y(-90).scale(0.1)

joint2_mesh = Mesh("Gimbal_3.obj").color("#00a2ff").rotate_y(-90).scale(0.1)
point_joint2 = vector(0, -26.1972, 0.0)

joint3_mesh = Mesh("Gimbal_4.obj").color("#12b000").rotate_y(-90).scale(0.1)
point_joint3 = vector(0, -26.2303, 11.7556)

# --- Zmienne kątów ---

base_x = 0.0
base_y = 0.0
base_z = 0.0

joint1_angle = 0.0
joint2_angle = 0.0
joint3_angle = 0.0

last_base_x = 0.0
last_base_y = 0.0
last_base_z = 0.0

last_joint1_angle = 0.0
last_joint2_angle = 0.0
last_joint3_angle = 0.0

def init_rotation():
    global last_base_x, last_base_y, last_base_z
    global last_joint1_angle, last_joint2_angle, last_joint3_angle

    base.rotate_z(-last_base_z)
    base.rotate_y(-last_base_y)
    base.rotate_x(-last_base_x)

    joint1.rotate_y(-last_joint1_angle)
    joint2.rotate_z(-last_joint2_angle)
    joint3.rotate_x(-last_joint3_angle)

    last_base_x = base_x
    last_base_y = base_y
    last_base_z = base_z

    last_joint1_angle = joint1_angle
    last_joint2_angle = joint2_angle
    last_joint3_angle = joint3_angle

def read_serial():
    global joint1_angle, joint2_angle, joint3_angle, base_x, base_y, base_z
    latest = None
    while ser.in_waiting > 0:
        latest = ser.readline().decode(errors='ignore').strip()

    if latest:
        print("ESP32 (ostatnie):", latest)
        angles = latest.split(',')
        joint1_angle = float(angles[0])
        # joint1_angle = -joint1_angle  # odwrócenie kierunku
        joint2_angle = float(angles[1])
        joint2_angle = -joint2_angle  # odwrócenie kierunku
        joint3_angle = float(angles[2]) #X

        base_x = float(angles[3])
        base_y = float(angles[5])
        base_z = float(angles[4])

def loop_func(event):

    read_serial()

    init_rotation()

    base.rotate_x(base_x)
    base.rotate_y(base_y)
    base.rotate_z(base_z)

    joint1.rotate_y(joint1_angle)
    joint2.rotate_z(joint2_angle)
    joint3.rotate_x(joint3_angle)

    txt.text(f"time: {event.time - t0:.2f} sec")
    plt.render()

# --- callbacki sliderów ---
# def set_base_x(widget, event):
#     global base_x
#     base_x = widget.value

# def set_base_y(widget, event):
#     global base_y
#     base_y = widget.value

# def set_base_z(widget, event):
#     global base_z
#     base_z = widget.value

# def set_joint1(widget, event):
#     global joint1_angle
#     joint1_angle = widget.value

# def set_joint2(widget, event):
#     global joint2_angle
#     joint2_angle = widget.value

# def set_joint3(widget, event):
#     global joint3_angle
#     joint3_angle = widget.value
    

# --- Tworzymy hierarchię ---
joint3 = Assembly([joint3_mesh])           # najgłębszy element
joint3.origin(point_joint3)
joint2 = Assembly([joint2_mesh, joint3])   # joint2 + joint3
joint2.origin(point_joint2)
joint1 = Assembly([joint1_mesh, joint2])   # joint1 + joint2 + joint3
base = Assembly([base_mesh, joint1])       # base + joint1 + joint2 + joint3

txt = Text2D(bg='yellow', font="Calco")
t0 = time()

while find_esp32_port() is None:
    print("Nie znaleziono ESP32.")
    t.sleep(0.1)

t.sleep(1)  # dodatkowa sekunda na ustabilizowanie połączenia
port = find_esp32_port()

print(f"Łączenie z {port}")
ser = serial.Serial(port, 115200, timeout=0.01)
# --- Okno i scena ---
plt = Plotter(bg='white', axes= 1, offscreen=False)
plt.add_callback("timer", loop_func)
plt.timer_callback("start")


# --- Dodanie sliderów ---
# plt.add_slider(set_base_x, -180, 180, value=0, title="Base X", pos=[(0.05, 0.3), (0.45, 0.3)])
# plt.add_slider(set_base_y, -180, 180, value=0, title="Base Y", pos=[(0.05, 0.2), (0.45, 0.2)])
# plt.add_slider(set_base_z, -180, 180, value=0, title="Base Z", pos=[(0.05, 0.1), (0.45, 0.1)])
# plt.add_slider(set_joint1, -60, 60, value=0, title="Joint1", pos=[(0.55, 0.3), (0.95, 0.3)])
# plt.add_slider(set_joint2, -45, 45, value=0, title="Joint2", pos=[(0.55, 0.2), (0.95, 0.2)])
# plt.add_slider(set_joint3, -60, 60, value=0, title="Joint3", pos=[(0.55, 0.1), (0.95, 0.1)])


plt.show(base, txt)
plt.close()

