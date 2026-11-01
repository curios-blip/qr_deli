import os, qrcode
TEX = {"room_101": "room_102", "room_102": "room_101"}   # swapped for D7
BASE = os.path.expanduser("~/qr_delivery")
for d in ("worlds", "signs", "launch"):
    os.makedirs(f"{BASE}/{d}", exist_ok=True)

T, H = 0.1, 1.2        # wall thickness, height
SIGN, SIGN_Z = 0.30, 0.20   # sign edge (m) and centre height; SIGN is what you vary for D4

# (x1, y1, x2, y2) wall segments; corridor walls have 1.2 m door gaps at x=2 and x=6
walls = [(0,-4,0,4), (8,-4,8,4), (0,4,8,4), (0,-4,8,-4), (4,1,4,4), (4,-4,4,-1)]
for y in (1, -1):
    walls += [(0,y,1.4,y), (2.6,y,5.4,y), (6.6,y,8,y)]

rooms = {"room_101": (2, 1), "room_102": (6, 1), "room_103": (2, -1), "room_104": (6, -1)}

def wall_xml(i, x1, y1, x2, y2):
    cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
    sx, sy = (abs(x2 - x1) + T, T) if y1 == y2 else (T, abs(y2 - y1) + T)
    geo = f"<geometry><box><size>{sx} {sy} {H}</size></box></geometry>"
    return f"""
      <collision name="c{i}"><pose>{cx} {cy} {H/2} 0 0 0</pose>{geo}</collision>
      <visual name="v{i}"><pose>{cx} {cy} {H/2} 0 0 0</pose>{geo}
        <material><ambient>0.8 0.8 0.75 1</ambient><diffuse>0.8 0.8 0.75 1</diffuse></material></visual>"""

def sign_xml(name, dx, side):
    qrcode.make(name, error_correction=qrcode.constants.ERROR_CORRECT_Q).save(f"{BASE}/signs/{name}.png")   # default border = quiet zone
      yaw = 0.0 if side > 0 else math.pi  
    return f"""
    <model name="sign_{name}"><static>true</static>
      <pose>{dx + 0.9} {side * 0.94} {SIGN_Z} 1.5708 0 {yaw}</pose>
      <link name="link"><visual name="v">
        <geometry><plane><normal>0 0 1</normal><size>{SIGN} {SIGN}</size></plane></geometry>
        <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>
          <pbr><metal><albedo_map>{BASE}/signs/{TEX.get(name, name)}.png</albedo_map>
          <metalness>0</metalness><roughness>1</roughness></metal></pbr></material>
      </visual></link></model>"""

world = f"""<?xml version="1.0"?>
<sdf version="1.10"><world name="rooms">
  <physics name="1ms" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>1.0</real_time_factor></physics>
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
  <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
  <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
  <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
  <light type="directional" name="sun"><cast_shadows>false</cast_shadows><pose>0 0 10 0 0 0</pose>
    <diffuse>1 1 1 1</diffuse><specular>0.2 0.2 0.2 1</specular><direction>-0.3 0.2 -0.9</direction></light>
  <light type="directional" name="fill"><cast_shadows>false</cast_shadows><pose>0 0 10 0 0 0</pose><diffuse>1 1 1 1</diffuse><specular>0 0 0 1</specular><direction>0.3 -0.2 -0.9</direction></light>
  <model name="ground"><static>true</static><link name="link">
    <collision name="c"><geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry></collision>
    <visual name="v"><geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry>
      <material><ambient>0.5 0.5 0.5 1</ambient><diffuse>0.5 0.5 0.5 1</diffuse></material></visual></link></model>
  <model name="walls"><static>true</static><link name="link">{''.join(wall_xml(i, *w) for i, w in enumerate(walls))}
  </link></model>
  {''.join(sign_xml(n, dx, s) for n, (dx, s) in rooms.items())}
</world></sdf>"""
open(f"{BASE}/worlds/rooms.sdf", "w").write(world)
print("wrote world and", len(rooms), "QR signs")
