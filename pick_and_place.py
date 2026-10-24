import time
import numpy as np
import mujoco, mujoco.viewer

ROBOT = "models/mujoco_menagerie/robotstudio_so101/scene.xml"
A, B = (0.20, -0.10), (0.20, 0.10)        # küpün alınacağı ve bırakılacağı yer (m)

spec = mujoco.MjSpec.from_file(ROBOT)     # SO101 sahnesini düzenlenebilir aç
cube = spec.worldbody.add_body(name="cube", pos=[*A, 0.015])
cube.add_freejoint()                      # küp serbest: düşer, itilir, tutulur
cube.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.015] * 3, mass=0.03,
              friction=[1.5, 0.005, 0.0005], condim=6, rgba=[0.9, 0.2, 0.2, 1])
for name, (x, y) in [("mark_A", A), ("mark_B", B)]:
    spec.worldbody.add_geom(name=name, type=mujoco.mjtGeom.mjGEOM_CYLINDER, pos=[x, y, 0.001],
                            size=[0.03, 0.001, 0], contype=0, conaffinity=0,
                            rgba=[0.2, 0.55, 1, 0.8] if name == "mark_A" else [0.2, 0.85, 0.45, 0.8])
model = spec.compile()                    # sahneyi fizik motoruna hazırla
data = mujoco.MjData(model)

PAN_X, SX, SZ = 0.039, 0.030, 0.117       # taban ekseni ve omuz ekseninin yeri (m)
L1, L2, TOOL = 0.116, 0.135, 0.160        # üst kol, ön kol, bilekten parmak ucuna
B1, B2, B3 = 1.327, 0.038, 0.075          # kolların kendi bükülme açıları (rad)
OPEN, CLOSE = 0.6, -0.17                  # tutucu açık / kapalı (rad)

def ik(x, y, z):
    """Parmak ucunu (x, y, z)'ye götüren eklem açıları."""
    yaw = -np.arctan2(y, x - PAN_X)                 # taban hedefe dönsün
    r, h = np.hypot(x - PAN_X, y) - SX, z + TOOL - SZ   # omuzdan bileğe: yatay, dikey
    c = (r*r + h*h - L1*L1 - L2*L2) / (2 * L1 * L2)
    phi = -np.arccos(np.clip(c, -1, 1))             # dirsek (kosinüs teoremi)
    a1 = np.arctan2(h, r) - np.arctan2(L2*np.sin(phi), L1 + L2*np.cos(phi))
    q1, q2 = B1 - a1, B2 - B1 - phi                 # omuz ve dirsek motoru
    q3 = a1 + phi + B3 - B2 + np.pi / 2             # bilek: tutucu hep aşağı baksın
    return [yaw, q1, q2, q3]

def smooth(t):                                      # yumuşak kalkış ve duruş
    return t * t * (3 - 2 * t)

HOME = (0.20, 0.0, 0.07)
plan = [(*A, 0.07, OPEN),  (*A, 0.012, OPEN), (*A, 0.012, CLOSE),
        (*A, 0.07, CLOSE), (*B, 0.07, CLOSE), (*B, 0.014, CLOSE),
        (*B, 0.014, OPEN), (*B, 0.07, OPEN),  (*HOME, OPEN)]

data.qpos[:4] = data.ctrl[:4] = ik(*HOME)           # başlangıç pozu
data.ctrl[5] = OPEN

with mujoco.viewer.launch_passive(model, data) as viewer:
    pos, grip = np.array(HOME), OPEN
    for *target, g in plan:
        start, target = pos.copy(), np.array(target)
        for i in range(240):                        # her adım 1.2 saniye
            s = smooth(i / 239)
            pos = start + (target - start) * s
            data.ctrl[:4] = ik(*pos)
            data.ctrl[5] = grip + (g - grip) * s
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(model.opt.timestep)
        grip = g
