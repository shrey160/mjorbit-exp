"""Project 4 (SPEC_4 P4.0): camera scene derived from the ESPA servicer scene.

Writes projects/p3_capture/p4_espa_cam_scene.xml (next to grasp_sim.py, which loads scenes
by name from there); with --arm arm2 (13 Nov), p4_espa_arm2_cam_scene.xml from the realistic
arm scene p3e_espa_arm2_scene.xml instead. Physics is unchanged: only cameras, a directional sun, materials for
specular randomisation and offscreen-buffer settings are added; no bodies, geoms, joints or
masses change. The frozen ESPA scene itself is not edited.

Cameras (MuJoCo cameras look along their -z axis):
  wrist_cam  on wrist3 (the gripper body: x = approach, y = bar axis, z = closing direction),
             above the open upper jaw, looking along the approach axis and down between the
             open jaws (an earlier mount behind the jaw was blocked by it)
  bus_cam    on the bus +x face, above and to the side of the shoulder, looking at the
             nominal bar position (pre-grasp distance ahead)
  bus_lidar  flash lidar 6 cm below bus_cam, same aim (real servicers fly visible + IR +
             flash lidar, e.g. NASA Raven); a depth image, see cams.py
"""
import sys
from pathlib import Path

import numpy as np

P3 = Path(__file__).resolve().parents[1] / "p3_capture"
ARMS = {"espa": ("p3e_espa_scene.xml", "p4_espa_cam_scene.xml", "p3e_espa", "p4_espa_cam"),
        "arm2": ("p3e_espa_arm2_scene.xml", "p4_espa_arm2_cam_scene.xml", "p3e_espa_arm2", "p4_espa_arm2_cam")}


def xyaxes(pos, look, up=(0.0, 0.0, 1.0)):
    """MuJoCo camera xyaxes for a camera at pos looking at look (parent-body frame)."""
    f = np.asarray(look, float) - np.asarray(pos, float)
    f /= np.linalg.norm(f)
    x = np.cross(f, up)
    x /= np.linalg.norm(x)
    y = np.cross(-f, x)  # camera z = -f, y = z x x
    return " ".join(f"{v:.4f}" for v in np.concatenate([x, y]))


def build(arm="espa"):
    src, dst, m_src, m_dst = ARMS[arm]
    SRC, DST = P3 / src, P3 / dst
    s = SRC.read_text(encoding="utf-8")

    def rep(a, b):
        nonlocal s
        assert s.count(a) == 1, a
        s = s.replace(a, b)

    rep(f'<mujoco model="{m_src}">', f'<mujoco model="{m_dst}">')
    rep('  <compiler angle="radian"/>\n', '''  <compiler angle="radian"/>
  <!-- P4 (SPEC_4): space-like lighting: no headlight, a little ambient, one directional sun
       (direction, intensity and ambient randomised per episode by p4_vision/cams.py) -->
  <visual>
    <global offwidth="640" offheight="480"/>
    <headlight active="0" ambient="0.05 0.05 0.05" diffuse="0 0 0" specular="0 0 0"/>
    <quality shadowsize="2048"/>
  </visual>
  <asset>
    <material name="mat_client" specular="0.3" shininess="0.3" reflectance="0"/>
    <material name="mat_bar" specular="0.5" shininess="0.5" reflectance="0"/>
    <material name="mat_serv" specular="0.3" shininess="0.3" reflectance="0"/>
  </asset>
''')
    rep('    <light pos="0 0 10" dir="0 0 -1"/>',
        '    <light name="sun" directional="true" castshadow="true" pos="0 0 10" dir="0 0 -1"'
        ' diffuse="1 1 1" specular="0.3 0.3 0.3"/>')
    rep('<geom name="client_bus" class="client_col" type="box"',
        '<geom name="client_bus" class="client_col" material="mat_client" type="box"')
    rep('<geom name="handle_bar" class="client_col" type="box"',
        '<geom name="handle_bar" class="client_col" material="mat_bar" type="box"')
    rep('<geom name="servicer_bus" type="box"', '<geom name="servicer_bus" material="mat_serv" type="box"')
    # above the open upper jaw (top at z ~0.11), sight line to the grip point passes below it
    wpos, wlook = (-0.03, 0.0, 0.14), (0.35, 0.0, 0.0)
    # P4.8 (13 Nov) palm_cam: between the jaws, 10 cm behind the grip point, looking along the
    # approach axis; sees both pads and the bar between them (wrist_cam never sees the pads:
    # probe_wristcam.py). Image up = closing direction.
    ppos, plook = (0.03, 0.0, 0.0), (0.35, 0.0, 0.0)
    rep('                  <site name="grip_site" pos="0.13 0 0" size="0.012" rgba="0.1 0.9 0.2 1"/>\n',
        '                  <site name="grip_site" pos="0.13 0 0" size="0.012" rgba="0.1 0.9 0.2 1"/>\n'
        f'                  <camera name="wrist_cam" mode="fixed" pos="{wpos[0]} {wpos[1]} {wpos[2]}"'
        f' xyaxes="{xyaxes(wpos, wlook)}" fovy="75"/>\n'
        f'                  <camera name="palm_cam" mode="fixed" pos="{ppos[0]} {ppos[1]} {ppos[2]}"'
        f' xyaxes="{xyaxes(ppos, plook)}" fovy="90"/>\n'
        # palm light (13 Nov): near the bar the gripper shadows it and the palm image goes black
        # (dbg_palm.py); a spot beside the camera, off by default and switched on only while
        # palm_cam renders (cams.CamRig.render_palm), so every other image is unchanged
        f'                  <light name="palm_light" active="false" pos="{ppos[0]} {ppos[1]} {ppos[2] + 0.015}"'
        ' dir="1 0 0" cutoff="70" exponent="1" diffuse="0.9 0.9 0.9" specular="0.2 0.2 0.2" castshadow="false"/>\n')
    bpos, blook = (0.26, 0.20, 0.30), (1.25, 0.0, 0.0)
    lpos = (0.26, 0.20, 0.24)
    rep('      <site name="bus_site" pos="0 0 0" size="0.02"/>\n',
        '      <site name="bus_site" pos="0 0 0" size="0.02"/>\n'
        f'      <camera name="bus_cam" mode="fixed" pos="{bpos[0]} {bpos[1]} {bpos[2]}"'
        f' xyaxes="{xyaxes(bpos, blook)}" fovy="40"/>\n'
        # flash lidar (Raven-like: visible + IR + flash lidar), 6 cm below the bus camera,
        # same aim point; rendered as a depth image by cams.py
        f'      <camera name="bus_lidar" mode="fixed" pos="{lpos[0]} {lpos[1]} {lpos[2]}"'
        f' xyaxes="{xyaxes(lpos, blook)}" fovy="45"/>\n')
    s = s.replace("<!--\n", f"<!--\n  GENERATED by projects/p4_vision/make_scene.py from {src}"
                  " (do not edit; re-run it).\n  Adds cameras, sun, materials only (physics unchanged).\n\n", 1)
    DST.write_text(s, encoding="utf-8")
    print("wrote", DST)


if __name__ == "__main__":
    build(sys.argv[sys.argv.index("--arm") + 1] if "--arm" in sys.argv else "espa")
