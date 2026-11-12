"""Check the robot-description wiring (SPEC_3 section 14): GraspSim's spec-derived values on
p3b must equal the old hard-coded constants exactly; print them for the ESPA scene."""
import numpy as np
from grasp_sim import STOW, GRIP_OPEN, GraspCfg, GraspSim

OLD_BOXES = (((0.0, 0.0, 0.0), (0.75, 0.5, 0.5)), ((-0.8375, 0.0, 0.0), (0.0875, 0.175, 0.025)))
for scene in ("p3b_scene.xml", "p3e_espa_scene.xml"):
    s = GraspSim(GraspCfg(scene=scene))
    print(f"== {scene}: m {s.m_serv:.2f} kg, I_bus {np.round(s.spec.I_serv, 3)}, s_m {s.s_m}, s_I {np.round(s.s_I, 3)}")
    print(f"   stow {s.stow}, ik_seed {s.ik_seed}, pregrasp {s.pregrasp_d}, r_serv {s.r_serv}, f_max {s.f_max}")
    print(f"   boxes {[(c.tolist(), h.tolist()) for c, h in s.boxes]}")
    if scene == "p3b_scene.xml":
        same = (np.array_equal(s.stow, STOW) and np.array_equal(s.grip_open, GRIP_OPEN) and s.ik_seed is None
                and s.pregrasp_d == 1.8 and s.r_serv == 0.87 and s.f_max == 0.75 and s.s_m == 1.0
                and np.array_equal(s.s_I, np.ones(3)) and s.kp_pos == 0.6 and s.kd_pos == 18.0
                and all(np.array_equal(c, o[0]) and np.array_equal(h, o[1]) for (c, h), o in zip(s.boxes, OLD_BOXES)))
        print("   identical to the old constants:", same)
