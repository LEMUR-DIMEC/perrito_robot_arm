import mujoco
import mujoco.viewer
import time
import numpy as np
from loop_rate_limiters import RateLimiter

model = mujoco.MjModel.from_xml_path("sim/model/scene.xml")
data = mujoco.MjData(model)
model.opt.gravity = (0, 0, -9.81)
mujoco.mj_resetDataKeyframe(model, data, 0)
with mujoco.viewer.launch_passive(model, data, show_left_ui=True,show_right_ui=True) as viewer:
  rate = RateLimiter(frequency=2000.0, warn=False)
  while viewer.is_running():
    mujoco.mj_step(model, data)
    # Pick up changes to the physics state, apply perturbations, update options from GUI.
    viewer.sync()
    rate.sleep()