import mujoco
import numpy as np
from mujoco import viewer

xml = """
<mujoco model="swarm_cube">
  <compiler angle="radian"/>
  <option timestep="0.01" gravity="0 0 0"/>

  <default>
    <geom friction="1 0.1 0.1"/>
    <joint damping="0.1"/>
  </default>

  <worldbody>
    <light name="sun" pos="0 0 5"/>

    <geom
      name="floor"
      type="plane"
      size="5 5 0.1"
      rgba="0.8 0.8 0.8 1"
    />

    <body name="robot_0" pos="0 0 0.05">
      <joint name="robot_0_x" type="slide" axis="1 0 0"/>
      <joint name="robot_0_y" type="slide" axis="0 1 0"/>

      <geom
        name="robot_0_box"
        type="box"
        size="0.05 0.05 0.05"
        mass="1.0"
        rgba="1 0 0 1"
      />
    </body>
  </worldbody>

  <actuator>
    <velocity name="robot_0_vx" joint="robot_0_x" kv="10" ctrllimited="true" ctrlrange="-0.2 0.2"/>
    <velocity name="robot_0_vy" joint="robot_0_y" kv="10" ctrllimited="true" ctrlrange="-0.2 0.2"/>
  </actuator>
</mujoco>

"""

model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)
data.ctrl[model.actuator("robot_0_vx").id] = 0.005
data.ctrl[model.actuator("robot_0_vy").id] = 0.0

with viewer.launch_passive(model, data) as v:
    while v.is_running():
        mujoco.mj_step(model, data)
        v.sync()