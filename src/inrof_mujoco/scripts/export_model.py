import mujoco
from mujoco import viewer

import sys

ROBOT_NUM = int(sys.argv[1]) if len(sys.argv) > 1 else 5
BALL_NUM = int(sys.argv[2]) if len(sys.argv) > 2 else 5
SPACING = 0.4

def make_robot_body(i: int, x: float, y: float) -> str:
    return f"""
    <body name="robot_{i}" pos="{x} {y} 0.05">
      <joint name="robot_{i}_x" type="slide" axis="1 0 0"/>
      <joint name="robot_{i}_y" type="slide" axis="0 1 0"/>

      <geom
        name="robot_{i}_box"
        type="box"
        size="0.05 0.05 0.05"
        mass="1.0"
        rgba="1 0 0 1"
      />
    </body>
    """

def make_robot_actuator(i: int) -> str:
    return f"""
    <velocity name="robot_{i}_vx" joint="robot_{i}_x" kv="10" ctrllimited="true" ctrlrange="-0.2 0.2"/>
    <velocity name="robot_{i}_vy" joint="robot_{i}_y" kv="10" ctrllimited="true" ctrlrange="-0.2 0.2"/>
    """

def make_ball_body(i: int, x: float, y: float) -> str:
    return f"""
    <body name="ball_{i}" pos="{x} {y} 0.05">
      <joint name="ball_{i}_x" type="slide" axis="1 0 0"/>
      <joint name="ball_{i}_y" type="slide" axis="0 1 0"/>

      <geom
        name="ball_{i}_geom"
        type="sphere"
        size="0.035"
        mass="0.03"
        rgba="0.1 0.7 1.0 1"
      />
    </body>
    """

bodies = "\n".join(
    make_robot_body(i, x=i * SPACING, y=0.0)
    for i in range(ROBOT_NUM)
)

actuators = "\n".join(
    make_robot_actuator(i)
    for i in range(ROBOT_NUM)
)
balls = "\n".join(
    make_ball_body(i, x=i * SPACING, y=2.0)
    for i in range(BALL_NUM)
)

xml = f"""
<mujoco model="five_robots">
  <compiler angle="radian"/>
  <option timestep="0.01" gravity="0 0 0"/>

  <worldbody>
    <light name="sun" pos="0 0 5"/>
    <geom name="floor" type="plane" size="5 5 0.1" rgba="0.8 0.8 0.8 1"/>

    {bodies}
    {balls}
  </worldbody>

  <actuator>
    {actuators}
  </actuator>
</mujoco>
"""

f = open("./models/boid.xml", "w")
f.write(xml)
f.close()
model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

with viewer.launch_passive(model, data) as v:
    while v.is_running():
        mujoco.mj_step(model, data)
        v.sync()
