from typing import NamedTuple

class FollowerParams(NamedTuple):
    max_vel: float
    k_separation: float
    k_alignment: float
    k_gravity: float
    k_wall: float
    k_follow: float
    field: float # 一旦使わない
    ir: float
    ir_min: float

class FollowerCoreMjx():
    def __init__(self):
        pass