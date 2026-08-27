"""limx_tron1 constants."""

from pathlib import Path

import mujoco

from mjlab import MJLAB_SRC_PATH
from mjlab.actuator import BuiltinPositionActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.utils.spec_config import CollisionCfg

##
# MJCF and assets.
##

LIMX_TRON1_XML: Path = (
  MJLAB_SRC_PATH / "asset_zoo" / "robots" / "limx_tron1" / "xmls" / "limx_tron1.xml"
)
assert LIMX_TRON1_XML.exists()


def get_spec() -> mujoco.MjSpec:
  return mujoco.MjSpec.from_file(str(LIMX_TRON1_XML))


##
# Actuator config.
##
MOTOR_ARMATURE = 0.1 # kg*m2
MOTOR_EFFORT_LIMIT = 80.0 #Nm
MOTOR_VELOCITY_LIMIT = 15.0 #rad/s

ACTUATOR = BuiltinPositionActuatorCfg(
  target_names_expr=(
    ".*_Joint",
  ),
  armature= MOTOR_ARMATURE,
  effort_limit=MOTOR_EFFORT_LIMIT,
  frictionloss=0.1,
  viscous_damping=0.01,
  damping=0.8,
  stiffness=45.0,
)

##
# Keyframe config.
##

HOME_KEYFRAME = EntityCfg.InitialStateCfg(
  pos=(0, 0, 0.8),
  joint_pos={
    ".*_Joint": 0.0,
  },
  joint_vel={".*": 0.0},
)

##
# Collision config.
##

# This enables all collisions, including self collisions.
# Self-collisions are given condim=1 while foot collisions
# are given condim=3.
FULL_COLLISION = CollisionCfg(
  geom_names_expr=(".*_collision",),
  contype=1,
  conaffinity=1,
  condim={".*_collision": 1},
  priority={".*_collision": 1},
  friction={".*_collision": (0.6,)},
)

##
# Final config.
##

LIMX_TRON1_ARTICULATION = EntityArticulationInfoCfg(
  actuators=(
    ACTUATOR,
  ),
  soft_joint_pos_limit_factor=0.9,
)


def get_limx_tron1_robot_cfg() -> EntityCfg:
  """Get a fresh G1 robot configuration instance.

  Returns a new EntityCfg instance each time to avoid mutation issues when
  the config is shared across multiple places.
  """
  return EntityCfg(
    init_state=HOME_KEYFRAME,
    collisions=(FULL_COLLISION,),
    spec_fn=get_spec,
    articulation=LIMX_TRON1_ARTICULATION,
  )


LIMX_TRON1_ACTION_SCALE: dict[str, float] = {}
for a in LIMX_TRON1_ARTICULATION.actuators:
  assert isinstance(a, BuiltinPositionActuatorCfg)
  e = a.effort_limit
  s = a.stiffness
  names = a.target_names_expr
  assert e is not None
  for n in names:
    LIMX_TRON1_ACTION_SCALE[n] = round(0.25 * e / s, 4)


if __name__ == "__main__":
  import mujoco.viewer as viewer

  from mjlab.entity.entity import Entity

  robot = Entity(get_limx_tron1_robot_cfg())

  viewer.launch(robot.spec.compile())
