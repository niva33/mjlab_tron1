"""bipedal_12dof constants."""

from pathlib import Path

import mujoco

from mjlab import MJLAB_SRC_PATH
from mjlab.actuator import BuiltinPositionActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.utils.spec_config import CollisionCfg

##
# MJCF and assets.
##

BIPEDAL_12DOF_XML: Path = (
  MJLAB_SRC_PATH / "asset_zoo" / "robots" / "bipedal" / "xmls" / "bipedal_12dof.xml"
)
assert BIPEDAL_12DOF_XML.exists()


def get_spec() -> mujoco.MjSpec:
  return mujoco.MjSpec.from_file(str(BIPEDAL_12DOF_XML))


##
# Actuator config.
##
AK7010_ARMATURE = 0.1   # kg*m2
AK7010_MAX_VEL = 10.0  # rad/s
AK7010_MAX_TORQUE = 20.0 # Nm

AK109_ARMATURE = 0.1  # kg*m2
AK109_MAX_VEL = 10.0  # rad/s
AK109_MAX_TORQUE = 50.0 # Nm

AK606_ARMATURE = 0.1  # kg*m2
AK606_MAX_VEL = 18.0  # rad/s
AK606_MAX_TORQUE = 8.0 * 2 # Nm

NATURAL_FREQ = 10 * 2.0 * 3.1415926535  # 10Hz
DAMPING_RATIO = 2.0

STIFFNESS_HIP_PITCH_ROLL_KNEE = NATURAL_FREQ ** 2 * AK109_ARMATURE
DAMPING_HIP_PITCH_ROLL_KNEE = 2.0 * DAMPING_RATIO * NATURAL_FREQ * AK109_ARMATURE

STIFFNESS_HIP_YAW = NATURAL_FREQ ** 2 * AK7010_ARMATURE
DAMPING_HIP_YAW = 2.0 * DAMPING_RATIO * NATURAL_FREQ * AK7010_ARMATURE

STIFFNESS_ANKLE = NATURAL_FREQ ** 2 * AK606_ARMATURE
DAMPING_ANKLE = 2.0 * DAMPING_RATIO * NATURAL_FREQ * AK606_ARMATURE

ACTUATOR_NAMES = {
  "hip_pitch": "AK109",
  "hip_roll": "AK109",
  "hip_yaw": "AK7010",
  "knee_pitch": "AK109",
  "ankle_pitch": "AK606",
  "ankle_roll": "AK606",
}

ACTUATOR_AK109 = BuiltinPositionActuatorCfg(
  target_names_expr=(".*_hip_pitch_joint", ".*_hip_roll_joint", ".*_knee_pitch_joint"),
  stiffness=STIFFNESS_HIP_PITCH_ROLL_KNEE,
  damping=DAMPING_HIP_PITCH_ROLL_KNEE,
  effort_limit=AK109_MAX_TORQUE,
)

ACTUATOR_AK7010 = BuiltinPositionActuatorCfg(
  target_names_expr=(".*_hip_yaw_joint",),  
  stiffness=STIFFNESS_HIP_YAW,
  damping=DAMPING_HIP_YAW,
  effort_limit=AK7010_MAX_TORQUE,
)

ACTUATOR_AK606 = BuiltinPositionActuatorCfg(
  target_names_expr=(".*_ankle_pitch_joint", ".*_ankle_roll_joint"),
  stiffness=STIFFNESS_ANKLE,
  damping=DAMPING_ANKLE,
  effort_limit=AK606_MAX_TORQUE,
)

ACTUATORS = (
  ACTUATOR_AK109,
  ACTUATOR_AK7010,
  ACTUATOR_AK606,
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

BIPEDAL_12DOF_ARTICULATION = EntityArticulationInfoCfg(
  actuators=ACTUATORS,
  soft_joint_pos_limit_factor=0.9,
)


def get_bipedal_12dof_robot_cfg() -> EntityCfg:
  """Get a fresh G1 robot configuration instance.

  Returns a new EntityCfg instance each time to avoid mutation issues when
  the config is shared across multiple places.
  """
  return EntityCfg(
    init_state=HOME_KEYFRAME,
    collisions=(FULL_COLLISION,),
    spec_fn=get_spec,
    articulation=BIPEDAL_12DOF_ARTICULATION,
  )


BIPEDAL_12DOF_ACTION_SCALE: dict[str, float] = {}
for a in BIPEDAL_12DOF_ARTICULATION.actuators:
  assert isinstance(a, BuiltinPositionActuatorCfg)
  e = a.effort_limit
  s = a.stiffness
  names = a.target_names_expr
  assert e is not None
  for n in names:
    BIPEDAL_12DOF_ACTION_SCALE[n] = round(0.25 * e / s, 4)


if __name__ == "__main__":
  import mujoco.viewer as viewer

  from mjlab.entity.entity import Entity

  robot = Entity(get_bipedal_12dof_robot_cfg())

  viewer.launch(robot.spec.compile())
