from mjlab.tasks.registry import register_mjlab_task
from mjlab.tasks.velocity.rl import VelocityOnPolicyRunner

from .env_cfgs import (
  bipedal_flat_env_cfg,
  bipedal_rough_env_cfg,
)
from .rl_cfg import bipedal_ppo_runner_cfg

register_mjlab_task(
  task_id="Mjlab-Velocity-Rough-Bipedal-12dof",
  env_cfg=bipedal_rough_env_cfg(),
  play_env_cfg=bipedal_rough_env_cfg(play=True),
  rl_cfg=bipedal_ppo_runner_cfg(),
  runner_cls=VelocityOnPolicyRunner,
)

register_mjlab_task(
  task_id="Mjlab-Velocity-Flat-Bipedal-12dof",
  env_cfg=bipedal_flat_env_cfg(),
  play_env_cfg=bipedal_flat_env_cfg(play=True),
  rl_cfg=bipedal_ppo_runner_cfg(),
  runner_cls=VelocityOnPolicyRunner,
)
