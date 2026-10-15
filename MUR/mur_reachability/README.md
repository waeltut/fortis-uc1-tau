# mur_reachability 0.7.1 — offline arm-specific collision checks

Each engine owns a model, solver and PlanningScene created from URDF/SRDF.
For left-arm generation, ignore all pairs involving the right moving branch;
for right-arm generation, ignore the left moving branch. The moving branch is
all descendants of the first active arm joint, including hand/tool geometry.
Fixed mounts and torso/base remain checked. All other SRDF collision rules remain.
Whole-model checks include the selected hand/tool outside the IK group.

No live scene/joint snapshot is taken. Non-IK joints use model-default values.
There is no claim of collision validity for all hand shapes or runtime payloads.
No world-obstacle or arm trajectory check is made. Other-arm position cannot
affect results because every collision pair involving that branch is ignored.

model_info() records policy self_body_ignore_other_arm_v1 and excluded links.
evaluate_offline() checks endpoint accuracy, bounds/margins and self/body collisions.
Both packages must be rebuilt, and older caches must be regenerated.
See the root README for migration and native test commands.
