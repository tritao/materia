# SimKit follow-ups

- An object held by a robot link (`nksim_session_hold_object` with a link
  body as carrier) follows the link's state from the previous tick, so it
  trails a fast-moving gripper by one tick. Actor parts don't lag: their pose
  for the tick is known before the step. Driving held objects from the link
  pose the step will produce needs the carrier's pose inside the host step.
