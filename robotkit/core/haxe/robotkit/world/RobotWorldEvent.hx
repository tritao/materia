package robotkit.world;

import robotkit.core.RobotId;

/** Adapter notifications applied by RobotWorld on its owner event-loop turn. */
enum RobotWorldEvent {
  RobotAttached(id:RobotId);
  RobotDetached(id:RobotId);
  RobotChanged(id:RobotId);
}
