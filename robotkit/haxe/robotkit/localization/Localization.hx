package robotkit.localization;

import robotkit.mobile.Pose2;
import robotkit.world.RobotSnapshot;

/** Service that derives a framed pose estimate from robot observations. */
interface Localization {
  function update(snapshot:RobotSnapshot):LocalizationState;
  function state():Null<LocalizationState>;
  function reset(?pose:Pose2):Void;
}
