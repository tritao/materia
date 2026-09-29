package robotkit.manipulation;

import robotkit.spatial.Transform3;

/** Robot-specific static payload envelope. Return null outside its rated domain. */
interface RobotLoadChart {
  public function limitsAt(q:Array<Float>, base_T_flange:Transform3):Null<RobotPayloadLimit>;
}
