package materia.automation.facility;

import robotkit.mobile.Pose2;

/** Station with a known charging interface and rated power. */
class Charger extends Station {
  public final connector:String;
  public final maximumPowerKilowatts:Float;

  public function new(id:String, name:String, zoneId:String, frameId:String, pose:Pose2,
      connector:String, maximumPowerKilowatts:Float) {
    super(id, name, zoneId, frameId, pose);
    if (connector == null || connector.length == 0 ||
        !Math.isFinite(maximumPowerKilowatts) || maximumPowerKilowatts <= 0.0)
      throw "Charger requires a connector and positive rated power";
    this.connector = connector;
    this.maximumPowerKilowatts = maximumPowerKilowatts;
  }
}
