package materia.automation.facility;

import robotkit.navigation.Path;

/** Directed travel corridor connecting two facility stations. */
class Lane {
  public final id:String;
  public final fromStationId:String;
  public final toStationId:String;
  public final centerline:Path;
  public final widthMeters:Float;
  public final maximumSpeedMetersPerSecond:Float;
  public final bidirectional:Bool;

  public function new(id:String, fromStationId:String, toStationId:String,
      centerline:Path, widthMeters:Float, maximumSpeedMetersPerSecond:Float,
      ?bidirectional:Bool = true) {
    if (id == null || id.length == 0 || fromStationId == null || fromStationId.length == 0 ||
        toStationId == null || toStationId.length == 0 || fromStationId == toStationId ||
        centerline == null || !Math.isFinite(widthMeters) || widthMeters <= 0.0 ||
        !Math.isFinite(maximumSpeedMetersPerSecond) || maximumSpeedMetersPerSecond <= 0.0)
      throw "Lane requires endpoints, a centerline, and positive width and speed";
    this.id = id;
    this.fromStationId = fromStationId;
    this.toStationId = toStationId;
    this.centerline = new Path(centerline.poses(), centerline.frameId);
    this.widthMeters = widthMeters;
    this.maximumSpeedMetersPerSecond = maximumSpeedMetersPerSecond;
    this.bidirectional = bidirectional;
  }
}
