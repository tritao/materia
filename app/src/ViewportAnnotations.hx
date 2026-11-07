package app;

/** Visibility of inspection aids, independent of machine geometry and playback. */
class ViewportAnnotations {
  public final enabled:Bool;
  public final sensors:Bool;
  public final readings:Bool;
  public final links:Bool;
  public final throughGeometry:Bool;
  public final collisions:Bool;
  public final robotId:Null<String>;
  public final sensorId:Null<String>;

  public function new(enabled:Bool = true, sensors:Bool = false, readings:Bool = false, links:Bool = false, throughGeometry:Bool = false, ?robotId:String,
      ?sensorId:String, collisions:Bool = false) {
    this.enabled = enabled;
    this.sensors = sensors;
    this.readings = readings;
    this.links = links;
    this.throughGeometry = throughGeometry;
    this.collisions = collisions;
    this.robotId = robotId;
    this.sensorId = sensorId;
  }

  public function selected(robot:String, sensor:String):Bool
    return robotId == robot && sensorId == sensor;

  public function mountVisible(robot:String, sensor:String):Bool
    return enabled && (sensors || selected(robot, sensor));

  public function readingsVisible(robot:String, sensor:String):Bool
    return enabled && (readings || selected(robot, sensor));

  public function linksVisible():Bool
    return enabled && links;

  public function any():Bool
    return enabled && (sensors || readings || links || (robotId != null && sensorId != null));

  public function collisionVisible(a:String, b:String, selectedObjects:Array<String>):Bool
    return enabled && (collisions || selectedObjects.indexOf(a) >= 0 || selectedObjects.indexOf(b) >= 0);

  public function key():String
    return '$enabled:$sensors:$readings:$links:$throughGeometry:$robotId:$sensorId:$collisions';

  /** Hidden dock tabs do not count as inspection, even though they remain open. */
  public static function panelVisible(node:haxeon.ui.docking.DockNode, id:String):Bool
    return switch node {
      case Empty: false;
      case Panel(panel): panel == id;
      case Tabs(_, active): active == id;
      case Split(_, _, first, second): panelVisible(first, id) || panelVisible(second, id);
    };

  /** Compare distances along the same picking ray; tolerate surface roundoff. */
  public static function unobstructed(targetDistance:Float, hitDistance:Null<Float>, tolerance:Float):Bool
    return hitDistance == null || hitDistance + tolerance >= targetDistance;
}
