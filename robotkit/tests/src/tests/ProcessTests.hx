package tests;

import haxe.Int64;
import motionkit.event.EventValue;
import motionkit.robot.ToolpathPosePath;
import robotkit.process.Toolpath;
import robotkit.process.ToolpathPoint;
import robotkit.spatial.Quat;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;
import robotkit.tool.ChannelToolAdapter;
import robotkit.tool.SimulatedSprayer;
import robotkit.world.FiredProcessEvent;
import robotkit.world.ProcessEventValue;

/** Toolpath authoring and B1 process-path conversion acceptance. */
class ProcessTests {
  static var assertions = 0;

  public static function run():Int {
    assertions = 0;
    testToolpathConversion();
    testScheduledToolEvents();
    Sys.println('RobotKit process tests passed ($assertions assertions)');
    return assertions;
  }

  static function testToolpathConversion():Void {
    var identity = Quat.identity();
    var p0 = new ToolpathPoint(new Transform3(new Vec3(0.0, 0.0, 0.0), identity), 0.1, false);
    var p1 = new ToolpathPoint(new Transform3(new Vec3(1.0, 0.0, 0.0), identity), 0.1, true);
    var p2 = new ToolpathPoint(new Transform3(new Vec3(1.0, 2.0, 0.0), identity), 0.1, true);
    var p3 = new ToolpathPoint(new Transform3(new Vec3(1.0, 2.0, 0.0), identity), 0.1, false);
    var toolpath = new Toolpath("work", [p0, p1, p2, p3]);

    check(approx(toolpath.length(), 3.0, 1e-9), "Toolpath length sums consecutive point distances");
    check(approx(toolpath.processOnLength(), 2.0, 1e-9),
      "Toolpath process-on length counts departing process moves");
    var segments = toolpath.segmentByProcess();
    check(segments.approach.points.length == 1 && segments.process.points.length == 2 &&
      segments.retract.points.length == 1, "Toolpath process runs retain their authored points");

    var path = ToolpathPosePath.convert(toolpath, "sprayer.flow");
    check(path.frameId == "work" && approx(path.length(), 3.0, 1e-9),
      "B1 conversion keeps the frame and geometric length");
    check(path.primitives.length == 2,
      "B1 conversion drops the duplicate zero-length terminal move");
    check(path.events.length == 2 &&
      approx(path.events[0].distance, 1.0, 1e-9) &&
      approx(path.events[1].distance, 3.0, 1e-9),
      "Process transitions are placed at path distance");
    check(switch [path.events[0].value, path.events[1].value] {
      case [EventValue.Digital(true), EventValue.Digital(false)]: true;
      case _: false;
    }, "Process transitions turn the tool on and off");
    var midpoint = path.waypointAt(2.0).pose;
    check(approx(midpoint.x, 1.0, 1e-9) && approx(midpoint.y, 1.0, 1e-9),
      "B1 path evaluates the process span in its frame");
  }

  static function testScheduledToolEvents():Void {
    var sprayer = new SimulatedSprayer();
    var adapter = new ChannelToolAdapter();
    adapter.bindSprayerFlow("sprayer.flow", sprayer, 1.5);
    adapter.apply(new FiredProcessEvent(Int64.ofInt(7), "sprayer.flow",
      ProcessEventValue.Digital(true), Int64.ofInt(200), Int64.ofInt(210), 1));
    adapter.apply(new FiredProcessEvent(Int64.ofInt(7), "sprayer.flow",
      ProcessEventValue.Digital(false), Int64.ofInt(400), Int64.ofInt(410), 1));
    check(sprayer.history.length == 2, "Each runtime record changes the sprayer once");
    check(sprayer.history[0].flow == 1.5 && sprayer.history[1].flow == 0.0,
      "Sprayer follows planned flow events");
    check(Int64.compare(sprayer.history[0].timestampNs, Int64.ofInt(200)) == 0 &&
      Int64.compare(sprayer.history[1].timestampNs, Int64.ofInt(400)) == 0,
      "Sprayer timestamps use scheduled trajectory time");
  }

  static function approx(a:Float, b:Float, tolerance:Float):Bool
    return Math.abs(a - b) <= tolerance;

  static function check(value:Bool, message:String):Void {
    assertions++;
    if (!value) throw 'assertion failed: $message';
  }
}
