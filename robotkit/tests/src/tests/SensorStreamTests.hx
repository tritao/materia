package tests;
import haxe.Int64;
import robotkit.core.SensorFrame;
import robotkit.core.RobotSnapshot;
import robotkit.streams.CameraImage;
import robotkit.streams.SensorStreams;
import robotkit.streams.SensorStreamSample;
class SensorStreamTests {
  static var assertions:Int;
  static function check(value:Bool):Void { assertions++; if (!value) throw "Sensor stream contract failed"; }
  public static function run():Int {
    assertions = 0;
    var imu = new SensorFrame("imu","imu","base",Int64.ofInt(1),Int64.ofInt(10),[1,2,3]);
    var lidar = new SensorFrame("scan","lidar","base",Int64.ofInt(2),Int64.ofInt(20),[1,2,3]);
    var camera = new SensorFrame("camera","camera","base",Int64.ofInt(3),Int64.ofInt(30),[],Int64.ofInt(40),"base",null,null,"camera.clock","host.clock",
      new CameraImage(4,4,"rgb8",haxe.io.Bytes.alloc(48)));
    var snapshot = new RobotSnapshot("robot",Int64.ofInt(1),Int64.ofInt(10),[0],[0],[0],0,0,null,[imu,lidar,camera]);
    check(snapshot.sensors.length == 1 && snapshot.sensors.get(0).sensorId == "imu");
    check(snapshot.streamSequences.length == 3);
    var world = new robotkit.world.WorldSnapshot(1, 1, Int64.ofInt(10), ["robot" => snapshot]);
    var published:RobotSnapshot = cast world.robot("robot");
    check(published.streamSequences.length == 3);
    var copy = snapshot.streamSequences; copy.resize(0);
    check(snapshot.streamSequences.length == 3);
    var streams = new SensorStreams(); var received = 0;
    var subscription = streams.subscribe("camera",function(value) {
      received++; check(value.sourceClockId == "camera.clock" && value.receivedClockId == "host.clock");
      check(Int64.compare(value.sourceTimestampNs,Int64.ofInt(30)) == 0);
    });
    streams.publish(SensorStreamSample.sensor(camera));
    check(received == 1 && streams.latestFrames().length == 1);
    streams.publish(SensorStreamSample.sensor(camera)); check(received == 1);
    check(streams.sequences()[0].streamId == "camera");
    subscription.cancel(); subscription.cancel();
    var latest = 0;
    var replay = streams.subscribe("*",function(_) latest++);
    check(latest == 1);
    replay.cancel(); streams.clear(); check(streams.latestFrames().length == 0);
    return assertions;
  }
}
