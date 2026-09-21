package tests;

import haxe.Int64;
import robotkit.world.RobotCapabilities;
import robotkit.world.RobotCommand;
import robotkit.world.RobotDescription;
import robotkit.world.RobotFault;
import robotkit.world.RobotId;
import robotkit.world.RobotInstance;
import robotkit.world.RobotSnapshot;
import robotkit.world.RobotStatus;
import robotkit.world.StopMode;
import robotkit.world.WorldHost;

class WorldHostTests {
  static var assertions = 0;

  // Keeps the optional native simulation façade in the package compile surface.
  static function simulationTypeCheck(value:robotkit.runtime.Simulation):robotkit.runtime.Simulation
    return value;

  public static function main():Void {
    testAttachDetachAndIdentity();
    testSequenceAndTopology();
    testImmutableSnapshots();
    testForwardingAndLifecycle();
    Sys.println('RobotKit world tests passed ($assertions assertions)');
  }

  static function testAttachDetachAndIdentity():Void {
    var world = new WorldHost();
    var robot = new FakeRobot("warehouse/forklift-17");
    world.attach(robot);
    equal(world.robots.get("warehouse/forklift-17"), robot, "attach registers logical ID");
    equal(world.snapshot().robotIds()[0], "warehouse/forklift-17", "snapshot keeps logical ID");
    equal(world.detach("missing"), null, "missing detach is harmless");
    equal(world.detach(robot.id()), robot, "detach returns ownership");
    equal(world.robots.get(robot.id()), null, "detach unregisters robot");
    check(!robot.closed, "detach does not close returned robot");
    robot.emitChange();
    equal(world.snapshot().sequence, 2, "detached robot no longer changes world");
    world.close();
    check(!robot.closed, "world does not close detached robot");
  }

  static function testSequenceAndTopology():Void {
    var world = new WorldHost();
    var left = new FakeRobot("left");
    var right = new FakeRobot("right");
    equal(world.snapshot().sequence, 0, "new world sequence");
    equal(world.snapshot().topologyRevision, 0, "new topology revision");
    world.attach(left);
    equal(world.snapshot().sequence, 1, "attach advances sequence");
    equal(world.snapshot().topologyRevision, 1, "attach advances topology");
    left.emitChange();
    equal(world.snapshot().sequence, 2, "state change advances sequence");
    equal(world.snapshot().topologyRevision, 1, "state change preserves topology");
    world.attach(right);
    equal(world.snapshot().sequence, 3, "second attach advances sequence");
    equal(world.snapshot().topologyRevision, 2, "second attach advances topology");
    world.detach(left.id());
    equal(world.snapshot().sequence, 4, "detach advances sequence");
    equal(world.snapshot().topologyRevision, 3, "detach advances topology");
    world.close();
  }

  static function testImmutableSnapshots():Void {
    var world = new WorldHost();
    var robot = new FakeRobot("arm");
    robot.positions = [1.0, 2.0];
    world.attach(robot);
    var first = world.snapshot();
    robot.positions[0] = 9.0;
    robot.emitChange();
    var second = world.snapshot();
    var firstRobot = first.robot("arm");
    var secondRobot = second.robot("arm");
    check(firstRobot != null && firstRobot.positions[0] == 1.0, "old snapshot owns copied arrays");
    check(secondRobot != null && secondRobot.positions[0] == 9.0, "new snapshot observes new state");
    first.robots.remove("arm");
    check(second.robot("arm") != null, "snapshot maps are independent");
    world.close();
  }

  static function testForwardingAndLifecycle():Void {
    var world = new WorldHost();
    var robot = new FakeRobot("arm");
    world.attach(robot);
    var command = RobotCommand.JointPosition(3, 1.25, Int64.ofInt(99));
    world.submit("arm", command);
    equal(Std.string(robot.lastCommand), Std.string(command), "command forwarded to selected robot");
    world.stop("arm", StopMode.Emergency);
    equal(Std.string(robot.lastStop), Std.string(StopMode.Emergency), "stop forwarded to selected robot");
    throws(function() world.attach(new FakeRobot("arm")), "duplicate logical ID rejected");
    world.close();
    check(robot.closed, "world closes attached robot it owns");
    equal(robot.listener, null, "world clears listener before close");
    world.close();
    equal(robot.closeCount, 1, "world close is idempotent");
    throws(function() world.submit("arm", command), "closed world rejects commands");
  }

  static function check(value:Bool, message:String):Void {
    assertions++;
    if (!value) throw 'assertion failed: $message';
  }

  static function equal(actual:Dynamic, expected:Dynamic, message:String):Void check(
    actual == expected,
    '$message (expected ${Std.string(expected)}, got ${Std.string(actual)})'
  );

  static function throws(action:Void -> Void, message:String):Void {
    var didThrow = false;
    try action() catch (_:Dynamic) didThrow = true;
    check(didThrow, message);
  }
}

private class FakeRobot implements RobotInstance {
  final logicalId:RobotId;
  public var positions:Array<Float> = [0.0];
  public var listener:Null < Void -> Void > = null;
  public var lastCommand:Null<RobotCommand> = null;
  public var lastStop:Null<StopMode> = null;
  public var closed = false;
  public var closeCount = 0;

  public function new(id:RobotId) logicalId = id;
  public function id():RobotId return logicalId;
  public function status():RobotStatus return Ready;
  public function description():RobotDescription return new RobotDescription(logicalId, logicalId, [], []);
  public function capabilities():RobotCapabilities return new RobotCapabilities(
    logicalId,
    positions.length,
    true,
    false,
    false,
    false
  );
  public function snapshot():RobotSnapshot return new RobotSnapshot(
    logicalId,
    Int64.ofInt(1),
    Int64.ofInt(10),
    positions,
    [],
    [],
    1,
    0
  );
  public function fault():Null < RobotFault > return null;
  public function submit(command:RobotCommand):Void lastCommand = command;
  public function stop(mode:StopMode):Void lastStop = mode;
  public function setChangeListener(value:Null < Void -> Void >):Void listener = value;
  public function emitChange():Void if (listener != null) listener();
  public function close():Void {
    closed = true;
    closeCount++;
  }
}
