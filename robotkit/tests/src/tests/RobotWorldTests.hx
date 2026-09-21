package tests;

import haxe.Int64;
import robotkit.runtime.RobotRuntimeBlueprint;
import robotkit.runtime.RobotRuntimeCompiler;
import robotkit.runtime.RobotRuntimeJointBlueprint;
import robotkit.runtime.RobotCompileException;
import robotkit.runtime.Simulation;
import robotkit.model.Joint;
import robotkit.model.JointType;
import robotkit.model.Link;
import robotkit.model.RobotModel;
import robotkit.world.RobotCapabilities;
import robotkit.world.RobotCommand;
import robotkit.world.RobotDescription;
import robotkit.world.RobotFault;
import robotkit.world.RobotId;
import robotkit.world.Robot;
import robotkit.world.RobotSnapshot;
import robotkit.world.RobotStatus;
import robotkit.world.RemoteRobot;
import robotkit.world.SimulatedRobot;
import robotkit.world.StopMode;
import robotkit.world.RobotWorld;

class RobotWorldTests {
  static var assertions = 0;

  public static function main():Void {
    testAttachDetachAndIdentity();
    testSequenceAndTopology();
    testImmutableSnapshots();
    testForwardingAndLifecycle();
    testMixedSimulatedAndRemoteWorld();
    testCompilerDiagnosticsAndTopology();
    Sys.println('RobotKit world tests passed ($assertions assertions)');
  }

  static function testCompilerDiagnosticsAndTopology():Void {
    var model = new RobotModel("diagnostic-arm");
    var base = model.addLink(new Link("base"));
    var tool = model.addLink(new Link("tool"));
    var shoulder = model.addJoint(new Joint("shoulder", JointType.Revolute, base, tool));
    shoulder.limits.lower = -1.0;
    shoulder.limits.upper = 1.0;
    var blueprint = RobotRuntimeCompiler.compile(model);
    equal(blueprint.jointCount, 1, "compiler lowers a valid topology");
    equal(blueprint.linkCount, 2, "compiler preserves link count");

    var invalid = new RobotModel("invalid-arm");
    var root = invalid.addLink(new Link("root"));
    var branch = invalid.addLink(new Link("branch"));
    invalid.addLink(new Link("branch"));
    var cycle = invalid.addJoint(new Joint("cycle", JointType.Revolute, root, branch));
    cycle.limits.lower = 2.0;
    cycle.limits.upper = -2.0;
    invalid.addJoint(new Joint("cycle", JointType.Floating, branch, root));
    var diagnostics = RobotRuntimeCompiler.validate(invalid);
    check(hasDiagnostic(diagnostics, "RK_LINK_DUPLICATE"),
      "compiler reports duplicate link names");
    check(hasDiagnostic(diagnostics, "RK_JOINT_DUPLICATE"),
      "compiler reports duplicate joint names");
    check(hasDiagnostic(diagnostics, "RK_LIMITS"),
      "compiler reports invalid joint limits");
    check(hasDiagnostic(diagnostics, "RK_JOINT_UNSUPPORTED"),
      "compiler reports unsupported joint types");
    check(hasDiagnostic(diagnostics, "RK_TOPOLOGY_CYCLE"),
      "compiler reports topology cycles");
    check(hasDiagnostic(diagnostics, "RK_TOPOLOGY_DISCONNECTED"),
      "compiler reports links disconnected from the root");
    var forest = new RobotModel("forest");
    forest.addLink(new Link("left"));
    forest.addLink(new Link("right"));
    check(hasDiagnostic(RobotRuntimeCompiler.validate(forest), "RK_TOPOLOGY_ROOT"),
      "compiler requires exactly one topology root");
    var threw = false;
    try {
      RobotRuntimeCompiler.compile(invalid);
    } catch (error:RobotCompileException) {
      threw = true;
      check(error.diagnostics.length >= 5,
        "compile exception retains all structured diagnostics");
      check(error.diagnostics[0].path != null && error.diagnostics[0].code != null,
        "compile diagnostics expose paths and stable codes");
    }
    check(threw, "invalid topology refuses native lowering");
  }

  static function hasDiagnostic(diagnostics:Array<robotkit.runtime.RobotCompileDiagnostic>,
      code:String):Bool {
    for (diagnostic in diagnostics)
      if (diagnostic.code == code) return true;
    return false;
  }

  static function testAttachDetachAndIdentity():Void {
    var world = new RobotWorld();
    var robot = new FakeRobot("warehouse/forklift-17");
    world.attach(robot);
    equal(world.robot("warehouse/forklift-17"), robot, "attach registers logical ID");
    equal(world.snapshot().robotIds()[0], "warehouse/forklift-17", "snapshot keeps logical ID");
    equal(world.detach("missing"), null, "missing detach is harmless");
    equal(world.detach(robot.id()), robot, "detach returns ownership");
    equal(world.robot(robot.id()), null, "detach unregisters robot");
    check(!robot.closed, "detach does not close returned robot");
    robot.emitChange();
    equal(world.snapshot().sequence, 2, "detached robot no longer changes world");
    world.close();
    check(!robot.closed, "world does not close detached robot");
  }

  static function testSequenceAndTopology():Void {
    var world = new RobotWorld();
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
    var world = new RobotWorld();
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
    first.robots().resize(0);
    check(second.robot("arm") != null, "snapshot maps are independent");
    world.close();
  }

  static function testForwardingAndLifecycle():Void {
    var world = new RobotWorld();
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

  static function testMixedSimulatedAndRemoteWorld():Void {
    var blueprint = new RobotRuntimeBlueprint(1, 1, 2);
    blueprint.addJoint(new RobotRuntimeJointBlueprint(
      0,
      RobotKitRuntimeConstants.RK_RUNTIME_JOINT_REVOLUTE,
      0,
      1,
      -3.14,
      3.14,
      100.0
    ));
    var simulation = new Simulation();
    var first = new SimulatedRobot(
      "sim-a",
      simulation.addRobot(blueprint),
      "simulated A",
      ["base", "tool"],
      ["shoulder"]
    );
    var second = new SimulatedRobot(
      "sim-b",
      simulation.addRobot(blueprint),
      "simulated B",
      ["base", "tool"],
      ["shoulder"]
    );
    var remote = new RemoteRobot("remote-c");
    var world = new RobotWorld();
    world.attach(first);
    world.attach(second);
    world.attach(remote);

    world.submit("sim-a", RobotCommand.JointPosition(0, 0.4, null));
    world.submit("sim-b", RobotCommand.JointPosition(0, -0.3, null));
    simulation.step(Int64.ofInt(1000));

    var value = world.snapshot();
    var firstState = value.robot("sim-a");
    var secondState = value.robot("sim-b");
    var remoteState = value.robot("remote-c");
    check(firstState != null, "mixed world contains first simulated robot");
    check(secondState != null, "mixed world contains second simulated robot");
    check(remoteState != null, "mixed world contains remote robot");
    var firstValue:RobotSnapshot = cast firstState;
    var secondValue:RobotSnapshot = cast secondState;
    var remoteValue:RobotSnapshot = cast remoteState;
    check(Math.abs(firstValue.positions[0] - 0.4) < 0.000000001,
      "first simulated command routed independently");
    check(Math.abs(secondValue.positions[0] + 0.3) < 0.000000001,
      "second simulated command routed independently");
    equal(firstValue.timestampNs, Int64.ofInt(1000), "shared simulation timestamp reaches first robot");
    equal(secondValue.timestampNs, Int64.ofInt(1000), "shared simulation timestamp reaches second robot");
    equal(remoteValue.sourceSequence, Int64.ofInt(0), "remote state remains independently sourced");
    var remoteStatus = remote.status();
    check(switch remoteStatus {
      case RobotStatus.Disconnected: true;
      case _: false;
    }, "unconnected remote remains disconnected");

    world.close();
    simulation.step(Int64.ofInt(2000));
    equal(simulation.stepIndex(), Int64.ofInt(2), "RobotWorld does not own shared simulation");
    simulation.dispose();
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

private class FakeRobot implements Robot {
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
