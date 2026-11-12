package machinekit.milling;

import machinekit.assembly.MachineAssembly;
import machinekit.component.*;
import machinekit.assembly.Transmission;
import machinekit.assembly.Sense;
import machinekit.transmission.TimingBelt;
import machinekit.standard.SocketHeadCapScrew;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** ATC-ready BT30 spindle packaging: cartridge, power drawbar and release cylinder, and the belt drive the carrier holds.
 * Both release joints are inspection motions, not a pneumatic control system.
 * Supplier release force/stroke, spring sizing and orientation feedback are pending. */
class MillSpindleHead extends MachineAssembly {
  public final gauge:AssemblyFrame;
  public function new(l:MillZAxisLayout, adjustment:Float, drive:MillSpindleDrivePart) {
    super();
    if (!Math.isFinite(adjustment)
      || Math.abs(adjustment) > MillSpindleMotorLayout.ADJUSTMENT) throw "Spindle belt adjustment exceeds its slots";
    gauge = F.translation(0, l.spindleY, l.headBottom - Bt30Cartridge.SEAT);
    addComponent("spindle", new Bt30Cartridge(), gauge);
    rotating(
      "rotor",
      new MillPowerDrawbarPart(new ComponentValues().setToken("role", "rotor")),
      gauge,
      "spindle",
      "rotor-turn"
    );
    for (role in ["springs", "pullStud", "holder", "reactionFrame", "cylinder"]) fixed(
      role,
      new MillPowerDrawbarPart(new ComponentValues().setToken("role", role)),
      gauge,
      (role == "reactionFrame" || role == "cylinder") ? "spindle" : "rotor"
    );
    moving("drawbar", new MillPowerDrawbarPart(new ComponentValues().setToken("role", "drawbar")), gauge, "rotor", 4.5);
    fixed("gripper", new MillPowerDrawbarPart(new ComponentValues().setToken("role", "gripper")), gauge, "drawbar");
    moving(
      "pistonRod",
      new MillPowerDrawbarPart(new ComponentValues().setToken("role", "pistonRod")),
      gauge,
      "cylinder",
      MillDrawbarLayout.RELEASE_STROKE
    );
    fixed(
      "piston",
      new MillPowerDrawbarPart(new ComponentValues().setToken("role", "piston")),
      gauge,
      "pistonRod"
    );
    for (i in 0...6) {
      var a = i * Math.PI / 3;
      fixed(
        'collarBolt$i',
        SocketHeadCapScrew.metric("M6", 30),
        F.fromRotationMatrix(
          53.5 * Math.cos(a),
          l.spindleY + 53.5 * Math.sin(a),
          gauge.z + 40,
          [1, 0, 0, 0, -1, 0, 0, 0, -1]
        ),
        "spindle"
      );
    }
    for (i => p in [[-37.5, 0.0], [37.5, 0.0], [0.0, -37.5], [0.0, 37.5]]) fixed(
      'reactionBolt${i + 1}',
      SocketHeadCapScrew.metric("M5", 12),
      F.translation(p[0], l.spindleY + p[1], gauge.z + 244.6),
      "reactionFrame"
    );
    for (x in [-MillDrawbarLayout.MOUNT_X, MillDrawbarLayout.MOUNT_X]) for (y in [
      -MillDrawbarLayout.MOUNT_Y,
      MillDrawbarLayout.MOUNT_Y
    ]) fixed(
      'cylinderBolt$x-$y',
      SocketHeadCapScrew.metric("M6", 16),
      F.fromRotationMatrix(x, l.spindleY + y, gauge.z + MillDrawbarLayout.BRIDGE_BOTTOM, [1, 0, 0, 0, -1, 0, 0, 0, -1]),
      "reactionFrame"
    );
    var motor = drive.motor();
    var face = gauge.z + drive.face(), mountBottom = l.headBottom + l.headHeight;
    var centres = drive.centres() + adjustment;
    fixed("motorCarrier", drive, gauge, "spindle");
    // Route catalog power/feedback connectors sideways, clear of the Z screw stack.
    var motorPose = F.fromRotationMatrix(
      0,
      l.spindleY + centres,
      face,
      [0, 1, 0, 1, 0, 0, 0, 0, -1]
    );
    fixed("motor", motor, motorPose, "motorCarrier");
    rotating("motorShaft", motor.shaft(), motorPose, "motor", "motor-turn", motor.rating.maxSpeed);
    fixed("spindlePulley", drive.spindlePulley(), F.translation(0, l.spindleY, gauge.z + 251), "rotor");
    fixed(
      "motorPulley",
      drive.motorPulley(),
      F.translation(0, l.spindleY + centres, gauge.z + 251),
      "motorShaft"
    );
    fixed("beltGuard", new MillSpindleDrivePart(drive.values().setToken("role", "guard")), gauge, "motorCarrier");
    var guardX = drive.guardBoltX();
    for (x in [-guardX, guardX]) fixed(
      'guardBolt$x',
      SocketHeadCapScrew.metric("M4", 10),
      F.fromRotationMatrix(
        x,
        l.spindleY + drive.guardBoltY(),
        face - 14,
        [1, 0, 0, 0, -1, 0, 0, 0, -1]
      ),
      "beltGuard"
    );
    fixed(
      "belt",
      TimingBelt.twoPulley(
        HTD5M,
        drive.spindleTeeth(),
        drive.motorTeeth(),
        centres,
        15,
        5000
      ),
      F.fromRotationMatrix(0, l.spindleY, gauge.z + 252.5, [0, -1, 0, 1, 0, 0, 0, 0, 1]),
      "spindle"
    );
    for (x in [-MillSpindleMotorLayout.FOOT_BOLT_X, MillSpindleMotorLayout.FOOT_BOLT_X]) for (y in [
      MillSpindleMotorLayout.FOOT_BOLT_FRONT,
      MillSpindleMotorLayout.FOOT_BOLT_REAR
    ]) fixed(
      'carrierBolt$x-$y',
      SocketHeadCapScrew.metric("M6", 20),
      F.translation(x, l.spindleY + y, mountBottom + 10),
      "motorCarrier"
    );
    var i = 0, size = drive.screwSize(), nut = machinekit.standard.HexNut.metric(size);
    var length = 5 * Math.ceil((12 + drive.flangeThickness() + nut.height + 2) / 5);
    for (p in drive.mountingHoles()) {
      fixed(
        'motorBolt${i}',
        SocketHeadCapScrew.metric(size, length),
        F.fromRotationMatrix(-p.x, l.spindleY + centres + p.y, face - 12, [1, 0, 0, 0, -1, 0, 0, 0, -1]),
        "motorCarrier"
      );
      fixed(
        'motorNut${i++}',
        machinekit.standard.HexNut.metric(size),
        F.translation(-p.x, l.spindleY + centres + p.y, face + drive.flangeThickness()),
        "motor"
      );
    }
    addMemberConnector("spindlePulley", "beltPlane", Solids.axial(0, 0, 1.5));
    addMemberConnector("motorPulley", "beltPlane", Solids.axial(0, 0, 1.5));
    addBeltPath({
      belt: "belt",
      wraps: [
        {instanceId: "spindlePulley", connectorName: "beltPlane"},
        {instanceId: "motorPulley", connectorName: "beltPlane"}
      ]
    });
    addTransmission(
      "spindle-belt",
      "rotor-turn",
      "motor-turn",
      Transmission.BeltReduction("belt", "spindlePulley", "motorPulley"),
      Same
    );
    exposeConnector("mount", "spindle", "mount");
  }
  function rotating(
    id:String,
    part:MachineComponent,
    pose:AssemblyFrame,
    parent:String,
    joint:String,
    ?maximumSpeed:Float
  ):Void {
    addComponent(id, part, pose);
    var positions = solvedPoses(), frame = F.translation(pose.x, pose.y, pose.z);
    addMemberConnector(parent, 'to-$id', F.compose(F.inverse(positions.get(parent)), frame));
    addMemberConnector(id, "seat", F.compose(F.inverse(pose), frame));
    addMateOnAxis(
      joint,
      "continuous",
      parent,
      'to-$id',
      id,
      "seat",
      {x: 0, y: 0, z: 1},
      0,
      {
        lower: null,
        upper: null,
        velocity: maximumSpeed ?? (MillBeltSpindle.MAXIMUM_RPM * 2 * Math.PI / 60),
        effort: null
      }
    );
  }

  function fixed(id:String, part:MachineComponent, pose:AssemblyFrame, parent:String):Void {
    addComponent(id, part, pose);
    var positions = solvedPoses();
    addMemberConnector(parent, 'to-$id', F.compose(F.inverse(positions.get(parent)), pose));
    addMemberConnector(id, "seat", F.identity());
    addMate('attach-$id', "fixed", parent, 'to-$id', id, "seat");
  }
  function moving(id:String, part:MachineComponent, pose:AssemblyFrame, parent:String, stroke:Float):Void {
    addComponent(id, part, pose);
    var positions = solvedPoses();
    addMemberConnector(parent, 'to-$id', F.compose(F.inverse(positions.get(parent)), pose));
    addMemberConnector(id, "seat", F.identity());
    addMateOnAxis(
      '$id-release',
      "prismatic",
      parent,
      'to-$id',
      id,
      "seat",
      {x: 0, y: 0, z: -1},
      0,
      {lower: 0, upper: stroke, velocity: null, effort: null}
    );
  }
}
