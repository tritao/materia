import cadkit.modeling.AssemblyState;
import gearkit.*;
import machinekit.assembly.*;
import machinekit.assembly.AxisBuilder;
import machinekit.component.*;
import machinekit.control.BissMaster;
import machinekit.design.DesignRequirement;
import machinekit.motion.*;
import machinekit.hobbing.*;
import machinekit.power.*;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.assembly.AssemblyDefinition.AssemblyJointType;

class HobberServiceAccess {
  final machine:FlexsplineHobber;
  public function new(machine:FlexsplineHobber) this.machine=machine;
  /** Every member below `root` in the mate tree, `root` not included. */
  function below(root:String):Array<String> {
    var parents = machine.mateParents(), out = [];
    for (c in machine.components()) {
      var at:Null<String> = c.id, guard = 0;
      while (at != null && at != root && guard++ < 200) at = parents.get(at);
      if (at == root && c.id != root) out.push(c.id);
    }
    return out;
  }

  /** The direction (assembly world frame) of `local` in member `id`'s frame at the rest pose, as a unit vector. */
  function axisOf(id:String, x:Float, y:Float, z:Float):Array<Float> {
    var p = machine.solvedPoses().get(id), v = F.transformVector(p, x, y, z);
    return [v.x, v.y, v.z];
  }

  /** Clearance added to every service path past the length that frees the part, mm. */
  public static final SERVICE_CLEARANCE = 5.0;

  function startsWith(id:String, prefix:String):Bool return StringTools.startsWith(id, prefix);

  /** The members of the B cartridge: the housing with the shaft, bearings, encoder, pulley and arbor, and the hob on it (not the servo, its pulley or the belt). */
  function cartridgeMembers():Array<String> {
    var motorGroup = ["spindle/motor", "spindle/motorShaft", "spindle/motorPulley", "spindle/motorEncoder", "spindle/belt"];
    return [for (c in machine.components()) if (startsWith(c.id, "spindle/") && motorGroup.indexOf(c.id) < 0) c.id].concat(["hob"]);
  }

  /** The declared removals. */
  public function serviceRemovals():Array<DesignRequirement> {
    var away = (d:Array<Float>) -> [for (c in d) -c];
    var out:Array<DesignRequirement> = [];
    for (axis in ["X", "Z"]) {
      var back = away(axisOf(HobberDrives.member(axis,"motor"), 0, 0, 1));
      out.push(ServiceRemoval(axis.toLowerCase() + "MotorWithdrawal", [HobberDrives.member(axis,"motor"), HobberDrives.member(axis,"motorShaft")],
        [{direction: back, distance: 30 + 10 + SERVICE_CLEARANCE}], [], []));
    }
    var nutGroup=below("Ymodule/nutCarrier").concat(["Ymodule/nutCarrier"]);
    var carriageBundle=below("Ymodule/carriage").concat(["Ymodule/carriage"]).filter(id->!nutGroup.contains(id)&&!StringTools.startsWith(id,"Ymodule/block")&&id!="Ymodule/scaleHead");
    out.push(ServiceRemoval("yCarriageWithdrawal",carriageBundle,[{direction:[0.0,-1,0],distance:HobberLayout.get().moduleSpec().carriageThickness+40}],
      ["Ymodule/scaleHead"],[{joint:"X",value:HobberLayout.get().xRetract}]));
    var shaft = machine.spindle.motor.rating.shaftLength;
    out.push(ServiceRemoval("bMotorWithdrawal", ["spindle/motor", "spindle/motorShaft", "spindle/motorEncoder"],
      [{direction: away(axisOf("spindle/motor", 0, 0, 1)), distance: shaft + 10 + SERVICE_CLEARANCE}], ["spindle/belt", "spindle/motorPulley"], []));
    out.push(ServiceRemoval("bBeltRemoval", ["spindle/belt"],
      [{direction: away(axisOf("spindle/housing", 0, 0, 1)), distance: 2 * HobSpindle.BELT_WIDTH + SERVICE_CLEARANCE}], [], []));
    out.push(ServiceRemoval("outboardRetainersWithdrawal", ["outboardLocknut", "outboardInnerSpacer"],
      [{direction: axisOf("spindle/housing", 0, 0, 1), distance: 16.0}], [], [{joint: "X", value: HobberLayout.get().xRetract}]));
    out.push(ServiceRemoval("outboardWithdrawal", HobberBSpindle.outboardMembers(),
      [{direction: axisOf("spindle/housing", 0, 0, 1), distance: 55.0}], ["outboardLocknut", "outboardInnerSpacer", "Ymodule/scaleHead"], [{joint: "X", value: HobberLayout.get().xRetract}]));
    out.push(ServiceRemoval("spindleCartridgeWithdrawal", cartridgeMembers(),
      [{direction: axisOf("spindle/housing", 0, 0, 1), distance: 180 + 60}], ["spindle/belt", "outboardLocknut", "outboardInnerSpacer", "Ymodule/scaleHead"].concat(HobberBSpindle.outboardMembers()), [{joint: "X", value: HobberLayout.get().xRetract}]));
    // The flange is motor-side. Release the nut flange, motor and BK12;
    // the BF12 stays and the barrel slides through the stationary nut block.
    var x = HobberDrives.x();
    var rearward = HobberLayout.get().xNominal().bedRear - (x.axis.start[0] - x.length) + SERVICE_CLEARANCE;
    out.push(ServiceRemoval("xScrewWithdrawal", ["Xscrew", "Xcoupling", "Xnut"],
      [{direction: [0.0,1,0], distance: rearward}],
      ["Xmotor", "Xmotor-shaft", "Xfixed", "XfloatingRing"], []));
    var bayPark = machine.travel.filter(t -> t.id == "X")[0].lower;
    var baySlide = 386 + bayPark - (115 + HobberChipCover.nutBounds().maxX + 3) + 3;
    out.push(ServiceRemoval("xMotorBayLid", ["XrearCover", "XrearPlugLeft", "XrearPlugRight"],
      [{direction:[0.0,0,1],distance:3.0}, {direction:[0.0,1,0],distance:baySlide},
       {direction:[0.0,0,1],distance:100.0}], [], [{joint:"X",value:bayPark}]));
    var stack = below("Xbridge").concat(["Xbridge"]).filter(id -> id != "Xnut");
    out.push(ServiceRemoval("xChipGuardWithdrawal",HobberChipCover.fixedMembers(),
      [{direction:[0.0,0,1],distance:100.0}],stack,[]));
    var head = below("Ymodule/base").concat(["Ymodule/base"]).filter(id -> id != "Ymodule/motor" && id != "Ymodule/motorShaft");
    var retract = [{joint:"X",value:HobberLayout.get().xRetract}];
    // Keep the head on the actuator: the adapter output screws are buried
    // behind the swivel plate. Release the saddle from its blocks and nut
    // block, and support the disconnected scale head on the fixed scale.
    out.push(ServiceRemoval("erobWithSaddle",["Zplate","swivel/housing","swivel/output","ZscaleArm"].concat(machine.zBellowsMounts.filter(id->id.indexOf("Moving")>=0)).concat(head),
      [{direction:[0.0,-1,0],distance:60.0}],["Ymodule/motor","Ymodule/motorShaft"],retract));
    return out;
  }

}
