import hobber.*;
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

class HobberBSpindle extends HobberUnit {
  public final spindle:HobSpindle;
  public final hob:Hob;
  public function new(skeleton:HobberSkeleton, spec:GearSpec, gashes:Int, detailedHob:Bool) {
    super("bSpindle", skeleton);
    var profile = spec.profile();
    var hobTool = Hob.fromDesign(new HobDesign(ConjugateRack.of(profile),
      1, 32, gashes), spec, 40, 16, false, detailedHob);
    hob = new Hob(hobTool.values().setNumber("faceKeyWidth", 5.2));
    var centre=0.0, e0=layout.spindleRotation;
    spindle = new HobSpindle();
    include("spindle", spindle, HobberFrame.partAt(-(layout.spindleNativeY), centre, heights.hobCentreZ, e0));
    // One milled split-clamp block holds the Ø80 cartridge (FRAME_PLAN R2): bore with a slit up the +z side, three clamp screws across it,
    // four M8 and two 4 mm drawing dowels into the plain Y carriage.
    externalPart("spindleSeat", new HobberSpindleSeat(), HobberFrame.partAt(0, centre, heights.hobCentreZ), "yStage", "carriage",
      F.alongY(-105, centre + 45, heights.hobCentreZ, 0, 1, 0));
    fixedMembers("spindle-mount", "spindleSeat", "spindle/housing", F.alongY(-150, centre + 40, heights.hobCentreZ, 0, 1, 0));
    // The belt-drive servo stands above the cartridge on a milled cradle bolted to the top of the spindle seat block (the stiffest part
    // near it); its flange is on the cradle wall.
    fixedPart("motorCradle", new HobberMotorCradle(), HobberFrame.partAt(0, centre, heights.hobCentreZ), "spindleSeat",
      F.alongY(-190, centre, heights.hobCentreZ + 58, 0, 0, 1));
    fixedMembers("motor-mount", "motorCradle", "spindle/motor",
      F.alongY(-(layout.spindleNativeY - HobSpindle.MOTOR_FACE_Z), centre - HobSpindle.MOTOR_OFFSET, HobberFrame.PART_DATUM_Z + (layout.nativeHobCentreZ + HobSpindle.motorHeight()), -1, 0, 0));
    fixedPart("hob", hob, HobberFrame.partAt(-20, centre, heights.hobCentreZ, e0), "spindle/arbor");
    group("Outboard support");
    externalPart("outboardCarrier", new HobberOutboardCarrier(), HobberFrame.partAt(0, centre, heights.hobCentreZ), "yStage", "carriage",
      F.alongY(-30, centre + 45, heights.hobCentreZ, 0, 1, 0));
    var supportPose = F.compose(HobberFrame.partAt(0, centre, heights.hobCentreZ), layout.outboardDatum());
    fixedPart("outboard", new OutboardSupport(), supportPose, "outboardCarrier",
      F.alongY(-(-100), centre + 30, heights.hobCentreZ, 0, 1, 0));
    for (role in ["sleeve", "gland", "cap"]) fixedPart("outboard" + (role == "sleeve" ? "Sleeve" : role == "gland" ? "Gland" : "Cap"),
      new OutboardBearingPart(new ComponentValues().setToken("role", role)),
      supportPose, role == "cap" ? "outboard" : role == "gland" ? "outboardSleeve" : "outboard");
    for (rear in [false, true]) {
      var id = rear ? "outboardBearingRear" : "outboardBearingFront";
      var pose = F.compose(supportPose, F.fromRotationMatrix(0, 0, rear ? OutboardSupport.PAIR_TO : OutboardSupport.PAIR_FROM,
        rear ? [1.0, 0, 0, 0, -1, 0, 0, 0, -1] : [1.0, 0, 0, 0, 1, 0, 0, 0, 1]));
      fixedPart(id, new SpindleBearing(), pose, "outboardSleeve");
      mountContacts.push({parent: "spindle/arbor", child: id});
    }
    fixedPart("outboardInnerSpacer", new OutboardBearingPart(new ComponentValues().setToken("role", "innerSpacer")),
      supportPose, "spindle/arbor");
    fixedPart("outboardLocknut", new machinekit.standard.FinePitchNut(),
      F.compose(supportPose, F.translation(0, 0, OutboardSupport.PAIR_TO + OutboardSupport.SPACER_LENGTH)), "spindle/arbor");
    group("Ungrouped");
      externalPart(
        "coolantPort",
        new HobberCoolantPort(),
        F.compose(HobberFrame.partAt(0, centre, heights.hobCentreZ), HobberCoolantPort.frameFor()),
        "yStage", "carriage",
        F.alongY(-64.35, centre + 45, HobberFrame.PART_DATUM_Z + 435, 0, 1, 0)
      );
    exposeConnector("hobCentre","hob","centre");

  }
  public static function outboardMembers():Array<String> return ["outboardCarrier", "outboard", "outboardSleeve", "outboardGland", "outboardCap", "outboardBearingFront", "outboardBearingRear"];


}
