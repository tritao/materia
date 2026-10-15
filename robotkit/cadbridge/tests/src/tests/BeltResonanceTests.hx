package tests;

import cadbridge.BeltResonanceCalibration;
import ResonanceTests;
import cadbridge.AssemblySimulationBridge;
import cadbridge.AssemblyPhysicalPartView;
import cadkit.modeling.AssemblyModel;
import machinekit.assembly.AssemblyPreview;
import materia.project.SceneArtifact;
import robotkit.model.DriveLoads;

class BeltResonanceTests {
  static function check(ok:Bool,message:String):Void { if (!ok) throw message; }
  public static function main():Void { ResonanceTests.run(); run(); }
  public static function run():Void {
    check(machinekit.motion.ServoMotor.basisFromToken("measured") == machinekit.transmission.ValueBasis.Measured,
      "shared engineering basis retains measured token support");
    var assembly=new CoreXyPrinter();
    var scene=AssemblyPreview.scene(assembly,"printer");
    var robot=AssemblySimulationBridge.toRobotModel(scene.assemblyDefinition,AssemblyPhysicalPartView.fromSceneArtifact(scene),scene.assemblyState).model;
    var pose:Map<String,Float> = ["x"=>-0.04,"y"=>0.03];
    function position(id:String):Float { var value=pose.get(id); if (value == null) throw "Missing calibration pose"; return value; }
    var before=DriveLoads.atPosition(robot,position,null,["x"])[0];
    var frequency=Math.sqrt(before.stiffness/before.mass)/(2*Math.PI)*1.12;
    var measurement=ResonanceTests.measurement(frequency,0.085);
    var partial=false;
    try new BeltResonanceCalibration(assembly,robot,["beltA"],measurement,pose) catch (_:Dynamic) partial=true;
    check(partial,"one CoreXY belt cannot be inferred from the common carriage mode");
    var proposal=new BeltResonanceCalibration(assembly,robot,["beltA","beltB"],measurement,pose);
    check(Math.abs(proposal.rigidityFactor-1.12*1.12)<1e-5,"EA follows frequency squared at the measured physical pose");
    check(robot.couplings[0].calibration == null,"fitting leaves the live model unchanged");
    var exposed=proposal.values, exposedBelt=exposed.get("beltA");
    if (exposedBelt == null) throw "Missing belt proposal";
    exposedBelt.rigidityPerWidth=1;
    var fresh=proposal.values.get("beltA");
    if (fresh == null) throw "Missing belt proposal";
    check(fresh.rigidityPerWidth>100,"proposal access cannot mutate its frozen update");
    var invalid=proposal.values, invalidBelt=invalid.get("beltB");
    if (invalidBelt == null) throw "Missing belt proposal";
    invalidBelt.dampingRatio=0.6;
    var sourceBefore=haxeon.wire.JsonWire.encode(assembly.describe()), invalidRefused=false;
    try assembly.calibrateBelts(invalid) catch (_:Dynamic) invalidRefused=true;
    check(invalidRefused && haxeon.wire.JsonWire.encode(assembly.describe())==sourceBefore,
      "invalid group data leaves every source transmission unchanged");
    var overlay=proposal.encode();
    var replay=new CoreXyPrinter();
    BeltResonanceCalibration.applySaved(replay,robot,overlay);
    check(replay.describe().machine.transmissions.filter(t->t.calibration!=null).length==4,"portable overlay updates source-generated printers after reload");
    var wrong=robotkit.model.RobotModelCodec.decode(robotkit.model.RobotModelCodec.encode(robot));
    wrong.links[1].mass=wrong.links[1].mass+0.1;
    var mismatch=false;
    try BeltResonanceCalibration.applySaved(new CoreXyPrinter(),wrong,overlay) catch (_:Dynamic) mismatch=true;
    check(mismatch,"saved measurement refuses a changed physical mass/model");
    proposal.apply();
    var calibratedScene=AssemblyPreview.scene(assembly,"printer");
    var measured=calibratedScene.assemblyDefinition;
    if (measured == null) throw "Calibrated printer lost its assembly";
    var laws=[for (c in measured.couplings) if (c.calibration!=null) c];
    check(laws.length==4,"both CoreXY belts update both axes together");
    for (c in laws) {
      var calibration=c.calibration;
      if (calibration == null) throw "Missing measured calibration";
      check(calibration.dampingRatio==measurement.dampingRatio && c.stiffnessLaw!=null,"measured damping retains geometry law");
      for (a in c.assumptions) check(a.quantity!="stiffness" && a.quantity!="dampingRatio","measured quantities are no longer assumptions");
    }
    var wire=materia.assembly.AssemblyDefinitionCodec.decode(materia.assembly.AssemblyDefinitionCodec.encode(measured));
    check(wire.couplings.filter(c->c.calibration!=null).length==4,"calibration evidence survives assembly wire round trip");
    var saved=assembly.describe();
    check(saved.machine.transmissions.filter(t->t.calibration!=null).length==4,"source description retains measured properties for rebuilding");
    var physical=AssemblyPhysicalPartView.fromSceneArtifact(calibratedScene);
    var actual=AssemblySimulationBridge.toRobotModel(wire,physical,calibratedScene.assemblyState).model;
    var copied=robotkit.model.RobotModelCodec.decode(robotkit.model.RobotModelCodec.encode(actual));
    check(copied.couplings.filter(c->c.calibration!=null).length==4,"RobotKit serialization preserves measurement provenance");
    var after=DriveLoads.atPosition(actual,position,null,["x"])[0];
    check(Math.abs(Math.sqrt(after.stiffness/after.mass)/(2*Math.PI)/measurement.frequencyHz-1)<1e-7,"rebuilt assembly matches its measured mode");
    var parent=new machinekit.assembly.MachineAssembly(); parent.include("printer",assembly);
    var nested=parent.definition(), nestedLaws=[for (c in nested.couplings) if (c.calibration!=null) c];
    check(nestedLaws.length==4,"nested printer keeps all measured motion relations");
    for (c in nestedLaws) {
      var evidence=c.calibration; if (evidence == null) throw "Missing nested evidence";
      check(evidence.axis == "printer/x" && evidence.pose[0].joint == "printer/x","measurement coordinates follow subassembly scope");
    }
    var beforeOverride=haxeon.wire.JsonWire.encode(assembly.describe()), rejected=false;
    try assembly.setTransmissionOverrides(laws[0].id,State(1000)) catch (_:Dynamic) rejected=true;
    check(rejected && haxeon.wire.JsonWire.encode(assembly.describe())==beforeOverride,
      "competing stated stiffness is refused without partially changing the measured source");
    var stale=false;
    try proposal.apply() catch (_:Dynamic) stale=true;
    check(stale,"a proposal cannot be applied twice or after a source edit");
    assembly.clearBeltCalibration("beltA"); assembly.clearBeltCalibration("beltB");
    var clearedScene=AssemblyPreview.scene(assembly,"printer");
    var cleared=AssemblySimulationBridge.toRobotModel(clearedScene.assemblyDefinition,
      AssemblyPhysicalPartView.fromSceneArtifact(clearedScene),clearedScene.assemblyState).model;
    var restored=DriveLoads.atPosition(cleared,position,null,["x"])[0];
    check(cleared.couplings.filter(c->c.calibration!=null).length==0 &&
      Math.abs(restored.stiffness/before.stiffness-1)<1e-10,"clearing calibration restores authored properties without stale measured metadata");
    Sys.println('CoreXY measured calibration passed: ${proposal.beforeFrequency} -> ${proposal.afterFrequency} Hz, EA x${proposal.rigidityFactor}');
  }
}
