import machinekit.power.BatteryPack;
import machinekit.power.IsolatedDcConverter;
import machinekit.assembly.PosedParts;
import cadkit.modeling.Part;
import materia.project.SceneArtifact;
import cadbridge.AssemblySimulationBridge;
import cadbridge.AssemblyPhysicalPartView;
class MobileWelderChecks {
  static function main():Void run();
  public static function run(checkDefault:Bool = true):Void {
    if (checkDefault) MobileBasePreview.MobileBaseChecks.run();
    var defaultBase = new MobileBase(new RobotArm(false));
    if (!defaultBase.hasPort("compressedAir")) throw "Suction tool services must still pass through";
    var welding = new RobotArm(false, new ArmWeldingTool());
    welding.replaceComponent("powerSupply", new IsolatedDcConverter(48, 48, 30, 6));
    var robot = new MobileBase(welding, 48, {length: 2400, width: 1000, payloadX: 700,
      batteryX: -600, battery: new BatteryPack(48, 25000, 1000, 600, 180, 230, 4)});
    if (robot.hasPort("compressedAir") || !robot.hasPort("torchPower")) throw "Base must expose the actual tool services";
    robot.connectPorts("arm-pack", "battery", "power3", "arm/powerSupply", "dc");
    if (robot.upstream("arm/driver1", "power").port.instanceId != "battery") throw "Arm power must trace to the floating pack";
    if (robot.length != 2400 || robot.width != 1000) throw "Payload platform dimensions were lost";
    if (Math.abs(robot.trackWidth() - defaultBase.trackWidth()) > 1e-6) throw "Changing deck dimensions must preserve the drive geometry";
    var carrier = new MobileWeldingRobot();
    var diagnostics = carrier.check();
    diagnostics.throwIfErrors();
    if (carrier.upstream("source", "mains").port.instanceId != "battery") throw "Welding source is not battery fed";
    if (carrier.upstreamChain("source", "mains").indexOf("inverter/mains") < 0) throw "Welder bypasses inverter";
    var parents = carrier.mateParents();
    for (name in ["source", "inverter", "cylinder"]) if (parents.get(name) != "deck") throw 'Equipment $name is not carried';
    var poses = carrier.solvedPoses();
    var parts = new PosedParts();
    var placed:Array<{id:String, part:Part, box:machinekit.assembly.PosedParts.PartBox}> = [];
    try {
      for (member in carrier.components()) {
        var part = parts.posed(member.component, poses.get(member.id));
        placed.push({id: member.id, part: part, box: PosedParts.boxOf(part)});
      }
      for (equipment in placed) if (["battery", "source", "inverter", "cylinder", "computerSupply"].indexOf(equipment.id) >= 0) {
        if (equipment.box.minX < -carrier.length / 2 - 1e-6 || equipment.box.maxX > carrier.length / 2 + 1e-6 ||
            equipment.box.minY < -carrier.width / 2 - 1e-6 || equipment.box.maxY > carrier.width / 2 + 1e-6)
          throw 'Equipment ${equipment.id} overhangs the platform footprint';
        for (other in placed) if (other.id != equipment.id && PosedParts.commonVolume(equipment.part, equipment.box, other.part, other.box) > 1)
          throw 'Equipment ${equipment.id} intersects ${other.id}';
      }
    } catch (error:Dynamic) {
      for (entry in placed) entry.part.close();
      parts.close();
      throw error;
    }
    for (entry in placed) entry.part.close();
    parts.close();
    var mass = carrier.massProperties();
    if (mass.unaccounted.length != 0 || mass.mass < 230 + 55 + 45 + 75) throw "Carried equipment mass is not accounted for";
    Sys.println('Carried mass ${Math.round(mass.mass * 10) / 10} kg; equipment footprint and clearance passed');
    var scene = SceneArtifact.decode(MobileWelderPreview.cell());
    var drive:materia.project.SceneArtifact.SceneArtifactMobileBase = cast scene.mobileBase;
    if (scene.mobileBase == null || scene.robotTools == null || scene.robotTools.length != 1 ||
        scene.robotTools[0].kind != "torch" || scene.mobileBase.footprintLength != carrier.length / 1000)
      throw "The mobile scene lost its CAD drive or torch";
    var definition:materia.assembly.AssemblyDefinition = cast scene.assemblyDefinition;
    var count = materia.assembly.AssemblyDefinitionFlattener.flatten(definition).occurrences.length;
    var physical = AssemblyPhysicalPartView.fromSceneArtifact(scene);
    var converted = AssemblySimulationBridge.toRobotModel(definition, physical, scene.assemblyState, null, null, scene.mobileBase);
    if (converted.partLinks.exists("work/basePlate") || !converted.partLinks.exists("robot/basePlate"))
      throw "Stationary welded surroundings must stay outside the mobile robot model";
    if (materia.assembly.AssemblyDefinitionFlattener.flatten(definition).occurrences.length != count)
      throw "Robot selection must not mutate the source CAD assembly";
    var rejected = false;
    try AssemblySimulationBridge.toRobotModel(definition, physical, scene.assemblyState, ["work/upright"], null, scene.mobileBase)
      catch (_:Dynamic) rejected = true;
    if (!rejected) throw "A genuinely dynamic part must still reject a fixed joint";
    var wrong:materia.project.SceneArtifact.SceneArtifactMobileBase = cast scene.mobileBase;
    var wanted = wrong.robot;
    wrong.robot = "robot/arm";
    rejected = false;
    try AssemblySimulationBridge.toRobotModel(definition, physical, scene.assemblyState, null, null, wrong)
      catch (_:Dynamic) rejected = true;
    wrong.robot = wanted;
    if (!rejected) throw "A robot selection must reject joints crossing its boundary";
    var torch:materia.project.SceneArtifact.SceneArtifactTorch = cast scene.robotTools[0].torch;
    if (torch.groundedWork.indexOf("work/basePlate") < 0 ||
        torch.groundedWork.indexOf("robot/basePlate") >= 0)
      throw "The work clamp must ground the work, not the floating chassis";
    if (drive.maxLinearAcceleration >= MobileBase.MAX_LINEAR_ACCELERATION)
      throw "Heavy payload acceleration must follow the available drive effort and carried mass";
    Sys.println('Mobile scene: ${Math.round(drive.maxLinearAcceleration * 1000) / 1000} m/s2, independently grounded work');
    Sys.println('Mobile payload layout and tool service checks passed; welding platform ${carrier.length} x ${carrier.width} mm');
  }
}
