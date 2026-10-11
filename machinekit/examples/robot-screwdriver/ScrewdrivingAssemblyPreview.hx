import machinekit.screwdriving.ScrewPresenter;
import machinekit.screwdriving.ScrewdrivingMission;
import machinekit.screwdriving.ScrewdrivingScene;
import machinekit.assembly.AssemblyPreview;
import machinekit.assembly.MachineAssembly;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyDefinitionFlattener;
import cadkit.modeling.AssemblyState;
import materia.project.SceneArtifact;
import materia.project.SceneArtifact.SceneArtifactData;
import materia.project.SceneScrewDefinition.SceneScrew;
import machinekit.robot.RobotScene;
import RobotArm.ArmBlock;
import haxe.io.Bytes;

/** Every flange fastening has its own supported supply and one physically separate replacement. */
class ScrewdrivingAssemblyPreview {
	public static function scene():SceneArtifactData {
		var cell = new ScrewdrivingCell(new machinekit.screwdriving.ScrewdriverBit(HexSocket(5),45,6,0.05),0,
			machinekit.standard.DeepGrooveBearing.metric("6003"));
		var positions = [[250.0,-450.0],[180.0,-380.0],[250.0,-380.0],[110.0,-450.0],[110.0,-380.0],[320.0,-450.0],[320.0,-380.0]];
		var supplyNames = ["presenter","supply2","supply3","supply4"];
		var spareNames = ["reserve1","reserve2","reserve3","reserve4"];
		var additional = supplyNames.slice(1).concat(spareNames);
		for (i in 0...additional.length) {
			var name = additional[i];
			cell.include(name,new ScrewPresenter(cell.presenter.screw),AssemblyFrames.translation(positions[i][0],positions[i][1],RobotArm.TABLE_TOP));
			cell.exposePort(name+"Control",name+"/escapement","control");
		}
		cell.include("reject",new ScrewRejectBin(),AssemblyFrames.translation(-180,-450,RobotArm.TABLE_TOP));
		var initial = cell.work.fastenings().require();
		for (fastening in initial) {var qualification = cell.work.join(fastening.screw.member,null,{minimum:0.2,maximum:1.0});}
		ScrewdrivingChecks.check(cell);
		var found = cell.work.fastenings(), ordered = ScrewdrivingMission.order(found.require());
		var scene = AssemblyPreview.scene(cell,"screwdriving-assembly");
		scene.assemblyState = ScrewdrivingPreview.state(cell,"screwdriving-assembly").record();
		scene.robotTools = RobotScene.robotTools(cell.arm.tool,"arm/tool");
		var definition = scene.assemblyDefinition;
		if (definition == null) throw "Whole screw cell lost its assembly";
		var flat = AssemblyDefinitionFlattener.flatten(definition), state = new AssemblyState(flat,scene.assemblyState);
		ScrewdrivingExecution.align(cell,state,"presenter/screw"); scene.assemblyState = state.record();
		var jobs:Array<SceneScrew> = [], staging:Array<ScrewdrivingExecution.ScrewStaging> = [];
		for (index in 0...ordered.length) {
			var fastening = ordered[index], supply = supplyNames[index], reserve = spareNames[index];
			var job = ScrewdrivingScene.fromFastening(cell.work,fastening,"work/",scene,supply+"/nest",supply+"/screw",supply+"/part-present",1,
				{runDownSpeed:20,finalSpeed:2,runDownTorqueLimit:1,finalTorqueLimit:19.5,maximumRunDownAngle:180,minimumStroke:2,maximumStroke:18},
				{leadInMm:0.5,maximumAxisAngle:0.1,prevailingTorque:0.2,stiffnessNmPerRad:40,stripTorque:22});
			var pickup = AssemblyFrames.compose(AssemblyFrames.inverse(state.worldPose(reserve+"/nest")),
				AssemblyFrames.compose(state.worldPose(reserve+"/screw"),cell.presenter.screw.driveFrame()));
			pickup.x *= scene.metresPerUnit; pickup.y *= scene.metresPerUnit; pickup.z *= scene.metresPerUnit;
			var spare = 'replacement/screw${index+1}';
			job.recovery = {replacement:{screw:spare,presenter:reserve+"/nest",presentSensor:reserve+"/part-present",pickup:pickup},
				rejectMember:"reject/floor",reject:{x:0,y:0,z:0.1,qx:1,qy:0,qz:0,qw:0}};
			jobs.push(job);
			staging.push({body:job.screw,template:supply+"/screw",like:job.screw});
			staging.push({body:spare,template:reserve+"/screw",like:job.screw});
		}
		var mission = ScrewdrivingMission.generate(found,(fastening) -> {
			var candidates = [for (job in jobs) if (job.fastening == fastening.name()) job];
			if (candidates.length != 1) throw "Whole screw cell lost a qualified connection";
			return candidates[0];
		});
		ScrewdrivingExecution.stage(scene,staging);
		scene.mission = {steps:mission.require()};
		var controllers = scene.controllers;
		if (controllers != null) {
			var owners = [for (controller in controllers) if (controller.kind == "robot" && controller.joints.indexOf("arm/j1") >= 0) controller];
			if (owners.length != 1) throw "Whole screw cell lost its arm controller";
			scene.mission.controllerId = owners[0].id;
		}
		return scene;
	}
	public static function assembly():Bytes return SceneArtifact.encode(scene());
}
function main():Void {
	var decoded = SceneArtifact.decode(ScrewdrivingAssemblyPreview.assembly());
	if (decoded.mission == null || decoded.mission.steps.length != 4) throw "Whole screw mission lost its four fastenings";
	trace("Screwdriving assembly CAD mission passed");
}

/** Rigid open bin; its named floor carries all four walls. */
private class ScrewRejectBin extends MachineAssembly {
	public function new() {
		super();
		addComponent("floor",new ArmBlock(80,80,4,"steel","Reject bin floor"));
		for (side in [-1,1]) {
			var label = side < 0 ? "Left" : "Right";
			for (axis in ["x","y"]) {
				var name = axis+label;
				addComponent(name,new ArmBlock(axis == "x" ? 4 : 72,axis == "x" ? 80 : 4,40,"steel","Reject bin wall"));
				addMemberConnector("floor",name,AssemblyFrames.translation(axis == "x" ? side*38 : 0,axis == "y" ? side*38 : 0,4));
				addMate(name+"-fixed","fixed","floor",name,name,"base");
			}
		}
	}
}
