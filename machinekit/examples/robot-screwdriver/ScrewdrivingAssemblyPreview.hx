import machinekit.screwdriving.ScrewPresenter;
import machinekit.screwdriving.ScrewdrivingMission;
import machinekit.screwdriving.ScrewdrivingScene;
import machinekit.assembly.AssemblyPreview;
import machinekit.assembly.MachineAssembly;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyDefinitionFlattener;
import cadkit.modeling.AssemblyState;
import cadkit.modeling.Part;
import cadkit.modeling.Location;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import materia.project.SceneArtifact;
import materia.project.SceneArtifact.SceneArtifactData;
import materia.project.SceneScrewDefinition.SceneScrew;
import machinekit.robot.RobotScene;
import RobotArm.ArmBlock;
import haxe.io.Bytes;

/** CAD design and qualification before free-body staging; derived afresh for each rebuild. */
typedef ScrewdrivingDesign = {
	var cell:ScrewdrivingCell;
	var scene:SceneArtifactData;
	var state:AssemblyState;
	var found:machinekit.screwdriving.Fastenings.FasteningsResult;
	var jobs:Array<SceneScrew>;
	var staging:Array<ScrewdrivingExecution.ScrewStaging>;
}

/** Every flange fastening has its own supported supply and one physically separate replacement. */
class ScrewdrivingAssemblyPreview {
	public static function design(bowlFed:Bool = false, blowFed:Bool = false, registered:Bool = false,
		forceSearch:Bool = false):ScrewdrivingDesign {
		var bitLength = forceSearch ? 85 : 45;
		var cell = new ScrewdrivingCell(new machinekit.screwdriving.ScrewdriverBit(HexSocket(5),bitLength,6,0.05),0,
			machinekit.standard.DeepGrooveBearing.metric("6003"),bowlFed,blowFed,registered ? 80 : 10,registered ? 80 : 15,
			registered ? 300 : 180, -450, registered);
		var positions = registered
			? [[-300.0,-360.0],[-220.0,-360.0],[-140.0,-360.0],[-60.0,-360.0],[60.0,-360.0],[140.0,-360.0],[220.0,-360.0]]
			: [[250.0,-450.0],[180.0,-380.0],[250.0,-380.0],[100.0,-450.0],[110.0,-380.0],[320.0,-450.0],[320.0,-380.0]];
		var reservePositions = registered
			? [positions[3],positions[4],positions[5],positions[6]]
			: [[320.0,-520.0],[320.0,-450.0],[320.0,-380.0],[320.0,-340.0]];
		var supplyNames = bowlFed || blowFed ? ["presenter","presenter","presenter","presenter"] : ["presenter","supply2","supply3","supply4"];
		var spareNames = ["reserve1","reserve2","reserve3","reserve4"];
		var additional = bowlFed || blowFed ? spareNames.copy() : supplyNames.slice(1).concat(spareNames);
		if (bowlFed || blowFed) {
			cell.include("feeder",new machinekit.screwdriving.ScrewBowlFeeder(cell.presenter.screw),AssemblyFrames.translation(180,-650,RobotArm.TABLE_TOP));
			cell.exposePort("feederControl","feeder/drive","control");
		}
		if (blowFed) {
			var tool:machinekit.screwdriving.BlowFeedTool = cast cell.tool;
			cell.addComponent("blowMeter",new machinekit.screwdriving.BlowFeedMeter(tool.outerDiameter));
			cell.addMate("blow-meter-track","fixed","feeder/trackLeft","outlet","blowMeter","inlet");
			cell.connectPorts("screw-delivery-tube","blowMeter","screw","arm/tool/feedTube","input");
			cell.exposePort("blowAir","blowMeter","air"); cell.exposePort("blowControl","blowMeter","control");
		}
		for (i in 0...additional.length) {
			var name = additional[i];
			var reserve = spareNames.indexOf(name);
			var position = reserve < 0 ? positions[i] : reservePositions[reserve];
			cell.include(name,new ScrewPresenter(cell.presenter.screw),AssemblyFrames.translation(position[0],position[1],RobotArm.TABLE_TOP));
			cell.exposePort(name+"Control",name+"/escapement","control");
		}
		// Keep the reject drop clear of reserve1, whose pickup nest occupies the
		// normal cell's former -180 mm bin station.
		cell.include("reject",new ScrewRejectBin(),AssemblyFrames.translation(-300,-450,RobotArm.TABLE_TOP));
		var initial = cell.work.fastenings().require();
		for (fastening in initial) {var qualification = cell.work.join(fastening.screw.member,null,{minimum:0.2,maximum:1.0});}
		ScrewdrivingChecks.check(cell);
		var found = cell.work.fastenings(), ordered = ScrewdrivingMission.order(found.require());
		var scene = AssemblyPreview.scene(cell,"screwdriving-assembly");
		scene.assemblyState = ScrewdrivingPreview.state(cell,"screwdriving-assembly").record();
		scene.robotTools = RobotScene.robotTools(cell.arm.tool,"arm/tool");
		materia.project.SceneArtifact.assignToolControllers(scene);
		scene.robotSensors = RobotScene.robotSensors(cell,"");
		var definition = scene.assemblyDefinition;
		if (definition == null) throw "Whole screw cell lost its assembly";
		var flat = AssemblyDefinitionFlattener.flatten(definition), state = new AssemblyState(flat,scene.assemblyState);
		if (!registered && !bowlFed && !blowFed) for (name in spareNames)
			Sys.println('Reserve diagnostic $name ${haxe.Json.stringify(state.worldPose(name+"/screw"))}');
		ScrewdrivingExecution.align(cell,state,blowFed ? null : "presenter/screw"); scene.assemblyState = state.record();
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
			if (blowFed) {job.delivery = BlowFeed("blow"); job.presenter = null; job.presentSensor = null; job.pickup = null;}
			jobs.push(job);
			staging.push({body:job.screw,template:supply+"/screw",like:job.screw});
			staging.push({body:spare,template:reserve+"/screw",like:job.screw});
		}
		return {cell:cell,scene:scene,state:state,found:found,jobs:jobs,staging:staging};
	}

	public static function scene(twoStage:Bool = false, bowlFed:Bool = false, blowFed:Bool = false,
		registered:Bool = false, forceSearch:Bool = false):SceneArtifactData {
		var design = design(bowlFed,blowFed,registered,forceSearch), cell = design.cell, scene = design.scene, state = design.state;
		var jobs = design.jobs, staging = design.staging, found = design.found;
		var tools = [for (tool in scene.robotTools) if (tool.screwdriver != null) tool];
		if (tools.length != 1) throw "Whole screw access lost its feed capability";
		var parts = new machinekit.assembly.PosedParts();
		var access = new ScrewdrivingAccess(cell,state,jobs,staging,parts,scene.metresPerUnit,tools[0]);
		var mission:machinekit.screwdriving.ScrewdrivingMission;
		try mission = ScrewdrivingMission.generate(found,(fastening) -> {
			var candidates = [for (job in jobs) if (job.fastening == fastening.name()) job];
			if (candidates.length != 1) throw "Whole screw cell lost a qualified connection";
			return candidates[0];
		},access,twoStage ? (_) -> 2 : null) catch (error:Dynamic) {parts.close(); throw error;}
		parts.close();
		ScrewdrivingExecution.stage(scene,staging);
		scene.missions = [{steps:mission.require()}];
		if (bowlFed || blowFed) {
			var metres = scene.metresPerUnit;
			var deposit = AssemblyFrames.compose(AssemblyFrames.inverse(state.worldPose("presenter/nest")),state.worldPose("presenter/screw"));
			deposit.x *= metres; deposit.y *= metres; deposit.z *= metres;
			if (bowlFed) scene.missions[0].screwFeeders = [{id:"bowl",presenter:"presenter/nest",presentSensor:"presenter/part-present",
				inventory:[for (job in jobs) job.screw],deposit:deposit,bufferCapacity:2,trackDelay:0.2,deliveryDelay:0.3,deliveryTimeout:3}];
			if (blowFed) {
				var tool:machinekit.screwdriving.BlowFeedTool = cast cell.tool;
				scene.missions[0].blowFeeds = [{id:"blow",receiver:"arm/tool/receiver",socketConnector:"receipt",
					receiptSensor:"arm/tool/screw-receipt",inventory:[for (job in jobs) job.screw],bufferCapacity:2,
					trackDelay:0.2,deliveryDelay:0.4,deliveryTimeout:3,airPulse:0.2,airPressureBar:4,
					tube:"arm/tool/feedTube",externalTubeLengthMm:tool.externalTubeLength,minimumBendRadiusMm:tool.minimumBendRadius,
					receivingAngularSpeed:0.01,receivingFeedSpeed:0.001}];
			}

			var flat = scene.assemblyDefinition;
			if (flat == null) throw "Fed cell lost its independent supplies";
			var execution = new AssemblyState(flat,scene.assemblyState);
			for (i in 0...jobs.length) {
				var stored = state.worldPose("presenter/screw");
				stored.x = 180+(i-1.5)*(cell.presenter.screw.spec.headDiameter+5);
				stored.y = -650; stored.z = RobotArm.TABLE_TOP+15+4+cell.presenter.screw.length;
				execution.setRootPose(jobs[i].screw,stored);
			}
			scene.assemblyState = execution.record();
		}
		if (registered) {
			var stagedDefinition = scene.assemblyDefinition;
			if (stagedDefinition == null) throw "Registered screw cell lost its staged assembly";
			var parked = new AssemblyState(AssemblyDefinitionFlattener.flatten(stagedDefinition),scene.assemblyState);
			var ready = cell.readyPose();
			for (index in 0...ready.length) parked.setJoint('arm/j${index+1}',ready[index]);
			scene.assemblyState = parked.record();
			var probeTools = [for (tool in scene.robotTools) if (tool.kind == "screwdriver") tool];
			if (probeTools.length != 1 || scene.missions == null) throw "Registered screw cell needs its one mounted screwdriver and mission";
			scene.missions[0].steps.insert(0, {kind: "findWork", at: probeTools[0].contact,
				contactWork: contactWork(cell, state, scene)});
		}
		var controllers = scene.controllers;
		if (controllers != null) {
			var owners = [for (controller in controllers) if (controller.kind == "robot" && controller.joints.indexOf("arm/j1") >= 0) controller];
			if (owners.length != 1) throw "Whole screw cell lost its arm controller";
			scene.missions[0].controllerId = owners[0].id;
		}
		if (forceSearch) {
			if (scene.robotTools == null || scene.robotSensors == null || scene.missions == null)
				throw "Force-search example lost its screwdriver, sensors or mission";
			var drivers = [for (tool in scene.robotTools) if (tool.kind == "screwdriver") tool];
			if (drivers.length != 1) throw "Force-search example needs its unique screwdriver";
			var contact = drivers[0].contact;
			var sensorId = "arm/tool/force-torque";
			scene.robotSensors.push({kind: "force_torque", id: sensorId, mount: contact});
			scene.missions[0].screwEntrySearch = forceSearchProfile(sensorId);
		}
		return scene;
	}
	public static function assembly():Bytes return SceneArtifact.encode(scene());
	public static function staged():Bytes return SceneArtifact.encode(scene(true));
	public static function fed():Bytes return SceneArtifact.encode(scene(false,true));
	public static function blown():Bytes return SceneArtifact.encode(scene(false,false,true));
	public static function registered():Bytes return SceneArtifact.encode(scene(false,false,false,true));
	public static function registeredForceSearch():Bytes return SceneArtifact.encode(scene(false,false,false,true,true));
	public static function forceSearch():Bytes return SceneArtifact.encode(scene(false,false,false,false,true));

	/**
	 * Simulation-only commissioning data for the force-search example, not a hardware safety recipe. The numbers are
	 * the ones `ScrewEntryPlantTests` exercises: a 5 N contact force, a 2 mm/s guarded descent and a 1.6 mm spiral with
	 * 0.4 mm pitch (below the clearance bore's capture diameter) at 2 mm/s. While the arm searches, the tool's feed
	 * slide is its 2 N/mm spring, so each axis is damped above what it pushes against times the 20 ms
	 * sample-and-step latency: 400 N s/m along the axis (the spring), 4000 N s/m sideways (rigid contact), each with
	 * a 20 ms relaxation. The 0.3 mm entry advance marks a drop into a bore, or the screw past the CAD entry plane
	 * into the tapped hole.
	 */
	static function forceSearchProfile(sensor:String):materia.project.SceneArtifact.SceneArtifactScrewEntrySearch {
		var axial = 400.0, lateral = 4000.0, relaxationSeconds = 0.02;
		return {sensor:sensor,maximumAgeSeconds:0.02,contactForceN:5,contactThresholdN:2,confirmationSamples:2,
			virtualMass:[lateral*relaxationSeconds,lateral*relaxationSeconds,axial*relaxationSeconds,1,1,1],
			damping:[lateral,lateral,axial,20,20,20],
			maximumLinearSpeed:0.004,maximumAngularSpeed:0.0002,maximumStepSeconds:0.01,approachSpeed:0.002,
			maximumForceN:25,maximumTorqueNm:2,maximumTravelMeters:0.045,maximumTravelRadians:0.05,maximumRunSeconds:120,
			regionHalfUMeters:0.0021,regionHalfVMeters:0.0021,spiralRadiusMeters:0.0016,spiralPitchMeters:0.0004,
			spiralSpeed:0.002,spiralTrackingGainPerSecond:10,spiralTrackingToleranceMeters:0.00015,
			entryAdvanceMeters:0.0003,stallSeconds:5};
	}

	/** The side stops move with the plate and provide exposed CAD reference faces. */
	static function contactWork(cell:ScrewdrivingCell, state:AssemblyState, scene:SceneArtifactData):materia.project.SceneContactRegistration.SceneContactWork {
		var frame = "work/plate";
		var parts:Map<String,Part> = new Map();
		parts.set(frame, cell.work.plate.geometry(machinekit.component.ComponentDetail.Envelope));
		var inverse = AssemblyFrames.inverse(state.worldPose(frame));
		for (name in ["fixtureLeft","fixtureRight"]) {
			var matches = [for (item in cell.components()) if (item.id == name) item];
			if (matches.length != 1) {
				for (part in parts) part.close();
				throw 'Touch registration needs one CAD reference stop named "$name"';
			}
			var relative = AssemblyFrames.compose(inverse,state.worldPose(name));
			var x = AssemblyFrames.transformVector(relative,1,0,0), z = AssemblyFrames.transformVector(relative,0,0,1);
			var local = matches[0].component.geometry(machinekit.component.ComponentDetail.Envelope);
			var placed:Part;
			try placed = local.placed(new Location(new Plane(new Vector(relative.x,relative.y,relative.z),
				new Vector(x.x,x.y,x.z),new Vector(z.x,z.y,z.z)))) catch (error:Dynamic) {
				local.close(); for (part in parts) part.close(); throw error;
			}
			local.close(); parts.set(name,placed);
		}
		var faces:Array<materia.project.SceneContactRegistration.SceneContactFace>;
		try faces = machinekit.welding.WeldProbeGeometry.of(parts, ["work/plate","fixtureLeft","fixtureRight"]).contactFaces(scene.metresPerUnit)
		catch (error:Dynamic) { for (part in parts) part.close(); throw error; }
		for (part in parts) part.close();
		var pose = state.worldPose(frame), scale = scene.metresPerUnit;
		var measurementError = 0.00001;
		return {frame: frame, nominal: {x: pose.x * scale, y: pose.y * scale, z: pose.z * scale,
			qx: pose.qx, qy: pose.qy, qz: pose.qz, qw: pose.qw}, translation: [0.02, 0.02, measurementError],
			rotation: [0, 0, 2 * Math.PI / 180], measurementError: measurementError, calibratedError: 0.0,
			contactOffset: 0.0, faces: faces};
	}
}
function main():Void {
	var decoded = SceneArtifact.decode(ScrewdrivingAssemblyPreview.assembly());
	if (decoded.missions == null || decoded.missions[0].steps.length != 4) throw "Whole screw mission lost its four fastenings";
	var registered = SceneArtifact.decode(ScrewdrivingAssemblyPreview.registered());
	if (registered.missions == null || registered.missions[0].steps.length != 5 || registered.missions[0].steps[0].kind != "findWork" ||
		registered.missions[0].steps[0].contactWork == null || registered.robotSensors == null ||
		[for (sensor in registered.robotSensors) if (sensor.kind == "tool_contact") sensor].length != 1)
		throw "Touch-registration project lost its CAD references, typed sensor or four named fastenings";
	var forceSearch = SceneArtifact.decode(ScrewdrivingAssemblyPreview.forceSearch());
	if (forceSearch.missions == null || forceSearch.missions[0].screwEntrySearch == null || forceSearch.robotSensors == null ||
		[for (sensor in forceSearch.robotSensors) if (sensor.id == "arm/tool/force-torque" && sensor.kind == "force_torque") sensor].length != 1)
		throw "Force-search project lost its commissioned sensor profile or CAD fastenings";
	ScrewdrivingAccessTests.run();
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
