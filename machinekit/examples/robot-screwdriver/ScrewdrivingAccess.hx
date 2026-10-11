import cadkit.modeling.AssemblyState;
import machinekit.assembly.PosedParts;
import machinekit.screwdriving.Fastening;
import machinekit.screwdriving.ScrewdrivingMission.ScrewAccess;
import machinekit.standard.ScrewFastener;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.project.SceneScrewDefinition.SceneScrew;
import materia.project.SceneArtifact.SceneArtifactRobotTool;

private typedef AccessPose = {
	var label:String;
	var member:String;
	var frame:AssemblyFrame;
	var feed:Float;
	var held:Bool;
	var threaded:Bool;
	var template:String;
};

/** Checks actual CAD arm/tool solids and identified supplies at qualified contact and retreat poses. */
class ScrewdrivingAccess implements ScrewAccess {
	final cell:ScrewdrivingCell;
	final state:AssemblyState;
	final jobs:Array<SceneScrew>;
	final supplies:Array<ScrewdrivingExecution.ScrewStaging>;
	final parts:PosedParts;
	final metres:Float;
	final feed:String;
	final tcpMember:String;
	final tcpConnector:String;
	final joints:Array<String>;
	final limits:Array<{lower:Float,upper:Float}>;
	final moving:Array<String>;
	final fixed:Array<String>;
	final carrierGroup:Int;
	final bodyGroups:Array<Int>;
	final allowed:Map<String,Bool> = [];

	public function new(cell:ScrewdrivingCell, state:AssemblyState, jobs:Array<SceneScrew>,
			supplies:Array<ScrewdrivingExecution.ScrewStaging>, parts:PosedParts, metres:Float, tool:SceneArtifactRobotTool) {
		this.cell = cell; this.state = state; this.jobs = jobs; this.supplies = supplies;
		this.parts = parts; this.metres = metres;
		var capacity = tool.screwdriver;
		if (capacity == null) throw "Access needs a declared screwdriver capability";
		feed = capacity.feedJoint; tcpMember = tool.contact.occurrence; tcpConnector = tool.contact.connector;
		var spindle = capacity.spindleJoint;
		joints = [for (i in 1...7) 'arm/j$i'];
		limits = [for (spec in cell.arm.specs) {lower:spec.lower,upper:spec.upper}];
		moving = [for (item in cell.components()) if (StringTools.startsWith(item.id,"arm/")) item.id];
		fixed = [for (item in cell.components()) if (!StringTools.startsWith(item.id,"arm/")) item.id];
		var model = state.model.kinematics.model;
		bodyGroups = kinematicskit.KinematicBodyPairs.rigidGroups(model);
		var identified = [for (joint in state.definition.joints) if (joint.id == spindle) joint];
		if (identified.length != 1) throw "Access needs its identified spindle joint";
		var carrier = model.bodyIndex(identified[0].child);
		if (carrier < 0) throw "Access lost its spindle carrier body";
		carrierGroup = bodyGroups[carrier];
		for (id in moving) if (model.bodyIndex(id) < 0) throw 'Access model has no CAD member "$id"';
		for (pair in kinematicskit.KinematicBodyPairs.of(model)) {
			allowed.set('${pair.a}:${pair.b}',true); allowed.set('${pair.b}:${pair.a}',true);
		}
	}

	public function problem(fastening:Fastening):Null<String> {
		var selected = [for (job in jobs) if (job.fastening == fastening.name()) job];
		if (selected.length != 1) return "access has no unique qualified job";
		var saved = [for (joint in joints) state.joint(joint)], savedFeed = state.joint(feed);
		var result:Null<String> = null;
		try result = check(selected[0]) catch (error:Dynamic) result = Std.string(error);
		for (i in 0...joints.length) state.setJoint(joints[i],saved[i]);
		state.setJoint(feed,savedFeed);
		return result;
	}

	function template(body:String):String {
		var found = [for (supply in supplies) if (supply.body == body) supply];
		if (found.length != 1) throw 'Supply "$body" has no unique presenter template';
		return found[0].template;
	}

	function world(member:String, frame:AssemblyFrame):AssemblyFrame {
		var local = {x:frame.x/metres,y:frame.y/metres,z:frame.z/metres,
			qx:frame.qx,qy:frame.qy,qz:frame.qz,qw:frame.qw};
		return AssemblyFrames.compose(state.worldPose(member),local);
	}

	function check(job:SceneScrew):Null<String> {
		var targets:Array<AccessPose> = [];
		var primary = template(job.screw);
		targets.push({label:"primary pickup",member:job.presenter,frame:job.pickup,feed:0,held:false,threaded:false,template:primary});
		targets.push({label:"primary lift",member:job.presenter,frame:job.pickup,feed:0,held:true,threaded:false,template:primary});
		var recovery = job.recovery;
		if (recovery != null) {
			var spare = recovery.replacement;
			var spareTemplate = template(spare.screw);
			targets.push({label:"replacement pickup",member:spare.presenter,frame:spare.pickup,feed:0,held:false,threaded:false,template:spareTemplate});
			targets.push({label:"replacement lift",member:spare.presenter,frame:spare.pickup,feed:0,held:true,threaded:false,template:spareTemplate});
			targets.push({label:"reject",member:recovery.rejectMember,frame:recovery.reject,feed:0,held:true,threaded:false,template:primary});
		}
		var stroke = (job.program.minimumStroke+job.program.maximumStroke)/2;
		targets.push({label:"thread entry",member:job.frame,frame:job.entry,feed:-stroke,held:true,threaded:true,template:primary});
		targets.push({label:"head seat",member:job.frame,frame:job.seat,feed:-stroke,held:true,threaded:true,template:primary});
		var source = [for (item in cell.components()) if (item.id == job.screw) item.component];
		if (source.length != 1 || !Std.isOfType(source[0],ScrewFastener)) throw "Access lost its identified driven fastener";
		var screw:ScrewFastener = cast source[0];
		var current = [for (joint in joints) state.joint(joint)];
		var distances = [0.0], approach = job.approach/metres;
		for (i in 1...Math.ceil(approach/10)+1) distances.push(Math.min(i*10.0,approach));
		for (target in targets) for (out in distances) {
			var socket = world(target.member,target.frame), direction = AssemblyFrames.transformVector(socket,0,0,1);
			socket.x -= direction.x*out; socket.y -= direction.y*out; socket.z -= direction.z*out;
			state.setJoint(feed,out == 0 ? target.feed : 0);
			var label = '${target.label} at $out mm out', found = false;
			var collision:Null<String> = null;
			var seeds = [current,cell.readyPose()];
			for (shoulder in [0.4,0.8,1.2]) for (elbow in [0.17,0.67,1.17])
				seeds.push([0.0,shoulder,elbow,0.0,Math.PI/2-shoulder+elbow,0.0]);
			for (seed in seeds) if (!found) {
				var q = machinekit.robot.ToolAxisReach.solveFrame(state,joints,limits,socket,seed,tcpMember,tcpConnector);
				if (q == null) continue;
				for (i in 0...joints.length) state.setJoint(joints[i],q[i]);
				// The active screw is staged, not simultaneously seated in its CAD
				// display pose. Other heads remain a conservative installed-state check.
				var obstacles = [for (id in fixed) if (id != job.screw && (!target.held || id != target.template)) id];
				try {
					parts.checkClear(cell,state,"Screwdriving access",moving,obstacles,
						(a,b,volume) -> '$label: $a hits $b ($volume mm³)');
					for (tool in moving) if (StringTools.startsWith(tool,"arm/tool/"))
						for (arm in moving) if (!StringTools.startsWith(arm,"arm/tool/")) {
							var a = state.model.kinematics.model.bodyIndex(tool), b = state.model.kinematics.model.bodyIndex(arm);
							if (allowed.exists('$a:$b')) continue;
							var volume = parts.volume(cell,state,"Screwdriving access",tool,arm);
							if (volume > 1e-3) throw '$label: $tool hits $arm ($volume mm³)';
						}
					if (target.held) {
						var payload = parts.posed(source[0],AssemblyFrames.compose(socket,AssemblyFrames.inverse(screw.driveFrame())));
						try {
							var box = PosedParts.boxOf(payload);
							var others = obstacles.concat([for (id in moving)
								if (bodyGroups[state.model.kinematics.model.bodyIndex(id)] != carrierGroup) id]);
							for (id in others) if (!target.threaded || id != job.frame) {
								var obstacle = parts.member(cell,state,"Screwdriving access",id);
								var volume = PosedParts.commonVolume(payload,box,obstacle,PosedParts.boxOf(obstacle));
								obstacle.close();
								if (volume > 1e-3) throw '$label: held ${job.screw} hits $id ($volume mm³)';
							}
						} catch (error:Dynamic) {payload.close(); throw error;}
						payload.close();
					}
					current = q; found = true;
				} catch (error:Dynamic) if (collision == null) collision = Std.string(error);
			}
			if (!found) return collision == null ? '$label: full socket orientation is unreachable' : collision;
		}
		return null;
	}
}
