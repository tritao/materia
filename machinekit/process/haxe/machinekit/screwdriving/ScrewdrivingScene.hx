package machinekit.screwdriving;

import cadkit.modeling.AssemblyState;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.assembly.AssemblyDefinitionFlattener;
import materia.project.SceneArtifact.SceneArtifactData;
import materia.project.SceneScrewDefinition;
import materia.project.SceneScrewDefinition.SceneScrew;
import materia.project.SceneScrewDefinition.ScrewSimulationSettings;
import materia.project.ThreadRenderData;
import machinekit.standard.ScrewFastener;
import machinekit.screwdriving.TighteningPrograms.TighteningPolicy;

/** Fresh mission geometry from a qualified connection and the exported named thread packet. */
class ScrewdrivingScene {
	static function scaled(value:AssemblyFrame, metres:Float):AssemblyFrame
		return {x:value.x*metres,y:value.y*metres,z:value.z*metres,qx:value.qx,qy:value.qy,qz:value.qz,qw:value.qw};
	static function hole(thread:ThreadRenderData, metres:Float):AssemblyFrame {
		var x = thread.xDirection, z = thread.direction;
		var y = [z[1]*x[2]-z[2]*x[1],z[2]*x[0]-z[0]*x[2],z[0]*x[1]-z[1]*x[0]];
		return AssemblyFrames.fromRotationMatrix(thread.origin[0]*metres,thread.origin[1]*metres,thread.origin[2]*metres,
			[x[0],y[0],z[0],x[1],y[1],z[1],x[2],y[2],z[2]]);
	}
	public static function fromFastening(source:FastenedAssembly, fastening:Fastening, prefix:String,
		scene:SceneArtifactData, presenter:String, presentedScrew:String, presentSensor:String,
		slot:Int, policy:TighteningPolicy, simulation:ScrewSimulationSettings, approach:Float = 0.04):SceneScrew {
		var definition = scene.assemblyDefinition;
		if (definition == null) throw '${fastening.name()}: screw mission needs an exported assembly';
		var flat = AssemblyDefinitionFlattener.flatten(definition);
		var state = new AssemblyState(flat,scene.assemblyState);
		var screwId = prefix+fastening.screw.member, frameId = prefix+fastening.threaded.member,
			clampedId = prefix+fastening.clamped.member;
		var members = [for (item in source.components()) if (item.id == fastening.screw.member) item.component];
		if (members.length != 1 || !Std.isOfType(members[0],ScrewFastener)) throw '${fastening.name()}: driven component is missing';
		var screw:ScrewFastener = cast members[0];
		var occurrences = [for (item in flat.occurrences) if (item.id == frameId) item];
		if (occurrences.length != 1) throw '${fastening.name()}: threaded occurrence "$frameId" is missing';
		var clampedOccurrences = [for (item in flat.occurrences) if (item.id == clampedId) item];
		if (clampedOccurrences.length != 1) throw '${fastening.name()}: clamped occurrence "$clampedId" is missing';
		var packets:Array<ThreadRenderData> = [];
		for (part in scene.parts) if (part.id == occurrences[0].definition) {
			var threads = part.threads;
			if (threads != null) for (thread in threads) if (thread.faceName == fastening.threaded.faceName) packets.push(thread);
		}
		if (packets.length != 1) throw '${fastening.name()}: exact named thread packet is missing or ambiguous';
		var metres = scene.metresPerUnit, thread = packets[0];
		var socket = screw.driveFrame(), target = state.worldPose(frameId);
		var seat = scaled(AssemblyFrames.compose(AssemblyFrames.inverse(target),
			AssemblyFrames.compose(state.worldPose(screwId),socket)),metres);
		var lead = hole(thread,metres), axis = AssemblyFrames.transformVector(lead,0,0,1);
		var local = source.solvedPoses().get(fastening.threaded.member);
		if (local == null) throw '${fastening.name()}: source threaded pose is missing';
		var tip = AssemblyFrames.transformPoint(AssemblyFrames.inverse(local),fastening.tip.x,fastening.tip.y,fastening.tip.z);
		var runDown = (tip.x*metres-lead.x)*axis.x+(tip.y*metres-lead.y)*axis.y+(tip.z*metres-lead.z)*axis.z;
		var entry:AssemblyFrame = {x:seat.x-axis.x*runDown,y:seat.y-axis.y*runDown,z:seat.z-axis.z*runDown,
			qx:seat.qx,qy:seat.qy,qz:seat.qz,qw:seat.qw};
		var pickup = scaled(AssemblyFrames.compose(AssemblyFrames.inverse(state.worldPose(presenter)),
			AssemblyFrames.compose(state.worldPose(presentedScrew),socket)),metres);
		var job:SceneScrew = {delivery:Presented,fastening:fastening.name(),screw:screwId,frame:frameId,
			threadFace:fastening.threaded.faceName,clamped:clampedId,
			presenter:presenter,presentSensor:presentSensor,socket:scaled(socket,metres),pickup:pickup,entry:entry,seat:seat,hole:lead,
			leadMm:fastening.thread.pitch*fastening.thread.starts,hand:switch fastening.thread.hand {case RightHand:1;case LeftHand:-1;},
			slot:slot,program:TighteningPrograms.fromFastening(fastening,policy).settings,simulation:simulation,approach:approach};
		SceneScrewDefinition.validateScene(job,flat,scene);
		return job;
	}
}
