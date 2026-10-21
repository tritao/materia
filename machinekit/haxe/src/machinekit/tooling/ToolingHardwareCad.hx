package machinekit.tooling;

import machinekit.assembly.MachineAssembly;
import machinekit.standard.FlatWasher;
import machinekit.standard.HexNut;
import machinekit.standard.SocketHeadCapScrew;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Nominal metric stack from the plan's selected screw diameter. */
class ToolingHardwareCad {
	public static function throughBolt(a:MachineAssembly, id:String, diameter:Float, grip:Float, pose:AssemblyFrame):Void {
		var thread = "M" + Std.string(diameter);
		var washer = FlatWasher.metric(thread), nut = HexNut.metric(thread);
		var length = 5 * Math.ceil((grip + 2 * washer.thickness + nut.height + 1) / 5);
		var screw=SocketHeadCapScrew.metric(thread,length,machinekit.standard.PropertyClass.Steel88);
		var screwPose=F.compose(pose,F.translation(0,0,washer.thickness));
		var nutPose=F.compose(pose,F.translation(0,0,-grip-washer.thickness-nut.height));
		ToolingFastenerFit.requireNut(screw,screwPose,nut,nutPose,id);
		a.addComponent(id,screw,screwPose);
		a.addComponent(materia.tooling.ToolingIds.derive("", id, "-top-washer"), washer, pose);
		a.addComponent(materia.tooling.ToolingIds.derive("", id, "-bottom-washer"), FlatWasher.metric(thread),
			F.compose(pose, F.translation(0, 0, -grip - washer.thickness)));
		a.addComponent(materia.tooling.ToolingIds.derive("", id, "-nut"), nut, nutPose);
	}

	/** A nominal study screw into a real source hole. Length excludes its washer.
	 * Validate the actual standard component before mutating the assembly.
	 */
	public static function fixtureScrew(a:MachineAssembly,id:String,hole:materia.casting.InsertHole,length:Float,
		pose:AssemblyFrame,holePose:AssemblyFrame):Void {
		var thread=hole.thread;
		if (thread==null) throw "Fixture screw requires a source steel thread: "+id;
		var sourceThread=machinekit.structural.InsertHoles.threadSpec(thread);
		var washer=FlatWasher.metric(thread.designation);
		var screw=SocketHeadCapScrew.metric(thread.designation,length+washer.thickness,machinekit.standard.PropertyClass.Steel88);
		if (!screw.threadSpec().matches(sourceThread)) throw "Fixture screw differs from source thread: "+id;
		var screwPose=F.compose(pose,F.translation(0,0,washer.thickness));
		var fits=materia.casting.ThreadedFastenerFit.checkIntervals(hole,holePose,
			{shankLength:screw.shankLength(),threadedLength:screw.threadedLength(),bearingFrame:F.compose(screwPose,screw.bearingFrame())},id);
		var failures=fits.filter(check -> !check.passed);
		if (failures.length>0) throw "Invalid fixture screw "+id+": "+[for (failure in failures) failure.code+": "+failure.message].join("; ");
		a.addComponent(id,screw,screwPose);
		a.addComponent(materia.tooling.ToolingIds.derive("",id,"-washer"),washer,pose);
	}
}
