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
		a.addComponent(id, SocketHeadCapScrew.metric(thread, length), F.compose(pose, F.translation(0, 0, washer.thickness)));
		a.addComponent(materia.tooling.ToolingIds.derive("", id, "-top-washer"), washer, pose);
		a.addComponent(materia.tooling.ToolingIds.derive("", id, "-bottom-washer"), FlatWasher.metric(thread),
			F.compose(pose, F.translation(0, 0, -grip - washer.thickness)));
		a.addComponent(materia.tooling.ToolingIds.derive("", id, "-nut"), nut, F.compose(pose, F.translation(0, 0, -grip - washer.thickness - nut.height)));
	}

	/** A cut-to-length study screw into a declared insert thread. Length excludes its washer. */
	public static function fixtureScrew(a:MachineAssembly, id:String, thread:String, length:Float, pose:AssemblyFrame):Void {
		var washer = FlatWasher.metric(thread);
		a.addComponent(id, SocketHeadCapScrew.metric(thread, length + washer.thickness), F.compose(pose, F.translation(0, 0, washer.thickness)));
		a.addComponent(materia.tooling.ToolingIds.derive("", id, "-washer"), washer, pose);
	}
}
