import machinekit.assembly.PosedParts;
import machinekit.robot.ToolAxisReach;
import machinekit.standard.ScrewFastener;
import materia.assembly.AssemblyFrames;
/** CAD design checks, with named failures; they do not plan a driving mission. */
class ScrewdrivingChecks {
	public static function check(cell:ScrewdrivingCell):Void {
		cell.check().throwIfErrors();
		var fastenings = cell.work.fastenings().require();
		for (fastening in fastenings) {
			var components = [for (member in cell.work.components()) if (member.id == fastening.screw.member) member.component];
			var screw:ScrewFastener = cast components[0];
			if (!cell.tool.spindle.bit.matches(screw.screwDrive())) throw 'Bit does not match ${fastening.name()} (${screw.screwDrive()})';
			if (fastening.tightening.torqueNm > cell.tool.spindle.capability.maxTorque)
				throw 'Spindle capacity is below ${fastening.name()} tightening torque';
		}
		if (!cell.tool.spindle.bit.matches(cell.presenter.screw.screwDrive())) throw "Bit does not match the presenter screw";
		var state = ScrewdrivingPreview.state(cell);
		var joints = [for (i in 1...7) 'arm/j$i'];
		var limits = [for (spec in cell.arm.specs) {lower: spec.lower, upper: spec.upper}];
		var ready = cell.readyPose();
		var targets:Array<{name:String, member:String, connector:String}> = [for (fastening in fastenings)
			{name: fastening.name(), member: "work/" + fastening.screw.member, connector: "head"}];
		if (!Std.isOfType(cell.tool,machinekit.screwdriving.BlowFeedTool)) targets.push({name: "presenter", member: "presenter/screw", connector: "head"});
		var moving = [for (member in cell.components()) if (StringTools.startsWith(member.id, "arm/tool/")) member.id];
		var fixed = [for (member in cell.components()) if (!StringTools.startsWith(member.id, "arm/")) member.id];
		PosedParts.scope(parts -> {
			for (target in targets) {
				var head = state.worldConnector(target.member, target.connector);
				// A screw's head connector has +Y along its shank; tool +Z points into the socket.
				var direction = AssemblyFrames.transformVector(head, 0, -1, 0);
				var component = [for (member in cell.components()) if (member.id == target.member) member.component][0];
				var screw:machinekit.standard.SocketHeadCapScrew = cast component;
				for (out in [0.0, 10.0, 20.0, 30.0, 40.0]) {
					var distance = screw.spec.headHeight + out;
					var position = {x: head.x - direction.x * distance, y: head.y - direction.y * distance, z: head.z - direction.z * distance};
					var found:Null<Array<Float>> = null;
					var seeds = [ready];
					for (shoulder in [0.4, 0.8, 1.2]) for (elbow in [0.17, 0.67, 1.17])
						seeds.push([0.0, shoulder, elbow, 0.0, Math.PI / 2 - shoulder + elbow, 0.0]);
					for (seed in seeds) if (found == null)
						found = ToolAxisReach.solve(state, joints, limits, position, direction, seed, "arm/tool/driver/carrier", "tcp");
					if (found == null) throw '${target.name}: tool axis is unreachable at $out mm approach';
					for (i in 0...6) state.setJoint(joints[i], found[i]);
					if (out > 0) parts.checkClear(cell, state, "Screwdriving cell", moving, fixed,
						(a, b, volume) -> '${target.name}: $a hits $b at approach ($volume mm³)');
				}
			}
		});
	}
}
