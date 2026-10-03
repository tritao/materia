package machinekit.transmission;

/** Resolve a saved source against the assembly's current members. */
class TransmissionResolver {
	public static function resolve(drive:machinekit.assembly.MachineAssemblyDescription.DriveRecord, requireMember:String->machinekit.component.MachineComponent):machinekit.transmission.TransmissionRelation {
		if (!(Math.abs(drive.alignment) == 1)) throw 'Drive "${drive.coupling}" needs an alignment of 1 or -1';
		function part(index:Int):machinekit.component.MachineComponent {
			if (drive.members.length <= index) throw 'Drive "${drive.coupling}" names too few parts';
			return requireMember(drive.members[index]);
		}
		function gear(index:Int):machinekit.transmission.SpurGear {
			var member = part(index);
			if (!Std.isOfType(member, machinekit.transmission.SpurGear))
				throw 'Drive "${drive.coupling}": "${drive.members[index]}" is not a spur gear';
			return cast member;
		}
		var relation = switch drive.kind {
			case "lead-screw":
				var member = part(0);
				if (!Std.isOfType(member, machinekit.motion.LeadScrew)) throw "Drive needs a lead screw";
				machinekit.motion.LeadScrew.relation(cast member, drive.alignment);
			case "gear-mesh": machinekit.transmission.GearPair.relation(gear(0), gear(1), drive.alignment);
			case "rack-and-pinion": machinekit.transmission.Rack.relation(gear(0), drive.alignment);
			case "belt":
				var member = part(0);
				if (Std.isOfType(member, machinekit.transmission.TimingPulley))
					machinekit.transmission.TimingPulley.relation(cast member, drive.alignment);
				else if (Std.isOfType(member, machinekit.transmission.Sprocket))
					machinekit.transmission.Sprocket.relation(cast member, drive.alignment);
				else throw "Drive needs a timing pulley or sprocket";
			case kind: throw 'Drive "${drive.coupling}" has unknown kind "$kind"';
		};
		relation.stiffness = drive.stiffness;
		if (drive.backlash != null) relation.backlash = drive.backlash;
		if (drive.drag != null) relation.drag = drive.drag;
		var near = drive.nearSupport, far = drive.farSupport;
		if (drive.kind == "lead-screw" && near != null && far != null) {
			var screw:machinekit.motion.LeadScrew = cast requireMember(drive.members[0]);
			relation.followerSpeedCap = screw.criticalSpeed(supportOf(near), supportOf(far), drive.unsupported,
				machinekit.assembly.DriveDefaults.CRITICAL_SPEED_MARGIN);
		}
		return relation;
	}

	static function supportOf(name:String):machinekit.motion.ScrewSupport
		return switch name {
			case "free": Free;
			case "simple": Simple;
			case "fixed": Fixed;
			case other: throw 'Unknown screw support "$other"';
		};
}
