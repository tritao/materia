package machinekit.transmission;

import machinekit.assembly.Transmission;
import machinekit.assembly.Sense.SenseTools;
import machinekit.assembly.MachineAssemblyDescription.TransmissionRecord;
import machinekit.component.MachineComponent;

/** Resolve a saved source against the assembly's current members. */
class TransmissionResolver {
	public static function resolve(record:TransmissionRecord, member:String->MachineComponent):TransmissionRelation {
		var alignment = SenseTools.sign(record.sense);
		function gear(id:String):SpurGear {
			var part = member(id);
			if (!Std.isOfType(part, SpurGear)) throw 'Transmission "${record.coupling}": "$id" is not a spur gear';
			return cast part;
		}
		var relation = switch record.source {
			case LeadScrew(screwId, nutId):
				var screw = member(screwId);
				if (!Std.isOfType(screw, machinekit.motion.LeadScrew)) throw "Transmission needs a lead screw";
				if (nutId != null && !Std.isOfType(member(nutId), machinekit.motion.LeadScrewNut)) throw "Transmission needs a lead screw nut";
				var result = machinekit.motion.LeadScrew.relation(cast screw, alignment);
				if (record.near != null && record.far != null)
					result.followerSpeedCap = (cast(screw, machinekit.motion.LeadScrew)).criticalSpeed(record.near,
						record.far, record.unsupported, machinekit.assembly.DriveDefaults.CRITICAL_SPEED_MARGIN);
				result;
			case GearMesh(driver, driven): GearPair.relation(gear(driver), gear(driven), alignment);
			case RackAndPinion(pinion, rack):
				if (rack != null && !Std.isOfType(member(rack), Rack)) throw "Transmission needs a rack";
				Rack.relation(gear(pinion), alignment);
			case TimingBelt(beltId, pulleyId, strand):
				var belt = member(beltId), pulley = member(pulleyId);
				if (!Std.isOfType(belt, TimingBelt)) throw "Transmission needs a timing belt";
				if (!Std.isOfType(pulley, TimingPulley)) throw "Transmission needs a timing pulley";
				var loop:TimingBelt = cast belt;
				if (strand < 0 || strand >= loop.strands().length) throw "Transmission belt strand is out of range";
				TimingPulley.relation(cast pulley, alignment);
			case RollerChain(chain, sprocketId):
				if (chain != null) member(chain);
				var sprocket = member(sprocketId);
				if (!Std.isOfType(sprocket, Sprocket)) throw "Transmission needs a sprocket";
				Sprocket.relation(cast sprocket, alignment);
		};
		relation.stiffness = record.stiffness;
		if (record.backlash != null) relation.backlash = record.backlash;
		if (record.drag != null) relation.drag = record.drag;
		return relation;
	}

	/** Keep member references with an included or extracted assembly's namespace. */
	public static function mapSource(source:Transmission, map:String->String):Transmission return switch source {
		case LeadScrew(screw, nut): LeadScrew(map(screw), nut == null ? null : map(nut));
		case GearMesh(driver, driven): GearMesh(map(driver), map(driven));
		case RackAndPinion(pinion, rack): RackAndPinion(map(pinion), rack == null ? null : map(rack));
		case TimingBelt(belt, pulley, strand): TimingBelt(map(belt), map(pulley), strand);
		case RollerChain(chain, sprocket): RollerChain(chain == null ? null : map(chain), map(sprocket));
	};
}
