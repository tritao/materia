package machinekit.welding;

import machinekit.assembly.MachineAssembly;

/** What a welding robot's simulated or real welder needs to know about the cell around its torch. */
typedef WeldingEquipmentData = {
	/** The power source's occurrence id. */
	var supply:String;
	var maxCurrentA:Float;
	/** Arc power delivered per watt drawn from the mains. */
	var efficiency:Float;
	var wireDiameterMm:Float;
	var maxWireSpeedMPerMin:Float;
	/** Fraction of the melted wire that reaches the weld. */
	var depositionEfficiency:Float;
	/** The occurrences the weld circuit returns through, in the assembly's own ids. */
	var groundedWork:Array<String>;
};

/**
 * Reads the welding equipment of an assembly from its parts' capabilities and its connections.
 *
 * Grounded work is derived, never listed: the power source's work lead (`weldNegative`) is traced
 * to the work clamp it is connected to, and the clamp's mates say what it sits on. The weld
 * circuit returns through that member and through every member welded to it (a `Weldment`'s
 * members are one conductor: welded joints carry current, and a seam's two sides are the same
 * metal). A mate does not make a conductor: a workpiece lying on a table is positioned there, not
 * bonded to it, so the table stays outside the circuit unless the clamp is on the table.
 */
class WeldingEquipment {
	/**
	 * The equipment of `assembly`, which has exactly one power source whose work lead reaches a work
	 * clamp, and a wire feeder. `weldments` are the welded workpieces in the assembly's own ids.
	 */
	public static function of(assembly:MachineAssembly, weldments:Array<Weldment>):WeldingEquipmentData {
		var supply:Null<String> = null, maxCurrent = 0.0, efficiency = 1.0;
		var feeder:Null<String> = null, wire = 0.0, wireSpeed = 0.0, deposited = 1.0;
		var clamps:Array<{id:String, lead:String}> = [];
		for (member in assembly.components()) {
			var source = WeldingSupplyFacet.of(member.component);
			if (source != null) {
				if (supply != null) throw 'The assembly has two welding power sources, "$supply" and "${member.id}"';
				supply = member.id;
				maxCurrent = source.maxCurrentA;
				efficiency = source.efficiency;
			}
			var feed = WireFeedFacet.of(member.component);
			if (feed != null) {
				if (feeder != null) throw 'The assembly has two wire feeders, "$feeder" and "${member.id}"';
				feeder = member.id;
				wire = feed.wireDiameterMm;
				wireSpeed = feed.maxSpeedMPerMin;
				deposited = feed.depositionEfficiency;
			}
			var clamp = WorkReturnFacet.of(member.component);
			if (clamp != null) clamps.push({id: member.id, lead: clamp.leadPort});
		}
		if (supply == null) throw "The assembly has no welding power source";
		if (feeder == null) throw "The assembly has no wire feeder";
		var returnEnd = supply + "/" + WeldingPowerSource.WORK_LEAD;
		var grounded:Array<String> = [];
		var found = false;
		for (clamp in clamps) {
			// A clamp counts when its lead is supplied by the power source's work lead, however many connections lie between.
			var chain = assembly.upstreamChain(clamp.id, clamp.lead);
			if (chain[chain.length - 1] != returnEnd) continue;
			found = true;
			for (partner in assembly.matedMembers(clamp.id)) {
				if (grounded.indexOf(partner) < 0) grounded.push(partner);
				for (weldment in weldments) if (weldment.members.indexOf(partner) >= 0)
					for (welded in weldment.members) if (grounded.indexOf(welded) < 0) grounded.push(welded);
			}
		}
		if (!found) throw 'The power source "$supply" has its work lead connected to no work clamp';
		if (grounded.length == 0) throw "The work clamp is not mated to anything: the weld circuit does not close";
		return {supply: supply, maxCurrentA: maxCurrent, efficiency: efficiency, wireDiameterMm: wire,
			maxWireSpeedMPerMin: wireSpeed, depositionEfficiency: deposited, groundedWork: grounded};
	}
}
