package machinekit.structural;

import machinekit.component.MachineComponent;
import machinekit.component.Connector;
import cadkit.threads.ThreadSpec;

typedef ElementMountBore = {
	var diameter:Float;
	var counterboreDiameter:Float;
	var counterboreDepth:Float;
	var reference:String;
}

/** An adapter to the element's own published drilling, not a new hole pattern.
 * New scale recipes can implement the same contract when they have sourced
 * mount connectors. Today's generic tape scale deliberately cannot supply it.
 */
class MountedElement {
	public final component:MachineComponent;
	public final connectors:Array<Connector>;
	public final bore:ElementMountBore;
	public final thread:ThreadSpec;

	public function new(component:MachineComponent,connectors:Array<Connector>,bore:ElementMountBore,thread:ThreadSpec) {
		if (connectors.length==0 || !thread.internal) throw "Mounted element requires connectors and an internal receiving thread";
		if (!Math.isFinite(bore.diameter+bore.counterboreDiameter+bore.counterboreDepth)
			|| bore.diameter<=thread.majorDiameter || bore.counterboreDiameter<=bore.diameter
			|| bore.counterboreDepth<=0 || StringTools.trim(bore.reference)=="")
			throw "Mounted element requires identified, finite counterbored drilling";
		var seen:Map<String,Bool>=[];
		for(c in connectors) {
			if(seen.exists(c.name)) throw "Duplicate element connector: "+c.name;
			seen.set(c.name,true);
			// Use the owning recipe's connector, not a caller's replacement frame.
			var actual=component.connector(c.name);
			if(haxe.Json.stringify(actual.frame)!=haxe.Json.stringify(c.frame)) throw "Element connector differs from its recipe";
		}
		this.component=component;this.connectors=[for(c in connectors)component.connector(c.name)];
		this.bore={diameter:bore.diameter,counterboreDiameter:bore.counterboreDiameter,
			counterboreDepth:bore.counterboreDepth,reference:bore.reference};
		this.thread=thread;
	}

	public static function of(component:MachineComponent):MountedElement {
		if (Std.isOfType(component,machinekit.motion.LinearRail)) {
			var rail:machinekit.motion.LinearRail=cast component;
			return new MountedElement(rail,rail.mountingConnectors(),rail.mountingHole(),rail.mountingThread());
		}
		if (Std.isOfType(component,machinekit.transmission.GearRack)) {
			var rack:machinekit.transmission.GearRack=cast component;
			return new MountedElement(rack,rack.mountingConnectors(),rack.mountingHole(),rack.mountingThread());
		}
		throw "Element recipe has no declared mounting drilling: "+component.designation;
	}

	public function fingerprint(memberId:String):String
		return haxe.crypto.Sha256.encode(materia.tooling.ToolingCodec.canonical({
			definition:machinekit.assembly.MachineAssembly.definitionKey(memberId,component),
			mountingHole:bore,thread:InsertHoles.threadRecord(thread),connectors:connectors}));
}
