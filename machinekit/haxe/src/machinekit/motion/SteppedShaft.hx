package machinekit.motion;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.ConnectorRole;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.standard.ParallelKey;

/** One constant-diameter length of a stepped shaft, in millimetres. */
typedef ShaftSection = {
	var diameter:Float;
	var length:Float;
}

/** A keyway cut for `key`, starting at `z0` and running the key's length.
 * Adds a `name` connector at the seat floor, mid-keyway, for mating the key with a fixed joint.
 */
typedef ShaftKeyway = {
	var name:String;
	var z0:Float;
	var key:ParallelKey;
}

/** A full-depth retaining-ring groove from `z0` to `z0 + width`, cut to `diameter`. */
typedef ShaftGroove = {
	var z0:Float;
	var width:Float;
	var diameter:Float;
}

/** Coaxial cylindrical sections stacked along +Z, producing square shoulders at each diameter
 * change. CAD frame: input end at z=0, output end at z=totalLength.
 * Connectors: `input` (axis, z=0), `output` (shaft, z=totalLength), plus any `namedFaces` (axis,
 * for bearing or collar seats) and one connector per keyway. Keyways are cut on the shaft's local
 * +Y side, so a `ParallelKey` mated through the keyway's connector sits flush in the slot.
 */
class SteppedShaft extends MachineComponent {
	public final sections:Array<ShaftSection>;
	public final totalLength:Float;
	final keyways:Array<ShaftKeyway>;
	final grooves:Array<ShaftGroove>;

	public function new(sections:Array<ShaftSection>, ?namedFaces:Array<{name:String, z:Float}>,
			?keyways:Array<ShaftKeyway>, ?grooves:Array<ShaftGroove>) {
		if (sections.length == 0) throw "Stepped shaft needs at least one section";
		for (section in sections)
			if (!(section.diameter > 0) || !(section.length > 0))
				throw "Stepped shaft section needs a positive diameter and length";
		var total = 0.0;
		for (section in sections) total += section.length;
		var name = "SHAFT-" + [for (section in sections) '${section.diameter}x${section.length}'].join("-");
		super(name, "Stepped shaft " + [for (section in sections) '${section.diameter}x${section.length}'].join(" / "),
			"steel C45");
		this.sections = sections.copy();
		totalLength = total;
		addConnector("input", Axis, Solids.axial(0, 0, 0));
		addConnector("output", Shaft, Solids.axial(0, 0, total));
		if (namedFaces != null)
			for (face in namedFaces) {
				if (face.z < 0 || face.z > total) throw 'Face "${face.name}" lies outside the shaft';
				addConnector(face.name, Face, Solids.axial(0, 0, face.z));
			}
		this.keyways = keyways == null ? [] : keyways.copy();
		for (keyway in this.keyways) {
			var end = keyway.z0 + keyway.key.length;
			if (keyway.z0 < 0 || end > total) throw 'Keyway "${keyway.name}" lies outside the shaft';
			if (diameterAt(keyway.z0) != diameterAt(end - 1e-6))
				throw 'Keyway "${keyway.name}" must lie within one shaft section';
			var radius = diameterAt(keyway.z0) / 2;
			if (!(keyway.key.spec.shaftDepth < radius))
				throw 'Keyway "${keyway.name}" is deeper than the shaft radius';
			addConnector(keyway.name, Face, Solids.axial(0, radius - keyway.key.spec.shaftDepth,
				keyway.z0 + keyway.key.length / 2));
		}
		this.grooves = grooves == null ? [] : grooves.copy();
		for (groove in this.grooves) {
			if (!(groove.width > 0)) throw "Retaining ring groove needs a positive width";
			if (groove.z0 < 0 || groove.z0 + groove.width > total) throw "Retaining ring groove lies outside the shaft";
			if (diameterAt(groove.z0) != diameterAt(groove.z0 + groove.width - 1e-6))
				throw "Retaining ring groove must lie within one shaft section";
			if (!(groove.diameter > 0) || !(groove.diameter < diameterAt(groove.z0)))
				throw "Retaining ring groove diameter must be smaller than the shaft";
		}
	}

	/** Diameter of the section containing `z`; a boundary belongs to the following section. */
	public function diameterAt(z:Float):Float {
		if (z < 0 || z > totalLength) throw 'Shaft position $z is outside 0..$totalLength';
		var start = 0.0;
		for (i in 0...sections.length) {
			var section = sections[i];
			var end = start + section.length;
			if (z < end || i == sections.length - 1) return section.diameter;
			start = end;
		}
		throw 'Shaft position $z is outside 0..$totalLength';
	}

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var z = 0.0, parts:Array<Part> = [];
		for (section in sections) {
			parts.push(Solids.cylinder(section.diameter / 2, z, z + section.length));
			z += section.length;
		}
		var body = Solids.union(parts);
		if (detail == Envelope) return body;
		var tools:Array<Part> = [];
		for (keyway in keyways) tools.push(keywayTool(keyway));
		for (groove in grooves) tools.push(Solids.cylinder(groove.diameter / 2, groove.z0, groove.z0 + groove.width));
		return tools.length == 0 ? body : Solids.cut(body, tools);
	}

	function keywayTool(keyway:ShaftKeyway):Part {
		var radius = diameterAt(keyway.z0) / 2, depth = keyway.key.spec.shaftDepth,
			width = keyway.key.spec.width, floor = radius - depth;
		return Solids.prism([
			new Vector(-width / 2, floor), new Vector(width / 2, floor),
			new Vector(width / 2, radius + depth), new Vector(-width / 2, radius + depth),
		], keyway.z0, keyway.z0 + keyway.key.length);
	}
}
