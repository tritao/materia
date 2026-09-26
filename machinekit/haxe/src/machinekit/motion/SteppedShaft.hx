package machinekit.motion;

import CadKit;
import cadkit.modeling.Axis;
import cadkit.modeling.Part;
import cadkit.modeling.Selection;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.ConnectorRole;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.standard.ParallelKey;
import machinekit.standard.BearingFit;
import machinekit.standard.BearingFit.BearingShaftFit;

/** One constant-diameter length of a stepped shaft, in millimetres. */
typedef ShaftSection = {
	var diameter:Float;
	var length:Float;
}

/** Semantic thread specification for a machined shaft end. Thread flanks are not modelled. */
typedef ShaftThreadEnd = {
	var diameter:Float;
	var pitch:Float;
	var length:Float;
}

/** Detail applied at a section shoulder. The relief diameter is the finished root diameter. */
typedef ShaftShoulderDetail = {
	var z:Float;
	var ?fillet:Float;
	var ?reliefWidth:Float;
	var ?reliefDiameter:Float;
}

/** Optional manufacturing details for a stepped shaft. */
typedef ShaftDetail = {
	var ?inputChamfer:Float;
	var ?outputChamfer:Float;
	var ?inputThread:ShaftThreadEnd;
	var ?outputThread:ShaftThreadEnd;
	var ?shoulders:Array<ShaftShoulderDetail>;
}

/** A keyway cut for `key`, starting at `z0` and running the key's length.
 * Adds a `name` connector at the seat floor, mid-keyway, for mating the key with a fixed joint.
 */
typedef ShaftKeyway = {
	var name:String;
	var z0:Float;
	var key:ParallelKey;
}

/** A full-depth retaining-ring groove from `z0` to `z0 + width`, cut to `diameter`.
 * A `name` connector (axis, groove mid-width) is added when given, for mating a `RetainingRing`.
 */
typedef ShaftGroove = {
	var ?name:String;
	var z0:Float;
	var width:Float;
	var diameter:Float;
}

/** Coaxial cylindrical sections stacked along +Z, producing square shoulders at each diameter
 * change. CAD frame: input end at z=0, output end at z=totalLength.
 * Connectors: `input` (axis, z=0), `output` (shaft, z=totalLength), plus any `namedFaces` (axis,
 * for bearing or collar seats), one connector per named keyway, and one per named groove. Keyways
 * are cut on the shaft's local +Y side, so a `ParallelKey` mated through the keyway's connector
 * sits flush in the slot; groove connectors are axial, for a `RetainingRing`.
 */
class SteppedShaft extends MachineComponent {
	public final sections:Array<ShaftSection>;
	public final totalLength:Float;
	final keyways:Array<ShaftKeyway>;
	final grooves:Array<ShaftGroove>;
	public final detail:ShaftDetail;

	public function new(sections:Array<ShaftSection>, ?namedFaces:Array<{name:String, z:Float}>,
			?keyways:Array<ShaftKeyway>, ?grooves:Array<ShaftGroove>, ?detail:ShaftDetail) {
		if (sections.length == 0) throw "Stepped shaft needs at least one section";
		for (section in sections)
			if (!(section.diameter > 0) || !(section.length > 0))
				throw "Stepped shaft section needs a positive diameter and length";
		var total = 0.0;
		for (section in sections) total += section.length;
		var sizes = [for (section in sections) '${Dimension.format(section.diameter)}x${Dimension.format(section.length)}'];
		var resolvedDetail:ShaftDetail = detail == null ? emptyDetail() : detail;
		var detailSuffix = detailDesignation(resolvedDetail);
		super("SHAFT-" + sizes.join("-") + detailSuffix, "Stepped shaft " + sizes.join(" / ") + detailDescription(resolvedDetail), "steel C45");
		this.sections = sections.copy();
		totalLength = total;
		this.detail = resolvedDetail;
		validateDetail();
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
			if (!withinOneSection(keyway.z0, end))
				throw 'Keyway "${keyway.name}" must lie within one shaft section';
			var radius = diameterAt(keyway.z0) / 2;
			if (!(keyway.key.spec.shaftDepth < radius))
				throw 'Keyway "${keyway.name}" is deeper than the shaft radius';
			if (!(keyway.key.spec.width < radius))
				throw 'Keyway "${keyway.name}" is too wide for the shaft';
			addConnector(keyway.name, Face, Solids.axial(0, radius - keyway.key.spec.shaftDepth,
				keyway.z0 + keyway.key.length / 2));
		}
		this.grooves = grooves == null ? [] : grooves.copy();
		for (groove in this.grooves) {
			if (!(groove.width > 0)) throw "Retaining ring groove needs a positive width";
			if (groove.z0 < 0 || groove.z0 + groove.width > total) throw "Retaining ring groove lies outside the shaft";
			if (!withinOneSection(groove.z0, groove.z0 + groove.width))
				throw "Retaining ring groove must lie within one shaft section";
			if (!(groove.diameter > 0) || !(groove.diameter < diameterAt(groove.z0)))
				throw "Retaining ring groove diameter must be smaller than the shaft";
			if (groove.name != null) addConnector(groove.name, Axis, Solids.axial(0, 0, groove.z0 + groove.width / 2));
		}
		var inputThread = resolvedDetail.inputThread, outputThread = resolvedDetail.outputThread;
		if (inputThread != null)
			addConnector("inputThread", Shaft, Solids.axial(0, 0, inputThread.length / 2));
		if (outputThread != null)
			addConnector("outputThread", Shaft, Solids.axial(0, 0, total - outputThread.length / 2));
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

	/** Finished journal diameter at `z` for a named shaft fit. Allowance is diametral. */
	public function journalDiameterAt(z:Float, fit:BearingShaftFit = Slip):Float
		return diameterAt(z) + BearingFit.shaftAllowance(fit, diameterAt(z));

	/** Finished journal diameter at `z` with an explicit diametral allowance. */
	public function journalDiameterAllowance(z:Float, allowance:Float):Float {
		var result = diameterAt(z) + allowance;
		if (!(result > 0) || !Math.isFinite(result)) throw "Shaft journal allowance leaves no shaft diameter";
		return result;
	}

	/** True when no shoulder lies strictly inside z0..z1. */
	function withinOneSection(z0:Float, z1:Float):Bool {
		var boundary = 0.0;
		for (i in 0...sections.length - 1) {
			boundary += sections[i].length;
			if (boundary > z0 + 1e-6 && boundary < z1 - 1e-6) return false;
		}
		return true;
	}

	function validateDetail():Void {
		validateChamfer(optionalFloat(detail.inputChamfer), diameterAt(0), "input");
		validateChamfer(optionalFloat(detail.outputChamfer), diameterAt(totalLength), "output");
		var inputThread = detail.inputThread, outputThread = detail.outputThread;
		if (inputThread != null)
			validateThread(inputThread, diameterAt(0), sections[0].length, "input");
		if (outputThread != null)
			validateThread(outputThread, diameterAt(totalLength), sections[sections.length - 1].length, "output");
		var shoulders = detail.shoulders == null ? [] : detail.shoulders;
		for (i in 0...shoulders.length) {
			var shoulder = shoulders[i];
			if (shoulderIndex(shoulder.z) < 0) throw 'Shoulder detail z=${shoulder.z} is not a section boundary';
			var prior = diameterAt(shoulder.z - 1e-5), next = diameterAt(shoulder.z + 1e-5);
			var smaller = Math.min(prior, next);
			var fillet = optionalFloat(shoulder.fillet);
			if (shoulder.fillet != null && (!(fillet > 0) || fillet >= smaller / 2))
				throw 'Shoulder fillet at z=${shoulder.z} is too large';
			if (shoulder.reliefWidth != null) {
				var width = optionalFloat(shoulder.reliefWidth);
				if (!(width > 0) || width >= 2 * Math.min(sectionLengthBefore(shoulder.z), sectionLengthAfter(shoulder.z)))
					throw 'Shoulder relief at z=${shoulder.z} has invalid width';
				var reliefDiameter = optionalFloat(shoulder.reliefDiameter);
				if (shoulder.reliefDiameter == null || !(reliefDiameter > 0) || reliefDiameter >= smaller)
					throw 'Shoulder relief at z=${shoulder.z} has invalid diameter';
			}
			for (j in 0...i)
				if (Math.abs(shoulders[j].z - shoulder.z) < 1e-6) throw 'Duplicate shoulder detail at z=${shoulder.z}';
		}
	}

	function validateChamfer(value:Float, diameter:Float, end:String):Void {
		if (value > 0 && value >= diameter / 2)
			throw 'Shaft $end chamfer must be positive and smaller than its radius';
		if (value < 0 || !Math.isFinite(value)) throw 'Shaft $end chamfer must be positive';
	}

	function validateThread(thread:ShaftThreadEnd, diameter:Float, sectionLength:Float, end:String):Void {
		if (!(thread.diameter > 0) || thread.diameter >= diameter || !Math.isFinite(thread.diameter))
			throw 'Shaft $end thread diameter must be below the shaft diameter';
		if (!(thread.pitch > 0) || !Math.isFinite(thread.pitch)) throw 'Shaft $end thread pitch must be positive';
		if (!(thread.length > 0) || thread.length > sectionLength || !Math.isFinite(thread.length))
			throw 'Shaft $end thread must fit within its end section';
	}

	function shoulderIndex(z:Float):Int {
		var boundary = 0.0;
		for (i in 0...sections.length - 1) {
			boundary += sections[i].length;
			if (Math.abs(boundary - z) < 1e-6) return i;
		}
		return -1;
	}

	function sectionLengthBefore(z:Float):Float
		return sections[shoulderIndex(z)].length;

	function sectionLengthAfter(z:Float):Float
		return sections[shoulderIndex(z) + 1].length;

	static function detailDesignation(detail:ShaftDetail):String {
		var result = "";
		var inputThread = detail.inputThread, outputThread = detail.outputThread;
		if (inputThread != null)
			result += '-TI${Dimension.format(inputThread.diameter)}x${Dimension.format(inputThread.pitch)}x${Dimension.format(inputThread.length)}';
		if (outputThread != null)
			result += '-TO${Dimension.format(outputThread.diameter)}x${Dimension.format(outputThread.pitch)}x${Dimension.format(outputThread.length)}';
		return result;
	}

	static function emptyDetail():ShaftDetail
		return {inputChamfer: null, outputChamfer: null, inputThread: null, outputThread: null, shoulders: null};

	static function optionalFloat(value:Null<Float>):Float
		return value == null ? 0.0 : value;

	static function detailDescription(detail:ShaftDetail):String {
		var result = "";
		if (detail.inputChamfer != null || detail.outputChamfer != null) result += ", chamfered ends";
		if (detail.inputThread != null || detail.outputThread != null) result += ", semantic threaded ends";
		if (detail.shoulders != null && detail.shoulders.length > 0) result += ", detailed shoulders";
		return result;
	}

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var z = 0.0, parts:Array<Part> = [];
		for (section in sections) {
			parts.push(Solids.cylinder(section.diameter / 2, z, z + section.length));
			z += section.length;
		}
		var body = Solids.union(parts);
		if (detail == Envelope) return body;
		body = applyEndDetails(body);
		var tools:Array<Part> = [];
		for (keyway in keyways) tools.push(keywayTool(keyway));
		for (groove in grooves) tools.push(grooveTool(groove));
		for (shoulder in (this.detail.shoulders == null ? [] : this.detail.shoulders))
			if (optionalFloat(shoulder.reliefWidth) > 0)
				tools.push(shoulderReliefTool(shoulder));
		return tools.length == 0 ? body : Solids.cut(body, tools);
	}

	function applyEndDetails(body:Part):Part {
		var result = body;
		var inputThread = detail.inputThread, outputThread = detail.outputThread;
		if (inputThread != null) result = applyThread(result, inputThread, true);
		if (outputThread != null) result = applyThread(result, outputThread, false);
		if (optionalFloat(detail.inputChamfer) > 0)
			result = finishEdge(result, 0, optionalFloat(detail.inputChamfer), false);
		if (optionalFloat(detail.outputChamfer) > 0)
			result = finishEdge(result, totalLength, optionalFloat(detail.outputChamfer), false);
		for (shoulder in (detail.shoulders == null ? [] : detail.shoulders))
			if (optionalFloat(shoulder.fillet) > 0)
				result = finishEdge(result, shoulder.z, optionalFloat(shoulder.fillet), true);
		return result;
	}

	function applyThread(body:Part, thread:ShaftThreadEnd, atInput:Bool):Part {
		var z0 = atInput ? 0 : totalLength - thread.length;
		var z1 = atInput ? thread.length : totalLength;
		var outer = diameterAt(atInput ? 0 : totalLength) / 2 + 1;
		var inner = thread.diameter / 2;
		return cutPart(body, Solids.cut(Solids.cylinder(outer, z0, z1),
			[Solids.cylinder(inner, z0 - 0.05, z1 + 0.05)]));
	}

	function finishEdge(body:Part, z:Float, distance:Float, fillet:Bool):Part {
		var selection = body.edges().curve(CadKit.CurveKind.Circle).filter(function(edge) {
			return Math.abs(Selection.center(edge).z - z) < 1e-6;
		});
		if (selection.count() == 0) {
			selection.close();
			throw 'No circular shaft edge at z=$z for ${fillet ? "fillet" : "chamfer"}';
		}
		try {
			var result = fillet ? body.fillet(selection, distance) : body.chamfer(selection, distance);
			selection.close();
			body.close();
			return result;
		} catch (error:Dynamic) {
			selection.close();
			body.close();
			throw error;
		}
	}

	function cutPart(body:Part, tool:Part):Part {
		try {
			var result = body.subtract(tool);
			body.close();
			tool.close();
			return result;
		} catch (error:Dynamic) {
			body.close();
			tool.close();
			throw error;
		}
	}

	function shoulderReliefTool(shoulder:ShaftShoulderDetail):Part {
		var width = optionalFloat(shoulder.reliefWidth);
		var rawDiameter = shoulder.reliefDiameter;
		if (rawDiameter == null) throw 'Shoulder relief at z=${shoulder.z} needs a diameter';
		var diameter = rawDiameter;
		var adjacent = Math.max(diameterAt(shoulder.z - 1e-5), diameterAt(shoulder.z + 1e-5));
		return Solids.cut(Solids.cylinder(adjacent / 2 + 1, shoulder.z - width / 2, shoulder.z + width / 2),
			[Solids.cylinder(diameter / 2, shoulder.z - width / 2 - 0.05, shoulder.z + width / 2 + 0.05)]);
	}

	/** Annulus from the groove diameter out past the shaft surface. */
	function grooveTool(groove:ShaftGroove):Part {
		var inner = groove.diameter / 2, outer = diameterAt(groove.z0) / 2 + 1, z1 = groove.z0 + groove.width;
		return Solids.revolve([{r: inner, z: groove.z0}, {r: outer, z: groove.z0}, {r: outer, z: z1}, {r: inner, z: z1}]);
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
