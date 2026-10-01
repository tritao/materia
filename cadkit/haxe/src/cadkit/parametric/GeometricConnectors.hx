package cadkit.parametric;

import CadKit;
import cadkit.Shape;
import haxe.Json;
import haxeon.wire.JsonWire;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinition.AssemblyComponentDefinition;
import materia.assembly.AssemblyDefinition.AssemblyMateKind;
import materia.assembly.AssemblyDefinitionCodec;
import materia.assembly.AssemblyDefinitionFlattener;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyConnector;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/**
	What a face or edge defines for a mate, independent of how it is cut:
	- `Plane` (a planar face): a plane and its normal; its centroid is no point of its own;
	- `Axis` (a cylindrical, conical or toroidal face): an axis line; where along it is arbitrary;
	- `Sphere` (a spherical face): a center; no direction of its own;
	- `Circle` (a circular or elliptical edge): a center, a plane and an axis;
	- `Line` (a straight edge): a line; its midpoint moves as it is trimmed.
**/
enum abstract GeometricFeatureKind(String) to String {
	var Plane = "plane";
	var Axis = "axis";
	var Sphere = "sphere";
	var Circle = "circle";
	var Line = "line";
}

/**
	A connector whose frame comes from a face or edge of the component's
	geometry (plan C4.4). In an assembly record it is an ordinary connector
	whose `reference` (see `GeometricConnectors.connector`) says how to find
	the face or edge again: first by its topology fingerprint (the same face,
	up to numerical noise), then, after an edit moved or resized it, as the
	one face or edge with its feature kind, `direction` and `radius` (what a
	mate on it depends on). Its frame's z axis is the feature's direction
	(see `GeometricConnectors.frameOf`); `flip` reverses it.
*/
class GeometricConnector {
	public final name:String;
	public final fingerprint:TopologyFingerprint;
	public final feature:GeometricFeatureKind;
	/** The feature's unit direction when captured: a plane's normal, an axis, a line's tangent. */
	public final direction:Array<Float>;
	/** The radius of an axial face, a sphere or a circular edge when captured; zero for planes and lines. */
	public final radius:Float;
	public final flip:Bool;

	public function new(name:String, fingerprint:TopologyFingerprint, feature:GeometricFeatureKind, direction:Array<Float>,
			radius:Float, flip:Bool = false) {
		if (name == null || name.length == 0) throw "A geometric connector needs a name";
		if (fingerprint.kind != CadKit.ShapeKind.Face && fingerprint.kind != CadKit.ShapeKind.Edge)
			throw 'Geometric connector "$name" must reference a face or an edge';
		if (direction == null || direction.length != 3) throw 'Geometric connector "$name" needs a direction';
		this.name = name;
		this.fingerprint = fingerprint;
		this.feature = feature;
		this.direction = direction.copy();
		this.radius = radius;
		this.flip = flip;
	}
}

/** A geometric connector that cannot be framed or mated: `code` is "unresolved", "ambiguous", "instance-dependent" or "incompatible-mate". */
class GeometricConnectorError {
	public final code:String;
	public final connector:String;
	public final message:String;

	public function new(code:String, connector:String, message:String) {
		this.code = code;
		this.connector = connector;
		this.message = message;
	}

	public function toString():String return 'Geometric connector "$connector": $message';
}

/** Where a face or edge offers a mate its frame, and the properties that identify it across edits. */
private class GeometricFeature {
	public final kind:GeometricFeatureKind;
	public final origin:Array<Float>;
	public final direction:Array<Float>;
	/** The geometry's own direction across `direction` (the frame's x), or null when it has none (a line). */
	public final reference:Null<Array<Float>>;
	public final radius:Float;
	/** Its direction's sign is the geometry's own (a plane's outward normal), not an arbitrary parametrisation. */
	public final signed:Bool;

	public function new(kind:GeometricFeatureKind, origin:Array<Float>, direction:Array<Float>, reference:Null<Array<Float>>,
			radius:Float, signed:Bool) {
		this.kind = kind;
		this.origin = origin;
		this.direction = direction;
		this.reference = reference;
		this.radius = radius;
		this.signed = signed;
	}
}

/** One face or edge a connector may be found on: its index among its shape's faces or edges, its feature and fingerprint. */
private class GeometricCandidate {
	public final index:Int;
	public final feature:GeometricFeature;
	public final fingerprint:TopologyFingerprint;

	public function new(index:Int, feature:GeometricFeature, fingerprint:TopologyFingerprint) {
		this.index = index;
		this.feature = feature;
		this.fingerprint = fingerprint;
	}
}

/**
	The faces and edges of one component's geometry that geometric connectors can be found on: from the
	B-rep (`ofShape`), or from the descriptors a producer wrote for an editor that holds only meshes
	(`ofDescriptors`, see `GeometricConnectors.describeFaces`; faces only). Both are matched alike.
*/
class GeometricCandidates {
	final shape:Null<Shape>;
	final faces:Null<Array<GeometricCandidate>>;
	var edges:Null<Array<GeometricCandidate>>;
	var shapeFaces:Null<Array<GeometricCandidate>>;

	function new(shape:Null<Shape>, faces:Null<Array<GeometricCandidate>>) {
		this.shape = shape;
		this.faces = faces;
	}

	/** Candidates measured on `shape` when first needed; the caller keeps owning `shape`. */
	public static function ofShape(shape:Shape):GeometricCandidates
		return new GeometricCandidates(shape, null);

	public static function ofDescriptors(descriptors:String):GeometricCandidates
		return new GeometricCandidates(null, @:privateAccess GeometricConnectors.parseDescriptors(descriptors));

	@:allow(cadkit.parametric.GeometricConnectors)
	function among(kind:CadKit.ShapeKind):Array<GeometricCandidate> {
		var descriptorFaces = faces;
		if (descriptorFaces != null) return kind == CadKit.ShapeKind.Face ? descriptorFaces : [];
		var source = shape;
		if (source == null) return [];
		if (kind == CadKit.ShapeKind.Face) {
			var known = shapeFaces;
			if (known != null) return known;
			var measured = @:privateAccess GeometricConnectors.measure(source, kind);
			shapeFaces = measured;
			return measured;
		}
		var known = edges;
		if (known != null) return known;
		var measured = @:privateAccess GeometricConnectors.measure(source, kind);
		edges = measured;
		return measured;
	}
}

/** Captures faces and edges as connectors, frames them again from the current geometry, and checks the mates on them. */
class GeometricConnectors {
	/** Marks a connector `reference` as CadKit's geometric connector. */
	static inline var REFERENCE_FORMAT:String = "cadkit.geometric-connector/1";
	static inline var PARALLEL:Float = 1e-9;
	static inline var AMBIGUOUS:Int = -2;
	static inline var DIRECTION_AGREEMENT:Float = 0.999999;
	static inline var RADIUS_TOLERANCE:Float = 1e-6;
	static inline var FRAME_TOLERANCE:Float = 1e-6;

	/** A connector on subshape `index` of `kind` in `shape` (the component's unplaced geometry). */
	public static function capture(name:String, shape:Shape, kind:CadKit.ShapeKind, index:Int, flip:Bool = false):GeometricConnector {
		var subshape = shape.subshape(kind, index);
		try {
			var feature = describe(subshape); // refuses what offers no frame before it is persisted
			var fingerprint = TopologyFingerprint.capture(subshape);
			subshape.close();
			return new GeometricConnector(name, fingerprint, feature.kind, feature.direction, feature.radius, flip);
		} catch (error:Dynamic) {
			subshape.close();
			throw error;
		}
	}

	/** `geometric` as an assembly connector: framed in `shape`, with the reference that frames it again. */
	public static function connector(geometric:GeometricConnector, shape:Shape):AssemblyConnector
		return {name: geometric.name, frame: frame(shape, geometric), reference: referenceText(geometric)};

	static function referenceText(geometric:GeometricConnector):String {
		var record:Dynamic = {
			format: REFERENCE_FORMAT,
			kind: geometric.fingerprint.kind == CadKit.ShapeKind.Edge ? "edge" : "face",
			feature: (geometric.feature : String),
			direction: geometric.direction,
			radius: geometric.radius,
			flip: geometric.flip,
			fingerprint: DocumentCodec.encodeFingerprint(geometric.fingerprint)
		};
		return Json.stringify(record);
	}

	/** The geometric connector behind an assembly connector, or null when it is not one (no reference, or another application's). */
	public static function reference(connector:AssemblyConnector):Null<GeometricConnector> {
		var text = connector.reference;
		if (text == null) return null;
		var record:Dynamic = null;
		try record = Json.parse(text) catch (_:Dynamic) {}
		if (record == null || Reflect.field(record, "format") != REFERENCE_FORMAT) return null;
		var kind = Reflect.field(record, "kind") == "edge" ? CadKit.ShapeKind.Edge : CadKit.ShapeKind.Face;
		var feature:String = Reflect.field(record, "feature");
		var direction:Array<Float> = Reflect.field(record, "direction");
		var radius:Float = Reflect.field(record, "radius");
		var flip:Null<Bool> = Reflect.field(record, "flip");
		return new GeometricConnector(connector.name, DocumentCodec.decodeFingerprint(Reflect.field(record, "fingerprint"), kind),
			featureKind(feature), direction, radius, flip == true);
	}

	/** The connector's frame in `shape`; throws `GeometricConnectorError` when its face or edge is gone or ambiguous. */
	public static function frame(shape:Shape, connector:GeometricConnector):AssemblyFrame
		return frameAmong(GeometricCandidates.ofShape(shape), connector);

	/**
		The connector's frame among `candidates`: its face or edge found by fingerprint (the same one, up to
		numerical noise) or, after an edit moved or resized it, as the one candidate with its feature kind,
		direction and radius. Throws `GeometricConnectorError` when none or several match.
	*/
	public static function frameAmong(candidates:GeometricCandidates, connector:GeometricConnector):AssemblyFrame {
		var list = candidates.among(connector.fingerprint.kind);
		var resolution = TopologyResolver.resolveAmong([for (candidate in list) candidate.fingerprint], connector.fingerprint);
		var found = resolution.state == ReferenceState.Resolved ? resolution.index : matchByProperties(list, connector);
		if (found == AMBIGUOUS) throw new GeometricConnectorError("ambiguous", connector.name, "several faces or edges match it");
		if (found < 0) throw new GeometricConnectorError("unresolved", connector.name, "its face or edge is no longer in the geometry");
		var feature = list[found].feature;
		// An axis or a line has no sign of its own: keep the one captured, so the frame does not turn over.
		var direction = feature.direction;
		if (!feature.signed && dot(direction, connector.direction) < 0) direction = negate(direction);
		return basis(feature.origin, connector.flip ? negate(direction) : direction, feature.reference);
	}

	/**
		What each face of `shape` offers a mate, for an editor that holds only its mesh: a JSON list of the
		faces that have a feature (index, feature, frame data, fingerprint). Read with
		`GeometricCandidates.ofDescriptors`, captured with `captureDescribed`.
	*/
	public static function describeFaces(shape:Shape):String {
		var records:Array<Dynamic> = [for (candidate in measure(shape, CadKit.ShapeKind.Face)) {
			var feature = candidate.feature;
			{
				index: candidate.index,
				feature: (feature.kind : String),
				origin: feature.origin,
				direction: feature.direction,
				reference: feature.reference,
				radius: feature.radius,
				signed: feature.signed,
				fingerprint: DocumentCodec.encodeFingerprint(candidate.fingerprint)
			};
		}];
		return Json.stringify(records);
	}

	/** A connector on face `faceIndex` of a component described by `descriptors` (see `describeFaces`), as an assembly connector. */
	public static function captureDescribed(name:String, descriptors:String, faceIndex:Int, flip:Bool = false):AssemblyConnector {
		var candidates = GeometricCandidates.ofDescriptors(descriptors);
		for (candidate in candidates.among(CadKit.ShapeKind.Face)) if (candidate.index == faceIndex) {
			var geometric = new GeometricConnector(name, candidate.fingerprint, candidate.feature.kind, candidate.feature.direction,
				candidate.feature.radius, flip);
			return {name: name, frame: frameAmong(candidates, geometric), reference: referenceText(geometric)};
		}
		throw new GeometricConnectorError("unresolved", name, 'face $faceIndex offers no frame for a mate');
	}

	/**
		The frame a face or edge offers a mate, z along its direction:
		- planar face: at its area centroid, z its outward normal;
		- cylindrical, conical or toroidal face: on its axis at the point nearest the face's centroid, z the axis;
		- spherical face: at its center;
		- circular edge: at its center, z its normal;
		- straight edge: at its midpoint, z its tangent.
		x is the geometry's own reference direction (its parametrisation's x), or for a straight edge the world
		axis least aligned with z made perpendicular to it.
	**/
	public static function frameOf(subshape:Shape, flip:Bool = false):AssemblyFrame {
		var feature = describe(subshape);
		return basis(feature.origin, flip ? negate(feature.direction) : feature.direction, feature.reference);
	}

	/**
		`definition` with every geometric connector framed again from `geometry(scope, component)`: the faces
		and edges of the unplaced geometry of that component's occurrences in that scope ("" for the root, else the subdefinition's id), all of
		which must place it alike. A component with no shapes keeps its last frames. The mates are checked against
		the features they name (see `checkMates`). Throws `GeometricConnectorError`.
	*/
	public static function reframe(definition:AssemblyDefinition, geometry:(String, String)->Array<GeometricCandidates>):AssemblyDefinition {
		var copy:AssemblyDefinition = JsonWire.decode(JsonWire.encode(definition));
		reframeScope("", copy.definitions, geometry);
		var nestedScopes = copy.assemblies;
		if (nestedScopes != null) for (nested in nestedScopes) reframeScope(nested.id, nested.definitions, geometry);
		AssemblyDefinitionCodec.validate(copy);
		checkMates(copy);
		return copy;
	}

	/** Throws `GeometricConnectorError` for a mate that asks a geometric connector for what its feature does not define. */
	public static function checkMates(definition:AssemblyDefinition):Void {
		var flat = AssemblyDefinitionFlattener.flatten(definition);
		var mates = flat.mates;
		if (mates == null) return;
		var components = new Map<String, AssemblyComponentDefinition>();
		for (component in flat.definitions) components.set(component.id, component);
		var definitionOf = new Map<String, String>();
		for (occurrence in flat.occurrences) definitionOf.set(occurrence.id, occurrence.definition);
		for (mate in mates)
			for (first in [true, false]) {
				var occurrence = first ? mate.first : mate.second, name = first ? mate.firstConnector : mate.secondConnector;
				var componentId = definitionOf.get(occurrence);
				var component = componentId == null ? null : components.get(componentId);
				if (component == null) continue;
				for (connector in component.connectors) if (connector.name == name) {
					var geometric = reference(connector);
					if (geometric != null && !compatible(geometric.feature, mate.kind))
						throw new GeometricConnectorError("incompatible-mate", name,
							'mate "${mate.id}" (${mate.kind}) needs ${needs(mate.kind)}, which a ${geometric.feature} does not define');
				}
			}
	}

	/**
		Whether a mate of `kind` can use a connector on a `feature` (on either side: a mate uses both connectors
		alike). Coincident and distance need a point (a sphere's or circle's center); coaxial an axis line;
		planar a plane (a planar face, or a circle's plane); parallel, perpendicular and angle a direction; lock
		a whole frame (only a circle's is the geometry's own).
	*/
	public static function compatible(feature:GeometricFeatureKind, kind:AssemblyMateKind):Bool {
		return switch kind {
			case AssemblyMateKind.Coincident, AssemblyMateKind.Distance: feature == Sphere || feature == Circle;
			case AssemblyMateKind.Coaxial: feature == Axis || feature == Circle || feature == Line;
			case AssemblyMateKind.Planar: feature == Plane || feature == Circle;
			case AssemblyMateKind.Parallel, AssemblyMateKind.Perpendicular, AssemblyMateKind.Angle: feature != Sphere;
			case AssemblyMateKind.Lock: feature == Circle;
			default: false;
		};
	}

	static function needs(kind:AssemblyMateKind):String
		return switch kind {
			case AssemblyMateKind.Coincident, AssemblyMateKind.Distance: "a point";
			case AssemblyMateKind.Coaxial: "an axis line";
			case AssemblyMateKind.Planar: "a plane";
			case AssemblyMateKind.Lock: "a whole frame";
			default: "a direction";
		};

	static function reframeScope(scope:String, components:Array<AssemblyComponentDefinition>,
			geometry:(String, String)->Array<GeometricCandidates>):Void {
		for (component in components) {
			var shapes:Array<GeometricCandidates> = [];
			var fetched = false;
			for (index in 0...component.connectors.length) {
				var connector = component.connectors[index];
				var geometric = reference(connector);
				if (geometric == null) continue;
				if (!fetched) {
					shapes = geometry(scope, component.id);
					fetched = true;
				}
				if (shapes.length == 0) continue;
				var resolved = frameAmong(shapes[0], geometric);
				for (k in 1...shapes.length)
					if (!sameFrame(resolved, frameAmong(shapes[k], geometric)))
						throw new GeometricConnectorError("instance-dependent", connector.name,
							'occurrences of "${component.id}" place it differently');
				component.connectors[index] = {name: connector.name, frame: resolved, reference: connector.reference};
			}
		}
	}

	static function describe(subshape:Shape):GeometricFeature {
		if (subshape.kind() == CadKit.ShapeKind.Face) {
			var surface = subshape.surfaceKind();
			var center = vector(subshape.center());
			if (surface == CadKit.SurfaceKind.Plane) {
				var plane = subshape.faceAxis();
				return new GeometricFeature(Plane, center, unit(vector(subshape.faceNormal())), unit(vector(plane.get_reference())), 0, true);
			}
			if (surface == CadKit.SurfaceKind.Sphere) {
				var sphere = subshape.faceAxis();
				return new GeometricFeature(Sphere, vector(sphere.get_origin()), unit(vector(sphere.get_direction())),
					unit(vector(sphere.get_reference())), sphere.get_radius(), false);
			}
			if (surface == CadKit.SurfaceKind.Cylinder || surface == CadKit.SurfaceKind.Cone || surface == CadKit.SurfaceKind.Torus) {
				var axis = subshape.faceAxis();
				var point = vector(axis.get_origin()), direction = unit(vector(axis.get_direction()));
				var along = (center[0] - point[0]) * direction[0] + (center[1] - point[1]) * direction[1] +
					(center[2] - point[2]) * direction[2];
				return new GeometricFeature(Axis, [point[0] + along * direction[0], point[1] + along * direction[1], point[2] + along * direction[2]],
					direction, unit(vector(axis.get_reference())), axis.get_radius(), false);
			}
			throw "A geometric connector needs a planar or axially symmetric face";
		}
		if (subshape.kind() == CadKit.ShapeKind.Edge) {
			var curve = subshape.curveKind();
			if (curve == CadKit.CurveKind.Circle || curve == CadKit.CurveKind.Ellipse) {
				var axis = subshape.edgeAxis();
				return new GeometricFeature(Circle, vector(axis.get_origin()), unit(vector(axis.get_direction())),
					unit(vector(axis.get_reference())), axis.get_radius(), false);
			}
			if (curve == CadKit.CurveKind.Line)
				return new GeometricFeature(Line, vector(subshape.positionAt(0.5)), unit(vector(subshape.tangentAt(0.5))), null, 0, false);
			throw "A geometric connector needs a straight or circular edge";
		}
		throw "A geometric connector needs a face or an edge";
	}

	/** The one candidate with the connector's feature kind, direction and radius; -1 if none, `AMBIGUOUS` if several. */
	static function matchByProperties(candidates:Array<GeometricCandidate>, connector:GeometricConnector):Int {
		var found = -1;
		for (position in 0...candidates.length) {
			var feature = candidates[position].feature;
			if (feature.kind != connector.feature) continue;
			var agreement = dot(feature.direction, connector.direction);
			if (!feature.signed) agreement = Math.abs(agreement);
			if (agreement < DIRECTION_AGREEMENT ||
				Math.abs(feature.radius - connector.radius) > RADIUS_TOLERANCE * Math.max(1, connector.radius))
				continue;
			if (found >= 0) return AMBIGUOUS;
			found = position;
		}
		return found;
	}

	/** Every face or edge of `shape` that offers a frame, with its feature and fingerprint. */
	static function measure(shape:Shape, kind:CadKit.ShapeKind):Array<GeometricCandidate> {
		var result:Array<GeometricCandidate> = [];
		for (index in 0...shape.subshapeCount(kind)) {
			var subshape = shape.subshape(kind, index);
			try {
				result.push(new GeometricCandidate(index, describe(subshape), TopologyFingerprint.capture(subshape)));
			} catch (_:Dynamic) {}
			subshape.close();
		}
		return result;
	}

	static function parseDescriptors(text:String):Array<GeometricCandidate> {
		var records:Array<Dynamic> = Json.parse(text);
		return [for (record in records) {
			var origin:Array<Float> = Reflect.field(record, "origin"), direction:Array<Float> = Reflect.field(record, "direction");
			var reference:Null<Array<Float>> = Reflect.field(record, "reference");
			var radius:Float = Reflect.field(record, "radius"), signed:Bool = Reflect.field(record, "signed") == true;
			var index:Int = Reflect.field(record, "index"), feature:String = Reflect.field(record, "feature");
			new GeometricCandidate(index, new GeometricFeature(featureKind(feature), origin, direction, reference, radius, signed),
				DocumentCodec.decodeFingerprint(Reflect.field(record, "fingerprint"), CadKit.ShapeKind.Face));
		}];
	}

	static function featureKind(name:String):GeometricFeatureKind
		return switch name {
			case "plane": Plane;
			case "axis": Axis;
			case "sphere": Sphere;
			case "circle": Circle;
			case "line": Line;
			default: throw 'Unknown geometric feature "$name"';
		};

	static function sameFrame(a:AssemblyFrame, b:AssemblyFrame):Bool {
		var agreement = Math.abs(a.qx * b.qx + a.qy * b.qy + a.qz * b.qz + a.qw * b.qw);
		return Math.abs(a.x - b.x) <= FRAME_TOLERANCE && Math.abs(a.y - b.y) <= FRAME_TOLERANCE &&
			Math.abs(a.z - b.z) <= FRAME_TOLERANCE && agreement >= 1 - FRAME_TOLERANCE;
	}

	/** A right-handed frame at `origin` with z along `z` and x along `hint` made perpendicular to z (or the world axis least aligned with z). */
	static function basis(origin:Array<Float>, z:Array<Float>, hint:Null<Array<Float>>):AssemblyFrame {
		z = unit(z);
		var x:Null<Array<Float>> = null;
		if (hint != null) x = across(hint, z);
		if (x == null) {
			var ax = Math.abs(z[0]), ay = Math.abs(z[1]), az = Math.abs(z[2]);
			x = across(ax <= ay && ax <= az ? [1.0, 0.0, 0.0] : ay <= az ? [0.0, 1.0, 0.0] : [0.0, 0.0, 1.0], z);
		}
		var xs:Array<Float> = x == null ? [1.0, 0.0, 0.0] : x;
		var y = [z[1] * xs[2] - z[2] * xs[1], z[2] * xs[0] - z[0] * xs[2], z[0] * xs[1] - z[1] * xs[0]];
		return AssemblyFrames.fromRotationMatrix(origin[0], origin[1], origin[2],
			[xs[0], y[0], z[0], xs[1], y[1], z[1], xs[2], y[2], z[2]]);
	}

	/** `value` without its component along unit `z`, normalised; null when `value` is (nearly) along z. */
	static function across(value:Array<Float>, z:Array<Float>):Null<Array<Float>> {
		var along = dot(value, z);
		var x = [value[0] - along * z[0], value[1] - along * z[1], value[2] - along * z[2]];
		var length = Math.sqrt(dot(x, x));
		return length > 1e-6 ? [x[0] / length, x[1] / length, x[2] / length] : null;
	}

	static function dot(a:Array<Float>, b:Array<Float>):Float
		return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];

	static function negate(value:Array<Float>):Array<Float>
		return [-value[0], -value[1], -value[2]];

	static function unit(value:Array<Float>):Array<Float> {
		var length = Math.sqrt(dot(value, value));
		if (!(length > PARALLEL)) throw "A geometric connector's direction is degenerate";
		return [value[0] / length, value[1] / length, value[2] / length];
	}

	static function vector(value:CadKit.Vec3):Array<Float>
		return [value.get_x(), value.get_y(), value.get_z()];
}
