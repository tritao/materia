package cadkit.parametric;

import CadKit;
import cadkit.Shape;
import haxe.Json;
import haxeon.wire.JsonWire;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinition.AssemblyComponentDefinition;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyConnector;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/**
	A connector whose frame comes from a face or edge of the component's
	geometry (plan C4.4). The face or edge is found again in the current
	geometry each time the frame is needed: first by its topology fingerprint
	(the same face, up to numerical noise), then, after an edit moved or
	resized it, as the one face or edge with its kind, `direction` and
	`radius` (what a mate on it depends on). The frame's z axis is the
	feature's direction (see `GeometricConnectors.frameOf`); `flip` reverses it.
*/
class GeometricConnector {
	public final name:String;
	public final fingerprint:TopologyFingerprint;
	public final flip:Bool;
	/** The feature's unit direction when captured: a plane's normal, an axis, a line's tangent. */
	public final direction:Array<Float>;
	/** The radius of an axial face or circular edge when captured; zero for planes and lines. */
	public final radius:Float;

	public function new(name:String, fingerprint:TopologyFingerprint, direction:Array<Float>, radius:Float, flip:Bool = false) {
		if (name == null || name.length == 0) throw "A geometric connector needs a name";
		if (fingerprint.kind != CadKit.ShapeKind.Face && fingerprint.kind != CadKit.ShapeKind.Edge)
			throw 'Geometric connector "$name" must reference a face or an edge';
		if (direction == null || direction.length != 3) throw 'Geometric connector "$name" needs a direction';
		this.name = name;
		this.fingerprint = fingerprint;
		this.direction = direction.copy();
		this.radius = radius;
		this.flip = flip;
	}
}

/** Where a face or edge offers a mate its frame, and the properties that identify it across edits. */
private class GeometricFeature {
	public final origin:Array<Float>;
	public final direction:Array<Float>;
	public final radius:Float;
	/** Its direction's sign is the geometry's own (a plane's outward normal), not an arbitrary parametrisation. */
	public final signed:Bool;

	public function new(origin:Array<Float>, direction:Array<Float>, radius:Float, signed:Bool) {
		this.origin = origin;
		this.direction = direction;
		this.radius = radius;
		this.signed = signed;
	}
}

/** A geometric connector that no longer matches one face or edge of its component. */
class GeometricConnectorError {
	public final connector:String;
	public final state:ReferenceState;
	public final message:String;

	public function new(connector:String, state:ReferenceState, message:String) {
		this.connector = connector;
		this.state = state;
		this.message = message;
	}

	public function toString():String return 'Geometric connector "$connector": $message';
}

/** Captures faces and edges as connectors, and turns them back into frames. */
class GeometricConnectors {
	/** Definition property holding a definition's geometric connectors. */
	public static inline var PROPERTY:String = "cadkit.assembly.geometricConnectors";
	static inline var PARALLEL:Float = 1e-9;
	static inline var AMBIGUOUS:Int = -2;
	static inline var DIRECTION_AGREEMENT:Float = 0.999999;
	static inline var RADIUS_TOLERANCE:Float = 1e-6;

	/** A connector on subshape `index` of `kind` in `shape` (the component's unplaced geometry). */
	public static function capture(name:String, shape:Shape, kind:CadKit.ShapeKind, index:Int, flip:Bool = false):GeometricConnector {
		var subshape = shape.subshape(kind, index);
		try {
			var feature = describe(subshape); // refuses what offers no frame before it is persisted
			var fingerprint = TopologyFingerprint.capture(subshape);
			subshape.close();
			return new GeometricConnector(name, fingerprint, feature.direction, feature.radius, flip);
		} catch (error:Dynamic) {
			subshape.close();
			throw error;
		}
	}

	/** The connector's frame in `shape`; throws `GeometricConnectorError` when its face or edge is gone or ambiguous. */
	public static function frame(shape:Shape, connector:GeometricConnector):AssemblyFrame {
		var kind = connector.fingerprint.kind;
		var index = -1;
		var resolution = TopologyResolver.resolve(shape, connector.fingerprint, kind);
		if (resolution.state == ReferenceState.Resolved || resolution.state == ReferenceState.Remapped) index = resolution.index;
		else index = matchByProperties(shape, connector);
		if (index == AMBIGUOUS)
			throw new GeometricConnectorError(connector.name, ReferenceState.Ambiguous, "several faces or edges match it");
		if (index < 0)
			throw new GeometricConnectorError(connector.name, ReferenceState.Unresolved, "its face or edge is no longer in the geometry");
		var feature = describeOwned(shape.subshape(kind, index));
		// An axis or a line has no sign of its own: keep the one captured, so the frame does not turn over.
		var direction = feature.direction;
		if (!feature.signed && dot(direction, connector.direction) < 0) direction = [-direction[0], -direction[1], -direction[2]];
		return basis(feature.origin, connector.flip ? [-direction[0], -direction[1], -direction[2]] : direction);
	}

	/** `describe(subshape)`, closing `subshape` either way. */
	static function describeOwned(subshape:Shape):GeometricFeature {
		try {
			var feature = describe(subshape);
			subshape.close();
			return feature;
		} catch (error:Dynamic) {
			subshape.close();
			throw error;
		}
	}

	/**
		The frame a face or edge offers a mate, z along its direction:
		- planar face: at its area centroid, z its outward normal;
		- cylindrical, conical, spherical or toroidal face: on its axis at the
		  point nearest the face's centroid, z the axis;
		- circular edge: at its center, z its normal;
		- straight edge: at its midpoint, z its tangent.
		x is the world axis least aligned with z, made perpendicular to it.
	**/
	public static function frameOf(subshape:Shape, flip:Bool = false):AssemblyFrame {
		var feature = describe(subshape), direction = feature.direction;
		return basis(feature.origin, flip ? [-direction[0], -direction[1], -direction[2]] : direction);
	}

	static function describe(subshape:Shape):GeometricFeature {
		if (subshape.kind() == CadKit.ShapeKind.Face) {
			var surface = subshape.surfaceKind();
			var center = vector(subshape.center());
			if (surface == CadKit.SurfaceKind.Plane) return new GeometricFeature(center, unit(vector(subshape.faceNormal())), 0, true);
			if (surface == CadKit.SurfaceKind.Cylinder || surface == CadKit.SurfaceKind.Cone ||
				surface == CadKit.SurfaceKind.Sphere || surface == CadKit.SurfaceKind.Torus) {
				var axis = subshape.faceAxis();
				var point = vector(axis.get_origin()), direction = unit(vector(axis.get_direction()));
				var along = (center[0] - point[0]) * direction[0] + (center[1] - point[1]) * direction[1] +
					(center[2] - point[2]) * direction[2];
				return new GeometricFeature([point[0] + along * direction[0], point[1] + along * direction[1], point[2] + along * direction[2]],
					direction, axis.get_radius(), false);
			}
			throw "A geometric connector needs a planar or axially symmetric face";
		}
		if (subshape.kind() == CadKit.ShapeKind.Edge) {
			var curve = subshape.curveKind();
			if (curve == CadKit.CurveKind.Circle || curve == CadKit.CurveKind.Ellipse) {
				var axis = subshape.edgeAxis();
				return new GeometricFeature(vector(axis.get_origin()), unit(vector(axis.get_direction())), axis.get_radius(), false);
			}
			if (curve == CadKit.CurveKind.Line)
				return new GeometricFeature(vector(subshape.positionAt(0.5)), unit(vector(subshape.tangentAt(0.5))), 0, false);
			throw "A geometric connector needs a straight or circular edge";
		}
		throw "A geometric connector needs a face or an edge";
	}

	/** The one subshape with the connector's surface or curve kind, direction and radius; -1 if none, `AMBIGUOUS` if several. */
	static function matchByProperties(shape:Shape, connector:GeometricConnector):Int {
		var fingerprint = connector.fingerprint, kind = fingerprint.kind, found = -1;
		for (index in 0...shape.subshapeCount(kind)) {
			var candidate = shape.subshape(kind, index);
			var matches = false;
			try {
				var sameKind = kind == CadKit.ShapeKind.Face ? candidate.surfaceKind() == fingerprint.surfaceKind
					: candidate.curveKind() == fingerprint.curveKind;
				if (sameKind) {
					var feature = describe(candidate);
					var agreement = dot(feature.direction, connector.direction);
					if (!feature.signed) agreement = Math.abs(agreement);
					matches = agreement >= DIRECTION_AGREEMENT &&
						Math.abs(feature.radius - connector.radius) <= RADIUS_TOLERANCE * Math.max(1, connector.radius);
				}
			} catch (_:Dynamic) {}
			candidate.close();
			if (!matches) continue;
			if (found >= 0) return AMBIGUOUS;
			found = index;
		}
		return found;
	}

	/** `definition` with each component's geometric connectors appended, their frames taken from `geometry(component)`. */
	public static function apply(definition:AssemblyDefinition, connectors:Map<String, Array<GeometricConnector>>,
			geometry:String->Shape):AssemblyDefinition {
		var copy:AssemblyDefinition = JsonWire.decode(JsonWire.encode(definition)); // unvalidated: its mates name the connectors added here
		for (component in copy.definitions) {
			var own = connectors.get(component.id);
			if (own != null && own.length > 0) append(component, own, geometry(component.id));
		}
		materia.assembly.AssemblyDefinitionCodec.validate(copy);
		return copy;
	}

	/** Adds `connectors`, resolved in `shape`, to `component`; a name it already has is refused. */
	public static function append(component:AssemblyComponentDefinition, connectors:Array<GeometricConnector>, shape:Shape):Void {
		for (connector in connectors) {
			for (existing in component.connectors)
				if (existing.name == connector.name)
					throw 'Component "${component.id}" already has a connector named "${connector.name}"';
			var resolved:AssemblyConnector = {name: connector.name, frame: frame(shape, connector)};
			component.connectors.push(resolved);
		}
	}

	/** The geometric connectors stored on a document definition (empty when it has none). */
	public static function read(definition:Definition):Array<GeometricConnector> {
		var property = definition.property(PROPERTY);
		if (property == null) return [];
		var text:String = cast property.value;
		var records:Array<Dynamic> = Json.parse(text);
		var result:Array<GeometricConnector> = [];
		for (record in records) {
			var kind = Reflect.field(record, "kind") == "edge" ? CadKit.ShapeKind.Edge : CadKit.ShapeKind.Face;
			var flip:Null<Bool> = Reflect.field(record, "flip");
			var direction:Array<Float> = Reflect.field(record, "direction");
			var radius:Float = Reflect.field(record, "radius");
			result.push(new GeometricConnector(Reflect.field(record, "name"),
				DocumentCodec.decodeFingerprint(Reflect.field(record, "fingerprint"), kind), direction, radius, flip == true));
		}
		return result;
	}

	/** Stores `connectors` on a document definition, replacing what it had (none removes the property). */
	public static function write(definition:Definition, connectors:Array<GeometricConnector>):Void {
		var names = new Map<String, Bool>();
		for (connector in connectors) {
			if (names.exists(connector.name)) throw 'Geometric connector "${connector.name}" is listed more than once';
			names.set(connector.name, true);
		}
		if (connectors.length == 0) {
			definition.removeProperty(PROPERTY);
			return;
		}
		var records:Array<Dynamic> = [for (connector in connectors) {
			name: connector.name,
			kind: connector.fingerprint.kind == CadKit.ShapeKind.Edge ? "edge" : "face",
			flip: connector.flip,
			direction: connector.direction,
			radius: connector.radius,
			fingerprint: DocumentCodec.encodeFingerprint(connector.fingerprint)
		}];
		definition.setProperty(TypedProperty.text(PROPERTY, Json.stringify(records)));
	}

	static function basis(origin:Array<Float>, z:Array<Float>):AssemblyFrame {
		var length = Math.sqrt(z[0] * z[0] + z[1] * z[1] + z[2] * z[2]);
		if (!(length > PARALLEL)) throw "A geometric connector's direction is degenerate";
		z = [z[0] / length, z[1] / length, z[2] / length];
		// The world axis least aligned with z, made perpendicular to it.
		var ax = Math.abs(z[0]), ay = Math.abs(z[1]), az = Math.abs(z[2]);
		var hint = ax <= ay && ax <= az ? [1.0, 0.0, 0.0] : ay <= az ? [0.0, 1.0, 0.0] : [0.0, 0.0, 1.0];
		var dot = hint[0] * z[0] + hint[1] * z[1] + hint[2] * z[2];
		var x = [hint[0] - dot * z[0], hint[1] - dot * z[1], hint[2] - dot * z[2]];
		var xLength = Math.sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);
		x = [x[0] / xLength, x[1] / xLength, x[2] / xLength];
		var y = [z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2], z[0] * x[1] - z[1] * x[0]];
		return AssemblyFrames.fromRotationMatrix(origin[0], origin[1], origin[2],
			[x[0], y[0], z[0], x[1], y[1], z[1], x[2], y[2], z[2]]);
	}

	static function dot(a:Array<Float>, b:Array<Float>):Float
		return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];

	static function unit(value:Array<Float>):Array<Float> {
		var length = Math.sqrt(dot(value, value));
		if (!(length > PARALLEL)) throw "A geometric connector's direction is degenerate";
		return [value[0] / length, value[1] / length, value[2] / length];
	}

	static function vector(value:CadKit.Vec3):Array<Float>
		return [value.get_x(), value.get_y(), value.get_z()];
}
