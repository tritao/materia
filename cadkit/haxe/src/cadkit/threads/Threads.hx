package cadkit.threads;

import CadKit;
import cadkit.ElementNames;
import cadkit.Shape;
import cadkit.modeling.Model;
import cadkit.modeling.Location;
import cadkit.modeling.Part;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import cadkit.threads.ThreadedFace.ThreadStart;

/** A named face's visible thread interval and its frame, derived from the current B-rep. */
class ResolvedThread {
	public final attribute:ThreadedFace;
	public final faceIndex:Int;
	public final frame:Plane;
	public final length:Float;
	public final radius:Float;
	public final partial:Bool;

	public function new(attribute:ThreadedFace, faceIndex:Int, frame:Plane, length:Float, radius:Float, partial:Bool) {
		this.attribute = attribute;
		this.faceIndex = faceIndex;
		this.frame = frame;
		this.length = length;
		this.radius = radius;
		this.partial = partial;
	}
}

/** Thread identity is topological. Dimensions never select or repair a threaded face. */
class Threads {
	public static function namedIndices(shape:Shape, name:String):Array<Int> {
		if (!ElementNames.isStrong(name)) throw 'Thread face name is weak: $name';
		var names = shape.elementNames(CadKit.ShapeKind.Face);
		var aliases = shape.elementAliases(CadKit.ShapeKind.Face);
		var matches = ElementNames.match(name, names);
		var indices:Array<Int> = [];
		for (index in 0...names.length) {
			var found = matches[index].grade == ElementNames.EXACT;
			if (!found && aliases[index].length > 0)
				for (match in ElementNames.match(name, aliases[index]))
					if (match.grade == ElementNames.EXACT) found = true;
			if (found) indices.push(index);
		}
		return indices;
	}

	public static function namedIndex(shape:Shape, name:String):Int {
		var indices = namedIndices(shape, name);
		if (indices.length != 1)
			throw 'Thread face $name is ${indices.length == 0 ? "missing" : "ambiguous"}';
		return indices[0];
	}

	/** Bounds along the actual surface axis, independent of world placement and tessellation. */
	public static function extent(face:Shape):{minimum:Float, maximum:Float} {
		var kind = face.surfaceKind();
		if (kind != CadKit.SurfaceKind.Cylinder && kind != CadKit.SurfaceKind.Cone)
			throw "Thread needs a cylindrical or conical face";
		var axis = face.faceAxis();
		var origin = Vector.fromNative(axis.get_origin());
		var direction = Vector.fromNative(axis.get_direction());
		var aligned = new Location(new Plane(origin, Vector.fromNative(axis.get_reference()), direction)).inverse().apply(face);
		var bounds:CadKit.Bounds;
		try { bounds = aligned.bounds(); aligned.close(); }
		catch (error:Dynamic) { aligned.close(); throw error; }
		var minimum = bounds.get_min().get_z(), maximum = bounds.get_max().get_z();
		if (!Math.isFinite(minimum) || maximum <= minimum) throw "Thread face has no axial extent";
		return {minimum: minimum, maximum: maximum};
	}

	/** Bind once at creation. Cuts later clip this interval without moving its start. */
	public static function attach(shape:Shape, name:String, spec:ThreadSpec, start:ThreadStart,
			length:Null<Float> = null, offset:Float = 0):Shape {
		if (!Math.isFinite(offset) || offset < 0) throw 'Invalid thread offset on $name';
		var face = shape.subshape(CadKit.ShapeKind.Face, namedIndex(shape, name));
		var span:{minimum:Float, maximum:Float};
		try { span = extent(face); face.close(); }
		catch (error:Dynamic) { face.close(); throw error; }
		var direction = start == AxisMinimum ? 1 : -1;
		var parameter = (direction == 1 ? span.minimum : span.maximum) + direction * offset;
		var attributes = shape.threadedFaces();
		attributes.push(new ThreadedFace(name, spec, parameter, direction, length));
		var copy = shape.cloneShape();
		copy.setThreadedFaces(attributes);
		return copy;
	}

	/** Common tap-drill tool, from z=0 into -Z. Its name survives into the cut. */
	public static function tappedBore(spec:ThreadSpec, tapDrill:Float, depth:Float):Part {
		if (!spec.internal) throw "Tapped bore needs an internal thread";
		if (!Math.isFinite(tapDrill) || tapDrill <= 0 || tapDrill >= spec.majorDiameter)
			throw 'Tap drill for ${spec.designation} must be positive and below its major diameter';
		var base = Part.cylinderSpan(tapDrill / 2, -depth, 0);
		var named = base.named("bore");
		base.close();
		try {
			var result = new Part(attach(named.shape, "bore:cyl.side", spec, AxisMaximum, depth));
			named.close();
			return result;
		} catch (error:Dynamic) {
			named.close();
			throw error;
		}
	}

	public static function of(part:Model):Array<ResolvedThread> return ofShape(part.shape);

	public static function ofShape(shape:Shape):Array<ResolvedThread> {
		var result:Array<ResolvedThread> = [];
		for (attribute in shape.threadedFaces()) {
			var index = namedIndex(shape, attribute.faceName);
			var face = shape.subshape(CadKit.ShapeKind.Face, index);
			try {
				var span = extent(face), axis = face.faceAxis();
				var end = attribute.length == null ? (attribute.direction == 1 ? span.maximum : span.minimum)
					: attribute.start + attribute.direction * attribute.length;
				var minimum = Math.max(span.minimum, Math.min(attribute.start, end));
				var maximum = Math.min(span.maximum, Math.max(attribute.start, end));
				if (maximum > minimum + 1e-8) {
					var start = attribute.direction == 1 ? minimum : maximum;
					var direction = Vector.fromNative(axis.get_direction());
					var origin = Vector.fromNative(axis.get_origin()).add(direction.scale(start));
					var frame = new Plane(origin, Vector.fromNative(axis.get_reference()), direction.scale(attribute.direction));
					result.push(new ResolvedThread(attribute, index, frame, maximum - minimum, axis.get_radius(),
						minimum > span.minimum + 1e-7 || maximum < span.maximum - 1e-7));
				}
				face.close();
			} catch (error:Dynamic) {
				face.close();
				throw 'Thread ${attribute.faceName}: $error';
			}
		}
		return result;
	}
}
