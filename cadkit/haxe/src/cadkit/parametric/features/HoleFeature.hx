package cadkit.parametric.features;

import cadkit.parametric.ParameterKind;
import cadkit.Shape;
import cadkit.modeling.Location;
import cadkit.modeling.Axis;
import cadkit.modeling.Plane;
import cadkit.modeling.Part;
import cadkit.modeling.Sketch;
import cadkit.modeling.Vector;
import cadkit.parametric.EvaluationContext;
import cadkit.parametric.EvaluationResult;
import cadkit.parametric.Feature;
import cadkit.parametric.FeatureId;
import cadkit.parametric.Parameter;
import cadkit.parametric.ParametricError;
import cadkit.parametric.SelectionRecipe;
import cadkit.sketch.FaceWorkplane;

/** Face-attached cutting tool for plain and counterbored holes. */
class HoleFeature extends Feature {
	public final target:Feature;
	public final selection:SelectionRecipe;
	public final xDirection:Vector;
	public final offset:Float;
	public final flipped:Bool;
	public final style:String;
	public final mode:String;
	public final x:Parameter;
	public final y:Parameter;
	public final diameter:Parameter;
	public final depth:Parameter;
	public final recessDiameter:Parameter;
	public final recessDepth:Parameter;
	public final includedAngle:Parameter;

	public function new(target:Feature, selection:SelectionRecipe, xDirection:Vector, style:String, mode:String,
		xValue:Float, yValue:Float, diameterValue:Float, depthValue:Float = 1,
		recessDiameterValue:Float = 1, recessDepthValue:Float = 1, offset:Float = 0, flipped:Bool = false,
		includedAngleValue:Float = Math.PI / 2) {
		super();
		if (style != "plain" && style != "counterbore" && style != "countersink")
			throw new ParametricError("hole style must be plain, counterbore, or countersink");
		if (mode != "blind" && mode != "through-all")
			throw new ParametricError("hole mode must be blind or through-all");
		if (!Math.isFinite(offset))
			throw new ParametricError("hole attachment offset must be finite");
		this.target = target;
		this.selection = selection;
		this.xDirection = xDirection;
		this.offset = offset;
		this.flipped = flipped;
		this.style = style;
		this.mode = mode;
		x = new Parameter(this, "hole.x", xValue, -1e300, false, 1e300, ParameterKind.Length);
		y = new Parameter(this, "hole.y", yValue, -1e300, false, 1e300, ParameterKind.Length);
		diameter = new Parameter(this, "hole.diameter", diameterValue, 0, false, 1e300, ParameterKind.Length);
		depth = new Parameter(this, "hole.depth", depthValue, 0, false, 1e300, ParameterKind.Length);
		recessDiameter = new Parameter(this, "hole.recessDiameter", recessDiameterValue, 0, false, 1e300, ParameterKind.Length);
		recessDepth = new Parameter(this, "hole.recessDepth", recessDepthValue, 0, false, 1e300, ParameterKind.Length);
		includedAngle = new Parameter(this, "hole.includedAngle", includedAngleValue, 0, false, 1e300, ParameterKind.Angle);
	}

	public static function plain(target:Feature, selection:SelectionRecipe, xDirection:Vector, mode:String,
		x:Float, y:Float, diameter:Float, depth:Float = 1, offset:Float = 0, flipped:Bool = false):HoleFeature {
		return new HoleFeature(target, selection, xDirection, "plain", mode, x, y, diameter, depth, 1, 1, offset, flipped);
	}

	public static function counterbore(target:Feature, selection:SelectionRecipe, xDirection:Vector, mode:String,
		x:Float, y:Float, diameter:Float, depth:Float, recessDiameter:Float, recessDepth:Float,
		offset:Float = 0, flipped:Bool = false):HoleFeature {
		return new HoleFeature(target, selection, xDirection, "counterbore", mode, x, y, diameter, depth,
			recessDiameter, recessDepth, offset, flipped);
	}

	public static function countersink(target:Feature, selection:SelectionRecipe, xDirection:Vector, mode:String,
		x:Float, y:Float, diameter:Float, depth:Float, mouthDiameter:Float, includedAngle:Float,
		offset:Float = 0, flipped:Bool = false):HoleFeature {
		return new HoleFeature(target, selection, xDirection, "countersink", mode, x, y, diameter, depth,
			mouthDiameter, 1, offset, flipped, includedAngle);
	}

	override public function serializationType():String {
		return "hole";
	}

	override public function dependencies():Array<FeatureId> {
		return [target.id];
	}

	override public function dependencyFeatures():Array<Feature> {
		return [target];
	}

	override public function evaluate(context:EvaluationContext):EvaluationResult {
		if (style == "counterbore") {
			if (recessDiameter.value <= diameter.value)
				throw new ParametricError("counterbore recess diameter must exceed the bore diameter");
			if (mode == "blind" && recessDepth.value >= depth.value)
				throw new ParametricError("counterbore recess depth must be less than the blind hole depth");
		}
		var targetShape = context.shape(target);
		var plane = FaceWorkplane.resolve(targetShape, selection, xDirection, offset, flipped);
		var span = projectedSpan(targetShape, plane.normal);
		var margin = Math.max(1e-7 * Math.max(1, span), 1e-7);
		var boreLength = mode == "blind" ? depth.value + margin : span + 2 * margin;
		var bore = cylinderAt(plane, diameter.value / 2, boreLength, margin);
		if (style == "plain")
			return EvaluationResult.fromShape(bore);
		if (style == "countersink") {
			if (recessDiameter.value <= diameter.value)
				throwInvalidCountersink(bore, "countersink mouth diameter must exceed the bore diameter");
			if (includedAngle.value <= 0 || includedAngle.value >= Math.PI)
				throwInvalidCountersink(bore, "countersink included angle must be between zero and pi radians");
			var sinkDepth = (recessDiameter.value - diameter.value) / 2 / Math.tan(includedAngle.value / 2);
			var availableDepth = mode == "blind" ? depth.value : span;
			if (!Math.isFinite(sinkDepth) || sinkDepth <= 0 || sinkDepth >= availableDepth)
				throwInvalidCountersink(bore, "countersink dimensions must define a recess shallower than the hole");
			var sink:Null<Shape> = null;
			try {
				sink = countersinkAt(plane, diameter.value / 2, recessDiameter.value / 2, sinkDepth, margin);
				var result = bore.fuse(sink);
				bore.close();
				sink.close();
				return EvaluationResult.fromShape(result);
			} catch (error:Dynamic) {
				bore.close();
				if (sink != null)
					sink.close();
				throw error;
			}
		}
		var recess:Null<Shape> = null;
		try {
			recess = cylinderAt(plane, recessDiameter.value / 2, recessDepth.value + margin, margin);
			var result = bore.fuse(recess);
			bore.close();
			recess.close();
			return EvaluationResult.fromShape(result);
		} catch (error:Dynamic) {
			bore.close();
			if (recess != null)
				recess.close();
			throw error;
		}
	}

	private static function throwInvalidCountersink(bore:Shape, message:String):Void {
		bore.close();
		throw new ParametricError(message);
	}

	private function countersinkAt(plane:Plane, boreRadius:Float, mouthRadius:Float, sinkDepth:Float, margin:Float):Shape {
		var profile:Null<Sketch> = null;
		var revolved:Null<Part> = null;
		try {
			profile = Sketch.polygon([
				new Vector(0, 0), new Vector(mouthRadius, 0), new Vector(mouthRadius, margin),
				new Vector(boreRadius, sinkDepth + margin), new Vector(0, sinkDepth + margin)
			], Plane.XZ());
			revolved = profile.revolve(Axis.Z());
			var localOrigin = plane.toWorld(new Vector(x.value, y.value, 0)).add(plane.normal.scale(margin));
			var placed = new Location(new Plane(localOrigin, plane.xDirection, plane.normal.scale(-1))).apply(revolved.shape);
			revolved.close();
			profile.close();
			return placed;
		} catch (error:Dynamic) {
			if (revolved != null)
				revolved.close();
			if (profile != null)
				profile.close();
			throw error;
		}
	}

	private function cylinderAt(plane:Plane, radius:Float, length:Float, margin:Float):Shape {
		var localOrigin = plane.toWorld(new Vector(x.value, y.value, 0)).add(plane.normal.scale(margin));
		var cylinder = Shape.cylinder(radius, length);
		try {
			var placed = new Location(new Plane(localOrigin, plane.xDirection, plane.normal.scale(-1))).apply(cylinder);
			cylinder.close();
			return placed;
		} catch (error:Dynamic) {
			cylinder.close();
			throw error;
		}
	}

	private static function projectedSpan(shape:Shape, direction:Vector):Float {
		var bounds = shape.bounds();
		var minimum = bounds.get_min();
		var maximum = bounds.get_max();
		return Math.abs(direction.x) * (maximum.get_x() - minimum.get_x())
			+ Math.abs(direction.y) * (maximum.get_y() - minimum.get_y())
			+ Math.abs(direction.z) * (maximum.get_z() - minimum.get_z());
	}
}
