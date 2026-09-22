package cadkit.parametric.recording;

import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import cadkit.parametric.Document;
import cadkit.parametric.Feature;
import cadkit.parametric.NamedParameter;
import cadkit.parametric.ParametricError;
import cadkit.parametric.SelectionRecipe;
import cadkit.parametric.features.BooleanFeature;
import cadkit.parametric.features.BooleanOperation;
import cadkit.parametric.features.BoxFeature;
import cadkit.parametric.features.ChamferFeature;
import cadkit.parametric.features.ExtrudeFeature;
import cadkit.parametric.features.FilletFeature;
import cadkit.parametric.features.GridFeature;
import cadkit.parametric.features.LoftFeature;
import cadkit.parametric.features.OffsetFeature;
import cadkit.parametric.features.PolylineFeature;
import cadkit.parametric.features.ProjectFeature;
import cadkit.parametric.features.RevolveFeature;
import cadkit.parametric.features.ShellFeature;
import cadkit.parametric.features.SketchFeature;
import cadkit.parametric.features.SweepFeature;
import cadkit.parametric.features.TransformFeature;
import cadkit.parametric.features.WireFeature;
import cadkit.parametric.features.ConstrainedSketchFeature;
import cadkit.sketch.ConstrainedSketch;

/** Explicit recorder for the serializable document feature set.
 * Methods borrow features and dimensions and return document-owned features.
 */
class DocumentBuilder {
	private final document:Document;
	private var finished:Bool;

	public function new() {
		document = new Document();
		finished = false;
	}

	private function check():Void {
		if (finished)
			throw new ParametricError("document builder is closed");
	}

	public function dimension(name:String, value:Float):NamedParameter {
		check();
		return document.defineParameter(name, value);
	}

	private function bind(dimension:NamedParameter, feature:Feature, slot:String):Void {
		if (dimension.document != document)
			throw new ParametricError("named parameter belongs to another builder");
		dimension.bind(feature.parameter(slot));
	}

	public function rectangle(width:NamedParameter, height:NamedParameter, ?plane:Plane):SketchFeature {
		check();
		var feature = document.add(new SketchFeature("rectangle", width.value, height.value, plane));
		bind(width, feature, "sketch.width");
		bind(height, feature, "sketch.height");
		return feature;
	}

	public function circle(radius:NamedParameter, ?plane:Plane):SketchFeature {
		check();
		var feature = document.add(new SketchFeature("circle", radius.value, 1, plane));
		bind(radius, feature, "sketch.width");
		return feature;
	}

	public function slot(length:NamedParameter, width:NamedParameter, ?plane:Plane):SketchFeature {
		check();
		var feature = document.add(new SketchFeature("slot", length.value, width.value, plane));
		bind(length, feature, "sketch.width");
		bind(width, feature, "sketch.height");
		return feature;
	}

	/** Record a constrained sketch and bind dimension constraint IDs to named parameters. */
	public function constrainedSketch(sketch:ConstrainedSketch, dimensions:Map<String, NamedParameter>):ConstrainedSketchFeature {
		check();
		var feature=document.add(new ConstrainedSketchFeature(sketch));
		for(constraintId in dimensions.keys()) {
			var dimension=dimensions.get(constraintId);
			if(dimension==null||dimension.document!=document)throw new ParametricError("named parameter belongs to another builder");
			dimension.bind(feature.dimension(constraintId));
		}
		return feature;
	}

	public function attachedConstrainedSketch(sketch:ConstrainedSketch, support:Feature, selection:SelectionRecipe,
		xDirection:Vector, dimensions:Map<String, NamedParameter>, offset:Float = 0, flip:Bool = false):ConstrainedSketchFeature {
		check();
		var feature = document.add(new ConstrainedSketchFeature(sketch, support, selection, xDirection, offset, flip));
		for (constraintId in dimensions.keys()) {
			var dimension = dimensions.get(constraintId);
			if (dimension == null || dimension.document != document)
				throw new ParametricError("named parameter belongs to another builder");
			dimension.bind(feature.dimension(constraintId));
		}
		return feature;
	}

	public function box(width:NamedParameter, depth:NamedParameter, height:NamedParameter):BoxFeature {
		check();
		var feature = document.add(new BoxFeature(width.value, depth.value, height.value));
		bind(width, feature, "box.width");
		bind(depth, feature, "box.depth");
		bind(height, feature, "box.height");
		return feature;
	}

	public function wire(source:Feature):WireFeature {
		check();
		return document.add(new WireFeature(source));
	}

	public function polyline(points:Array<Vector>, closed:Bool = false):PolylineFeature {
		check();
		return document.add(new PolylineFeature(points, closed));
	}

	public function grid(source:Feature, columns:Int, rows:Int, spacingX:NamedParameter, spacingY:NamedParameter):GridFeature {
		check();
		var feature = document.add(new GridFeature(source, columns, rows, spacingX.value, spacingY.value));
		bind(spacingX, feature, "grid.spacingX");
		bind(spacingY, feature, "grid.spacingY");
		return feature;
	}

	public function add(first:Feature, second:Feature):BooleanFeature {
		check();
		return document.add(new BooleanFeature(first, second, BooleanOperation.Fuse));
	}

	public function subtract(first:Feature, second:Feature):BooleanFeature {
		check();
		return document.add(new BooleanFeature(first, second, BooleanOperation.Cut));
	}

	public function intersect(first:Feature, second:Feature):BooleanFeature {
		check();
		return document.add(new BooleanFeature(first, second, BooleanOperation.Common));
	}

	public function extrude(source:Feature, amount:NamedParameter, ?direction:Vector, reversed:Bool = false,
		symmetric:Bool = false):ExtrudeFeature {
		check();
		if (direction == null) {
			if (Std.isOfType(source, ConstrainedSketchFeature)) {
				var constrained:ConstrainedSketchFeature = cast source;
				direction = constrained.sketch().plane.normal;
			} else if (Std.isOfType(source, SketchFeature)) {
				var primitive:SketchFeature = cast source;
				direction = primitive.plane.normal;
			} else {
				direction = Vector.Z();
			}
		}
		var feature = document.add(ExtrudeFeature.along(source, amount.value, direction, reversed, symmetric));
		bind(amount, feature, "extrude.amount");
		return feature;
	}

	public function translate(source:Feature, x:Float, y:Float, z:Float):TransformFeature {
		check();
		return document.add(new TransformFeature(source, x, y, z));
	}

	public function revolve(source:Feature, origin:Vector, axis:Vector, angle:NamedParameter):RevolveFeature {
		check();
		var feature = document.add(new RevolveFeature(source, origin.x, origin.y, origin.z, axis.x, axis.y, axis.z, angle.value));
		bind(angle, feature, "revolve.angle");
		return feature;
	}

	public function loft(sections:Array<Feature>, ruled:Bool = false):LoftFeature {
		check();
		return document.add(new LoftFeature(sections, ruled));
	}

	public function sweep(profile:Feature, path:Feature):SweepFeature {
		check();
		return document.add(new SweepFeature(profile, path));
	}

	public function offset(source:Feature, distance:NamedParameter):OffsetFeature {
		check();
		var feature = document.add(new OffsetFeature(source, distance.value));
		bind(distance, feature, "offset.distance");
		return feature;
	}

	public function shell(source:Feature, thickness:NamedParameter, selection:SelectionRecipe):ShellFeature {
		check();
		var feature = document.add(new ShellFeature(source, thickness.value, selection));
		bind(thickness, feature, "shell.thickness");
		return feature;
	}

	public function project(source:Feature, target:Feature, direction:Vector):ProjectFeature {
		check();
		return document.add(new ProjectFeature(source, target, direction));
	}

	public function fillet(source:Feature, radius:NamedParameter, selection:SelectionRecipe):FilletFeature {
		check();
		var feature = document.add(new FilletFeature(source, radius.value, null, null, selection));
		bind(radius, feature, "fillet.radius");
		return feature;
	}

	public function chamfer(source:Feature, distance:NamedParameter, selection:SelectionRecipe):ChamferFeature {
		check();
		var feature = document.add(new ChamferFeature(source, distance.value, null, null, selection));
		bind(distance, feature, "chamfer.distance");
		return feature;
	}

	public function output(feature:Feature):Void {
		check();
		document.setOutput(feature);
	}

	public function unsupported(operation:String):Feature {
		check();
		throw new ParametricError("operation cannot be recorded: " + operation);
	}

	/** Recompute and transfer ownership to the caller. */
	public function finish():Document {
		check();
		try {
			for (parameter in document.namedParameters())
				if (parameter.bindings().length == 0)
					throw new ParametricError("named parameter has no feature bindings: " + parameter.name);
			document.outputFeature();
			document.recompute();
			finished = true;
			return document;
		} catch (error:Dynamic) {
			finished = true;
			document.close();
			throw error;
		}
	}

	public function close():Void {
		if (finished)
			return;
		finished = true;
		document.close();
	}

	public static function build(callback:DocumentBuilder->Void):Document {
		var builder = new DocumentBuilder();
		try {
			callback(builder);
			return builder.finish();
		} catch (error:Dynamic) {
			builder.close();
			throw error;
		}
	}
}
