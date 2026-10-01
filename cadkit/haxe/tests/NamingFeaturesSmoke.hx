import CadKit;
import cadkit.ElementNames;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import cadkit.parametric.Document;
import cadkit.parametric.Feature;
import cadkit.parametric.SelectionRecipe;
import cadkit.parametric.features.BooleanFeature;
import cadkit.parametric.features.BooleanOperation;
import cadkit.parametric.features.BoxFeature;
import cadkit.parametric.features.ChamferFeature;
import cadkit.parametric.features.CylinderFeature;
import cadkit.parametric.features.ExtrudeFeature;
import cadkit.parametric.features.FilletFeature;
import cadkit.parametric.features.GridFeature;
import cadkit.parametric.features.HoleFeature;
import cadkit.parametric.features.LinearPatternFeature;
import cadkit.parametric.features.LoftFeature;
import cadkit.parametric.features.MirrorFeature;
import cadkit.parametric.features.PocketFeature;
import cadkit.parametric.features.PolarPatternFeature;
import cadkit.parametric.features.PolylineFeature;
import cadkit.parametric.features.RevolveFeature;
import cadkit.parametric.features.RotationFeature;
import cadkit.parametric.features.ShellFeature;
import cadkit.parametric.features.SketchFeature;
import cadkit.parametric.features.SweepFeature;
import cadkit.parametric.features.ToolCollectionFeature;
import cadkit.parametric.features.TransformFeature;
import cadkit.parametric.features.WireFeature;

/**
	Every built-in solid feature names every face of its output strongly and uniquely (plans/TOPOLOGICAL_NAMING.md,
	TN4.5): a weak or repeated name could only be found again by geometry, so references to it would not survive edits.
*/
class NamingFeaturesSmoke {
	public static function run():Void {
		var failures:Array<String> = [];
		for (model in MODELS) {
			var document = new Document();
			var output = build(model, document);
			document.recompute();
			var names = output.currentShape().elementNames(CadKit.ShapeKind.Face);
			var seen:Map<String, Bool> = [];
			var bad:Array<String> = [];
			for (name in names) {
				if (!ElementNames.isStrong(name) || seen.exists(name))
					bad.push(name);
				seen.set(name, true);
			}
			if (bad.length > 0)
				failures.push('$model: ${bad.join(", ")}');
			document.close();
		}
		if (failures.length > 0)
			throw "NamingFeaturesSmoke: weak or repeated face names\n  " + failures.join("\n  ");
	}

	static final MODELS = [
		"box", "cylinder", "fillet", "chamfer", "fuse", "cut", "common", "extrude rectangle", "extrude circle", "extrude slot",
		"revolve", "loft", "sweep", "shell", "pocket", "hole", "counterbore", "countersink", "counterbore pattern", "linear pattern",
		"polar pattern", "grid", "mirror both", "mirror fuse", "rotation", "tool collection"
	];

	static function build(model:String, d:Document):Feature {
		var top = new SelectionRecipe("face", "plane", Vector.Z(), "max", Vector.Z(), 1);
		if (model == "box")
			return d.add(new BoxFeature(10, 20, 30));
		if (model == "cylinder")
			return d.add(new CylinderFeature(5, 10));
		if (model == "fillet")
			return d.add(new FilletFeature(d.add(new BoxFeature(10, 20, 30)), 1));
		if (model == "chamfer")
			return d.add(new ChamferFeature(d.add(new BoxFeature(10, 20, 30)), 1));
		if (model == "fuse" || model == "cut" || model == "common") {
			var a = d.add(new BoxFeature(20, 20, 10));
			var b = d.add(new TransformFeature(d.add(new CylinderFeature(4, 30)), 10, 10, -5));
			return d.add(new BooleanFeature(a, b, model == "fuse" ? BooleanOperation.Fuse : model == "cut" ? BooleanOperation.Cut
				: BooleanOperation.Common));
		}
		if (model == "extrude rectangle")
			return d.add(ExtrudeFeature.along(d.add(new SketchFeature("rectangle", 20, 10)), 5, Vector.Z()));
		if (model == "extrude circle")
			return d.add(ExtrudeFeature.along(d.add(new SketchFeature("circle", 4)), 5, Vector.Z()));
		if (model == "extrude slot")
			return d.add(ExtrudeFeature.along(d.add(new SketchFeature("slot", 20, 6)), 5, Vector.Z()));
		if (model == "revolve") {
			var plane = new Plane(new Vector(15, 0, 0), Vector.X(), new Vector(0, -1, 0));
			return d.add(new RevolveFeature(d.add(new SketchFeature("rectangle", 10, 20, plane)), 0, 0, 0, 0, 0, 1, Math.PI));
		}
		if (model == "loft") {
			var wire = d.add(new WireFeature(d.add(new SketchFeature("rectangle", 10, 10))));
			return d.add(new LoftFeature([wire, d.add(new TransformFeature(wire, 0, 0, 5))]));
		}
		if (model == "sweep") {
			var path = d.add(new PolylineFeature([new Vector(), new Vector(0, 0, 10), new Vector(10, 0, 20)]));
			return d.add(new SweepFeature(d.add(new SketchFeature("rectangle", 2, 2)), path));
		}
		if (model == "shell")
			return d.add(new ShellFeature(d.add(new BoxFeature(10, 10, 10)), -1, top));
		if (model == "pocket") {
			var box = d.add(new BoxFeature(20, 30, 10));
			var profile = d.add(new cadkit.parametric.features.ConstrainedSketchFeature(rectangle(8, 6), box, top, Vector.X()));
			return d.add(PocketFeature.blind(box, profile, 3));
		}
		if (model == "hole" || model == "counterbore" || model == "countersink" || model == "counterbore pattern") {
			var target = d.add(new BoxFeature(30, 30, 10));
			var tool:Feature = model == "hole" ? d.add(HoleFeature.plain(target, top, Vector.X(), "blind", 0, 0, 4, 3))
				: model == "countersink" ? d.add(HoleFeature.countersink(target, top, Vector.X(), "through-all", 0, 0, 4, 1, 8, Math.PI / 2))
				: d.add(HoleFeature.counterbore(target, top, Vector.X(), "through-all", 0, 0, 4, 1, 8, 2));
			if (model == "counterbore pattern")
				tool = d.add(new LinearPatternFeature(tool, 2, 16, Vector.X(), 2, 16, Vector.Y()));
			return d.add(new BooleanFeature(target, tool, BooleanOperation.Cut));
		}
		var plate = d.add(new BoxFeature(80, 80, 10));
		var boss = d.add(new TransformFeature(d.add(new CylinderFeature(3, 5)), 40, 40, 10));
		if (model == "linear pattern")
			return d.add(new BooleanFeature(plate, d.add(new LinearPatternFeature(boss, 3, 15, Vector.X())), BooleanOperation.Fuse));
		if (model == "polar pattern")
			return d.add(new PolarPatternFeature(d.add(new CylinderFeature(2, 5)), 4, 20, 2 * Math.PI, new Vector(), Vector.Z(), Vector.X()));
		if (model == "grid")
			return d.add(new BooleanFeature(plate, d.add(new GridFeature(boss, 2, 2, 15, 15)), BooleanOperation.Fuse));
		var offCenter = d.add(new TransformFeature(d.add(new BoxFeature(10, 10, 10)), 5, 0, 0));
		if (model == "mirror both")
			return d.add(new MirrorFeature(offCenter, new Vector(), Vector.X(), "both"));
		if (model == "mirror fuse")
			return d.add(new MirrorFeature(d.add(new TransformFeature(d.add(new BoxFeature(10, 10, 10)), -2, 0, 0)), new Vector(), Vector.X(),
				"fuse"));
		if (model == "rotation")
			return d.add(new RotationFeature(offCenter, new Vector(), Vector.Z(), Math.PI / 2));
		var first = d.add(new CylinderFeature(2, 5));
		var second = d.add(new TransformFeature(d.add(new CylinderFeature(2, 5)), 10, 0, 0));
		return d.add(new ToolCollectionFeature([first, second], ["first", "second"]));
	}

	static function rectangle(width:Float, height:Float):cadkit.sketch.ConstrainedSketch {
		var sketch = new cadkit.sketch.ConstrainedSketch();
		var corners = [[-width / 2, -height / 2], [width / 2, -height / 2], [width / 2, height / 2], [-width / 2, height / 2]];
		for (index in 0...4) {
			sketch.addPoint(new cadkit.sketch.SketchPoint("p" + index, corners[index][0], corners[index][1]));
			sketch.addConstraint(cadkit.sketch.SketchConstraint.fixed("fixed" + index, "p" + index));
		}
		for (index in 0...4)
			sketch.addEntity(cadkit.sketch.SketchEntity.line("edge" + index, "p" + index, "p" + ((index + 1) % 4)));
		return sketch;
	}
}
