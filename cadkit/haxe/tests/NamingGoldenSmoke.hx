import CadKit;
import cadkit.Shape;
import cadkit.modeling.Vector;
import cadkit.parametric.Document;
import cadkit.parametric.Feature;
import cadkit.parametric.features.BooleanFeature;
import cadkit.parametric.features.BooleanOperation;
import cadkit.parametric.features.BoxFeature;
import cadkit.parametric.features.ConstrainedSketchFeature;
import cadkit.parametric.features.CylinderFeature;
import cadkit.parametric.features.ExtrudeFeature;
import cadkit.parametric.features.FilletFeature;
import cadkit.parametric.features.LinearPatternFeature;
import cadkit.parametric.features.TransformFeature;
import cadkit.sketch.ConstrainedSketch;
import cadkit.sketch.SketchConstraint;
import cadkit.sketch.SketchEntity;
import cadkit.sketch.SketchPoint;

/**
	Names are document format (plans/TOPOLOGICAL_NAMING.md, TN-D13): the face names of a small corpus are pinned
	here, so a change to the naming rules fails this test and has to bump the naming scheme on purpose.
	`CADKIT_NAMING_GOLDEN=print` prints the current names instead of checking them.
*/
class NamingGoldenSmoke {
	static var GOLDEN:Map<String, String> = [
		"box fillet" => "f1:box.+x ; f1:box.+y ; f1:box.+z ; f1:box.-x ; f1:box.-y ; f1:box.-z ; f2:corner(V(f1:box.+x|f1:box.+y|f1:box.+z)) ; f2:corner(V(f1:box.+x|f1:box.+y|f1:box.-z)) ; f2:corner(V(f1:box.+x|f1:box.+z|f1:box.-y)) ; f2:corner(V(f1:box.+x|f1:box.-y|f1:box.-z)) ; f2:corner(V(f1:box.+y|f1:box.+z|f1:box.-x)) ; f2:corner(V(f1:box.+y|f1:box.-x|f1:box.-z)) ; f2:corner(V(f1:box.+z|f1:box.-x|f1:box.-y)) ; f2:corner(V(f1:box.-x|f1:box.-y|f1:box.-z)) ; f2:fillet(E(f1:box.+x|f1:box.+y)) ; f2:fillet(E(f1:box.+x|f1:box.+z)) ; f2:fillet(E(f1:box.+x|f1:box.-y)) ; f2:fillet(E(f1:box.+x|f1:box.-z)) ; f2:fillet(E(f1:box.+y|f1:box.+z)) ; f2:fillet(E(f1:box.+y|f1:box.-x)) ; f2:fillet(E(f1:box.+y|f1:box.-z)) ; f2:fillet(E(f1:box.+z|f1:box.-x)) ; f2:fillet(E(f1:box.+z|f1:box.-y)) ; f2:fillet(E(f1:box.-x|f1:box.-y)) ; f2:fillet(E(f1:box.-x|f1:box.-z)) ; f2:fillet(E(f1:box.-y|f1:box.-z))",
		"plate hole slot" => "f1:box.+x ; f1:box.+y{f1:box.+x,f4:box.+x} ; f1:box.+y{f1:box.-x,f4:box.-x} ; f1:box.+z{f1:box.+x,f4:box.+x} ; f1:box.+z{f1:box.-x,f2:cyl.side,f4:box.-x} ; f1:box.-x ; f1:box.-y{f1:box.+x,f4:box.+x} ; f1:box.-y{f1:box.-x,f4:box.-x} ; f1:box.-z{f1:box.+x,f4:box.+x} ; f1:box.-z{f1:box.-x,f2:cyl.side,f4:box.-x} ; f2:cyl.side ; f4:box.+x ; f4:box.-x",
		"sketch extrude" => "f2:end(f1:r.line0+line1+line2+line3) ; f2:side(f1:e.line0) ; f2:side(f1:e.line1) ; f2:side(f1:e.line2) ; f2:side(f1:e.line3) ; f2:start(f1:r.line0+line1+line2+line3)",
		"pattern" => "f1:box.+x ; f1:box.+y ; f1:box.+z ; f1:box.-x ; f1:box.-y ; f1:box.-z ; f4:i0:f2:cyl.side ; f4:i0:f2:cyl.top ; f4:i1:f2:cyl.side ; f4:i1:f2:cyl.top"
	];

	public static function run():Void {
		var printing = Sys.getEnv("CADKIT_NAMING_GOLDEN") == "print";
		var failures:Array<String> = [];
		for (model in ["box fillet", "plate hole slot", "sketch extrude", "pattern"]) {
			var document = new Document();
			var output = build(model, document);
			document.recompute();
			var names = output.currentShape().elementNames(CadKit.ShapeKind.Face);
			names.sort(Reflect.compare);
			var text = names.join(" ; ");
			if (printing)
				Sys.println('\t\t"$model" => "$text",');
			else if (GOLDEN.get(model) != text)
				failures.push('$model:\n    expected ${GOLDEN.get(model)}\n    got      $text');
			document.close();
		}
		if (Shape.namingScheme() != 1)
			failures.push("naming scheme changed: update GOLDEN and the documents' migration together");
		if (!printing && failures.length > 0)
			throw "NamingGoldenSmoke:\n  " + failures.join("\n  ");
	}

	static function build(model:String, document:Document):Feature {
		if (model == "box fillet")
			return document.add(new FilletFeature(document.add(new BoxFeature(10, 20, 30)), 1));
		if (model == "plate hole slot") {
			var plate = document.add(new BoxFeature(60, 40, 10));
			var hole = document.add(new TransformFeature(document.add(new CylinderFeature(3, 30)), 15, 20, -5));
			var slot = document.add(new TransformFeature(document.add(new BoxFeature(4, 60, 30)), 28, -10, -5));
			var drilled = document.add(new BooleanFeature(plate, hole, BooleanOperation.Cut));
			return document.add(new BooleanFeature(drilled, slot, BooleanOperation.Cut));
		}
		if (model == "sketch extrude") {
			var sketch = new ConstrainedSketch();
			var corners = [[0.0, 0.0], [20.0, 0.0], [20.0, 10.0], [0.0, 10.0]];
			for (index in 0...4) {
				sketch.addPoint(new SketchPoint("p" + index, corners[index][0], corners[index][1]));
				sketch.addConstraint(SketchConstraint.fixed("fixed" + index, "p" + index));
			}
			for (index in 0...4)
				sketch.addEntity(SketchEntity.line("line" + index, "p" + index, "p" + ((index + 1) % 4)));
			return document.add(ExtrudeFeature.along(document.add(new ConstrainedSketchFeature(sketch)), 5, Vector.Z()));
		}
		var plate = document.add(new BoxFeature(80, 30, 10));
		var boss = document.add(new TransformFeature(document.add(new CylinderFeature(3, 5)), 40, 15, 10));
		var bosses = document.add(new LinearPatternFeature(boss, 2, 15, Vector.X()));
		return document.add(new BooleanFeature(plate, bosses, BooleanOperation.Fuse));
	}
}
