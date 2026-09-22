import CadKit;
import cadkit.Shape;
import cadkit.modeling.Vector;
import cadkit.modeling.Axis;
import cadkit.modeling.Plane;
import cadkit.modeling.Location;
import cadkit.modeling.Locations;
import cadkit.modeling.Curve;
import cadkit.modeling.Sketch;
import cadkit.modeling.Part;
import cadkit.modeling.Scope;
import cadkit.modeling.Selection;
import cadkit.modeling.BuildLine;
import cadkit.modeling.BuildSketch;
import cadkit.modeling.BuildPart;
import sys.FileSystem;
import cadkit.parametric.Document;
import cadkit.parametric.DocumentCodec;
import cadkit.parametric.features.SketchFeature;
import cadkit.parametric.features.ExtrudeFeature;

class ModelingSmoke {
	static function check(value:Bool, message:String):Void {
		if (!value)
			throw message;
	}

	static function near(a:Float, b:Float):Void {
		check(Math.abs(a - b) < 0.00001, "measurement mismatch: " + a + " / " + b);
	}

	public static function run():Void {
		var plane = new Plane(new Vector(3, 4, 5), Vector.X(), Vector.Y().scale(-1));
		var v = new Vector(1, 2, 3);
		var local = plane.toLocal(plane.toWorld(v));
		near(local.x, 1);
		near(local.y, 2);
		near(local.z, 3);
		var location = Location.translation(new Vector(4, 5, 6)).compose(Location.rotation(Axis.Z(), Math.PI / 2));
		var roundtrip = location.inverse().plane.toWorld(location.plane.toWorld(v));
		near(roundtrip.x, 1);
		near(roundtrip.y, 2);
		near(roundtrip.z, 3);
		var sketch = BuildSketch.build(function(b) {
			b.rectangle(80, 50);
			b.circles(3, Locations.grid(2, 2, 60, 30), Subtract);
		});
		near(sketch.shape.area(), 4000 - 36 * Math.PI);
		var plate = sketch.extrude(6);
		near(plate.volume(), (4000 - 36 * Math.PI) * 6);
		check(plate.valid() && plate.solidCount() == 1, "plate topology");
		var selected = plate.edges().parallel(Axis.Z()).filter(function(s) {
			return Math.abs(Selection.center(s).x) > 39;
		});
		check(selected.count() == 4, "four corner edges");
		var rounded = plate.fillet(selected, 2);
		selected.close();
		check(rounded.valid() && rounded.volume() < plate.volume(), "rounded plate");
		var path = "/tmp/cadkit-modeling-haxe.step";
		rounded.exportStep(path);
		var imported = Shape.importStep(path);
		near(imported.volume(), rounded.volume());
		imported.close();
		FileSystem.deleteFile(path);
		var generated = plate.generated(CadKit.ShapeKind.Face);
		check(generated.count() > 0, "extrusion history");
		generated.close();
		var top = plate.faces().parallel(Axis.Z()).extreme(Axis.Z());
		var topShape = top.unique();
		near(topShape.center().get_z(), 6);
		topShape.close();
		var topEdges = top.children(CadKit.ShapeKind.Edge);
		check(topEdges.count() == 8, "nested edges");
		topEdges.close();
		top.close();
		var faces = plate.faces();
		var groups = faces.groupBy(function(s) {
			return Selection.center(s).z;
		});
		check(groups.length == 3, "face groups");
		for (group in groups)
			group.close();
		faces.close();
		var placed = sketch.placed(plane.location());
		var sideways = placed.extrude(2);
		check(sideways.valid(), "workplane extrusion");
		sideways.close();
		placed.close();
		var slot = Sketch.slot(20, 4);
		near(slot.shape.area(), 64 + 4 * Math.PI);
		slot.close();
		var outline = BuildLine.build(function(b) {
			b.line(new Vector(0, 0), new Vector(10, 0));
			b.arc(new Vector(10, 0), new Vector(15, 5), new Vector(10, 10));
			b.line(new Vector(10, 10), new Vector(0, 10));
			b.line(new Vector(0, 10), new Vector(0, 0));
		});
		var profile = Sketch.face(outline);
		check(profile.valid(), "line builder profile");
		profile.close();
		outline.close();
		var spline = Curve.spline([new Vector(), new Vector(1, 1), new Vector(2, 0)]);
		check(spline.valid(), "spline");
		spline.close();
		var c0 = Curve.circle(2);
		var c1 = Curve.circle(2, Plane.XY().offset(5));
		var loft = Part.loft([c0, c1]);
		near(loft.volume(), 20 * Math.PI);
		loft.close();
		var offset = c0.offset(1);
		check(offset.valid(), "offset");
		offset.close();
		var disk = Sketch.circle(2);
		var spine = Curve.polyline([new Vector(), new Vector(0, 0, 5)]);
		var swept = disk.sweep(spine);
		near(swept.volume(), 20 * Math.PI);
		var projection = c1.project(disk, new Vector(0, 0, -1));
		check(projection.valid(), "projection");
		projection.close();
		swept.close();
		spine.close();
		disk.close();
		c1.close();
		c0.close();
		var box = Part.box(10, 10, 10);
		var lid = box.faces().parallel(Axis.Z()).extreme(Axis.Z());
		var shell = box.shell(lid, -1);
		near(shell.volume(), 424);
		shell.close();
		lid.close();
		box.close();
		var result = MountingPlate.build();
		near(result.volume(), rounded.volume());
		result.close();
		var scoped = Scope.run(function(s:Scope) {
			var base = s.own(Part.box(10, 10, 2));
			var tool = s.own(Part.cylinder(1, 2));
			return s.own(base.subtract(tool));
		});
		near(scoped.volume(), 200 - 2 * Math.PI);
		scoped.close();
		var temp:Null<Part> = null;
		var failed = false;
		try {
			Scope.run(function(s:Scope) {
				temp = s.own(Part.box(1, 1, 1));
				if (temp != null)
					throw "deliberate scope failure";
				return temp;
			});
		} catch (error:Dynamic) {
			failed = true;
		}
		check(failed && temp != null && temp.shape.isClosed(), "scope failure cleanup");
		var empty = BuildPart.build(function(b) {});
		check(empty.isEmpty() && empty.solidCount() == 0, "empty part");
		empty.close();
		var pattern = BuildPart.build(function(b) {
			var seed = Part.cylinder(1, 2);
			try {
				b.pattern(seed, Locations.polar(4, 10));
				seed.close();
			} catch (error:Dynamic) {
				seed.close();
				throw error;
			}
		});
		check(pattern.solidCount() == 4, "multiple solids");
		pattern.close();
		var wrongShape = sketch.shape.cloneShape();
		var typeFailed = false;
		try {
			var wrong = new Part(wrongShape);
			wrong.close();
		} catch (error:Dynamic) {
			typeFailed = true;
		}
		check(typeFailed && wrongShape.isClosed(), "invalid typed model cleanup");
		var tied = plate.faces().parallel(Axis.Z());
		var ambiguous = false;
		try {
			var wrong = tied.unique();
			wrong.close();
		} catch (error:Dynamic) {
			ambiguous = true;
		}
		check(ambiguous && tied.count() == 2, "selection preserves ambiguous matches");
		tied.close();
		var borrowed = Part.box(2, 2, 2);
		var builderFailed = false;
		try {
			BuildPart.build(function(b) {
				b.add(borrowed);
				var selection = b.edges();
				try {
					b.fillet(selection, -1);
					selection.close();
				} catch (error:Dynamic) {
					selection.close();
					throw error;
				}
			});
		} catch (error:Dynamic) {
			builderFailed = true;
		}
		check(builderFailed && !borrowed.shape.isClosed(), "failed builder preserves borrowed input");
		near(borrowed.volume(), 8);
		borrowed.close();
		rounded.close();
		plate.close();
		sketch.close();
		var document = new Document();
		var feature = document.add(new SketchFeature("rectangle", 10, 20, Plane.XY().offset(3)));
		var extrusion = document.add(new ExtrudeFeature(feature, 0, 0, 4));
		document.recompute();
		near(extrusion.currentShape().volume(), 800);
		var transaction = document.beginTransaction();
		feature.width.set(15);
		transaction.commit();
		document.recompute();
		near(extrusion.currentShape().volume(), 1200);
		check(document.undo(), "profile undo");
		document.recompute();
		near(extrusion.currentShape().volume(), 800);
		check(document.redo(), "profile redo");
		document.recompute();
		near(extrusion.currentShape().volume(), 1200);
		var loaded = DocumentCodec.decode(DocumentCodec.encode(document));
		near(loaded.featureAt(1).currentShape().volume(), 1200);
		near(loaded.featureAt(1).currentShape().bounds().get_min().get_z(), 3);
		loaded.close();
		document.close();
	}
}
