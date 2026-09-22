import CadKit;
import cadkit.modeling.Vector;
import cadkit.parametric.Document;
import cadkit.parametric.DocumentCodec;
import cadkit.parametric.features.BooleanFeature;
import cadkit.parametric.features.BooleanOperation;
import cadkit.parametric.features.BoxFeature;
import cadkit.parametric.features.CylinderFeature;
import cadkit.parametric.features.LinearPatternFeature;
import cadkit.parametric.features.PolarPatternFeature;
import cadkit.parametric.features.TransformFeature;
import cadkit.parametric.features.RotationFeature;
import cadkit.parametric.features.MirrorFeature;
import cadkit.parametric.features.FilletFeature;
import cadkit.parametric.features.SketchFeature;

class PatternSmoke {
	static function check(value:Bool, message:String):Void {
		if (!value)
			throw message;
	}

	static function near(value:Float, expected:Float):Void {
		check(Math.abs(value - expected) < 1e-5, 'expected $expected, got $value');
	}

	public static function run():Void {
		var document = new Document();
		var target = document.add(new BoxFeature(30, 10, 5));
		var tool = document.add(new CylinderFeature(1, 5));
		var positionedTool = document.add(new TransformFeature(tool, 15, 5, 0));
		var holes = document.add(new LinearPatternFeature(positionedTool, 3, 8, Vector.X()));
		var count = document.defineParameter("holes.count", 3);
		var spacing = document.defineParameter("holes.spacing", 8);
		count.bind(holes.count);
		spacing.bind(holes.spacing);
		var cut = document.add(new BooleanFeature(target, holes, BooleanOperation.Cut));
		document.setOutput(cut);
		document.recompute();
		near(document.result().volume(), 30 * 10 * 5 - 3 * Math.PI * 5);

		count.set(5);
		spacing.set(5);
		document.recompute();
		near(document.result().volume(), 30 * 10 * 5 - 5 * Math.PI * 5);
		check(document.undo(), "linear spacing undo");
		check(document.undo(), "linear count undo");
		document.recompute();
		near(document.result().volume(), 30 * 10 * 5 - 3 * Math.PI * 5);
		check(document.redo(), "linear count redo");
		check(document.redo(), "linear spacing redo");
		document.recompute();
		near(document.result().volume(), 30 * 10 * 5 - 5 * Math.PI * 5);

		var committedLinearCut = document.result();
		var failed = false;
		try count.set(3.5) catch (error:Dynamic) failed = true;
		check(failed && count.value == 5 && document.result() == committedLinearCut,
			"linear count rejects non-integers atomically");
		var restored = DocumentCodec.decode(DocumentCodec.encode(document));
		near(restored.result().volume(), document.result().volume());
		restored.parameter("holes.count").set(3);
		restored.recompute();
		near(restored.result().volume(), 30 * 10 * 5 - 3 * Math.PI * 5);
		restored.close();
		document.close();

		var additionDocument = new Document();
		var additionTarget = additionDocument.add(new BoxFeature(30, 10, 5));
		var boss = additionDocument.add(new BoxFeature(1, 1, 1));
		var positionedBoss = additionDocument.add(new TransformFeature(boss, 15, 5, 5));
		var bosses = additionDocument.add(new LinearPatternFeature(positionedBoss, 3, 5, Vector.X()));
		var added = additionDocument.add(new BooleanFeature(additionTarget, bosses, BooleanOperation.Fuse));
		additionDocument.setOutput(added);
		additionDocument.recompute();
		near(additionDocument.result().volume(), 30 * 10 * 5 + 3);
		additionDocument.close();

		var twoAxisDocument = new Document();
		var seed = twoAxisDocument.add(new BoxFeature(1, 1, 1));
		var array = twoAxisDocument.add(new LinearPatternFeature(seed, 3, 4, new Vector(1, 1, 0), 2, 6, Vector.Z()));
		twoAxisDocument.setOutput(array);
		twoAxisDocument.recompute();
		near(twoAxisDocument.result().volume(), 6);
		check(array.currentShape().subshapeCount(CadKit.ShapeKind.Solid) == 6, "two-axis linear pattern instance count");
		twoAxisDocument.close();

		var polarDocument = new Document();
		var flange = polarDocument.add(new CylinderFeature(30, 6));
		var boltHole = polarDocument.add(new CylinderFeature(2, 6));
		var boltHoles = polarDocument.add(new PolarPatternFeature(boltHole, 6, 20, 2 * Math.PI,
			new Vector(), Vector.Z(), Vector.X()));
		var boltCount = polarDocument.defineParameter("bolts.count", 6);
		var boltRadius = polarDocument.defineParameter("bolts.circle-radius", 20);
		var boltSpan = polarDocument.defineParameter("bolts.angular-span", 2 * Math.PI);
		boltCount.bind(boltHoles.count);
		boltRadius.bind(boltHoles.radius);
		boltSpan.bind(boltHoles.angularSpan);
		var drilledFlange = polarDocument.add(new BooleanFeature(flange, boltHoles, BooleanOperation.Cut));
		polarDocument.setOutput(drilledFlange);
		polarDocument.recompute();
		near(polarDocument.result().volume(), Math.PI * 30 * 30 * 6 - 6 * Math.PI * 2 * 2 * 6);
		check(boltHoles.currentShape().subshapeCount(CadKit.ShapeKind.Solid) == 6,
			"full polar pattern does not duplicate its endpoint");
		boltCount.set(8);
		polarDocument.recompute();
		near(polarDocument.result().volume(), Math.PI * 30 * 30 * 6 - 8 * Math.PI * 2 * 2 * 6);
		check(polarDocument.undo(), "polar count undo");
		polarDocument.recompute();
		near(polarDocument.result().volume(), Math.PI * 30 * 30 * 6 - 6 * Math.PI * 2 * 2 * 6);
		check(polarDocument.redo(), "polar count redo");
		polarDocument.recompute();
		var restoredPolar = DocumentCodec.decode(DocumentCodec.encode(polarDocument));
		near(restoredPolar.result().volume(), polarDocument.result().volume());
		check(restoredPolar.parameter("bolts.count").value == 8, "polar count reload");
		restoredPolar.close();
		polarDocument.close();

		var orientationDocument = new Document();
		var orientationSeed = orientationDocument.add(new BoxFeature(2, 1, 1));
		var fixedOrientation = orientationDocument.add(new PolarPatternFeature(orientationSeed, 2, 10, Math.PI / 2,
			new Vector(), Vector.Z(), Vector.X(), false));
		var rotatingOrientation = orientationDocument.add(new PolarPatternFeature(orientationSeed, 2, 10, Math.PI / 2,
			new Vector(), Vector.Z(), Vector.X(), true));
		orientationDocument.setOutput(rotatingOrientation);
		orientationDocument.recompute();
		near(fixedOrientation.currentShape().bounds().get_max().get_y(), 11);
		near(rotatingOrientation.currentShape().bounds().get_max().get_y(), 12);
		orientationDocument.close();

		var flangeExample = new ParametricFlange();
		near(flangeExample.finish.currentShape().volume(), Math.PI * 30 * 30 * 6 - 6 * Math.PI * 2 * 2 * 6);
		flangeExample.resize(36, 8, 25);
		near(flangeExample.finish.currentShape().volume(), Math.PI * 36 * 36 * 6 - 8 * Math.PI * 2 * 2 * 6);
		var restoredFlange = DocumentCodec.decode(DocumentCodec.encode(flangeExample.document));
		near(restoredFlange.result().volume(), flangeExample.finish.currentShape().volume());
		restoredFlange.close();
		flangeExample.close();

		var ventilationExample = new VentilatedEnclosure();
		near(ventilationExample.finish.currentShape().volume(), 60 * 40 * 5 - 5 * 4 * 20 * 5);
		ventilationExample.resize(80, 7, 10);
		near(ventilationExample.finish.currentShape().volume(), 80 * 40 * 5 - 7 * 4 * 20 * 5);
		near(ventilationExample.slot.currentShape().center().get_x(), 40);
		var restoredVentilation = DocumentCodec.decode(DocumentCodec.encode(ventilationExample.document));
		near(restoredVentilation.result().volume(), ventilationExample.finish.currentShape().volume());
		restoredVentilation.close();
		var committedVentilation = ventilationExample.finish.currentShape();
		var failedResize = false;
		try ventilationExample.resize(30, 7, 10) catch (error:Dynamic) failedResize = true;
		check(failedResize && ventilationExample.finish.currentShape() == committedVentilation,
			"failed patterned enclosure edit preserves committed geometry");
		ventilationExample.close();

		var rotationDocument = new Document();
		var rotationSource = rotationDocument.add(new BoxFeature(2, 1, 1));
		var rotation = rotationDocument.add(new RotationFeature(rotationSource, new Vector(1, 0, 0), Vector.Z(), Math.PI / 2));
		var rotationAngle = rotationDocument.defineParameter("rotation.angle", Math.PI / 2);
		rotationAngle.bind(rotation.angle);
		rotationDocument.setOutput(rotation);
		rotationDocument.recompute();
		near(rotationDocument.result().bounds().get_min().get_x(), 0);
		near(rotationDocument.result().bounds().get_max().get_x(), 1);
		near(rotationDocument.result().bounds().get_min().get_y(), -1);
		near(rotationDocument.result().bounds().get_max().get_y(), 1);
		check(rotation.provenance != null, "rotation retains operation history");
		rotationAngle.set(Math.PI);
		rotationDocument.recompute();
		near(rotationDocument.result().bounds().get_min().get_x(), 0);
		near(rotationDocument.result().bounds().get_max().get_x(), 2);
		check(rotationDocument.undo(), "rotation angle undo");
		rotationDocument.recompute();
		near(rotationDocument.result().bounds().get_max().get_y(), 1);
		var restoredRotation = DocumentCodec.decode(DocumentCodec.encode(rotationDocument));
		near(restoredRotation.result().bounds().get_max().get_y(), 1);
		restoredRotation.close();
		rotationDocument.close();

		var mirrorDocument = new Document();
		var mirrorSource = mirrorDocument.add(new BoxFeature(10, 5, 3));
		var reflected = mirrorDocument.add(new MirrorFeature(mirrorSource, new Vector(), Vector.X(), "copy"));
		var retained = mirrorDocument.add(new MirrorFeature(mirrorSource, new Vector(), Vector.X(), "both"));
		var fusedMirror = mirrorDocument.add(new MirrorFeature(mirrorSource, new Vector(), Vector.X(), "fuse"));
		var downstreamFillet = mirrorDocument.add(new FilletFeature(fusedMirror, 0.4));
		mirrorDocument.setOutput(downstreamFillet);
		mirrorDocument.recompute();
		near(reflected.currentShape().bounds().get_min().get_x(), -10);
		near(reflected.currentShape().bounds().get_max().get_x(), 0);
		near(reflected.currentShape().volume(), 150);
		check(CadKit.shapeValidChecked(reflected.currentShape().borrowHandle()) != 0, "reflected solid is valid");
		check(retained.currentShape().subshapeCount(CadKit.ShapeKind.Solid) == 2, "mirror both retains two solids");
		near(retained.currentShape().volume(), 300);
		near(fusedMirror.currentShape().volume(), 300);
		check(CadKit.shapeValidChecked(fusedMirror.currentShape().borrowHandle()) != 0, "fused mirror is valid");
		check(CadKit.shapeValidChecked(downstreamFillet.currentShape().borrowHandle()) != 0, "mirrored solid supports downstream fillets");
		var minimumNormal = 0.0;
		var maximumNormal = 0.0;
		var reflectedFaces = reflected.currentShape().faces();
		for (index in 0...reflectedFaces.count()) {
			var face = reflectedFaces.at(index);
			var center = face.center();
			if (Math.abs(center.get_x() + 10) < 1e-6)
				minimumNormal = face.normal().get_x();
			if (Math.abs(center.get_x()) < 1e-6)
				maximumNormal = face.normal().get_x();
			face.close();
		}
		near(minimumNormal, -1);
		near(maximumNormal, 1);
		var restoredMirror = DocumentCodec.decode(DocumentCodec.encode(mirrorDocument));
		near(restoredMirror.result().volume(), downstreamFillet.currentShape().volume());
		restoredMirror.close();
		var committedMirror = mirrorDocument.result();
		downstreamFillet.radius.set(100);
		var mirrorFailure = false;
		try mirrorDocument.recompute() catch (error:Dynamic) mirrorFailure = true;
		check(mirrorFailure && mirrorDocument.result() == committedMirror,
			"downstream mirror failure preserves committed geometry");
		mirrorDocument.close();

		var profileMirrorDocument = new Document();
		var profile = profileMirrorDocument.add(new SketchFeature("rectangle", 2, 4));
		var movedProfile = profileMirrorDocument.add(new TransformFeature(profile, 3, 0, 0));
		var mirroredProfile = profileMirrorDocument.add(new MirrorFeature(movedProfile, new Vector(), Vector.X(), "copy"));
		profileMirrorDocument.setOutput(mirroredProfile);
		profileMirrorDocument.recompute();
		near(mirroredProfile.currentShape().area(), 8);
		near(Math.abs(mirroredProfile.currentShape().faceNormal().get_z()), 1);
		check(CadKit.shapeValidChecked(mirroredProfile.currentShape().borrowHandle()) != 0, "reflected profile is valid");
		profileMirrorDocument.close();

		var bracket = new MirroredMountingBracket();
		var bracketVolume = bracket.finish.currentShape().volume();
		check(bracketVolume > 0, "mirrored mounting bracket initial solid");
		bracket.resize(100, 70);
		check(bracket.finish.currentShape().volume() > bracketVolume, "mirrored bracket resize");
		near(bracket.finish.currentShape().bounds().get_min().get_x(), -50);
		near(bracket.finish.currentShape().bounds().get_max().get_x(), 50);
		var restoredBracket = DocumentCodec.decode(DocumentCodec.encode(bracket.document));
		near(restoredBracket.result().volume(), bracket.finish.currentShape().volume());
		restoredBracket.close();
		bracket.close();
	}
}
