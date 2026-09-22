import cadkit.sketch.ConstrainedSketch;
import cadkit.sketch.SketchConstraint;
import cadkit.sketch.SketchEntity;
import cadkit.sketch.SketchPoint;
import cadkit.sketch.SketchSolveError;
import cadkit.sketch.SketchProfile;
import cadkit.sketch.ProfileError;
import cadkit.parametric.Document;
import cadkit.parametric.DocumentCodec;
import cadkit.parametric.features.ConstrainedSketchFeature;
import cadkit.parametric.features.ExtrudeFeature;
import cadkit.sketch.SolverSettings;

class ConstrainedSketchSmoke {
	static function check(value:Bool, message:String):Void { if (!value) throw message; }
	static function near(value:Float, expected:Float):Void { check(Math.abs(value - expected) < 1e-5, 'expected $expected, got $value'); }

	public static function run():Void {
		var sketch = new ConstrainedSketch();
		sketch.addPoint(new SketchPoint("p0", 0.2, -0.1)).addPoint(new SketchPoint("p1", 9.7, 0.3))
			.addPoint(new SketchPoint("p2", 10.4, 5.1)).addPoint(new SketchPoint("p3", -0.3, 4.8));
		sketch.addEntity(SketchEntity.line("bottom", "p0", "p1")).addEntity(SketchEntity.line("right", "p1", "p2"))
			.addEntity(SketchEntity.line("top", "p2", "p3")).addEntity(SketchEntity.line("left", "p3", "p0"));
		sketch.addConstraint(SketchConstraint.fixed("origin", "p0"))
			.addConstraint(SketchConstraint.horizontal("h0", "bottom")).addConstraint(SketchConstraint.horizontal("h1", "top"))
			.addConstraint(SketchConstraint.vertical("v0", "right")).addConstraint(SketchConstraint.vertical("v1", "left"))
			.addConstraint(SketchConstraint.distance("width", "p0", "p1", 10))
			.addConstraint(SketchConstraint.distance("height", "p1", "p2", 5));
		var solved = sketch.solve();
		near(solved.x("p1") - solved.x("p0"), 10); near(solved.y("p2") - solved.y("p1"), 5);
		check(solved.diagnostic.converged, "rectangle converges");

		var circle = new ConstrainedSketch();
		circle.addPoint(new SketchPoint("c", 2, 3)).addEntity(SketchEntity.circle("circle", "c", 1.2));
		circle.addConstraint(SketchConstraint.fixed("center", "c")).addConstraint(SketchConstraint.radius("r", "circle", 4));
		near(circle.solve().radius("circle"), 4);

		var loose = new ConstrainedSketch();
		loose.addPoint(new SketchPoint("a", 0, 0)).addPoint(new SketchPoint("b", 2, 1)).addEntity(SketchEntity.line("l", "a", "b"));
		loose.addConstraint(SketchConstraint.horizontal("horizontal", "l"));
		check(loose.solve().diagnostic.status == "under-constrained", "local degrees of freedom");
		loose.addConstraint(SketchConstraint.horizontal("duplicate", "l"));
		check(loose.solve().diagnostic.status == "redundant", "redundancy diagnosis");

		var conflict = new ConstrainedSketch();
		conflict.addPoint(new SketchPoint("a", 0, 0)).addPoint(new SketchPoint("b", 1, 0));
		conflict.addConstraint(SketchConstraint.fixed("fa", "a")).addConstraint(SketchConstraint.fixed("fb", "b"))
			.addConstraint(SketchConstraint.distance("impossible", "a", "b", 2));
		var failed = false;
		try conflict.solve() catch (error:SketchSolveError) {
			failed = error.diagnostic.status == "conflicting" && error.diagnostic.constraintIds.length > 0;
		}
		check(failed, "conflicting constraints are distinct");

		var limited = new ConstrainedSketch(null, "mm", new SolverSettings(1e-12, 1e-7, 1, 1e-3));
		limited.addPoint(new SketchPoint("a", 0, 0)).addPoint(new SketchPoint("b", 100, 50));
		limited.addConstraint(SketchConstraint.fixed("fixed", "a")).addConstraint(SketchConstraint.distance("target", "a", "b", 2));
		failed = false;
		try limited.solve() catch (error:SketchSolveError) failed = error.diagnostic.status == "nonconvergent";
		check(failed, "iteration exhaustion is nonconvergence");

		var invalid = new ConstrainedSketch(); invalid.addPoint(new SketchPoint("bad", 0 / 0, 0));
		failed = false;
		try invalid.solve() catch (error:SketchSolveError) failed = error.diagnostic.status == "invalid" && error.diagnostic.constraintIds[0] == "bad";
		check(failed, "non-finite authored input is invalid");

		var mixed=new ConstrainedSketch();
		for(p in [new SketchPoint("m0",0,0),new SketchPoint("m1",2,0),new SketchPoint("m2",0,1),new SketchPoint("m3",2,1),
			new SketchPoint("m4",0,2),new SketchPoint("mc1",0,4),new SketchPoint("mc2",2,4),new SketchPoint("ms0",-1,2),new SketchPoint("ms1",1,2)])mixed.addPoint(p);
		mixed.addEntity(SketchEntity.line("ml1","m0","m1")).addEntity(SketchEntity.line("ml2","m2","m3"))
			.addEntity(SketchEntity.line("ml3","m0","m4")).addEntity(SketchEntity.circle("mcircle1","mc1",1))
			.addEntity(SketchEntity.circle("mcircle2","mc2",1)).addEntity(SketchEntity.circle("mcircle3","mc1",1))
			.addEntity(SketchEntity.arc("marc","mc1",1,0,Math.PI,false,true));
		mixed.addConstraint(SketchConstraint.equal("equal.lines","ml1","ml2")).addConstraint(SketchConstraint.parallel("parallel","ml1","ml2"))
			.addConstraint(SketchConstraint.perpendicular("perpendicular","ml1","ml3")).addConstraint(SketchConstraint.angle("angle","ml1","ml3",Math.PI/2))
			.addConstraint(SketchConstraint.concentric("concentric","mcircle1","mcircle3")).addConstraint(SketchConstraint.pointOn("point.on","m1","ml1"))
			.addConstraint(SketchConstraint.tangent("tangent.circles","mcircle1","mcircle2")).addConstraint(SketchConstraint.symmetric("symmetric","ms0","ms1","ml3"))
			.addConstraint(SketchConstraint.equal("equal.radii","mcircle1","marc"));
		check(mixed.solve().diagnostic.converged,"expanded constraint set");

		var open=new ConstrainedSketch();open.addPoint(new SketchPoint("o0",0,0)).addPoint(new SketchPoint("o1",1,0)).addEntity(SketchEntity.line("open.line","o0","o1"));
		failed=false;try SketchProfile.build(open,open.solve()) catch(error:ProfileError) failed=error.kind=="open"&&error.entityIds.length>0;
		check(failed,"open profiles identify entities");

		var profile = new ConstrainedSketch();
		profile.addPoint(new SketchPoint("q0", -10, -5)).addPoint(new SketchPoint("q1", 10, -5))
			.addPoint(new SketchPoint("q2", 10, 5)).addPoint(new SketchPoint("q3", -10, 5));
		profile.addEntity(SketchEntity.line("qb", "q0", "q1")).addEntity(SketchEntity.line("qr", "q1", "q2"))
			.addEntity(SketchEntity.line("qt", "q2", "q3")).addEntity(SketchEntity.line("ql", "q3", "q0"));
		for (i in 0...4) {
			var id = "hc" + i;
			profile.addPoint(new SketchPoint(id, i % 2 == 0 ? -6 : 6, i < 2 ? -2 : 2));
			profile.addEntity(SketchEntity.circle("hole" + i, id, 1));
			profile.addConstraint(SketchConstraint.fixed("fix" + i, id));
			profile.addConstraint(SketchConstraint.radius("holeRadius" + i, "hole" + i, 1));
		}
		for (i in 0...4) profile.addConstraint(SketchConstraint.fixed("corner" + i, "q" + i));
		var face = SketchProfile.build(profile, profile.solve());
		var solid = face.extrude(3);
		near(solid.shape.volume(), (200 - 4 * Math.PI) * 3);
		solid.close(); face.close();

		var capsule = new ConstrainedSketch();
		for (point in [
			new SketchPoint("cap.left", -3, 0), new SketchPoint("cap.right", 3, 0),
			new SketchPoint("cap.bl", -3, -2), new SketchPoint("cap.br", 3, -2),
			new SketchPoint("cap.tl", -3, 2), new SketchPoint("cap.tr", 3, 2),
			new SketchPoint("cap.hole", 0, 0), new SketchPoint("cap.island", 0, 0)
		]) capsule.addPoint(point);
		capsule.addEntity(SketchEntity.line("cap.bottom", "cap.bl", "cap.br"))
			.addEntity(SketchEntity.arc("cap.rightArc", "cap.right", 2, -Math.PI / 2, Math.PI / 2))
			// Authored left-to-right so loop assembly must reverse this edge.
			.addEntity(SketchEntity.line("cap.top", "cap.tl", "cap.tr"))
			.addEntity(SketchEntity.arc("cap.leftArc", "cap.left", 2, Math.PI / 2, 3 * Math.PI / 2))
			.addEntity(SketchEntity.circle("cap.holeCircle", "cap.hole", 1))
			.addEntity(SketchEntity.circle("cap.islandCircle", "cap.island", 0.4));
		var capsuleFace = SketchProfile.build(capsule, capsule.solve());
		near(capsuleFace.shape.area(), 24 + 4 * Math.PI - Math.PI + 0.16 * Math.PI);
		capsuleFace.close();

		var overlap = new ConstrainedSketch();
		overlap.addPoint(new SketchPoint("overlap.a", 0, 0)).addPoint(new SketchPoint("overlap.b", 1, 0))
			.addEntity(SketchEntity.circle("overlap.first", "overlap.a", 2))
			.addEntity(SketchEntity.circle("overlap.second", "overlap.b", 2));
		failed = false;
		try SketchProfile.build(overlap, overlap.solve()) catch (error:ProfileError)
			failed = error.kind == "overlapping" && error.entityIds.length == 2;
		check(failed, "overlapping curved boundaries identify entities");

		var document=new Document();
		var feature=document.add(new ConstrainedSketchFeature(sketch));
		var width=document.defineParameter("width",10);width.bind(feature.dimension("width"));
		var extrude=document.add(new ExtrudeFeature(feature,0,0,2));document.setOutput(extrude);document.recompute();
		near(document.result().volume(),100);
		for (nextWidth in [7.0, 14.0, 9.0]) {
			width.set(nextWidth);
			document.recompute();
			near(document.result().volume(), nextWidth * 5 * 2);
		}
		width.set(10);document.recompute();near(document.result().volume(),100);
		width.set(12);document.recompute();near(document.result().volume(),120);
		check(document.undo(),"constrained sketch undo");document.recompute();near(document.result().volume(),100);
		var baseConstraintCount = feature.sketch().constraints().length;
		var detached = feature.sketch();
		detached.addConstraint(SketchConstraint.parallel("detached", "bottom", "top"));
		check(feature.sketch().constraints().length == baseConstraintCount, "feature sketch snapshots are detached");
		feature.addConstraint(SketchConstraint.parallel("edit.parallel", "bottom", "top"));
		document.recompute();
		check(feature.sketch().constraints().length == baseConstraintCount + 1, "constraint addition");
		check(document.undo(), "constraint addition undo"); document.recompute();
		check(feature.sketch().constraints().length == baseConstraintCount, "constraint addition undone");
		check(document.redo(), "constraint addition redo"); document.recompute();
		check(feature.sketch().constraints().length == baseConstraintCount + 1, "constraint addition redone");
		var editedReload = DocumentCodec.decode(DocumentCodec.encode(document));
		var editedFeature:ConstrainedSketchFeature = cast editedReload.featureAt(0);
		check(editedFeature.sketch().constraints().length == baseConstraintCount + 1, "constraint addition reload");
		editedReload.close();
		feature.removeConstraint("edit.parallel"); document.recompute();
		check(document.undo(), "constraint removal undo"); document.recompute();
		check(feature.sketch().constraints().length == baseConstraintCount + 1, "constraint removal undone");
		check(document.redo(), "constraint removal redo"); document.recompute();
		check(feature.sketch().constraints().length == baseConstraintCount, "constraint removal redone");
		failed = false;
		try feature.removePoint("p0") catch (error:Dynamic) failed = true;
		check(failed && feature.sketch().points().length == 4, "referenced point removal is rejected atomically");
		var widthIdentity = feature.dimension("width");
		width.set(12); document.recompute();
		feature.removeConstraint("width"); document.recompute();
		check(document.undo(), "dimensional constraint removal undo");
		check(feature.dimension("width") == widthIdentity && feature.dimension("width").value == 12,
			"dimensional constraint undo preserves identity and value");
		check(document.undo(), "dimension value undo after constraint restore");
		check(feature.dimension("width") == widthIdentity && feature.dimension("width").value == 10,
			"parameter history retains restored identity");
		check(document.redo(), "dimension value redo after constraint restore");
		check(feature.dimension("width").value == 12, "parameter redo uses restored identity");
		feature.replaceConstraint(SketchConstraint.distance("width", "p0", "p1", 20));
		check(feature.dimension("width") == widthIdentity && feature.dimension("width").value == 20,
			"dimensional replacement preserves identity and applies its value");
		document.recompute(); near(document.result().volume(), 200);
		check(document.undo(), "dimensional replacement undo"); document.recompute();
		check(feature.dimension("width").value == 12, "dimensional replacement restores its value");
		width.set(10); document.recompute();

		var loaded=DocumentCodec.decode(DocumentCodec.encode(document));near(loaded.result().volume(),100);
		loaded.parameter("width").set(14);loaded.recompute();near(loaded.result().volume(),140);
		loaded.close();document.close();

		var locked = new ConstrainedSketch();
		for (point in [new SketchPoint("l0", 0, 0), new SketchPoint("l1", 10, 0), new SketchPoint("l2", 10, 5),
			new SketchPoint("l3", 0, 5)]) locked.addPoint(point);
		locked.addEntity(SketchEntity.line("locked.bottom", "l0", "l1"))
			.addEntity(SketchEntity.line("locked.right", "l1", "l2"))
			.addEntity(SketchEntity.line("locked.top", "l2", "l3"))
			.addEntity(SketchEntity.line("locked.left", "l3", "l0"));
		for (index in 0...4)
			locked.addConstraint(SketchConstraint.fixed("locked.fixed" + index, "l" + index));
		locked.addConstraint(SketchConstraint.distance("locked.width", "l0", "l1", 10));
		var failureDocument = new Document();
		var lockedFeature = failureDocument.add(new ConstrainedSketchFeature(locked));
		failureDocument.setOutput(lockedFeature);
		failureDocument.recompute();
		var committedShape = failureDocument.result();
		lockedFeature.dimension("locked.width").set(12);
		failed = false;
		try failureDocument.recompute() catch (error:Dynamic) failed = true;
		check(failed && failureDocument.result() == committedShape && lockedFeature.lastDiagnostic != null
			&& lockedFeature.lastAttemptDiagnostic != null && lockedFeature.lastAttemptDiagnostic.status == "conflicting",
			"failed attempt diagnostics are separate from committed geometry");
		failureDocument.close();

		var plate=new ConstrainedMountingPlate();
		var oldVolume=plate.finish.currentShape().volume();
		plate.resize(100,60,12,4);check(plate.finish.currentShape().volume()>oldVolume,"mounting plate recompute");
		check(plate.document.undo(),"mounting plate undo");plate.document.recompute();near(plate.finish.currentShape().volume(),oldVolume);
		check(plate.document.redo(),"mounting plate redo");plate.document.recompute();
		var saved=DocumentCodec.decode(DocumentCodec.encode(plate.document));near(saved.result().volume(),plate.finish.currentShape().volume());saved.close();
		var previous=plate.finish.currentShape();failed=false;try plate.resize(20,20,9,3) catch(error:Dynamic) failed=true;
		check(failed&&previous==plate.finish.currentShape(),"contradictory plate edit preserves solid");plate.close();

		var bracket = new ConstrainedSlottedBracket();
		var bracketVolume = bracket.finish.currentShape().volume();
		check(bracketVolume > 0, "slotted bracket initial solid");
		for (dimensions in [
			[90.0, 55.0, 3.5, 7.0, 2.0],
			[100.0, 60.0, 4.0, 8.0, 2.5],
			[85.0, 48.0, 2.5, 5.0, 1.5]
		]) {
			bracket.resize(dimensions[0], dimensions[1], dimensions[2], dimensions[3], dimensions[4]);
			check(bracket.finish.currentShape().volume() > 0, "slotted bracket dimension set");
		}
		check(bracket.document.undo(), "slotted bracket undo");
		bracket.document.recompute();
		check(bracket.finish.currentShape().volume() > 0, "slotted bracket undo recompute");
		bracket.close();
	}
}
