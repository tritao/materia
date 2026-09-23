import cadkit.sketch.ConstrainedSketch;
import cadkit.sketch.SketchConstraint;
import cadkit.sketch.SketchEntity;
import cadkit.sketch.SketchPoint;
import cadkit.sketch.SketchSolveError;
import cadkit.sketch.SketchProfile;
import cadkit.sketch.ProfileError;
import cadkit.sketch.SketchSession;
import cadkit.parametric.Document;
import cadkit.parametric.DocumentCodec;
import cadkit.parametric.features.ConstrainedSketchFeature;
import cadkit.parametric.features.ExtrudeFeature;
import cadkit.sketch.SolverSettings;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import cadkit.parametric.features.SketchFeature;
import cadkit.parametric.recording.DocumentBuilder;
import cadkit.parametric.SelectionRecipe;
import cadkit.parametric.features.BoxFeature;
import cadkit.parametric.features.PocketFeature;
import cadkit.parametric.features.BooleanFeature;
import cadkit.parametric.features.BooleanOperation;
import cadkit.parametric.features.TransformFeature;

class ConstrainedSketchSmoke {
	static function check(value:Bool, message:String):Void { if (!value) throw message; }
	static function near(value:Float, expected:Float):Void { check(Math.abs(value - expected) < 1e-5, 'expected $expected, got $value'); }
	static function failObserver():Void { throw "observer failure"; }
	static function fixedRectangle(width:Float, height:Float):ConstrainedSketch {
		var sketch = new ConstrainedSketch();
		var coordinates = [
			[-width / 2, -height / 2], [width / 2, -height / 2],
			[width / 2, height / 2], [-width / 2, height / 2]
		];
		for (index in 0...4) {
			sketch.addPoint(new SketchPoint("rectangle.point" + index, coordinates[index][0], coordinates[index][1]));
			sketch.addConstraint(SketchConstraint.fixed("rectangle.fixed" + index, "rectangle.point" + index));
		}
		for (index in 0...4)
			sketch.addEntity(SketchEntity.line("rectangle.edge" + index, "rectangle.point" + index,
				"rectangle.point" + ((index + 1) % 4)));
		return sketch;
	}

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

		var openSketch = new ConstrainedSketch();
		openSketch.addPoint(new SketchPoint("draft.a", 0, 0)).addPoint(new SketchPoint("draft.b", 4, 1));
		openSketch.addEntity(SketchEntity.line("draft.edge", "draft.a", "draft.b"));
		var sketchSession = new SketchSession(openSketch);
		var underconstrainedSolution:cadkit.sketch.SolvedSketch = cast sketchSession.solution;
		check(sketchSession.isSolved && sketchSession.degreesOfFreedom == 4
			&& underconstrainedSolution.diagnostic.status == "under-constrained",
			"unfinished open sketches expose a solved state and remaining freedom");
		var openProfileRejected = false;
		try sketchSession.buildProfile() catch (error:ProfileError) openProfileRejected = error.kind == "open";
		check(openProfileRejected, "profile construction remains a separate closed-boundary requirement");
		check(sketchSession.edit(function(draft) {
			draft.addConstraint(SketchConstraint.fixed("draft.lock-a", "draft.a"));
			draft.addConstraint(SketchConstraint.fixed("draft.lock-b", "draft.b"));
		}), "draft accepts a solvable edit");
		var previousDraftSolution = sketchSession.lastValidSolution;
		check(!sketchSession.edit(function(draft)
			draft.addConstraint(SketchConstraint.distance("draft.conflict", "draft.a", "draft.b", 10))),
			"conflicting draft edit is reported without discarding the authored draft");
		var conflictDiagnostic:cadkit.sketch.SolveDiagnostic = cast sketchSession.diagnostic;
		check(!sketchSession.isSolved && conflictDiagnostic.status == "conflicting"
			&& sketchSession.conflictingConstraintIds.indexOf("draft.conflict") >= 0
			&& sketchSession.lastValidSolution == previousDraftSolution,
			"draft conflict exposes implicated constraints and retains the previous valid solution");
		check(sketchSession.edit(function(draft) draft.removeConstraint("draft.conflict"))
			&& sketchSession.isSolved && sketchSession.degreesOfFreedom == 0,
			"draft can recover after a conflicting constraint is removed");

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

		var translated = new ConstrainedSketch();
		translated.addPoint(new SketchPoint("translated.a", 1e9, 1e9))
			.addPoint(new SketchPoint("translated.b", 1e9 + 10, 1e9))
			.addConstraint(SketchConstraint.fixed("translated.fixed", "translated.a"))
			.addConstraint(SketchConstraint.distance("translated.distance", "translated.a", "translated.b", 12));
		var translatedResult = translated.solve();
		near(translatedResult.x("translated.b") - translatedResult.x("translated.a"), 12);
		check(translatedResult.diagnostic.iterations > 0, "translation does not relax linear tolerance");
		var translatedAngle = new ConstrainedSketch();
		translatedAngle.addPoint(new SketchPoint("angle.a", 1e9, 1e9))
			.addPoint(new SketchPoint("angle.b", 1e9 + 10, 1e9))
			.addPoint(new SketchPoint("angle.c", 1e9 + 1, 1e9 + 5))
			.addEntity(SketchEntity.line("angle.horizontal", "angle.a", "angle.b"))
			.addEntity(SketchEntity.line("angle.vertical", "angle.a", "angle.c"))
			.addConstraint(SketchConstraint.fixed("angle.fixedA", "angle.a"))
			.addConstraint(SketchConstraint.fixed("angle.fixedB", "angle.b"))
			.addConstraint(SketchConstraint.distance("angle.length", "angle.a", "angle.c", 5))
			.addConstraint(SketchConstraint.perpendicular("angle.perpendicular", "angle.horizontal", "angle.vertical"));
		var translatedAngleResult = translatedAngle.solve();
		near(translatedAngleResult.x("angle.c") - translatedAngleResult.x("angle.a"), 0);

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
		failed = false;
		var widthRemovalError = "";
		try feature.removeConstraint("width") catch (error:Dynamic) {
			var detail:Dynamic = Reflect.field(error, "message");
			widthRemovalError = detail == null ? Std.string(error) : cast detail;
			failed = widthRemovalError.indexOf("named parameter width") >= 0;
		}
		check(failed && feature.sketch().constraints().length == baseConstraintCount
			&& feature.dimension("width") == widthIdentity,
			"bound dimensional constraint removal explains its dependency; error=" + widthRemovalError);
		width.unbind(widthIdentity);
		feature.removeConstraint("width"); document.recompute();
		failed = false;
		try widthIdentity.set(13) catch (error:Dynamic) failed = true;
		check(failed, "removed dimension parameters cannot be edited through stale handles");
		var removedDimensionReload = DocumentCodec.decode(DocumentCodec.encode(document), false, false);
		var removedDimensionFeature:ConstrainedSketchFeature = cast removedDimensionReload.featureAt(0);
		var removedNamedWidth = removedDimensionReload.parameter("width");
		failed = false;
		try removedDimensionFeature.dimension("width") catch (error:Dynamic) failed = true;
		check(failed && removedNamedWidth.bindings().length == 0 && removedNamedWidth.value == 12,
			"removed dimensions and their bindings do not persist as stale parameters");
		removedDimensionReload.close();
		check(document.undo(), "dimensional constraint removal undo");
		check(feature.dimension("width") == widthIdentity && feature.dimension("width").value == 12,
			"dimensional constraint undo preserves identity and value");
		width.bind(feature.dimension("width"));
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
		var anchorBefore=feature.sketch().points()[0];
		feature.replacePoint(new SketchPoint(anchorBefore.id,anchorBefore.x+2,anchorBefore.y+3));
		document.recompute();
		check(feature.sketch().points()[0].x==anchorBefore.x+2,"authored point replacement moves a constrained profile");
		check(document.undo(),"authored point replacement undo");document.recompute();
		check(feature.sketch().points()[0].x==anchorBefore.x,"authored point replacement restores its prior position");
		check(document.redo(),"authored point replacement redo");document.recompute();
		check(feature.sketch().points()[0].x==anchorBefore.x+2,"authored point replacement replays");
		feature.replacePoint(anchorBefore);document.recompute();
		width.set(10); document.recompute();
		var cancelAnchor=feature.sketch().points()[0];
		var cancelTransaction=document.beginTransaction();
		width.set(13);
		feature.replacePoint(new SketchPoint(cancelAnchor.id,cancelAnchor.x+4,cancelAnchor.y));
		cancelTransaction.cancel();
		check(width.value==10&&feature.dimension("width").value==10
			&&feature.sketch().points()[0].x==cancelAnchor.x,
			"cancel restores parameter bindings after authored sketch changes");
		document.recompute();

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

		var stagedDocument = new Document();
		var stagedSketch = stagedDocument.add(new ConstrainedSketchFeature(sketch));
		var stagedExtrude = stagedDocument.add(new ExtrudeFeature(stagedSketch, 0, 0, 2));
		stagedDocument.setOutput(stagedExtrude);
		stagedDocument.recompute();
		var committedSolution = stagedSketch.solvedSketch();
		var committedDiagnostic = stagedSketch.lastDiagnostic;
		var committedSolid = stagedDocument.result();
		stagedSketch.dimension("width").set(16);
		stagedExtrude.z.set(0);
		failed = false;
		try stagedDocument.recompute() catch (error:Dynamic) failed = true;
		check(failed && stagedDocument.result() == committedSolid && stagedSketch.solvedSketch() == committedSolution
			&& stagedSketch.lastDiagnostic == committedDiagnostic,
			"downstream failure rolls back staged solver state with geometry");
		stagedExtrude.z.set(2);
		stagedDocument.recompute();
		check(stagedSketch.solvedSketch() != committedSolution, "successful recompute commits staged solver state");
		stagedDocument.close();

		var observerDocument = new Document();
		var observerBox = observerDocument.add(new BoxFeature(10, 8, 2));
		observerDocument.setOutput(observerBox);
		observerDocument.recompute();
		observerBox.width.set(12);
		observerDocument.afterRecompute = failObserver;
		failed = false;
		try observerDocument.recompute() catch (error:Dynamic) failed = true;
		check(failed, "post-recompute observer failures are reported");
		near(observerDocument.result().volume(), 192);
		var publishedShape = observerBox.currentShape();
		check(publishedShape != null && !publishedShape.isClosed(),
			"observer failure leaves the newly published shape open");
		observerDocument.afterRecompute = null;
		observerDocument.close();

		var workplaneDocument = DocumentBuilder.build(function(builder) {
			var width = builder.dimension("workplane.width", 20);
			var height = builder.dimension("workplane.height", 10);
			var amount = builder.dimension("workplane.amount", 5);
			var workplaneProfile = builder.rectangle(width, height, Plane.YZ());
			builder.output(builder.extrude(workplaneProfile, amount));
		});
		near(workplaneDocument.result().volume(), 1000);
		near(workplaneDocument.result().bounds().get_max().get_x(), 5);
		var restoredWorkplane = DocumentCodec.decode(DocumentCodec.encode(workplaneDocument));
		near(restoredWorkplane.result().volume(), 1000);
		restoredWorkplane.close(); workplaneDocument.close();

		var directionalDocument = new Document();
		var directionalProfile = directionalDocument.add(new SketchFeature("rectangle", 20, 10));
		var arbitrary = directionalDocument.add(ExtrudeFeature.along(directionalProfile, 5, new Vector(1, 0, 1)));
		directionalDocument.setOutput(arbitrary); directionalDocument.recompute();
		near(directionalDocument.result().volume(), 1000 / Math.pow(2, 0.5));
		directionalDocument.close();

		var reversedDocument = new Document();
		var reversedProfile = reversedDocument.add(new SketchFeature("rectangle", 20, 10));
		var reversedExtrude = reversedDocument.add(ExtrudeFeature.along(reversedProfile, 5, Vector.Z(), true));
		reversedDocument.setOutput(reversedExtrude); reversedDocument.recompute();
		near(reversedDocument.result().bounds().get_min().get_z(), -5);
		near(reversedDocument.result().bounds().get_max().get_z(), 0);
		reversedDocument.close();

		var symmetricDocument = new Document();
		var symmetricProfile = symmetricDocument.add(new SketchFeature("rectangle", 20, 10));
		var symmetricExtrude = symmetricDocument.add(ExtrudeFeature.along(symmetricProfile, 5, Vector.Z(), false, true));
		symmetricDocument.setOutput(symmetricExtrude); symmetricDocument.recompute();
		near(symmetricDocument.result().bounds().get_min().get_z(), -2.5);
		near(symmetricDocument.result().bounds().get_max().get_z(), 2.5);
		symmetricDocument.close();

		var attachedModel = new ConstrainedSketch();
		attachedModel.addPoint(new SketchPoint("attached.center", 0, 0))
			.addEntity(SketchEntity.circle("attached.circle", "attached.center", 2))
			.addConstraint(SketchConstraint.fixed("attached.fixed", "attached.center"))
			.addConstraint(SketchConstraint.radius("attached.radius", "attached.circle", 2));
		var topSelection = new SelectionRecipe("face", "plane", Vector.Z(), "max", Vector.Z(), 1);
		var attachedDocument = new Document();
		var attachedBox = attachedDocument.add(new BoxFeature(20, 30, 10));
		var attachedFeature = attachedDocument.add(new ConstrainedSketchFeature(attachedModel, attachedBox, topSelection, Vector.X(), 2));
		var flippedFeature = attachedDocument.add(new ConstrainedSketchFeature(attachedModel, attachedBox, topSelection, Vector.X(), 2, true));
		attachedDocument.setOutput(attachedFeature); attachedDocument.recompute();
		near(attachedDocument.result().center().get_z(), 12);
		near(flippedFeature.currentShape().center().get_z(), 8);
		attachedBox.height.set(15); attachedBox.width.set(24); attachedDocument.recompute();
		near(attachedDocument.result().center().get_x(), 12);
		near(attachedDocument.result().center().get_z(), 17);
		near(flippedFeature.currentShape().center().get_z(), 13);
		var restoredAttached = DocumentCodec.decode(DocumentCodec.encode(attachedDocument));
		near(restoredAttached.result().center().get_z(), 17);
		restoredAttached.close(); attachedDocument.close();

		var ambiguousDocument = new Document();
		var ambiguousBox = ambiguousDocument.add(new BoxFeature(20, 30, 10));
		var ambiguousFaces = new SelectionRecipe("face", "plane", Vector.Z(), "all", Vector.Z(), 1);
		var ambiguousAttached = ambiguousDocument.add(new ConstrainedSketchFeature(attachedModel, ambiguousBox, ambiguousFaces, Vector.X()));
		ambiguousDocument.setOutput(ambiguousAttached);
		failed = false;
		try ambiguousDocument.recompute() catch (error:Dynamic) failed = true;
		check(failed, "ambiguous attached face selection is rejected");
		ambiguousDocument.close();

		var pocketDocument = new Document();
		var pocketBox = pocketDocument.add(new BoxFeature(20, 30, 10));
		var pocketProfile = pocketDocument.add(new ConstrainedSketchFeature(fixedRectangle(8, 6), pocketBox, topSelection, Vector.X()));
		var blindPocket = pocketDocument.add(PocketFeature.blind(pocketBox, pocketProfile, 3));
		var pocketDepth = pocketDocument.defineParameter("pocket.depth", 3);
		pocketDepth.bind(blindPocket.depth);
		pocketDocument.setOutput(blindPocket);
		pocketDocument.recompute();
		near(pocketDocument.result().volume(), 20 * 30 * 10 - 8 * 6 * 3);
		pocketDepth.set(4);
		pocketDocument.recompute();
		near(pocketDocument.result().volume(), 20 * 30 * 10 - 8 * 6 * 4);
		check(pocketDocument.undo(), "pocket depth undo");
		pocketDocument.recompute();
		near(pocketDocument.result().volume(), 20 * 30 * 10 - 8 * 6 * 3);
		check(pocketDocument.redo(), "pocket depth redo");
		pocketDocument.recompute();
		near(pocketDocument.result().volume(), 20 * 30 * 10 - 8 * 6 * 4);
		pocketBox.height.set(15);
		pocketDocument.recompute();
		near(pocketDocument.result().volume(), 20 * 30 * 15 - 8 * 6 * 4);
		near(pocketProfile.currentShape().center().get_z(), 15);
		var restoredPocket = DocumentCodec.decode(DocumentCodec.encode(pocketDocument));
		near(restoredPocket.result().volume(), pocketDocument.result().volume());
		check(restoredPocket.parameter("pocket.depth").value == 4, "pocket named depth reload");
		restoredPocket.close();
		pocketDocument.close();

		var throughDocument = new Document();
		var throughBox = throughDocument.add(new BoxFeature(20, 30, 10));
		var throughProfile = throughDocument.add(new ConstrainedSketchFeature(fixedRectangle(8, 6), throughBox, topSelection, Vector.X()));
		var throughPocket = throughDocument.add(PocketFeature.throughAll(throughBox, throughProfile));
		throughDocument.setOutput(throughPocket);
		throughDocument.recompute();
		near(throughDocument.result().volume(), (20 * 30 - 8 * 6) * 10);
		throughBox.height.set(15);
		throughDocument.recompute();
		near(throughDocument.result().volume(), (20 * 30 - 8 * 6) * 15);
		var restoredThrough = DocumentCodec.decode(DocumentCodec.encode(throughDocument));
		near(restoredThrough.result().volume(), throughDocument.result().volume());
		restoredThrough.close();
		throughDocument.close();

		var enclosure = new AttachedPocketEnclosure();
		var enclosureVolume = enclosure.finish.currentShape().volume();
		check(enclosureVolume > 0, "attached pocket enclosure initial solid");
		enclosure.resize(70, 50, 35);
		check(enclosure.finish.currentShape().volume() > enclosureVolume, "attached pockets survive enclosure resize");
		near(enclosure.topProfile.currentShape().center().get_z(), 35);
		near(enclosure.sideProfile.currentShape().bounds().get_min().get_x(), 70);
		near(enclosure.sideProfile.currentShape().bounds().get_max().get_x(), 70);
		var restoredEnclosure = DocumentCodec.decode(DocumentCodec.encode(enclosure.document));
		near(restoredEnclosure.result().volume(), enclosure.finish.currentShape().volume());
		restoredEnclosure.close();
		enclosure.close();

		var selectionFailureDocument = new Document();
		var selectionBase = selectionFailureDocument.add(new BoxFeature(20, 30, 10));
		var selectionCutter = selectionFailureDocument.add(new BoxFeature(2, 30, 20));
		var movingCutter = selectionFailureDocument.add(new TransformFeature(selectionCutter, 30, 0, 0));
		var changingSupport = selectionFailureDocument.add(new BooleanFeature(selectionBase, movingCutter, BooleanOperation.Cut));
		var failureProfile = selectionFailureDocument.add(new ConstrainedSketchFeature(fixedRectangle(4, 4), changingSupport,
			topSelection, Vector.X()));
		var failurePocket = selectionFailureDocument.add(PocketFeature.blind(changingSupport, failureProfile, 2));
		selectionFailureDocument.setOutput(failurePocket);
		selectionFailureDocument.recompute();
		var committedPocket = selectionFailureDocument.result();
		movingCutter.x.set(9);
		failed = false;
		try selectionFailureDocument.recompute() catch (error:Dynamic) failed = true;
		check(failed && selectionFailureDocument.result() == committedPocket,
			"failed attached face selection preserves committed pocket geometry");
		selectionFailureDocument.close();

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
