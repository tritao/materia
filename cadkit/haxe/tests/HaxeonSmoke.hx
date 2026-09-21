import CadKit;
import cadkit.Edge;
import cadkit.Geometry;
import cadkit.Operation;
import cadkit.Shape;
import cadkit.SelectionError;
import cadkit.SelectionErrorKind;
import cadkit.parametric.Document;
import cadkit.parametric.DocumentCodec;
import cadkit.parametric.ReferenceState;
import cadkit.parametric.RecomputeError;
import cadkit.parametric.features.BooleanFeature;
import cadkit.parametric.features.BooleanOperation;
import cadkit.parametric.features.BoxFeature;
import cadkit.parametric.features.CylinderFeature;
import cadkit.parametric.features.ExtrudeFeature;
import cadkit.parametric.features.FaceFeature;
import cadkit.parametric.features.FilletFeature;
import cadkit.parametric.features.ChamferFeature;
import cadkit.parametric.features.RevolveFeature;
import cadkit.parametric.features.TransformFeature;
import cadkit.parametric.TopologyReference;
import cadkit.parametric.TopologyFingerprint;
import cadkit.parametric.TopologyHistoryMap;
import sys.FileSystem;

class HaxeonSmoke {
	static function historyRelationIsMapped(
		operation:Operation,
		relation:CadKit.HistoryRelation):Bool {
		var count = operation.historyCount(relation);
		if (count == 0)
			return true;
		var source = operation.historySourceAt(relation, 0);
		var result = new TopologyHistoryMap(operation).remap(source, source.kind());
		var valid:Bool;
		if (relation == CadKit.HistoryRelation.Deleted) {
			valid = result.state == ReferenceState.Deleted && result.shape == null;
		} else {
			valid = (result.state == ReferenceState.Remapped ||
				result.state == ReferenceState.Ambiguous);
			if (result.state == ReferenceState.Remapped && result.shape == null)
				valid = false;
		}
		if (result.shape != null)
			result.shape.close();
		source.close();
		return valid;
	}

	static function selectorReportsAmbiguous(box:Shape):Bool {
		try {
			box.faces().query()
				.surface(CadKit.SurfaceKind.Plane)
				.normalParallelTo(Geometry.vec3(0.0, 0.0, 2.0))
				.unique();
		} catch (error:Dynamic) {
			var selectionError:SelectionError = cast error;
			return selectionError.kind == SelectionErrorKind.Ambiguous;
		}
		return false;
	}

	static function main():Int {
		var box = Shape.box(10.0, 20.0, 30.0);
		var bounds = box.bounds();
		if (bounds.get_max().get_z() != 30.0)
			return 1;
		var stepPath = "/tmp/cadkit-haxeon-smoke.step";
		if (FileSystem.exists(stepPath))
			FileSystem.deleteFile(stepPath);
		box.exportStep(stepPath);
		if (!FileSystem.exists(stepPath))
			return 79;
		var importedStep = Shape.importStep(stepPath);
		if (Math.abs(importedStep.volume() - box.volume()) > 0.000001 ||
			importedStep.bounds().get_max().get_z() != 30.0)
			return 80;
		importedStep.close();
		FileSystem.deleteFile(stepPath);
		if (box.kind() != CadKit.ShapeKind.Solid)
			return 2;
		var faces = box.faces();
		if (faces.count() != 6)
			return 3;

		var face = faces.at(0);
		if (face.surfaceKind() != CadKit.SurfaceKind.Plane)
			return 4;
		if (face.area() <= 0.0)
			return 5;
		var faceNormal = face.normal();
		if (faceNormal.get_x() * faceNormal.get_x() +
			faceNormal.get_y() * faceNormal.get_y() +
			faceNormal.get_z() * faceNormal.get_z() < 0.99)
			return 6;
		var faceAgain = faces.at(0);
		if (!face.sameAs(faceAgain))
			return 7;
		var upward = faces.first(function(candidate) return candidate.normal().get_z() > 0.99);
		if (upward == null)
			return 8;
		face.center();
		face.close();
		faceAgain.close();
		upward.close();

		var top = box.faces().query()
			.surface(CadKit.SurfaceKind.Plane)
			.centerNear(Geometry.vec3(5.0, 10.0, 30.0), 0.001)
			.unique();
		if (Math.abs(top.area() - 200.0) > 0.000001)
			return 46;
		top.close();
		var parallelFaces = box.faces().query()
			.surface(CadKit.SurfaceKind.Plane)
			.normalParallelTo(Geometry.vec3(0.0, 0.0, 2.0))
			.all();
		if (parallelFaces.length != 2)
			return 47;
		for (parallelFace in parallelFaces)
			parallelFace.close();
		if (!selectorReportsAmbiguous(box))
			return 48;

		var edges = box.edges();
		if (edges.count() != 12)
			return 9;
		var edge = edges.at(0);
		if (edge.curveKind() != CadKit.CurveKind.Line || edge.length() <= 0.0)
			return 10;
		var edgeTangent = edge.tangentAt();
		if (edgeTangent.get_x() * edgeTangent.get_x() +
			edgeTangent.get_y() * edgeTangent.get_y() +
			edgeTangent.get_z() * edgeTangent.get_z() < 0.99)
			return 11;
		edge.close();
		var tenEdges = box.edges().query()
			.curve(CadKit.CurveKind.Line)
			.tangentParallelTo(Geometry.vec3(2.0, 0.0, 0.0))
			.lengthAtLeast(9.99)
			.lengthAtMost(10.01)
			.all();
		if (tenEdges.length != 4)
			return 49;
		for (tenEdge in tenEdges)
			tenEdge.close();

		var vertices = box.vertices();
		if (vertices.count() != 8)
			return 12;
		var vertex = vertices.at(0);
		vertex.position();
		vertex.close();
		var origin = box.vertices().query()
			.near(Geometry.vec3(0.0, 0.0, 0.0), 0.001)
			.unique();
		origin.close();

		var mesh = box.tessellate();
		if (mesh.vertexCount <= 0 || mesh.indexCount <= 0)
			return 13;
		if (mesh.vertices.length != mesh.vertexCount * 24 || mesh.normals.length != mesh.vertexCount * 24)
			return 14;
		if (mesh.indices.length != mesh.indexCount * 4)
			return 15;
		if (mesh.faceRanges.length != faces.count())
			return 59;
		var expectedIndex = 0;
		for (faceRangeIndex in 0...mesh.faceRanges.length) {
			var faceRange = mesh.faceRanges[faceRangeIndex];
			if (faceRange.faceIndex != faceRangeIndex ||
				faceRange.firstIndex != expectedIndex)
				return 60;
			expectedIndex = faceRange.endIndex();
		}
		if (expectedIndex != mesh.indexCount)
			return 61;
		var meshFace = box.faces().at(0);
		var meshFaceRange = mesh.rangeFor(meshFace);
		if (meshFaceRange == null || meshFaceRange.indexCount <= 0)
			return 62;
		meshFace.close();
		var cachedMesh = box.tessellate();
		if (cachedMesh.faceRanges.length != mesh.faceRanges.length)
			return 63;

		var moved = box.translate(Geometry.vec3(2.0, 3.0, 4.0));
		if (moved.volume() != 6000.0)
			return 16;
		var filletShape = box.fillet(1.0);
		if (filletShape.volume() <= 0.0 || filletShape.volume() >= box.volume())
			return 72;
		var selectedEdge = box.edges().at(0);
		var selectedFilletShape = box.filletEdges([selectedEdge], 1.0);
		if (selectedFilletShape.volume() <= filletShape.volume() ||
			selectedFilletShape.volume() >= box.volume())
			return 76;
		var selectedFilletOperation = box.filletEdgesOperation([selectedEdge], 1.0);
		var selectedFilletResult = selectedFilletOperation.resultShape();
		if (selectedFilletResult.volume() <= filletShape.volume() ||
			selectedFilletOperation.historyCount(CadKit.HistoryRelation.Generated) +
				selectedFilletOperation.historyCount(CadKit.HistoryRelation.Modified) +
				selectedFilletOperation.historyCount(CadKit.HistoryRelation.Deleted) <= 0)
			return 77;
		var filletOperation = box.filletOperation(1.0);
		var filletResult = filletOperation.resultShape();
		if (filletResult.volume() <= 0.0 ||
			filletOperation.historyCount(CadKit.HistoryRelation.Generated) +
				filletOperation.historyCount(CadKit.HistoryRelation.Modified) +
				filletOperation.historyCount(CadKit.HistoryRelation.Deleted) <= 0)
			return 73;
		var chamferShape = box.chamfer(1.0);
		if (chamferShape.volume() <= 0.0 || chamferShape.volume() >= box.volume())
			return 74;
		var selectedChamferShape = box.chamferEdges([selectedEdge], 1.0);
		if (selectedChamferShape.volume() <= 0.0 || selectedChamferShape.volume() >= box.volume())
			return 78;
		var selectedChamferOperation = box.chamferEdgesOperation([selectedEdge], 1.0);
		var selectedChamferResult = selectedChamferOperation.resultShape();
		if (selectedChamferResult.volume() <= 0.0 ||
			selectedChamferOperation.historyCount(CadKit.HistoryRelation.Generated) +
				selectedChamferOperation.historyCount(CadKit.HistoryRelation.Modified) +
				selectedChamferOperation.historyCount(CadKit.HistoryRelation.Deleted) <= 0)
			return 79;
		var chamferOperation = box.chamferOperation(1.0);
		var chamferResult = chamferOperation.resultShape();
		if (chamferResult.volume() <= 0.0 ||
			chamferOperation.historyCount(CadKit.HistoryRelation.Generated) +
				chamferOperation.historyCount(CadKit.HistoryRelation.Modified) +
				chamferOperation.historyCount(CadKit.HistoryRelation.Deleted) <= 0)
			return 75;
		filletShape.close();
		selectedEdge.close();
		selectedFilletShape.close();
		selectedFilletResult.close();
		selectedFilletOperation.close();
		filletResult.close();
		filletOperation.close();
		chamferShape.close();
		selectedChamferShape.close();
		selectedChamferResult.close();
		selectedChamferOperation.close();
		chamferResult.close();
		chamferOperation.close();

		var profileFace = box.faces().at(0);
		var profile = profileFace.cloneShape();
		var profileNormal = profile.faceNormal();
		var extrusionDelta = Geometry.vec3(
			profileNormal.get_x() * 5.0,
			profileNormal.get_y() * 5.0,
			profileNormal.get_z() * 5.0);
		var extruded = profile.extrude(extrusionDelta);
		if (extruded.volume() <= 0.0)
			return 64;
		var extrudeOperation = profile.extrudeOperation(extrusionDelta);
		var extrudeResult = extrudeOperation.resultShape();
		if (extrudeResult.volume() <= 0.0 ||
			extrudeOperation.historyCount(CadKit.HistoryRelation.Generated) +
				extrudeOperation.historyCount(CadKit.HistoryRelation.Modified) +
				extrudeOperation.historyCount(CadKit.HistoryRelation.Deleted) <= 0)
			return 65;
		var revolved = profile.revolve(
			Geometry.vec3(0.0, 0.0, 0.0),
			Geometry.vec3(0.0, 0.0, 1.0),
			1.5707963267948966);
		if (revolved.volume() <= 0.0)
			return 66;
		var revolveOperation = profile.revolveOperation(
			Geometry.vec3(0.0, 0.0, 0.0),
			Geometry.vec3(0.0, 0.0, 1.0),
			1.5707963267948966);
		var revolveResult = revolveOperation.resultShape();
		if (revolveResult.volume() <= 0.0 ||
			revolveOperation.historyCount(CadKit.HistoryRelation.Generated) +
				revolveOperation.historyCount(CadKit.HistoryRelation.Modified) +
				revolveOperation.historyCount(CadKit.HistoryRelation.Deleted) <= 0)
			return 67;
		profileFace.close();
		extruded.close();
		extrudeResult.close();
		extrudeOperation.close();
		revolved.close();
		revolveResult.close();
		revolveOperation.close();
		profile.close();

		var hole = Shape.cylinder(2.0, 30.0).translate(Geometry.vec3(5.0, 10.0, 0.0));
		var cut = box.cut(hole);
		if (cut.volume() >= box.volume())
			return 17;

		var operation = box.cutOperation(hole);
		var operationCut = operation.resultShape();
		if (operationCut.volume() >= box.volume())
			return 18;
		var history = operation.history();
		var generated = history.count(CadKit.HistoryRelation.Generated);
		var modified = history.count(CadKit.HistoryRelation.Modified);
		var deleted = history.count(CadKit.HistoryRelation.Deleted);
		if (generated + modified + deleted <= 0)
			return 19;
		if (!historyRelationIsMapped(operation, CadKit.HistoryRelation.Generated))
			return 52;
		if (!historyRelationIsMapped(operation, CadKit.HistoryRelation.Modified))
			return 53;
		if (!historyRelationIsMapped(operation, CadKit.HistoryRelation.Deleted))
			return 54;
		if (modified > 0) {
			var source = history.sourceAt(CadKit.HistoryRelation.Modified, 0);
			var target = history.targetAt(CadKit.HistoryRelation.Modified, 0);
			source.close();
			target.close();
		} else if (generated > 0) {
			var source = history.sourceAt(CadKit.HistoryRelation.Generated, 0);
			var target = history.targetAt(CadKit.HistoryRelation.Generated, 0);
			source.close();
			target.close();
		} else {
			var source = history.sourceAt(CadKit.HistoryRelation.Deleted, 0);
			source.close();
		}
		operationCut.close();
		operation.close();

		var document = new Document();
		var baseFeature = document.add(new BoxFeature(10.0, 20.0, 30.0));
		var rawToolFeature = document.add(new CylinderFeature(2.0, 30.0));
		var toolFeature = document.add(new TransformFeature(rawToolFeature, 5.0, 10.0, 0.0));
		var resultFeature = document.add(
			new BooleanFeature(baseFeature, toolFeature, BooleanOperation.Cut));
		var extrudePrototype = new ExtrudeFeature(baseFeature, 0.0, 0.0, 5.0);
		var revolvePrototype = new RevolveFeature(
			baseFeature, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
			1.5707963267948966);
		if (extrudePrototype.serializationType() != "extrude" ||
			revolvePrototype.serializationType() != "revolve")
			return 68;
		extrudePrototype.close();
		revolvePrototype.close();
		var foreignDocument = new Document();
		var foreignFeature = foreignDocument.add(new BoxFeature(1.0, 1.0, 1.0));
		var foreignDependencyRejected = false;
		try {
			document.add(new BooleanFeature(baseFeature, foreignFeature, BooleanOperation.Cut));
		} catch (error:Dynamic) {
			foreignDependencyRejected = true;
		}
		if (!foreignDependencyRejected)
			return 50;
		foreignDocument.close();
		document.recompute();
		var initialShape = resultFeature.currentShape();
		if (initialShape == null || initialShape.volume() >= 6000.0)
			return 20;
		var originalVolume = initialShape.volume();
		var baseShape = baseFeature.currentShape();
		if (baseShape == null)
			return 26;
		var baseFace = baseShape.faces().at(0);
		var baseReference = TopologyReference.fromFace(baseFeature, baseFace);
		baseFace.close();
		if (!baseReference.isResolved())
			return 27;
		var ambiguousReference = TopologyReference.fromFingerprint(
			baseFeature,
			CadKit.ShapeKind.Vertex,
			TopologyFingerprint.fromData(
				CadKit.ShapeKind.Vertex,
				CadKit.SurfaceKind.Unknown,
				CadKit.CurveKind.Unknown,
				5.0, 10.0, 15.0,
				0.0, 0.0, 0.0,
				0.0));
		baseFeature.remapTopologyReferences();
		if (ambiguousReference.state != ReferenceState.Ambiguous ||
			ambiguousReference.isResolved())
			return 51;
		ambiguousReference.close();
		var resultFace = initialShape.faces().at(0);
		var resultReference = TopologyReference.fromFace(resultFeature, resultFace);
		resultFace.close();
		if (!resultReference.isResolved())
			return 28;

		var initialReferenceShape = baseReference.currentShape();
		if (initialReferenceShape == null ||
			initialReferenceShape.surfaceKind() != CadKit.SurfaceKind.Plane)
			return 29;

		var serialized = DocumentCodec.encode(document);
		if (serialized.length == 0)
			return 37;
		var loaded = DocumentCodec.decode(serialized);
		if (loaded.featureCount() != 4)
			return 38;
		var loadedBase:BoxFeature = cast loaded.featureAt(0);
		var loadedResult = loaded.featureAt(3).currentShape();
		if (loadedResult == null || loadedResult.volume() != originalVolume)
			return 39;
		var loadedReference = loadedBase.topologyReferenceAt(0);
		if (!loadedReference.isResolved())
			return 40;
		if (loadedReference.state != ReferenceState.Remapped ||
			loaded.lastRemapReport.remapped <= 0)
			return 55;

		var loadedTransaction = loaded.beginTransaction();
		loadedBase.width.set(14.0);
		loadedTransaction.commit();
		loaded.recompute();
		var loadedExpanded = loaded.featureAt(3).currentShape();
		if (loadedExpanded == null || loadedExpanded.volume() <= originalVolume)
			return 41;
		if (!loaded.undo())
			return 42;
		loaded.recompute();
		var loadedUndone = loaded.featureAt(3).currentShape();
		if (loadedUndone == null || loadedUndone.volume() != originalVolume)
			return 43;
		if (!loaded.redo())
			return 44;
		loaded.recompute();
		if (!loadedReference.isResolved())
			return 45;
		loaded.close();

		var transaction = document.beginTransaction();
		baseFeature.width.set(12.0);
		transaction.commit();
		document.recompute();
		var expandedShape = resultFeature.currentShape();
		if (expandedShape == null || expandedShape.volume() <= originalVolume)
			return 30;
		if (!baseReference.isResolved() || !resultReference.isResolved())
			return 31;
		if (baseReference.state != ReferenceState.Remapped ||
			resultReference.state != ReferenceState.Remapped ||
			document.lastRemapReport.remapped < 2)
			return 56;
		var expandedReferenceShape = baseReference.currentShape();
		if (expandedReferenceShape == null ||
			expandedReferenceShape.surfaceKind() != CadKit.SurfaceKind.Plane)
			return 32;

		var transformTransaction = document.beginTransaction();
		toolFeature.x.set(6.0);
		transformTransaction.commit();
		document.recompute();
		if (!resultReference.isResolved() ||
			resultReference.state != ReferenceState.Remapped)
			return 57;
		if (!document.undo())
			return 58;
		document.recompute();

		if (!document.undo())
			return 22;
		document.recompute();
		var undoneShape = resultFeature.currentShape();
		if (undoneShape == null || undoneShape.volume() != originalVolume)
			return 33;
		if (!baseReference.isResolved() || !resultReference.isResolved())
			return 34;

		if (!document.redo())
			return 24;
		document.recompute();
		var redoneShape = resultFeature.currentShape();
		if (redoneShape == null || redoneShape.volume() <= originalVolume)
			return 35;
		if (!baseReference.isResolved() || !resultReference.isResolved())
			return 36;
		document.close();

		var sweepDocument = new Document();
		var sweepBase = sweepDocument.add(new BoxFeature(10.0, 20.0, 30.0));
		sweepDocument.recompute();
		var sweepBaseShape = sweepBase.currentShape();
		if (sweepBaseShape == null)
			return 69;
		var sweepFace = sweepBaseShape.faces().at(0);
		var sweepNormal = sweepFace.normal();
		sweepFace.close();
		var sweepProfile = sweepDocument.add(new FaceFeature(sweepBase, 0));
		var sweepExtrude = sweepDocument.add(new ExtrudeFeature(
			sweepProfile,
			sweepNormal.get_x() * 5.0,
			sweepNormal.get_y() * 5.0,
			sweepNormal.get_z() * 5.0));
		var sweepRevolve = sweepDocument.add(new RevolveFeature(
			sweepProfile, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
			1.5707963267948966));
		var sweepFillet = sweepDocument.add(new FilletFeature(sweepBase, 1.0));
		var sweepChamfer = sweepDocument.add(new ChamferFeature(sweepBase, 1.0));
		sweepDocument.recompute();
		if (sweepExtrude.currentShape() == null ||
			sweepExtrude.currentShape().volume() <= 0.0 ||
			sweepRevolve.currentShape() == null ||
			sweepRevolve.currentShape().volume() <= 0.0 ||
			sweepFillet.currentShape() == null ||
			sweepFillet.currentShape().volume() <= 0.0 ||
			sweepChamfer.currentShape() == null ||
			sweepChamfer.currentShape().volume() <= 0.0 ||
			sweepProfile.fingerprintData() == null)
			return 76;
		var sweepTransaction = sweepDocument.beginTransaction();
		sweepBase.width.set(12.0);
		sweepTransaction.commit();
		sweepDocument.recompute();
		if (sweepProfile.currentShape() == null ||
			sweepExtrude.currentShape() == null ||
			sweepExtrude.currentShape().volume() <= 0.0)
			return 77;
		var sweepSerialized = DocumentCodec.encode(sweepDocument);
		var loadedSweep = DocumentCodec.decode(sweepSerialized);
		if (loadedSweep.featureCount() != 6 ||
			loadedSweep.featureAt(2).currentShape() == null ||
			loadedSweep.featureAt(3).currentShape() == null ||
			loadedSweep.featureAt(4).currentShape() == null ||
			loadedSweep.featureAt(5).currentShape() == null ||
			loadedSweep.featureAt(2).currentShape().volume() <= 0.0 ||
			loadedSweep.featureAt(3).currentShape().volume() <= 0.0 ||
			loadedSweep.featureAt(4).currentShape().volume() <= 0.0 ||
			loadedSweep.featureAt(5).currentShape().volume() <= 0.0)
			return 78;
		loadedSweep.close();
		sweepDocument.close();

		var selectedDocument = new Document();
		var selectedBase = selectedDocument.add(new BoxFeature(10.0, 20.0, 30.0));
		selectedDocument.recompute();
		var selectedBaseShape = selectedBase.currentShape();
		if (selectedBaseShape == null)
			return 81;
		var selectedParametricEdge = selectedBaseShape.edges().at(0);
		var selectedFilletFeature = selectedDocument.add(new FilletFeature(
			selectedBase, 1.0, [selectedParametricEdge]));
		var selectedChamferFeature = selectedDocument.add(new ChamferFeature(
			selectedBase, 1.0, [selectedParametricEdge]));
		selectedParametricEdge.close();
		selectedDocument.recompute();
		if (selectedFilletFeature.edgeReferences.length != 1 ||
			selectedChamferFeature.edgeReferences.length != 1 ||
			!selectedFilletFeature.edgeReferences[0].isResolved() ||
			!selectedChamferFeature.edgeReferences[0].isResolved() ||
			selectedFilletFeature.currentShape() == null ||
			selectedFilletFeature.currentShape().volume() <= 0.0 ||
			selectedChamferFeature.currentShape() == null ||
			selectedChamferFeature.currentShape().volume() <= 0.0)
			return 82;

		var selectedTransaction = selectedDocument.beginTransaction();
		selectedBase.width.set(12.0);
		selectedTransaction.commit();
		selectedDocument.recompute();
		if (!selectedFilletFeature.edgeReferences[0].isResolved() ||
			!selectedChamferFeature.edgeReferences[0].isResolved() ||
			selectedDocument.lastRemapReport.remapped < 2)
			return 83;

		var selectedSerialized = DocumentCodec.encode(selectedDocument);
		if (selectedSerialized.indexOf("\"edges\"") < 0)
			return 84;
		var loadedSelected = DocumentCodec.decode(selectedSerialized);
		if (loadedSelected.featureCount() != 3)
			return 85;
		var loadedSelectedFillet:FilletFeature = cast loadedSelected.featureAt(1);
		var loadedSelectedChamfer:ChamferFeature = cast loadedSelected.featureAt(2);
		if (loadedSelectedFillet.edgeReferences.length != 1 ||
			loadedSelectedChamfer.edgeReferences.length != 1 ||
			!loadedSelectedFillet.edgeReferences[0].isResolved() ||
			!loadedSelectedChamfer.edgeReferences[0].isResolved() ||
			loadedSelectedFillet.currentShape() == null ||
			loadedSelectedFillet.currentShape().volume() <= 0.0 ||
			loadedSelectedChamfer.currentShape() == null ||
			loadedSelectedChamfer.currentShape().volume() <= 0.0)
			return 86;
		loadedSelected.close();
		selectedDocument.close();

		var deletedDocument = new Document();
		var deletedBase = deletedDocument.add(new BoxFeature(10.0, 20.0, 30.0));
		var deletedCylinder = deletedDocument.add(new CylinderFeature(2.0, 30.0));
		var deletedTool = deletedDocument.add(
			new TransformFeature(deletedCylinder, 5.0, 10.0, 0.0));
		var deletedSource = deletedDocument.add(
			new BooleanFeature(deletedBase, deletedTool, BooleanOperation.Cut));
		deletedDocument.recompute();
		var deletedSourceShape = deletedSource.currentShape();
		if (deletedSourceShape == null)
			return 87;
		var deletedEdge:Null<Edge> = null;
		for (index in 0...deletedSourceShape.edges().count()) {
			var candidate = deletedSourceShape.edges().at(index);
			if (deletedEdge == null && candidate.curveKind() == CadKit.CurveKind.Circle)
				deletedEdge = candidate;
			else
				candidate.close();
		}
		if (deletedEdge == null)
			return 88;
		var deletedFillet = deletedDocument.add(new FilletFeature(
			deletedSource, 1.0, [deletedEdge]));
		deletedEdge.close();
		deletedDocument.recompute();
		var deletedTransaction = deletedDocument.beginTransaction();
		deletedTool.x.set(20.0);
		deletedTransaction.commit();
		var deletedFailure:Null<RecomputeError> = null;
		try {
			deletedDocument.recompute();
		} catch (error:Dynamic) {
			var recomputeError:RecomputeError = cast error;
			deletedFailure = recomputeError;
		}
		if (deletedFailure == null ||
			deletedFillet.edgeReferences[0].state != ReferenceState.Deleted ||
			deletedDocument.lastRemapReport.deleted != 1 ||
			deletedFailure.referenceState != ReferenceState.Deleted)
			return 89;
		deletedFailure = null;
		try {
			deletedDocument.recompute();
		} catch (error:Dynamic) {
			var recomputeError:RecomputeError = cast error;
			deletedFailure = recomputeError;
		}
		if (deletedFailure == null || deletedDocument.lastRemapReport.deleted != 1)
			return 95;
		if (!deletedDocument.undo())
			return 91;
		deletedDocument.recompute();
		if (!deletedFillet.edgeReferences[0].isResolved() ||
			deletedDocument.lastRemapReport.remapped != 1)
			return 92;
		if (!deletedDocument.redo())
			return 93;
		deletedFailure = null;
		try {
			deletedDocument.recompute();
		} catch (error:Dynamic) {
			var recomputeError:RecomputeError = cast error;
			deletedFailure = recomputeError;
		}
		if (deletedFailure == null ||
			deletedFillet.edgeReferences[0].state != ReferenceState.Deleted ||
			deletedDocument.lastRemapReport.deleted != 1 ||
			deletedFailure.referenceState != ReferenceState.Deleted)
			return 94;
		deletedDocument.close();

		var ambiguousDocument = new Document();
		var ambiguousBase = ambiguousDocument.add(new BoxFeature(10.0, 20.0, 30.0));
		ambiguousDocument.recompute();
		var ambiguousFillet = ambiguousDocument.add(new FilletFeature(
			ambiguousBase,
			1.0,
			null,
			[TopologyFingerprint.fromData(
				CadKit.ShapeKind.Edge,
				CadKit.SurfaceKind.Unknown,
				CadKit.CurveKind.Line,
				0.0, 10.0, 15.0,
				1.0, 0.0, 0.0,
				10.0)]));
		var ambiguousFailure:Null<RecomputeError> = null;
		try {
			ambiguousDocument.recompute();
		} catch (error:Dynamic) {
			var recomputeError:RecomputeError = cast error;
			ambiguousFailure = recomputeError;
		}
		if (ambiguousFailure == null ||
			ambiguousFillet.edgeReferences[0].state != ReferenceState.Ambiguous ||
			ambiguousDocument.lastRemapReport.ambiguous != 1 ||
			ambiguousFailure.referenceState != ReferenceState.Ambiguous)
			return 90;
		ambiguousDocument.close();

		cut.close();
		hole.close();
		moved.close();
		box.close();
		return 0;
	}
}
