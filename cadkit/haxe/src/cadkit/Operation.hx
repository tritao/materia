package cadkit;

import CadKit;
import cadkit.threads.ThreadedFace;
import cadkit.threads.Threads;

/** Owning result and topology history for one modeling operation. */
class Operation {
	private var threadSources:Array<{attribute:ThreadedFace, face:Shape}>;
	private var native:CadKit.OwnedOperationHandle;

	public function new(native:CadKit.OwnedOperationHandle, inputs:Array<Shape> = null) {
		this.native = native;
		threadSources = [];
		try {
			if (inputs != null) for (input in inputs) for (attribute in input.threadedFaces())
				threadSources.push({attribute: attribute, face: input.subshape(CadKit.ShapeKind.Face,
					Threads.namedIndex(input, attribute.faceName))});
		} catch (error:Dynamic) {
			close();
			throw error;
		}
	}

	public function resultShape():Shape {
		var result = Shape.fromOwnedHandle(CadKit.operationResultShapeChecked(native.borrow()));
		try {
			var attributes:Array<ThreadedFace> = [];
			for (source in threadSources) {
				var named = Threads.namedIndices(result, source.attribute.faceName);
				if (named.length == 1) {
					attributes.push(source.attribute.renamed(result.elementName(CadKit.ShapeKind.Face, named[0])));
					continue;
				}
				var targets:Array<Int> = [];
				for (index in 0...historyCount(CadKit.HistoryRelation.Modified)) {
					var original = historySourceAt(CadKit.HistoryRelation.Modified, index);
					var same = original.sameAs(source.face);
					original.close();
					if (!same) continue;
					var target = historyTargetAt(CadKit.HistoryRelation.Modified, index);
					for (faceIndex in 0...result.subshapeCount(CadKit.ShapeKind.Face)) {
						var face = result.subshape(CadKit.ShapeKind.Face, faceIndex);
						var found = face.sameAs(target);
						face.close();
						if (found && targets.indexOf(faceIndex) < 0) targets.push(faceIndex);
					}
					target.close();
				}
				if (targets.length > 0) {
					for (index in targets) attributes.push(source.attribute.renamed(result.elementName(CadKit.ShapeKind.Face, index)));
					continue;
				}
				var deleted = false;
				for (index in 0...historyCount(CadKit.HistoryRelation.Deleted)) {
					var original = historySourceAt(CadKit.HistoryRelation.Deleted, index);
					if (original.sameAs(source.face)) deleted = true;
					original.close();
				}
				if (!deleted) throw 'Thread face ${source.attribute.faceName} lost without proven deletion';
			}
			result.setThreadedFaces(attributes);
			return result;
		} catch (error:Dynamic) {
			result.close();
			throw error;
		}
	}

	public function history():OperationHistory {
		return new OperationHistory(this);
	}

	public function historyCount(relation:CadKit.HistoryRelation):Int {
		return CadKit.operationHistoryCountChecked(native.borrow(), relation);
	}

	public function historySourceAt(
		relation:CadKit.HistoryRelation,
		index:Int):Shape {
		return Shape.fromOwnedHandle(
			CadKit.operationHistorySourceAtChecked(native.borrow(), relation, index));
	}

	public function historyTargetAt(
		relation:CadKit.HistoryRelation,
		index:Int):Shape {
		return Shape.fromOwnedHandle(
			CadKit.operationHistoryTargetAtChecked(native.borrow(), relation, index));
	}

	public function close():Bool {
		for (source in threadSources) source.face.close();
		threadSources.resize(0);
		return native.close();
	}

	public function isClosed():Bool {
		return native.isClosed();
	}
}
