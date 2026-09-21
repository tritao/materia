package cadkit.parametric;

import cadkit.Shape;
import cadkit.parametric.ChangeSet;
import cadkit.parametric.EvaluationContext;
import cadkit.parametric.EvaluationResult;
import cadkit.parametric.Feature;
import cadkit.parametric.FeatureId;
import cadkit.parametric.ParametricError;
import cadkit.parametric.RecomputeError;
import cadkit.parametric.Transaction;

/** Haxeon-owned parametric feature document. */
class Document {
	private static var nextToken:Int = 1;

	private var nextId:Int;
	private final token:Int;
	private final features:Array<Feature>;
	private var byId:Map<Int, Feature>;
	private var activeTransaction:Null<Transaction>;
	private final undoStack:Array<ChangeSet>;
	private final redoStack:Array<ChangeSet>;
	public var lastRemapReport(default, null):TopologyRemapReport;

	public function new() {
		token = nextToken;
		nextToken++;
		nextId = 1;
		features = [];
		byId = new Map<Int, Feature>();
		activeTransaction = null;
		undoStack = [];
		redoStack = [];
		lastRemapReport = new TopologyRemapReport();
	}

	public function add<T:Feature>(feature:T):T {
		if (feature.document != null)
			throw new ParametricError("feature is already attached to a document");

		for (dependency in feature.dependencyFeatures()) {
			if (dependency.ownerToken != token || dependency.id.toInt() == 0 ||
				!byId.exists(dependency.id.toInt()))
				throw new ParametricError("feature dependency is not in this document");
			if (dependency.id.toInt() == feature.id.toInt() && feature.id.toInt() != 0)
				throw new ParametricError("feature cannot depend on itself");
		}

		var featureId = new FeatureId(nextId);
		nextId++;
		feature.attach(this, featureId, token);
		features.push(feature);
		byId.set(featureId.toInt(), feature);
		return feature;
	}

	public function featureCount():Int {
		return features.length;
	}

	public function featureAt(index:Int):Feature {
		if (index < 0 || index >= features.length)
			throw new ParametricError("feature index is out of range");
		return features[index];
	}

	public function featureById(featureId:Int):Null<Feature> {
		return byId.get(featureId);
	}

	public function recompute():Void {
		var order = topologicalOrder();
		var context = new EvaluationContext(this);
		var stagedFeatures:Array<Feature> = [];
		var stagedResults:Array<EvaluationResult> = [];
		var current:Null<Feature> = null;

		try {
			for (feature in order) {
				current = feature;
				if (!feature.dirty)
					continue;

				var result:EvaluationResult = feature.evaluate(context);
				stagedFeatures.push(feature);
				stagedResults.push(result);
				context.stage(feature, result.getShape());
			}

			for (index in 0...stagedFeatures.length)
				stagedFeatures[index].install(stagedResults[index]);
			lastRemapReport = new TopologyRemapReport();
			for (feature in stagedFeatures)
				lastRemapReport.merge(feature.remapTopologyReferences());
		} catch (error:Dynamic) {
			for (index in 0...stagedResults.length) {
				var reverse = stagedResults.length - index - 1;
				stagedResults[reverse].dispose();
			}
			if (current != null)
				throw new RecomputeError(current.id, error);
			throw error;
		}
	}

	public function beginTransaction():Transaction {
		if (activeTransaction != null)
			throw new ParametricError("nested transactions are not supported");
		activeTransaction = new Transaction(this);
		return activeTransaction;
	}

	public function undo():Bool {
		if (activeTransaction != null)
			throw new ParametricError("finish the active transaction first");
		if (undoStack.length == 0)
			return false;

		var changes = undoStack.pop();
		for (index in 0...changes.changes.length) {
			var reverse = changes.changes.length - index - 1;
			changes.changes[reverse].parameter.restore(changes.changes[reverse].oldValue);
		}
		redoStack.push(changes);
		return true;
	}

	public function redo():Bool {
		if (activeTransaction != null)
			throw new ParametricError("finish the active transaction first");
		if (redoStack.length == 0)
			return false;

		var changes = redoStack.pop();
		for (change in changes.changes)
			change.parameter.restore(change.newValue);
		undoStack.push(changes);
		return true;
	}

	public function close():Void {
		if (activeTransaction != null)
			activeTransaction.cancel();
		for (feature in features)
			feature.close();
		features.resize(0);
		byId = new Map<Int, Feature>();
	}

	public function recordParameterChange(parameter:Parameter, oldValue:Float):Void {
		if (activeTransaction != null) {
			activeTransaction.record(parameter, oldValue);
			return;
		}

		var change = new ParameterChange(parameter, oldValue, parameter.value);
		undoStack.push(new ChangeSet([change]));
		redoStack.resize(0);
	}

	public function commitTransaction(transaction:Transaction):Void {
		if (activeTransaction == null || activeTransaction.identity != transaction.identity)
			throw new ParametricError("transaction does not belong to this document");
		activeTransaction = null;
		if (transaction.changes.length == 0)
			return;
		undoStack.push(new ChangeSet(transaction.changes.copy()));
		redoStack.resize(0);
	}

	public function cancelTransaction(transaction:Transaction):Void {
		if (activeTransaction == null || activeTransaction.identity != transaction.identity)
			throw new ParametricError("transaction does not belong to this document");
		activeTransaction = null;
		for (index in 0...transaction.changes.length) {
			var reverse = transaction.changes.length - index - 1;
			var change = transaction.changes[reverse];
			change.parameter.restore(change.oldValue);
		}
	}

	public function invalidate(feature:Feature):Void {
		feature.dirty = true;
		var visited = new Map<Int, Bool>();
		invalidateDependents(feature, visited);
	}

	private function invalidateDependents(source:Feature, visited:Map<Int, Bool>):Void {
		var sourceId = source.id.toInt();
		if (visited.exists(sourceId))
			return;
		visited.set(sourceId, true);
		for (candidate in features) {
			for (dependency in candidate.dependencies()) {
				if (dependency.toInt() == sourceId) {
					candidate.dirty = true;
					invalidateDependents(candidate, visited);
					break;
				}
			}
		}
	}

	private function topologicalOrder():Array<Feature> {
		var order:Array<Feature> = [];
		var visiting = new Map<Int, Bool>();
		var visited = new Map<Int, Bool>();
		for (feature in features)
			visit(feature, visiting, visited, order);
		return order;
	}

	private function visit(
		feature:Feature,
		visiting:Map<Int, Bool>,
		visited:Map<Int, Bool>,
		order:Array<Feature>):Void {
		var key = feature.id.toInt();
		if (visited.exists(key))
			return;
		if (visiting.exists(key))
			throw new ParametricError("feature dependency cycle detected");

		visiting.set(key, true);
		for (dependency in feature.dependencies()) {
			var dependencyFeature = byId.get(dependency.toInt());
			if (dependencyFeature == null)
				throw new ParametricError("feature dependency is missing");
			visit(dependencyFeature, visiting, visited, order);
		}
		visiting.remove(key);
		visited.set(key, true);
		order.push(feature);
	}
}
