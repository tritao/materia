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
import cadkit.parametric.ExpressionValue;
import cadkit.parametric.NamedParameterChange;
import cadkit.parametric.ParameterExpression;
import cadkit.parametric.ParameterKind;
import cadkit.parametric.ParameterExpressionChange;
import cadkit.parametric.UnitConversion;
import cadkit.parametric.DocumentId;
import cadkit.parametric.Element;
import cadkit.parametric.ElementId;
import cadkit.parametric.LevelElement;
import cadkit.parametric.ReferencePlaneElement;
import cadkit.parametric.ElementReference;
import cadkit.parametric.ElementChanges.ElementCreateChange;
import cadkit.parametric.ElementChanges.ElementRemoveChange;
import cadkit.parametric.ElementChanges.ElementNameChange;
import cadkit.parametric.ElementChanges.ElementOutputChange;
import cadkit.parametric.DatumChanges.LevelElevationChange;
import cadkit.parametric.DatumChanges.ReferencePlaneChange;
import cadkit.modeling.Plane;

/** Haxeon-owned parametric feature document. */
class Document {
	private static var nextToken:Int = 1;

	private var nextId:Int;
	public final id:DocumentId;
	private final elements:Array<Element>;
	private var elementsById:Map<String, Element>;
	private var issuedElementIds:Map<String, Bool>;
	private final dimensions:Array<NamedParameter>;
	private var updatingNamedParameter:Bool;
	private var selectedOutput:Null<Feature>;
	private var closed:Bool;
	private final token:Int;
	private final features:Array<Feature>;
	private var byId:Map<Int, Feature>;
	private var activeTransaction:Null<Transaction>;
	private final undoStack:Array<ChangeSet>;
	private final redoStack:Array<ChangeSet>;

	public var lastRemapReport(default, null):TopologyRemapReport;

	public function new(?id:DocumentId) {
		this.id = id == null ? new DocumentId() : id;
		token = nextToken;
		nextToken++;
		nextId = 1;
		dimensions = [];
		elements = [];
		elementsById = new Map<String, Element>();
		issuedElementIds = new Map<String, Bool>();
		updatingNamedParameter = false;
		selectedOutput = null;
		closed = false;
		features = [];
		byId = new Map<Int, Feature>();
		activeTransaction = null;
		undoStack = [];
		redoStack = [];
		lastRemapReport = new TopologyRemapReport();
	}

	public function createElement(name:String, output:Feature):Element {
		var identity = new ElementId();
		while (issuedElementIds.exists(identity.value))
			identity = new ElementId();
		var result = installElement(name, output, identity);
		recordDocumentChange(new ElementCreateChange(this, result, elements.length - 1));
		return result;
	}

	/** Codec path: installs a persisted record without creating undo history. */
	public function installElement(name:String, output:Feature, id:ElementId):Element {
		ensureOpen();
		validateElementName(name);
		validateElementOutput(output);
		if (issuedElementIds.exists(id.value))
			throw new ParametricError("duplicate or previously issued element ID: " + id.value);
		var result = new Element(this, id, name, "geometry", output);
		elements.push(result);
		elementsById.set(id.value, result);
		issuedElementIds.set(id.value, true);
		return result;
	}

	public function createLevel(name:String,elevation:Float,offset:Float=0,?relativeTo:ElementReference):LevelElement {
		var id=newElementId(); var result=new LevelElement(this,id,name,elevation,offset,relativeTo); installRecord(result); recordDocumentChange(new ElementCreateChange(this,result,elements.length-1)); return result;
	}
	public function createReferencePlane(name:String,plane:Plane):ReferencePlaneElement {
		var id=newElementId(); var result=new ReferencePlaneElement(this,id,name,plane); installRecord(result); recordDocumentChange(new ElementCreateChange(this,result,elements.length-1)); return result;
	}
	public function installLevel(name:String,id:ElementId,elevation:Float,offset:Float=0,?relativeTo:ElementReference):LevelElement { var result=new LevelElement(this,id,name,elevation,offset,relativeTo);installRecord(result);return result; }
	public function installReferencePlane(name:String,id:ElementId,plane:Plane):ReferencePlaneElement { var result=new ReferencePlaneElement(this,id,name,plane);installRecord(result);return result; }
	private function newElementId():ElementId { var value=new ElementId(); while(issuedElementIds.exists(value.value)) value=new ElementId(); return value; }
	private function installRecord(result:Element):Void { validateElementName(result.name); if(issuedElementIds.exists(result.id.value)) throw new ParametricError("duplicate or previously issued element ID: "+result.id.value); elements.push(result);elementsById.set(result.id.value,result);issuedElementIds.set(result.id.value,true); }
	public function setLevelElevation(level:LevelElement,value:Float):Void { validateOwnedElement(level);if(!Math.isFinite(value))throw new ParametricError("level elevation must be finite");var old=level.elevation;if(old==value)return;level.restoreElevation(value);recordDocumentChange(new LevelElevationChange(level,old,value)); }
	public function setReferencePlane(datum:ReferencePlaneElement,value:Plane):Void { validateOwnedElement(datum);var old=datum.plane;datum.restorePlane(value);recordDocumentChange(new ReferencePlaneChange(datum,old,value)); }

	public function duplicateElement(source:Element, ?name:String):Element {
		validateOwnedElement(source);
		if(source.output==null) throw new ParametricError("datum duplication requires its typed API");
		return createElement(name == null ? source.name + " copy" : name, cast source.output);
	}

	public function removeElement(id:ElementId):Void {
		var target = element(id);
		var index = elements.indexOf(target);
		restoreElementRemoval(target);
		recordDocumentChange(new ElementRemoveChange(this, target, index));
	}

	public function renameElement(target:Element, name:String):Void {
		validateOwnedElement(target);
		validateElementName(name);
		if (target.name == name) return;
		var previous = target.name;
		target.restoreName(name);
		recordDocumentChange(new ElementNameChange(target, previous, name));
	}

	public function setElementOutput(target:Element, output:Feature):Void {
		validateOwnedElement(target);
		validateElementOutput(output);
		if (target.output == output) return;
		var previous = target.output;
		target.restoreOutput(output);
		recordDocumentChange(new ElementOutputChange(target, previous, output));
	}

	public function allElements():Array<Element> {
		ensureOpen();
		return elements.copy();
	}

	public function elementCount():Int {
		ensureOpen();
		return elements.length;
	}

	public function elementAt(index:Int):Element {
		ensureOpen();
		if (index < 0 || index >= elements.length)
			throw new ParametricError("element index is out of range");
		return elements[index];
	}

	public function element(id:ElementId):Element {
		ensureOpen();
		var result = elementsById.get(id.value);
		if (result == null)
			throw new ParametricError("unresolved element: " + id.value);
		return result;
	}

	public function findElement(id:ElementId):Null<Element> {
		ensureOpen();
		return elementsById.get(id.value);
	}

	public function restoreElementInsertion(element:Element, index:Int):Void {
		if (element.document != this || elementsById.exists(element.id.value))
			throw new ParametricError("cannot restore element: " + element.id.value);
		var insertion = index < 0 ? 0 : (index > elements.length ? elements.length : index);
		elements.insert(insertion, element);
		elementsById.set(element.id.value, element);
	}

	public function restoreElementRemoval(element:Element):Void {
		validateOwnedElement(element);
		elements.remove(element);
		elementsById.remove(element.id.value);
	}

	private function validateOwnedElement(element:Element):Void {
		if (element == null || element.document != this || elementsById.get(element.id.value) != element)
			throw new ParametricError("element belongs to another document or has been removed");
	}

	private function validateElementName(name:String):Void {
		if (name == null || StringTools.trim(name) == "")
			throw new ParametricError("element name must not be empty");
	}

	private function validateElementOutput(output:Feature):Void {
		if (output == null || output.document != this || featureById(output.id.toInt()) != output)
			throw new ParametricError("element output belongs to another document");
	}

	public function add<T:Feature>(feature:T):T {
		ensureOpen();
		if (feature.document != null)
			throw new ParametricError("feature is already attached to a document");

		for (dependency in feature.dependencyFeatures()) {
			if (dependency.ownerToken != token || dependency.id.toInt() == 0 || !byId.exists(dependency.id.toInt()))
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

	public function isClosed():Bool {
		return closed;
	}

	private function ensureOpen():Void {
		if (closed)
			throw new ParametricError("document is closed");
	}

	public function defineParameter(name:String, value:Float):NamedParameter {
		return defineTypedParameter(name, value, ParameterKind.Scalar, "1");
	}

	public function defineTypedParameter(name:String, value:Float, kind:String, unit:String):NamedParameter {
		ensureOpen();
		if (name == null || StringTools.trim(name) == "" || !Math.isFinite(value))
			throw new ParametricError("named parameter needs a nonempty name and finite value");
		for (existing in dimensions)
			if (existing.name == name)
				throw new ParametricError("duplicate named parameter: " + name);
		var validatedKind = ParameterKind.validate(kind);
		var validatedUnit = UnitConversion.validateUnit(validatedKind, unit);
		var result = new NamedParameter(this, name, UnitConversion.toCanonical(value, validatedKind, validatedUnit),
			validatedKind, validatedUnit);
		dimensions.push(result);
		return result;
	}

	public function defineExpression(name:String, kind:String, unit:String, source:String):NamedParameter {
		ensureOpen();
		if (name == null || StringTools.trim(name) == "")
			throw new ParametricError("named parameter needs a nonempty name");
		for (existing in dimensions)
			if (existing.name == name)
				throw new ParametricError("duplicate named parameter: " + name);
		var parsed = new ParameterExpression(source);
		for (dependency in parsed.dependencies) {
			if (dependency == name)
				throw new ParametricError("parameter expression cycle detected: " + name);
			parameter(dependency);
		}
		var validatedKind = ParameterKind.validate(kind);
		var validatedUnit = UnitConversion.validateUnit(validatedKind, unit);
		var result = new NamedParameter(this, name, 0, validatedKind, validatedUnit, parsed);
		dimensions.push(result);
		try {
			var evaluated = expressionValue(result);
			result.synchronize(evaluated);
		} catch (error:Dynamic) {
			dimensions.pop();
			throw error;
		}
		return result;
	}

	public function setExpression(name:String, source:String):Void {
		installParameterExpression(name, source, true);
	}

	/** Codec path: install persisted expression structure without creating undo history. */
	public function installExpression(name:String, source:String):Void {
		installParameterExpression(name, source, false);
	}

	private function installParameterExpression(name:String, source:String, record:Bool):Void {
		ensureOpen();
		var named = parameter(name);
		var parsed = new ParameterExpression(source);
		for (dependency in parsed.dependencies)
			parameter(dependency);
		var previous = named.expression;
		var previousValue = named.value;
		named.replaceExpression(parsed);
		var nextValue:Float;
		try {
			nextValue = expressionValue(named);
			named.synchronize(nextValue);
		} catch (error:Dynamic) {
			named.replaceExpression(previous);
			named.synchronize(previousValue);
			throw error;
		}
		if (record)
			recordDocumentChange(new ParameterExpressionChange(named, previous, parsed, previousValue, nextValue));
	}

	public function parameter(name:String):NamedParameter {
		ensureOpen();
		for (value in dimensions)
			if (value.name == name)
				return value;
		throw new ParametricError("unknown named parameter: " + name);
	}

	public function namedParameters():Array<NamedParameter> {
		ensureOpen();
		return dimensions.copy();
	}

	public function setOutput(feature:Feature):Void {
		ensureOpen();
		if (feature.document != this || featureById(feature.id.toInt()) != feature)
			throw new ParametricError("output belongs to another document");
		selectedOutput = feature;
	}

	/** Without an explicit selection, the last feature remains the primary output. */
	public function outputFeature():Feature {
		ensureOpen();
		if (selectedOutput != null)
			return selectedOutput;
		if (features.length == 0)
			throw new ParametricError("document has no output feature");
		return features[features.length - 1];
	}

	/** Borrow the last successfully committed output. Recompute dirty edits explicitly. */
	public function result():Shape {
		var shape = outputFeature().currentShape();
		if (shape == null)
			throw new ParametricError("document output has not been evaluated");
		return shape;
	}

	/** Internal Parameter.set routing. Validate every bound slot before touching any value. */
	public function setNamedParameter(parameter:Parameter, next:Float):Bool {
		ensureOpen();
		if (updatingNamedParameter)
			return false;
		for (named in dimensions) {
			if (!named.contains(parameter))
				continue;
			if (named.expression != null)
				throw new ParametricError("expression parameters are read-only: " + named.name);
			named.validateCanonical(next);
			var bindings = named.bindings();
			for (binding in bindings)
				binding.validateValue(next);
			if (named.value == next)
				return true;
			var ownTransaction = activeTransaction == null;
			var transaction = ownTransaction ? beginTransaction() : activeTransaction;
			updatingNamedParameter = true;
			try {
				for (binding in bindings)
					binding.set(next);
				updatingNamedParameter = false;
			} catch (error:Dynamic) {
				updatingNamedParameter = false;
				if (ownTransaction)
					transaction.cancel();
				throw error;
			}
			if (ownTransaction)
				transaction.commit();
			return true;
		}
		return false;
	}

	public function setStandaloneNamedParameter(parameter:NamedParameter, next:Float):Void {
		ensureOpen();
		if (parameter.document != this || this.parameter(parameter.name) != parameter)
			throw new ParametricError("named parameter belongs to another document");
		parameter.validateCanonical(next);
		var previous = parameter.value;
		if (previous == next)
			return;
		parameter.restoreStored(next);
		recordDocumentChange(new NamedParameterChange(parameter, previous, next));
	}

	public function expressionValue(parameter:NamedParameter):Float {
		var visiting = new Map<String, Bool>();
		var cached = new Map<String, ExpressionValue>();
		return evaluateNamed(parameter, visiting, cached).value;
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
		ensureOpen();
		synchronizeExpressions();
		var order = topologicalOrder();
		var context = new EvaluationContext(this);
		var stagedFeatures:Array<Feature> = [];
		var stagedResults:Array<EvaluationResult> = [];
		var current:Null<Feature> = null;
		var previousStates:Map<Int, Array<Int>> = new Map();
		for (feature in features) {
			var generations:Array<Int> = [];
			for (index in 0...feature.topologyReferenceCount()) {
				var reference = feature.topologyReferenceAt(index);
				generations.push(reference.stateGeneration());
			}
			previousStates.set(feature.id.toInt(), generations);
		}

		try {
			for (feature in order) {
				current = feature;
				if (!feature.dirty)
					continue;

				var result:EvaluationResult = feature.evaluate(context);
				stagedFeatures.push(feature);
				stagedResults.push(result);
				context.stage(feature, result);
			}

			for (index in 0...stagedFeatures.length)
				stagedFeatures[index].install(stagedResults[index]);
			for (feature in stagedFeatures)
				feature.commitEvaluation();
			lastRemapReport = new TopologyRemapReport();
			for (feature in stagedFeatures)
				lastRemapReport.merge(feature.remapTopologyReferences());
			for (element in elements)
				element.commitOutput();
		} catch (error:Dynamic) {
			for (feature in features)
				feature.discardEvaluation();
			lastRemapReport = new TopologyRemapReport();
			var recomputeError:Null<RecomputeError> = null;
			if (current != null) {
				recomputeError = new RecomputeError(current.id, error);
				if (recomputeError.referenceState != null)
					lastRemapReport.add(recomputeError.referenceState);
			}
			if (recomputeError == null || recomputeError.referenceState == null) {
				for (feature in features) {
					var generations = previousStates.get(feature.id.toInt());
					if (generations != null)
						lastRemapReport.merge(feature.topologyReferenceReport(generations));
				}
			}
			for (index in 0...stagedResults.length) {
				var reverse = stagedResults.length - index - 1;
				stagedResults[reverse].dispose();
			}
			if (recomputeError != null)
				throw recomputeError;
			throw error;
		}
	}

	private function synchronizeExpressions():Void {
		var visiting = new Map<String, Bool>();
		var cached = new Map<String, ExpressionValue>();
		var staged:Array<{parameter:NamedParameter, value:Float}> = [];
		for (named in dimensions) {
			if (named.expression == null)
				continue;
			var value = evaluateNamed(named, visiting, cached).value;
			named.validateSynchronized(value);
			staged.push({parameter: named, value: value});
		}
		for (entry in staged)
			entry.parameter.synchronize(entry.value);
	}

	private function evaluateNamed(named:NamedParameter, visiting:Map<String, Bool>, cached:Map<String, ExpressionValue>):ExpressionValue {
		var known = cached.get(named.name);
		if (known != null)
			return known;
		if (visiting.exists(named.name))
			throw new ParametricError("parameter expression cycle detected: " + named.name);
		visiting.set(named.name, true);
		var result:ExpressionValue;
		if (named.expression == null) {
			result = new ExpressionValue(named.value, named.kind);
		} else {
			result = named.expression.evaluate(function(dependency:String) {
				return evaluateNamed(parameter(dependency), visiting, cached);
			});
			if (result.kind != named.kind)
				throw new ParametricError("expression for " + named.name + " produces " + result.kind + ", expected " + named.kind);
			if (named.kind == ParameterKind.Count && result.value != Std.int(result.value))
				throw new ParametricError("count expression must produce an integer: " + named.name);
		}
		visiting.remove(named.name);
		cached.set(named.name, result);
		return result;
	}

	public function beginTransaction():Transaction {
		ensureOpen();
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
		for (index in 0...changes.documentChanges.length) {
			var reverse = changes.documentChanges.length - index - 1;
			changes.documentChanges[reverse].undo();
		}
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
		for (change in changes.documentChanges)
			change.redo();
		undoStack.push(changes);
		return true;
	}

	public function close():Void {
		if (closed)
			return;
		if (activeTransaction != null)
			activeTransaction.cancel();
		for (feature in features)
			feature.close();
		features.resize(0);
		byId = new Map<Int, Feature>();
		dimensions.resize(0);
		elements.resize(0);
		elementsById = new Map<String, Element>();
		issuedElementIds = new Map<String, Bool>();
		undoStack.resize(0);
		redoStack.resize(0);
		selectedOutput = null;
		closed = true;
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

	public function recordDocumentChange(change:DocumentChange):Void {
		if (activeTransaction != null) {
			activeTransaction.recordDocumentChange(change);
			return;
		}
		undoStack.push(new ChangeSet([], [change]));
		redoStack.resize(0);
	}

	public function commitTransaction(transaction:Transaction):Void {
		if (activeTransaction == null || activeTransaction.identity != transaction.identity)
			throw new ParametricError("transaction does not belong to this document");
		activeTransaction = null;
		if (transaction.changes.length == 0 && transaction.documentChanges.length == 0)
			return;
		undoStack.push(new ChangeSet(transaction.changes.copy(), transaction.documentChanges.copy()));
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
		for (index in 0...transaction.documentChanges.length) {
			var reverse = transaction.documentChanges.length - index - 1;
			transaction.documentChanges[reverse].undo();
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

	private function visit(feature:Feature, visiting:Map<Int, Bool>, visited:Map<Int, Bool>, order:Array<Feature>):Void {
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
