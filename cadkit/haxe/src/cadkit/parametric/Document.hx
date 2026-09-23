package cadkit.parametric;

import cadkit.Shape;
import cadkit.parametric.ChangeSet;
import cadkit.parametric.EvaluationContext;
import cadkit.parametric.EvaluationResult;
import cadkit.parametric.Feature;
import cadkit.parametric.FeatureId;
import cadkit.parametric.FeatureActiveChange;
import cadkit.parametric.OutputSelectionChange;
import cadkit.parametric.ParametricError;
import cadkit.parametric.RecomputeError;
import cadkit.parametric.EvaluationCancelled;
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
import cadkit.parametric.Placement;
import cadkit.parametric.PlacementChange;
import cadkit.parametric.Definition;
import cadkit.parametric.DefinitionId;
import cadkit.parametric.DefinitionInput;
import cadkit.parametric.DefinitionOutput;
import cadkit.parametric.InstanceElement;
import cadkit.parametric.DefinitionChanges.DefinitionDefaultChange;
import cadkit.parametric.DefinitionChanges.InstanceOverrideChange;
import cadkit.parametric.DefinitionChanges.DefinitionCreateChange;
import cadkit.parametric.DefinitionChanges.InstanceDefinitionChange;

/** Haxeon-owned parametric feature document. */
class Document {
	private static var nextToken:Int = 1;

	private var nextId:Int;

	public final id:DocumentId;

	private final elements:Array<Element>;
	private var elementsById:Map<String, Element>;
	private var issuedElementIds:Map<String, Bool>;
	private final dimensions:Array<NamedParameter>;
	private final definitions:Array<Definition>;
	private var definitionsById:Map<String, Definition>;
	private final definitionCache:Map<String, Shape>;
	private final issuedDefinitionIds:Map<String, Bool>;

	public var definitionEvaluationCount(default, null):Int;
	public var recomputeAttemptCount(default, null):Int;
	public var lastRecomputeSeconds(default, null):Float;
	public var lastRecomputeFeatureCount(default, null):Int;
	public var lastSketchSolveSeconds(default, null):Float;
	public var lastSketchSolveCount(default, null):Int;
	public var lastSketchProfileSeconds(default, null):Float;

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

	/** Optional layer-owned validation run before any staged feature evaluation. */
	public var beforeRecompute:Null<Void->Void>;

	public var afterRecompute:Null<Void->Void>;

	/** Checked between feature evaluations and by cooperative sketch solving. */
	public var evaluationCancellationCheck:Null<Void->Bool>;

	public function new(?id:DocumentId) {
		this.id = id == null ? new DocumentId() : id;
		token = nextToken;
		nextToken++;
		nextId = 1;
		dimensions = [];
		definitions = [];
		definitionsById = new Map();
		definitionCache = new Map();
		issuedDefinitionIds = new Map();
		definitionEvaluationCount = 0;
		recomputeAttemptCount = 0;
		lastRecomputeSeconds = 0;
		lastRecomputeFeatureCount = 0;
		lastSketchSolveSeconds = 0;
		lastSketchSolveCount = 0;
		lastSketchProfileSeconds = 0;
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
		beforeRecompute = null;
		afterRecompute = null;
		evaluationCancellationCheck = null;
	}

	public function createDefinition(name:String, recipe:String, inputs:Array<DefinitionInput>, outputs:Array<DefinitionOutput>):Definition {
		var result = installDefinition(new DefinitionId(), name, recipe, inputs, outputs);
		recordDocumentChange(new DefinitionCreateChange(this, result, definitions.length - 1));
		return result;
	}

	/** Snapshot a feature document as a reusable, input-bound, named-output definition. */
	public function createSubgraphDefinition(name:String, graph:Document, inputs:Array<DefinitionInput>,
		outputs:Array<DefinitionOutput>, inputBindings:Map<String, String>, outputFeatures:Map<String, Int>):Definition {
		ensureOpen();
		if (graph == null || graph == this || graph.isClosed())
			throw new ParametricError("definition subgraph must be a separate open document");
		if (inputs == null || outputs == null || inputBindings == null || outputFeatures == null)
			throw new ParametricError("definition subgraph ports must not be null");

		var inputNames = new Map<String, Bool>();
		var boundParameters = new Map<String, Bool>();
		for (input in inputs) {
			inputNames.set(input.name, true);
			var parameterName = inputBindings.get(input.name);
			if (parameterName == null)
				throw new ParametricError("definition input has no subgraph binding: " + input.name);
			if (boundParameters.exists(parameterName))
				throw new ParametricError("subgraph parameter is bound to more than one definition input: " + parameterName);
			var parameter = graph.parameter(parameterName);
			if (parameter.kind != input.kind)
				throw new ParametricError("definition input type does not match subgraph parameter: " + input.name);
			if (parameter.expression != null)
				throw new ParametricError("definition input cannot bind an expression parameter: " + parameterName);
			boundParameters.set(parameterName, true);
		}
		for (name in inputBindings.keys())
			if (!inputNames.exists(name))
				throw new ParametricError("subgraph binding references an unknown definition input: " + name);

		var outputNames = new Map<String, Bool>();
		for (output in outputs) {
			outputNames.set(output.name, true);
			var featureId = outputFeatures.get(output.name);
			if (featureId == null)
				throw new ParametricError("definition output has no subgraph feature: " + output.name);
			var feature = graph.featureById(featureId);
			if (feature == null || !feature.active)
				throw new ParametricError("definition output feature is missing or inactive: " + output.name);
		}
		for (name in outputFeatures.keys())
			if (!outputNames.exists(name))
				throw new ParametricError("subgraph output references an unknown definition output: " + name);

		var primaryGeometry = false;
		for (output in outputs)
			if (output.purpose == DefinitionOutput.Geometry)
				primaryGeometry = true;
		if (!primaryGeometry)
			throw new ParametricError("subgraph definition requires a geometry output");

		var subgraph = new DefinitionSubgraph(DocumentCodec.encode(graph), inputBindings, outputFeatures);
		var result = installDefinition(new DefinitionId(), name, Definition.SubgraphRecipe, inputs, outputs, subgraph);
		recordDocumentChange(new DefinitionCreateChange(this, result, definitions.length - 1));
		return result;
	}

	public function installDefinition(id:DefinitionId, name:String, recipe:String, inputs:Array<DefinitionInput>,
		outputs:Array<DefinitionOutput>, ?subgraph:DefinitionSubgraph):Definition {
		if (issuedDefinitionIds.exists(id.value))
			throw new ParametricError("duplicate or previously issued definition ID: " + id.value);
		if ((recipe == Definition.SubgraphRecipe) != (subgraph != null))
			throw new ParametricError("feature subgraph recipe and graph payload do not match");
		var result = new Definition(this, id, name, recipe, inputs, outputs, subgraph);
		definitions.push(result);
		definitionsById.set(id.value, result);
		issuedDefinitionIds.set(id.value, true);
		return result;
	}

	public function restoreDefinitionRemoval(definition:Definition):Void {
		definitions.remove(definition);
		definitionsById.remove(definition.id.value);
	}

	public function restoreDefinitionInsertion(definition:Definition, index:Int):Void {
		if (definitionsById.exists(definition.id.value))
			throw new ParametricError("definition is already registered");
		definitions.insert(index, definition);
		definitionsById.set(definition.id.value, definition);
	}

	public function definition(id:DefinitionId):Definition {
		var result = definitionsById.get(id.value);
		if (result == null)
			throw new ParametricError("unresolved definition: " + id.value);
		return result;
	}

	public function allDefinitions():Array<Definition>
		return definitions.copy();

	public function createInstance(name:String, definition:Definition):InstanceElement {
		if (definition.document != this)
			throw new ParametricError("definition belongs to another document");
		definition.primaryGeometryOutput();
		var id = newElementId();
		var result = new InstanceElement(this, id, name, definition.id);
		installRecord(result);
		result.restoreDirectShape(resolveInstanceShape(result));
		recordDocumentChange(new ElementCreateChange(this, result, elements.length - 1));
		return result;
	}

	public function installInstance(name:String, id:ElementId, definitionId:DefinitionId, overrides:Map<String, Float>):InstanceElement {
		definition(definitionId).primaryGeometryOutput();
		var result = new InstanceElement(this, id, name, definitionId);
		for (key in overrides.keys()) {
			definition(definitionId).input(key);
			result.restoreOverride(key, overrides.get(key));
		}
		installRecord(result);
		result.restoreDirectShape(resolveInstanceShape(result));
		return result;
	}

	public function duplicateInstance(source:InstanceElement, ?name:String):InstanceElement {
		validateOwnedElement(source);
		var overrides = new Map<String, Float>();
		for (key in source.overrideNames())
			overrides.set(key, cast source.overrideValue(key));
		var result = installInstance(name == null ? source.name + " copy" : name, newElementId(), source.definitionId, overrides);
		recordDocumentChange(new ElementCreateChange(this, result, elements.length - 1));
		return result;
	}

	/** Detach one instance from its shared definition without changing its element identity. */
	public function makeInstanceUnique(instance:InstanceElement, ?definitionName:String):Definition {
		validateOwnedElement(instance);
		var source = definition(instance.definitionId);
		var inputs:Array<DefinitionInput> = [];
		for (input in source.inputs())
			inputs.push(new DefinitionInput(input.name, input.kind, input.unit,
				UnitConversion.fromCanonical(input.defaultValue, input.kind, input.unit)));
		var outputs:Array<DefinitionOutput> = [];
		for (output in source.outputs())
			outputs.push(new DefinitionOutput(output.name, output.purpose));

		var subgraph:Null<DefinitionSubgraph> = null;
		var sourceSubgraph = source.subgraph;
		if (sourceSubgraph != null) {
			var inputBindings = new Map<String, String>();
			for (input in source.inputs())
				inputBindings.set(input.name, sourceSubgraph.parameterName(input.name));
			var outputFeatures = new Map<String, Int>();
			for (output in source.outputs())
				outputFeatures.set(output.name, sourceSubgraph.featureId(output.name));
			subgraph = new DefinitionSubgraph(sourceSubgraph.graph, inputBindings, outputFeatures);
		}

		var name = definitionName == null ? instance.name + " definition" : definitionName;
		var unique = installDefinition(new DefinitionId(), name, source.recipe, inputs, outputs, subgraph);
		var previousDefinition = instance.definitionId;
		try {
			restoreInstanceDefinition(instance, unique.id);
		} catch (error:Dynamic) {
			restoreDefinitionRemoval(unique);
			throw error;
		}
		recordDocumentChange(new DefinitionCreateChange(this, unique, definitions.length - 1));
		recordDocumentChange(new InstanceDefinitionChange(this, instance, previousDefinition, unique.id));
		return unique;
	}

	/** Restore an instance's definition and published shape for undo/redo. */
	public function restoreInstanceDefinition(instance:InstanceElement, definitionId:DefinitionId):Void {
		validateOwnedElement(instance);
		var nextDefinition = definition(definitionId);
		nextDefinition.primaryGeometryOutput();
		var previousDefinition = instance.definitionId;
		if (previousDefinition.value == definitionId.value)
			return;
		instance.restoreDefinitionId(definitionId);
		var nextShape:Shape;
		try {
			nextShape = resolveInstanceShape(instance);
		} catch (error:Dynamic) {
			instance.restoreDefinitionId(previousDefinition);
			throw error;
		}
		instance.restoreDirectShape(nextShape);
	}

	private function instanceKey(instance:InstanceElement, output:String):String {
		var definition = definition(instance.definitionId);
		definition.output(output);
		var parts = [definition.id.value, Std.string(definition.revision), output];
		for (input in definition.inputs())
			parts.push(input.name + "=" + Std.string(instance.resolved(input.name)));
		return parts.join("|");
	}

	private function resolveInstanceShape(instance:InstanceElement):Shape {
		var definition = definition(instance.definitionId);
		return definitionOutput(instance, definition.primaryGeometryOutput().name);
	}

	public function definitionOutput(instance:InstanceElement, output:String):Shape {
		validateOwnedElement(instance);
		var key = instanceKey(instance, output);
		var known = definitionCache.get(key);
		if (known != null)
			return known;
		var result = evaluateDefinition(definition(instance.definitionId), instance, output);
		definitionCache.set(key, result);
		return result;
	}

	private function evaluateDefinition(definition:Definition, instance:InstanceElement, output:String):Shape {
		definitionEvaluationCount++;
		return DefinitionEvaluatorRegistry.evaluate(definition, instance, output);
	}

	private function refreshDefinition(definition:Definition):Void {
		var staged:Array<{instance:InstanceElement, shape:Shape}> = [];
		for (element in elements)
			if (element.kind == "instance") {
				var instance:InstanceElement = cast element;
				if (instance.definitionId.value == definition.id.value)
					staged.push({instance: instance, shape: resolveInstanceShape(instance)});
			}
		for (value in staged)
			value.instance.restoreDirectShape(value.shape);
		invalidateFrom(DependencyNode.DefinitionNode(definition.id.value));
	}

	public function setDefinitionDefault(definition:Definition, name:String, value:Float, ?unit:String):Void {
		var input = definition.input(name);
		var canonical = UnitConversion.toCanonical(value, input.kind, unit == null ? input.unit : unit);
		var before = input.defaultValue;
		var revision = definition.revision;
		if (before == canonical)
			return;
		definition.restoreDefault(name, canonical, revision + 1);
		try {
			if (beforeRecompute != null)
				beforeRecompute();
			refreshDefinition(definition);
		} catch (e:Dynamic) {
			definition.restoreDefault(name, before, revision);
			throw e;
		}
		recordDocumentChange(new DefinitionDefaultChange(this, definition, name, before, revision, canonical, revision + 1));
	}

	public function restoreDefinitionDefault(definition:Definition, name:String, value:Float, revision:Int):Void {
		definition.restoreDefault(name, value, revision);
		refreshDefinition(definition);
	}

	public function setInstanceOverride(instance:InstanceElement, name:String, value:Float, ?unit:String):Void {
		validateOwnedElement(instance);
		var input = definition(instance.definitionId).input(name);
		var canonical = UnitConversion.toCanonical(value, input.kind, unit == null ? input.unit : unit);
		var before = instance.overrideValue(name);
		instance.restoreOverride(name, canonical);
		try {
			if (beforeRecompute != null)
				beforeRecompute();
			instance.restoreDirectShape(resolveInstanceShape(instance));
		} catch (e:Dynamic) {
			instance.restoreOverride(name, before);
			throw e;
		}
		invalidateFrom(DependencyNode.ElementNode(instance.id.value));
		recordDocumentChange(new InstanceOverrideChange(this, instance, name, before, canonical));
	}

	public function removeInstanceOverride(instance:InstanceElement, name:String):Void {
		validateOwnedElement(instance);
		var before = instance.overrideValue(name);
		if (before == null)
			return;
		instance.restoreOverride(name, null);
		try {
			if (beforeRecompute != null)
				beforeRecompute();
			instance.restoreDirectShape(resolveInstanceShape(instance));
		} catch (error:Dynamic) {
			instance.restoreOverride(name, before);
			throw error;
		}
		invalidateFrom(DependencyNode.ElementNode(instance.id.value));
		recordDocumentChange(new InstanceOverrideChange(this, instance, name, before, null));
	}

	public function restoreInstanceOverride(instance:InstanceElement, name:String, value:Null<Float>):Void {
		instance.restoreOverride(name, value);
		instance.restoreDirectShape(resolveInstanceShape(instance));
		invalidateFrom(DependencyNode.ElementNode(instance.id.value));
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

	public function createLevel(name:String, elevation:Float, offset:Float = 0, ?relativeTo:ElementReference):LevelElement {
		if (relativeTo != null)
			resolveElement(relativeTo, "level");
		var id = newElementId();
		var result = new LevelElement(this, id, name, elevation, offset, relativeTo);
		installRecord(result);
		recordDocumentChange(new ElementCreateChange(this, result, elements.length - 1));
		return result;
	}

	public function createReferencePlane(name:String, plane:Plane):ReferencePlaneElement {
		var id = newElementId();
		var result = new ReferencePlaneElement(this, id, name, plane);
		installRecord(result);
		recordDocumentChange(new ElementCreateChange(this, result, elements.length - 1));
		return result;
	}

	public function installLevel(name:String, id:ElementId, elevation:Float, offset:Float = 0, ?relativeTo:ElementReference):LevelElement {
		if (relativeTo != null)
			resolveElement(relativeTo, "level");
		var result = new LevelElement(this, id, name, elevation, offset, relativeTo);
		installRecord(result);
		return result;
	}

	public function installReferencePlane(name:String, id:ElementId, plane:Plane):ReferencePlaneElement {
		var result = new ReferencePlaneElement(this, id, name, plane);
		installRecord(result);
		return result;
	}

	private function newElementId():ElementId {
		var value = new ElementId();
		while (issuedElementIds.exists(value.value))
			value = new ElementId();
		return value;
	}

	private function installRecord(result:Element):Void {
		validateElementName(result.name);
		if (issuedElementIds.exists(result.id.value))
			throw new ParametricError("duplicate or previously issued element ID: " + result.id.value);
		elements.push(result);
		elementsById.set(result.id.value, result);
		issuedElementIds.set(result.id.value, true);
	}

	public function setLevelElevation(level:LevelElement, value:Float):Void {
		validateOwnedElement(level);
		if (!Math.isFinite(value))
			throw new ParametricError("level elevation must be finite");
		var old = level.elevation;
		if (old == value)
			return;
		level.restoreElevation(value);
		recordDocumentChange(new LevelElevationChange(level, old, value));
	}

	public function setReferencePlane(datum:ReferencePlaneElement, value:Plane):Void {
		validateOwnedElement(datum);
		var old = datum.plane;
		datum.restorePlane(value);
		recordDocumentChange(new ReferencePlaneChange(datum, old, value));
	}

	public function datumChanged(datum:Element):Void {
		invalidateFrom(DependencyNode.ElementNode(datum.id.value));
	}

	public function worldPlacement(element:Element):Placement {
		validateOwnedElement(element);
		return resolvePlacement(element, new Map<String, Bool>());
	}

	private function resolvePlacement(element:Element, visiting:Map<String, Bool>):Placement {
		if (visiting.exists(element.id.value))
			throw new ParametricError("placement parent cycle: " + element.id.value);
		visiting.set(element.id.value, true);
		var result = element.localPlacement;
		if (element.placementParent != null) {
			var parent = resolveElement(element.placementParent);
			if (!isPlaceable(parent))
				throw new ParametricError("incompatible placement parent kind: " + parent.kind);
			result = resolvePlacement(parent, visiting).compose(result);
		}
		visiting.remove(element.id.value);
		return result;
	}

	public function setElementPlacement(element:Element, value:Placement):Void {
		validateOwnedElement(element);
		if (element.placementDerived)
			throw new ParametricError("element placement is derived: " + element.id.value);
		if (!isPlaceable(element))
			throw new ParametricError("element kind does not support placement: " + element.kind);
		var old = element.localPlacement;
		if (old == value)
			return;
		restoreElementPlacement(element, value, element.placementParent);
		recordDocumentChange(new PlacementChange(this, element, old, element.placementParent, value, element.placementParent));
	}

	public function reparentElement(element:Element, parent:Null<ElementReference>, preserveWorld:Bool):Void {
		validateOwnedElement(element);
		if (element.placementDerived)
			throw new ParametricError("element placement is derived: " + element.id.value);
		if (!isPlaceable(element))
			throw new ParametricError("element kind does not support placement parents: " + element.kind);
		validatePlacementParent(element, parent);
		var oldParent = element.placementParent;
		var oldLocal = element.localPlacement;
		var world = worldPlacement(element);
		var next = oldLocal;
		if (preserveWorld) {
			var parentWorld = parent == null ? Placement.identity() : worldPlacement(resolveElement(parent));
			next = parentWorld.inverse().compose(world);
		}
		restoreElementPlacement(element, next, parent);
		recordDocumentChange(new PlacementChange(this, element, oldLocal, oldParent, next, parent));
	}

	private function isPlaceable(element:Element):Bool
		return element.kind == "geometry" || element.kind == "instance";

	private function validatePlacementParent(element:Element, parent:Null<ElementReference>):Void {
		var cursor = parent;
		var seen = new Map<String, Bool>();
		while (cursor != null) {
			var candidate = resolveElement(cursor);
			if (!isPlaceable(candidate))
				throw new ParametricError("incompatible placement parent kind: " + candidate.kind);
			if (candidate == element)
				throw new ParametricError("placement parent cycle: " + element.id.value);
			if (seen.exists(candidate.id.value))
				throw new ParametricError("placement parent cycle: " + candidate.id.value);
			seen.set(candidate.id.value, true);
			cursor = candidate.placementParent;
		}
	}

	public function restoreElementPlacement(element:Element, value:Placement, parent:Null<ElementReference>):Void {
		element.restorePlacement(value, parent);
		invalidateFrom(DependencyNode.ElementNode(element.id.value));
	}

	public function duplicateElement(source:Element, ?name:String):Element {
		validateOwnedElement(source);
		if (source.output == null)
			throw new ParametricError("datum duplication requires its typed API");
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
		if (target.name == name)
			return;
		var previous = target.name;
		target.restoreName(name);
		recordDocumentChange(new ElementNameChange(target, previous, name));
	}

	public function setElementOutput(target:Element, output:Feature):Void {
		validateOwnedElement(target);
		validateElementOutput(output);
		if (target.output == output)
			return;
		var previous = target.output;
		target.restoreOutput(output);
		invalidateFrom(DependencyNode.ElementNode(target.id.value));
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

	public function resolveElement(reference:ElementReference, ?expectedKind:String):Element {
		var state = reference.state(this, expectedKind);
		if (state != "resolved")
			throw new ParametricError("element reference is " + state + ": " + reference.elementId.value);
		return element(reference.elementId);
	}

	public function levelElevation(reference:ElementReference):Float {
		return resolveLevel(reference, new Map<String, Bool>());
	}

	private function resolveLevel(reference:ElementReference, visiting:Map<String, Bool>):Float {
		var level:LevelElement = cast resolveElement(reference, "level");
		if (visiting.exists(level.id.value))
			throw new ParametricError("level reference cycle: " + level.id.value);
		visiting.set(level.id.value, true);
		var value = level.elevation + level.offset;
		if (level.relativeTo != null)
			value += resolveLevel(level.relativeTo, visiting);
		visiting.remove(level.id.value);
		return value;
	}

	public function restoreElementInsertion(element:Element, index:Int):Void {
		if (element.document != this || elementsById.exists(element.id.value))
			throw new ParametricError("cannot restore element: " + element.id.value);
		var insertion = index < 0 ? 0 : (index > elements.length ? elements.length : index);
		elements.insert(insertion, element);
		elementsById.set(element.id.value, element);
		datumChanged(element);
	}

	public function restoreElementRemoval(element:Element):Void {
		validateOwnedElement(element);
		datumChanged(element);
		element.clearPlacedShape();
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

	/** Retain a new graph node for identity, but deactivate it if its transaction is undone. */
	public function trackFeatureCreation(feature:Feature):Void {
		if (activeTransaction == null || feature.document != this)
			throw new ParametricError("tracked feature creation requires an active document transaction");
		recordDocumentChange(new FeatureActiveChange(feature, false, true));
	}

	public function setFeatureActive(feature:Feature, value:Bool):Void {
		if (activeTransaction == null || feature.document != this)
			throw new ParametricError("feature activation requires an active document transaction");
		if (feature.active == value)
			return;
		var before = feature.active;
		feature.restoreActive(value);
		recordDocumentChange(new FeatureActiveChange(feature, before, value));
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
		var result = new NamedParameter(this, name, UnitConversion.toCanonical(value, validatedKind, validatedUnit), validatedKind, validatedUnit);
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

	public function setOutputTracked(feature:Null<Feature>):Void {
		if (activeTransaction == null)
			throw new ParametricError("tracked output selection requires an active document transaction");
		var before = selectedOutput;
		if (feature == null)
			selectedOutput = null;
		else
			setOutput(feature);
		if (before != feature)
			recordDocumentChange(new OutputSelectionChange(this, before, feature));
	}

	public function restoreOutputSelection(value:Null<Feature>):Void
		selectedOutput = value;

	/** Without an explicit selection, the last feature remains the primary output. */
	public function outputFeatureOrNull():Null<Feature> {
		ensureOpen();
		if (selectedOutput != null && selectedOutput.active)
			return selectedOutput;
		for (index in 0...features.length) {
			var feature = features[features.length - index - 1];
			if (feature.active)
				return feature;
		}
		return null;
	}

	public function outputFeature():Feature {
		var result = outputFeatureOrNull();
		if (result == null)
			throw new ParametricError("document has no output feature");
		return result;
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
		invalidateFrom(DependencyNode.ParameterNode(parameter.name));
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
		var started = Sys.time();
		var context = new EvaluationContext(this);
		var order:Array<Feature>;
		try {
			if (beforeRecompute != null)
				beforeRecompute();
			context.checkCancelled();
			synchronizeExpressions();
			order = topologicalOrder();
		} catch (error:Dynamic) {
			recordRecomputeMetrics(started, 0, context);
			throw error;
		}
		var stagedFeatures:Array<Feature> = [];
		var stagedResults:Array<EvaluationResult> = [];
		var evaluatedFeatureCount = 0;
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
				context.checkCancelled();
				current = feature;
				if (!feature.active || !feature.dirty)
					continue;

				evaluatedFeatureCount++;
				var result:EvaluationResult = feature.evaluate(context);
				stagedFeatures.push(feature);
				stagedResults.push(result);
				context.stage(feature, result);
			}
			context.checkCancelled();

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
			recordRecomputeMetrics(started, evaluatedFeatureCount, context);
			if (Std.isOfType(error, EvaluationCancelled))
				throw error;
			if (recomputeError != null)
				throw recomputeError;
			throw error;
		}
		recordRecomputeMetrics(started, evaluatedFeatureCount, context);

		// Publication succeeded. Observer failures are reported to the caller, but cannot
		// roll back or dispose resources that are now owned by committed features.
		if (afterRecompute != null)
			afterRecompute();
	}

	public function isEvaluationCancelled():Bool
		return evaluationCancellationCheck != null && evaluationCancellationCheck();

	private function recordRecomputeMetrics(started:Float, evaluated:Int, context:EvaluationContext):Void {
		recomputeAttemptCount++;
		lastRecomputeSeconds = Math.max(0, Sys.time() - started);
		lastRecomputeFeatureCount = evaluated;
		lastSketchSolveSeconds = context.sketchSolveSeconds;
		lastSketchSolveCount = context.sketchSolveCount;
		lastSketchProfileSeconds = context.sketchProfileSeconds;
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

	/** Discard local undo records when an enclosing editor owns project history. */
	public function clearHistory():Void {
		ensureOpen();
		if (activeTransaction != null)
			throw new ParametricError("finish the active transaction before clearing history");
		undoStack.resize(0);
		redoStack.resize(0);
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
		definitions.resize(0);
		for (element in elements)
			element.clearPlacedShape();
		for (shape in definitionCache)
			shape.close();
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
		for (index in 0...transaction.documentChanges.length) {
			var reverse = transaction.documentChanges.length - index - 1;
			transaction.documentChanges[reverse].undo();
		}
		for (index in 0...transaction.changes.length) {
			var reverse = transaction.changes.length - index - 1;
			var change = transaction.changes[reverse];
			change.parameter.restore(change.oldValue);
		}
		discardCancelledFeatures(transaction.initialFeatureCount);
	}

	private function discardCancelledFeatures(start:Int):Void {
		for (index in start...features.length)
			if (features[index].active)
				return;
		if (selectedOutput != null && selectedOutput.id.toInt() > start)
			return;
		for (element in elements)
			if (element.output != null && element.output.id.toInt() > start)
				return;
		for (index in 0...start)
			for (dependency in features[index].dependencies())
				if (dependency.toInt() > start)
					return;
		while (features.length > start) {
			var feature = features.pop();
			byId.remove(feature.id.toInt());
			feature.close();
		}
		nextId = start + 1;
	}

	public function invalidate(feature:Feature):Void {
		invalidateFrom(DependencyNode.FeatureNode(feature.id.toInt()));
	}

	private function invalidateFrom(source:DependencyNode):Void {
		var index = new DependencyIndex(this);
		var pending:Array<DependencyNode> = [source];
		var visited = new Map<String, Bool>();
		var cursor = 0;
		while (cursor < pending.length) {
			var current = pending[cursor++];
			var currentKey = DependencyIndex.key(current);
			if (visited.exists(currentKey))
				continue;
			visited.set(currentKey, true);

			switch (current) {
				case FeatureNode(featureId):
					var feature = featureById(featureId);
					if (feature != null)
						feature.dirty = true;
				case ElementNode(elementId):
					var element = findElement(new ElementId(elementId));
					if (element != null)
						element.clearPlacedShape();
				case DefinitionNode(_), ParameterNode(_):
			}

			for (dependent in index.dependents(current))
				pending.push(dependent);
		}
	}

	private function topologicalOrder():Array<Feature> {
		var order:Array<Feature> = [];
		var visiting = new Map<Int, Bool>();
		var visited = new Map<Int, Bool>();
		for (feature in features)
			if (feature.active)
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
			if (!dependencyFeature.active)
				throw new ParametricError("active feature depends on an inactive feature");
			visit(dependencyFeature, visiting, visited, order);
		}
		visiting.remove(key);
		visited.set(key, true);
		order.push(feature);
	}
}
