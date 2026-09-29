package cadkit.parametric;

import haxeon.wire.JsonWire;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinition.AssemblyComponentDefinition;
import materia.assembly.AssemblyDefinition.AssemblyComponentOccurrence;
import materia.assembly.AssemblyDefinition.AssemblyExposedConnector;
import materia.assembly.AssemblyDefinition.AssemblyJointCoupling;
import materia.assembly.AssemblyDefinition.AssemblyJointLimits;
import materia.assembly.AssemblyDefinition.AssemblySubdefinition;
import materia.assembly.AssemblyDefinition.AssemblyVector;
import materia.assembly.AssemblyDefinition.KinematicJoint;
import materia.assembly.AssemblyDefinitionCodec;
import materia.assembly.AssemblyRecord.AssemblyFrame;

private class AssemblyDocumentScope {
	public final definitions:Array<AssemblyComponentDefinition> = [];
	public final occurrences:Array<AssemblyComponentOccurrence> = [];
	public final joints:Array<KinematicJoint> = [];
	public final couplings:Array<AssemblyJointCoupling> = [];
	public var hasCouplings:Bool = false;
}

typedef AssemblyDefinitionResolver = (String, AssemblyComponentOccurrence,
	Null<AssemblyComponentDefinition>)->Null<Definition>;

/** An owned relationship whose document endpoint has been removed. */
class AssemblyDocumentDiagnostic {
	public final severity:String = "error";
	public final code:String;
	public final subject:String;
	public final message:String;

	public function new(code:String, subject:String, message:String) {
		this.code = code;
		this.subject = subject;
		this.message = message;
	}

	public function toString():String return '$code: $subject: $message';
}

/** Maps mechanical definitions to persistent CadKit document objects and relationships. */
class AssemblyDocuments {
	public static inline var JOINT:String = "cadkit.joint";
	public static inline var COUPLING:String = "cadkit.coupling";
	static inline var PREFIX:String = "cadkit.assembly.";

	public static function fromDefinition(document:Document, definition:AssemblyDefinition,
			?resolve:AssemblyDefinitionResolver):Element {
		AssemblyDefinitionCodec.validate(definition);
		var ownTransaction = !document.hasActiveTransaction();
		var transaction = ownTransaction ? document.beginTransaction() : null;
		try {
			var result = writeDefinition(document, definition, resolve);
			if (transaction != null) transaction.commit();
			return result;
		} catch (error:Dynamic) {
			if (transaction != null) transaction.cancel();
			throw error;
		}
	}

	static function writeDefinition(document:Document, definition:AssemblyDefinition,
			resolve:Null<AssemblyDefinitionResolver>):Element {
		AssemblyMemberEvaluator.register();
		for (existing in document.allElements()) if (readOrNull(existing, "kind") == "root" &&
			readOrNull(existing, "id") == definition.id) removeAssembly(document, existing);
		var root = document.createObject(definition.id);
		put(root, "kind", "root");
		put(root, "id", definition.id);
		root.setProperty(TypedProperty.integer(PREFIX + "schemaVersion", definition.schemaVersion));
		root.setProperty(TypedProperty.boolean(PREFIX + "hasAssemblies", definition.assemblies != null));
		if (definition.lengthUnit != null) put(root, "lengthUnit", definition.lengthUnit);
		if (definition.exposedConnectors != null)
			put(root, "exposed", JsonWire.encode(definition.exposedConnectors));
		var rootMembers = writeScope(document, root, "", root, definition.definitions, definition.occurrences,
			definition.couplings, definition.exposedConnectors, resolve, "");
		var scopes = new Map<String, Map<String, Element>>();
		scopes.set("", rootMembers);
		var scopePaths = new Map<String, String>();
		if (definition.assemblies != null) {
			var byId = new Map<String, AssemblySubdefinition>();
			for (nested in definition.assemblies) byId.set(nested.id, nested);
			collectScopePaths(definition.occurrences, byId, "", scopePaths);
		}
		if (definition.assemblies != null) for (nested in definition.assemblies) {
			var holder = document.createObject(nested.id);
			put(holder, "kind", "subdefinition");
			putOwner(holder, root);
			put(holder, "scope", nested.id);
			scopes.set(nested.id, writeScope(document, root, nested.id, holder, nested.definitions,
				nested.occurrences, nested.couplings, nested.exposedConnectors, resolve, scopePaths.get(nested.id)));
		}
		writeJoints(document, root, "", definition.joints, definition.couplings, rootMembers);
		if (definition.assemblies != null) for (nested in definition.assemblies)
			writeJoints(document, root, nested.id, nested.joints, nested.couplings, scopes.get(nested.id));
		return root;
	}

	static function collectScopePaths(occurrences:Array<AssemblyComponentOccurrence>,
			byId:Map<String, AssemblySubdefinition>, prefix:String, paths:Map<String, String>):Void {
		for (occurrence in occurrences) if (occurrence.assembly != null) {
			var nested = byId.get(occurrence.assembly);
			if (nested == null) throw 'Unknown nested assembly "${occurrence.assembly}"';
			var path = prefix + occurrence.id + "/";
			if (!paths.exists(nested.id)) paths.set(nested.id, path);
			collectScopePaths(nested.occurrences, byId, path, paths);
		}
	}

	static function removeAssembly(document:Document, root:Element):Void {
		var owned = [for (element in document.allElements()) if (belongsTo(element, root)) element];
		var ownedIds:Map<String, Bool> = [];
		ownedIds.set(root.id.value, true);
		for (element in owned) ownedIds.set(element.id.value, true);
		for (relationship in document.allRelationships()) if (
			ownedIds.exists(relationship.source.elementId.value) ||
			ownedIds.exists(relationship.target.elementId.value)) document.removeRelationship(relationship.id);
		for (element in owned) document.removeElement(element.id);
		document.removeElement(root.id);
	}

	public static function toDefinition(root:Element):AssemblyDefinition {
		if (root == null || read(root, "kind") != "root") throw "Element is not an assembly root";
		var document = root.document;
		var scopes = new Map<String, AssemblyDocumentScope>();
		var rootScope = new AssemblyDocumentScope();
		rootScope.hasCouplings = bool(root, "hasCouplings");
		scopes.set("", rootScope);
		var subdefinitions:Array<AssemblySubdefinition> = [];
		for (element in document.allElements()) if (belongsTo(element, root) && read(element, "kind") == "subdefinition") {
			var scope = read(element, "scope");
			if (scopes.exists(scope)) throw 'Duplicate assembly scope "$scope"';
			var data = new AssemblyDocumentScope();
			data.hasCouplings = bool(element, "hasCouplings");
			scopes.set(scope, data);
			var nested:AssemblySubdefinition = {id: scope, definitions: data.definitions,
				occurrences: data.occurrences, joints: data.joints};
			if (data.hasCouplings || data.couplings.length > 0) nested.couplings = data.couplings;
			var exposure = readOrNull(element, "exposed");
			if (exposure != null) nested.exposedConnectors = decodeExposed(exposure);
			subdefinitions.push(nested);
		}
		var byElement = new Map<String, {scope:String, id:String}>();
		var seenDefinitions:Map<String, Bool> = [];
		for (element in document.allElements()) if (belongsTo(element, root)) {
			var kind = read(element, "kind");
			if (kind != "component" && kind != "occurrence") continue;
			var scopeName = read(element, "scope"), scope = scopes.get(scopeName);
			if (scope == null) throw 'Unknown assembly scope "$scopeName"';
			if (kind == "component") {
				var record:AssemblyComponentDefinition = JsonWire.decode(read(element, "record"));
				scope.definitions.push(record);
			} else {
				var recordText = readOrNull(element, "record");
				if (recordText != null) {
					var record:AssemblyComponentDefinition = JsonWire.decode(recordText);
					var key = scopeName + "/" + record.id;
					if (!seenDefinitions.exists(key)) {
						scope.definitions.push(record);
						seenDefinitions.set(key, true);
					}
				}
				var occurrence:AssemblyComponentOccurrence = {id: read(element, "id"),
					definition: read(element, "definition"),
					initialPose: element.kind == "instance" ? PlacementFrames.toAssemblyFrame(element.localPlacement) :
						decodeFrame(read(element, "initialPose"))};
				var assembly = readOrNull(element, "assembly");
				if (assembly != null) occurrence.assembly = assembly;
				scope.occurrences.push(occurrence);
				byElement.set(element.id.value, {scope: scopeName, id: occurrence.id});
			}
		}
		for (relationship in document.allRelationships()) if (relationship.typeName == JOINT) {
			var parent = byElement.get(relationship.source.elementId.value);
			var child = byElement.get(relationship.target.elementId.value);
			if (parent == null || child == null) {
				if (readRelationshipOrNull(relationship, "owner") == root.id.value)
					throw new AssemblyDocumentDiagnostic("assembly.missing-joint-endpoint",
						readRelationship(relationship, "id"), "Joint endpoint is missing");
				continue;
			}
			var scopeName = readRelationship(relationship, "scope");
			if (parent.scope != scopeName || child.scope != scopeName) throw "Joint crosses assembly scopes";
			var scope = scopes.get(scopeName);
			if (scope == null) throw 'Unknown joint scope "$scopeName"';
			var axis:AssemblyVector = readAxis(relationship);
			var limits:AssemblyJointLimits = readLimits(relationship);
			var joint:KinematicJoint = {id: readRelationship(relationship, "id"), type: cast readRelationship(relationship, "type"),
				role: cast readRelationship(relationship, "role"), parent: parent.id, parentConnector: readRelationship(relationship, "parentConnector"),
				child: child.id, childConnector: readRelationship(relationship, "childConnector"), axis: axis, limits: limits,
				defaultValue: number(relationship, "defaultValue")};
			var tolerance = relationship.property(PREFIX + "closureTolerance");
			if (tolerance != null) joint.closureTolerance = cast tolerance.value;
			scope.joints.push(joint);
		}
		for (relationship in document.allRelationships()) if (relationship.typeName == COUPLING) {
			var source = byElement.get(relationship.source.elementId.value);
			var target = byElement.get(relationship.target.elementId.value);
			if (source == null || target == null) {
				if (readRelationshipOrNull(relationship, "owner") == root.id.value)
					throw new AssemblyDocumentDiagnostic("assembly.missing-coupling-endpoint",
						readRelationship(relationship, "id"), "Coupling endpoint is missing");
				continue;
			}
			var scopeName = readRelationship(relationship, "scope"), scope = scopes.get(scopeName);
			if (scope == null) throw 'Unknown coupling scope "$scopeName"';
			if (source.scope != scopeName || target.scope != scopeName) throw "Coupling crosses assembly scopes";
			scope.couplings.push({id: readRelationship(relationship, "id"), source: drivingJoint(scope.joints, source.id),
				target: drivingJoint(scope.joints, target.id), ratio: number(relationship, "ratio"),
				offset: number(relationship, "offset")});
		}
		var definition:AssemblyDefinition = {schemaVersion: integer(root, "schemaVersion"), id: read(root, "id"),
			definitions: rootScope.definitions, occurrences: rootScope.occurrences, joints: rootScope.joints};
		if (rootScope.hasCouplings || rootScope.couplings.length > 0) definition.couplings = rootScope.couplings;
		if (bool(root, "hasAssemblies") || subdefinitions.length > 0) definition.assemblies = subdefinitions;
		var unit = readOrNull(root, "lengthUnit");
		if (unit != null) definition.lengthUnit = unit;
		var exposed = readOrNull(root, "exposed");
		if (exposed != null) definition.exposedConnectors = decodeExposed(exposed);
		sortScope(rootScope);
		for (scope in scopes) if (scope != rootScope) sortScope(scope);
		subdefinitions.sort((a, b) -> Reflect.compare(a.id, b.id));
		AssemblyDefinitionCodec.validate(definition);
		return definition;
	}

	static function writeScope(document:Document, root:Element, scope:String, holder:Element,
			definitions:Array<AssemblyComponentDefinition>, occurrences:Array<AssemblyComponentOccurrence>,
			couplings:Array<AssemblyJointCoupling>, exposed:Array<AssemblyExposedConnector>,
			resolve:Null<AssemblyDefinitionResolver>, path:Null<String>):Map<String, Element> {
		holder.setProperty(TypedProperty.boolean(PREFIX + "hasCouplings", couplings != null));
		if (scope != "" && exposed != null) put(holder, "exposed", JsonWire.encode(exposed));
		var definitionsById = new Map<String, AssemblyComponentDefinition>();
		var fallback = new Map<String, Definition>();
		for (definition in definitions) {
			definitionsById.set(definition.id, definition);
		}
		var members = new Map<String, Element>();
		for (occurrence in occurrences) {
			var component = occurrence.assembly == null ? definitionsById.get(occurrence.definition) : null;
			var instanceDefinition = resolve == null ? null : resolve(path == null ? "" : path, occurrence, component);
			if (instanceDefinition == null) {
				instanceDefinition = fallback.get(occurrence.definition);
				if (instanceDefinition == null) {
					instanceDefinition = document.createDefinition(occurrence.definition,
						AssemblyMemberEvaluator.RECIPE, [], [new DefinitionOutput("body", DefinitionOutput.Geometry)]);
					var noConnectors:Array<materia.assembly.AssemblyRecord.AssemblyConnector> = [];
					instanceDefinition.setProperty(TypedProperty.text(PREFIX + "connectors",
						JsonWire.encode(component == null ? noConnectors : component.connectors)));
					fallback.set(occurrence.definition, instanceDefinition);
				}
			}
			var element = document.createInstance(occurrence.id, instanceDefinition);
			element.setPlacement(PlacementFrames.fromAssemblyFrame(occurrence.initialPose));
			put(element, "kind", "occurrence"); putOwner(element, root); put(element, "scope", scope);
			if (path != null) put(element, "path", path + occurrence.id);
			put(element, "id", occurrence.id); put(element, "definition", occurrence.definition);
			if (component != null) put(element, "record", JsonWire.encode(component));
			if (occurrence.assembly != null) put(element, "assembly", occurrence.assembly);
			members.set(occurrence.id, element);
		}
		return members;
	}

	static function writeJoints(document:Document, root:Element, scope:String, joints:Array<KinematicJoint>,
			couplings:Array<AssemblyJointCoupling>, members:Map<String, Element>):Void {
		var children = new Map<String, Element>();
		for (joint in joints) {
			var relationship = document.createRelationship(JOINT, reference(document, members.get(joint.parent)),
				reference(document, members.get(joint.child)));
			putRelationship(relationship, "owner", root.id.value);
			putRelationship(relationship, "scope", scope); putRelationship(relationship, "id", joint.id);
			putRelationship(relationship, "type", joint.type); putRelationship(relationship, "role", joint.role);
			putRelationship(relationship, "parentConnector", joint.parentConnector); putRelationship(relationship, "childConnector", joint.childConnector);
			quantity(relationship, "axisX", joint.axis.x);
			quantity(relationship, "axisY", joint.axis.y);
			quantity(relationship, "axisZ", joint.axis.z);
			if (joint.limits.lower != null) quantity(relationship, "limitLower", joint.limits.lower);
			if (joint.limits.upper != null) quantity(relationship, "limitUpper", joint.limits.upper);
			if (joint.limits.velocity != null) quantity(relationship, "limitVelocity", joint.limits.velocity);
			if (joint.limits.effort != null) quantity(relationship, "limitEffort", joint.limits.effort);
			relationship.setProperty(TypedProperty.quantity(PREFIX + "defaultValue", QuantityKind.Scalar, joint.defaultValue, "1"));
			if (joint.closureTolerance != null)
				relationship.setProperty(TypedProperty.quantity(PREFIX + "closureTolerance",
					QuantityKind.Scalar, joint.closureTolerance, "1"));
			children.set(joint.id, members.get(joint.child));
		}
		if (couplings != null) for (coupling in couplings) {
			var relationship = document.createRelationship(COUPLING, reference(document, children.get(coupling.source)),
				reference(document, children.get(coupling.target)));
			putRelationship(relationship, "owner", root.id.value);
			putRelationship(relationship, "scope", scope); putRelationship(relationship, "id", coupling.id);
			relationship.setProperty(TypedProperty.quantity(PREFIX + "ratio", QuantityKind.Scalar, coupling.ratio, "1"));
			relationship.setProperty(TypedProperty.quantity(PREFIX + "offset", QuantityKind.Scalar, coupling.offset, "1"));
		}
	}

	static function reference(document:Document, element:Element):ElementReference {
		if (element == null) throw "Assembly relationship has a missing occurrence";
		return new ElementReference(document.id, element.id);
	}

	static function drivingJoint(joints:Array<KinematicJoint>, child:String):String {
		for (joint in joints) if (joint.child == child && joint.role == materia.assembly.AssemblyDefinition.AssemblyJointRole.Tree &&
			AssemblyDefinitionCodec.hasCoordinate(joint.type)) return joint.id;
		throw 'Coupling endpoint "$child" has no driving joint';
	}

	static function decodeExposed(value:String):Array<AssemblyExposedConnector>
		return JsonWire.decode(value);

	static function decodeFrame(value:String):AssemblyFrame
		return JsonWire.decode(value);

	static function quantity(relationship:Relationship, name:String, value:Float):Void
		relationship.setProperty(TypedProperty.quantity(PREFIX + name, QuantityKind.Scalar, value, "1"));

	static function optionalNumber(relationship:Relationship, name:String):Null<Float> {
		var property = relationship.property(PREFIX + name);
		return property == null ? null : cast property.value;
	}

	static function readAxis(relationship:Relationship):AssemblyVector {
		if (relationship.property(PREFIX + "axisX") == null)
			return JsonWire.decode(readRelationship(relationship, "axis"));
		return {x: number(relationship, "axisX"), y: number(relationship, "axisY"),
			z: number(relationship, "axisZ")};
	}

	static function readLimits(relationship:Relationship):AssemblyJointLimits {
		if (relationship.property(PREFIX + "axisX") == null)
			return JsonWire.decode(readRelationship(relationship, "limits"));
		return {lower: optionalNumber(relationship, "limitLower"),
			upper: optionalNumber(relationship, "limitUpper"),
			velocity: optionalNumber(relationship, "limitVelocity"),
			effort: optionalNumber(relationship, "limitEffort")};
	}

	static function sortScope(scope:AssemblyDocumentScope):Void {
		scope.definitions.sort((a, b) -> Reflect.compare(a.id, b.id));
		scope.occurrences.sort((a, b) -> Reflect.compare(a.id, b.id));
		scope.joints.sort((a, b) -> Reflect.compare(a.id, b.id));
		scope.couplings.sort((a, b) -> Reflect.compare(a.id, b.id));
	}

	static function put(element:Element, name:String, value:String):Void
		element.setProperty(TypedProperty.text(PREFIX + name, value));

	static function putOwner(element:Element, root:Element):Void
		element.setProperty(TypedProperty.elementReference(PREFIX + "owner", new ElementReference(root.document.id, root.id)));

	static function belongsTo(element:Element, root:Element):Bool {
		var property = element.property(PREFIX + "owner");
		if (property == null || property.type != TypedProperty.TypeElementReference) return false;
		var reference:PersistentReference = cast property.value;
		return reference.documentId == root.document.id.value && reference.targetId == root.id.value;
	}

	static function putRelationship(relationship:Relationship, name:String, value:String):Void
		relationship.setProperty(TypedProperty.text(PREFIX + name, value));

	static function read(element:Element, name:String):String {
		var property = element.property(PREFIX + name);
		if (property == null) throw 'Missing assembly property "$name"';
		return cast property.value;
	}

	static function readRelationship(relationship:Relationship, name:String):String {
		var property = relationship.property(PREFIX + name);
		if (property == null) throw 'Missing assembly relationship property "$name"';
		return cast property.value;
	}

	static function readRelationshipOrNull(relationship:Relationship, name:String):Null<String> {
		var property = relationship.property(PREFIX + name);
		return property == null ? null : cast property.value;
	}

	static function readOrNull(element:Element, name:String):Null<String> {
		var property = element.property(PREFIX + name);
		return property == null ? null : cast property.value;
	}

	static function bool(element:Element, name:String):Bool {
		var property = element.property(PREFIX + name);
		return property != null && property.value == true;
	}

	static function integer(element:Element, name:String):Int {
		var property = element.property(PREFIX + name);
		if (property == null) throw 'Missing assembly integer property "$name"';
		return cast property.value;
	}

	static function number(relationship:Relationship, name:String):Float {
		var property = relationship.property(PREFIX + name);
		if (property == null) throw 'Missing assembly numeric property "$name"';
		return cast property.value;
	}
}
