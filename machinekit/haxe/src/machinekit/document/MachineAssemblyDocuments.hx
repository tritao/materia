package machinekit.document;

import cadkit.parametric.AssemblyDocuments;
import cadkit.parametric.Document;
import cadkit.parametric.Element;
import cadkit.parametric.ElementReference;
import cadkit.parametric.PersistentReference;
import cadkit.parametric.TypedProperty;
import haxeon.wire.JsonWire;
import machinekit.assembly.MachineAssembly;
import machinekit.assembly.MachineAssemblyCodec;
import machinekit.assembly.MachineAssemblyDescription;
import machinekit.component.ComponentRegistry;
import machinekit.component.MachineKitComponents;
import machinekit.robotics.EndEffector;
import machinekit.robotics.EndEffectorDescription;
import machinekit.robotics.EndEffectorSet;
import cadkit.parametric.Definition;
import cadkit.parametric.InstanceElement;
import materia.assembly.AssemblyDefinitionFlattener;

/** What kind of MachineKit assembly a document holds, with the data its kind adds. */
@:wire enum MachineDocumentKind {
	@:id(1) Assembly;
	@:id(2) Effector(endEffector:EndEffectorRecord);
	@:id(3) EffectorSet(endEffector:EndEffectorRecord, changer:ChangerRecord, tools:Array<ChangerToolRecord>);
}

/**
 * The MachineKit side of an assembly document. Members and port connections are left out of the
 * stored copy: the document's instances and relationships hold them, so editing those edits the assembly.
 */
@:wire typedef MachineDocumentRecord = {
	@:id(1) var assembly:MachineAssemblyDescription;
	@:id(2) var kind:MachineDocumentKind;
}

/** One level of a description: its path from the root and its facts. */
private typedef DocumentLevel = {var path:String; var machine:MachineLevelRecord;}

/** Stores mechanical data through CadKit and MachineKit facts beside it. */
class MachineAssemblyDocuments {
	public static inline var PORT_CONNECTION:String = "machinekit.port-connection";
	static inline var PREFIX:String = "machinekit.assembly.";

	public static function defineAssembly(document:Document, assembly:MachineAssembly):Element {
		var ownTransaction = !document.hasActiveTransaction();
		var transaction = ownTransaction ? document.beginTransaction() : null;
		try {
			var root = writeAssembly(document, assembly);
			if (transaction != null) transaction.commit();
			return root;
		} catch (error:Dynamic) {
			if (transaction != null) transaction.cancel();
			throw error;
		}
	}

	public static function describeDocument(assembly:MachineAssembly):MachineDocumentRecord {
		if (Std.isOfType(assembly, EndEffectorSet)) {
			var set:EndEffectorSet = cast assembly;
			var description = set.describeSet();
			return {assembly: description.base.assembly,
				kind: MachineDocumentKind.EffectorSet(description.base.endEffector, description.changer, description.tools)};
		}
		if (Std.isOfType(assembly, EndEffector)) {
			var effector:EndEffector = cast assembly;
			var description = effector.describeEndEffector();
			return {assembly: description.assembly, kind: MachineDocumentKind.Effector(description.endEffector)};
		}
		return {assembly: assembly.describe(), kind: MachineDocumentKind.Assembly};
	}

	static function writeAssembly(document:Document, assembly:MachineAssembly):Element {
		// The generated codec checks all members, including tools, before creating document objects.
		var record:MachineDocumentRecord = JsonWire.decode(JsonWire.encode(describeDocument(assembly)));
		var description = record.assembly;
		MachineAssemblyCodec.encode(description);
		var tools = toolsOf(record.kind);
		for (tool in tools) MachineAssemblyCodec.encode(tool.tool.assembly);
		rebuild(record);
		for (existing in document.allElements())
			if (propertyText(existing, "cadkit.assembly.kind") == "root" &&
				propertyText(existing, "cadkit.assembly.id") == description.mechanical.id)
				removeToolInstances(document, existing);
		var members = membersByPath(description);
		var recipes:Map<String, Definition> = [];
		var root = AssemblyDocuments.fromDefinition(document, description.mechanical,
			(scope, occurrence, component) -> {
				if (component == null) return null;
				var member = members.get(scope + occurrence.id);
				if (member == null) return null;
				return recipeDefinition(document, member, recipes);
			});
		var stored:MachineDocumentRecord = {assembly: stripped(description),
			kind: switch record.kind {
				case EffectorSet(effector, changer, tools):
					MachineDocumentKind.EffectorSet(effector, changer, [for (tool in tools)
						{id: tool.id, tool: {assembly: stripped(tool.tool.assembly), endEffector: tool.tool.endEffector}}]);
				case kind: kind;
			}};
		root.setProperty(TypedProperty.text(PREFIX + "schemaVersion", Std.string(MachineAssembly.SCHEMA_VERSION)));
		root.setProperty(TypedProperty.text(PREFIX + "side", JsonWire.encode(stored)));
		var endpoints = new Map<String, Element>();
		for (element in document.allElements()) if (element.kind == "instance" && belongsToCadKit(element, root)) {
			var path = propertyText(element, "cadkit.assembly.path");
			if (path != null && members.exists(path)) {
				element.setProperty(TypedProperty.text(PREFIX + "occurrence", path));
				element.setProperty(TypedProperty.text("machinekit.occurrence", path));
				endpoints.set(path, element);
			}
		}
		for (level in levels(description)) for (connection in level.machine.portConnections)
			writeConnection(document, connection, level.path, null, endpoints);
		for (tool in tools) {
			var toolEndpoints:Map<String, Element> = [];
			var flatTool = AssemblyDefinitionFlattener.flatten(tool.tool.assembly.mechanical);
			var toolMembers = membersByPath(tool.tool.assembly);
			for (path in toolMembers.keys()) {
				var member = toolMembers.get(path);
				var definition = recipeDefinition(document, member, recipes);
				if (definition == null) throw 'Tool member "${tool.id}/$path" has no recipe';
				var instance = document.createInstance(tool.id + "/" + path, definition);
				for (occurrence in flatTool.occurrences) if (occurrence.id == path)
					instance.setPlacement(cadkit.parametric.PlacementFrames.fromAssemblyFrame(occurrence.initialPose));
				instance.setProperty(TypedProperty.elementReference(PREFIX + "owner",
					new ElementReference(document.id, root.id)));
				instance.setProperty(TypedProperty.text(PREFIX + "tool", tool.id));
				instance.setProperty(TypedProperty.text(PREFIX + "occurrence", path));
				instance.setProperty(TypedProperty.text("machinekit.occurrence", tool.id + "/" + path));
				toolEndpoints.set(path, instance);
			}
			for (level in levels(tool.tool.assembly)) for (connection in level.machine.portConnections)
				writeConnection(document, connection, level.path, tool.id, toolEndpoints);
		}
		return root;
	}

	static function writeConnection(document:Document, connection:PortConnectionRecord, level:String,
			tool:Null<String>, endpoints:Map<String, Element>):Void {
		var source = endpoints.get(join(level, connection.fromInstance));
		var target = endpoints.get(join(level, connection.toInstance));
		var scope = tool == null ? "" : '$tool/';
		if (source == null || target == null)
			throw 'Port connection "$scope${join(level, connection.id)}" references a missing member';
		var relationship = document.createRelationship(PORT_CONNECTION,
			new ElementReference(document.id, source.id), new ElementReference(document.id, target.id));
		if (tool != null) put(relationship, "tool", tool);
		if (level.length > 0) put(relationship, "level", level);
		put(relationship, "id", connection.id);
		put(relationship, "fromPort", connection.fromPort);
		put(relationship, "toPort", connection.toPort);
	}

	static function toolsOf(kind:MachineDocumentKind):Array<ChangerToolRecord> return switch kind {
		case EffectorSet(_, _, tools): tools;
		case _: [];
	};

	/** A copy with no members or port connections; the document holds those. */
	static function stripped(description:MachineAssemblyDescription):MachineAssemblyDescription {
		var copy:MachineAssemblyDescription = JsonWire.decode(JsonWire.encode(description));
		for (level in levels(copy)) {
			level.machine.members = [];
			level.machine.portConnections = [];
		}
		return copy;
	}

	static function levels(description:MachineAssemblyDescription):Array<DocumentLevel> {
		var result:Array<DocumentLevel> = [{path: "", machine: description.machine}];
		for (record in description.subassemblies) result.push({path: record.path, machine: record.machine});
		return result;
	}

	static function membersByPath(description:MachineAssemblyDescription):Map<String, MemberRecord> {
		var result = new Map<String, MemberRecord>();
		for (level in levels(description)) for (member in level.machine.members)
			result.set(join(level.path, member.occurrence), member);
		return result;
	}

	/** Each member path of the nested definition, with the level that owns it and its local name. */
	static function memberLevels(description:MachineAssemblyDescription):Map<String, {level:String, local:String}> {
		var result = new Map<String, {level:String, local:String}>();
		var table = new Map<String, materia.assembly.AssemblyDefinition.AssemblySubdefinition>();
		var mechanical = description.mechanical;
		if (mechanical.assemblies != null) for (entry in mechanical.assemblies) table.set(entry.id, entry);
		walkMembers(mechanical.occurrences, "", table, result);
		return result;
	}

	static function walkMembers(occurrences:Array<materia.assembly.AssemblyDefinition.AssemblyComponentOccurrence>, level:String,
			table:Map<String, materia.assembly.AssemblyDefinition.AssemblySubdefinition>,
			result:Map<String, {level:String, local:String}>):Void
		for (occurrence in occurrences) {
			var path = join(level, occurrence.id);
			if (occurrence.assembly == null) result.set(path, {level: level, local: occurrence.id});
			else {
				var entry = table.get(occurrence.assembly);
				if (entry == null) throw 'Unknown nested assembly "${occurrence.assembly}"';
				walkMembers(entry.occurrences, path, table, result);
			}
		}

	static function join(level:String, id:String):String return level.length == 0 ? id : level + "/" + id;

	static function removeToolInstances(document:Document, root:Element):Void {
		var owned = [for (element in document.allElements())
			if (belongsToMachineKit(element, root)) element];
		var ids:Map<String, Bool> = [];
		for (element in owned) ids.set(element.id.value, true);
		for (relationship in document.allRelationships()) if (
			ids.exists(relationship.source.elementId.value) ||
			ids.exists(relationship.target.elementId.value)) document.removeRelationship(relationship.id);
		for (element in owned) document.removeElement(element.id);
	}

	static function recipeDefinition(document:Document, member:MemberRecord, cache:Map<String, Definition>):Null<Definition> {
		var typeId:String;
		var values = new machinekit.component.ComponentValues();
		switch member.source {
			case Code(_): return null;
			case Typed(id, saved):
				typeId = id;
				for (entry in saved) values.set(entry.name, MachineAssemblyCodec.componentValue(entry.value));
		}
		values.setToken("material", member.material);
		var type = MachineKitComponents.defaultRegistry().byId(typeId);
		var key = type.key(values);
		var definition = cache.get(key);
		if (definition == null) {
			definition = MachineKitDocuments.define(document, type, values);
			cache.set(key, definition);
		}
		return definition;
	}

	static function propertyText(element:Element, name:String):Null<String> {
		var property = element.property(name);
		return property == null ? null : cast property.value;
	}

	static function belongsToCadKit(element:Element, root:Element):Bool {
		var owner = element.property("cadkit.assembly.owner");
		if (owner == null || owner.type != TypedProperty.TypeElementReference) return false;
		var reference:PersistentReference = cast owner.value;
		return reference.documentId == root.document.id.value && reference.targetId == root.id.value;
	}

	static function belongsToMachineKit(element:Element, root:Element):Bool {
		var owner = element.property(PREFIX + "owner");
		if (owner == null || owner.type != TypedProperty.TypeElementReference) return false;
		var reference:PersistentReference = cast owner.value;
		return reference.documentId == root.document.id.value && reference.targetId == root.id.value;
	}

	static function memberRecord(element:InstanceElement, local:String):MemberRecord {
		var component = MachineKitRecipes.component(element);
		if (component.componentType() == null) throw 'Saved member "$local" has no recipe';
		return {occurrence: local, material: component.materialSpec(), source: MachineAssemblyCodec.memberSource(component)};
	}

	/** The document's assembly as data, with members and port connections read from the document. */
	public static function describeAssembly(root:Element):MachineDocumentRecord {
		var version = propertyText(root, PREFIX + "schemaVersion");
		if (version != Std.string(MachineAssembly.SCHEMA_VERSION))
			throw 'schema v$version is unsupported; expected v${MachineAssembly.SCHEMA_VERSION}';
		var property = root.property(PREFIX + "side");
		if (property == null) throw "Assembly document has no MachineKit side record";
		var savedText:String = cast property.value;
		var record:MachineDocumentRecord = JsonWire.decode(savedText);
		var description = record.assembly;
		description.mechanical = AssemblyDocuments.toDefinition(root);
		var owners = memberLevels(description);
		var levelByPath = new Map<String, MachineLevelRecord>();
		for (level in levels(description)) levelByPath.set(level.path, level.machine);
		var tools = new Map<String, MachineAssemblyDescription>();
		for (tool in toolsOf(record.kind)) tools.set(tool.id, tool.tool.assembly);
		var toolOwners = new Map<String, Map<String, {level:String, local:String}>>();
		var toolLevels = new Map<String, Map<String, MachineLevelRecord>>();
		for (id in tools.keys()) {
			var tool = tools.get(id);
			toolOwners.set(id, memberLevels(tool));
			var byPath = new Map<String, MachineLevelRecord>();
			for (level in levels(tool)) byPath.set(level.path, level.machine);
			toolLevels.set(id, byPath);
		}
		var endpoints = new Map<String, String>();
		var toolEndpoints:Map<String, {tool:String, occurrence:String}> = [];
		for (element in root.document.allElements()) {
			var path = propertyText(element, PREFIX + "occurrence");
			if (path == null) continue;
			var toolId = propertyText(element, PREFIX + "tool");
			if (toolId != null && belongsToMachineKit(element, root)) {
				var ownersOfTool = toolOwners.get(toolId), levelsOfTool = toolLevels.get(toolId);
				var owner = ownersOfTool == null ? null : ownersOfTool.get(path);
				if (owner == null || levelsOfTool == null) throw 'Tool instance "$toolId/$path" is not a tool member';
				toolEndpoints.set(element.id.value, {tool: toolId, occurrence: path});
				requireLevel(levelsOfTool, owner.level).members.push(memberRecord(cast element, owner.local));
				continue;
			}
			if (!belongsToCadKit(element, root)) continue;
			var owner = owners.get(path);
			if (owner == null) throw 'Instance "$path" is not an assembly member';
			endpoints.set(element.id.value, path);
			requireLevel(levelByPath, owner.level).members.push(memberRecord(cast element, owner.local));
		}
		for (relationship in root.document.allRelationships()) if (relationship.typeName == PORT_CONNECTION) {
			var level = propertyTextRelationship(relationship, "level");
			if (level == null) level = "";
			var toolId = propertyTextRelationship(relationship, "tool");
			var from:Null<String>, to:Null<String>, target:Null<MachineLevelRecord>;
			if (toolId != null) {
				var fromTool = toolEndpoints.get(relationship.source.elementId.value);
				var toTool = toolEndpoints.get(relationship.target.elementId.value);
				if (fromTool == null || toTool == null) continue;
				if (fromTool.tool != toolId || toTool.tool != toolId)
					throw 'Tool port connection crosses tool scopes: "$toolId"';
				from = fromTool.occurrence;
				to = toTool.occurrence;
				var levelsOfTool = toolLevels.get(toolId);
				target = levelsOfTool == null ? null : levelsOfTool.get(level);
			} else {
				from = endpoints.get(relationship.source.elementId.value);
				to = endpoints.get(relationship.target.elementId.value);
				if (from == null && to == null) continue;
				if (from == null || to == null) throw "Port connection crosses assembly documents";
				target = levelByPath.get(level);
			}
			if (target == null) throw 'Port connection names unknown level "$level"';
			var prefix = level.length == 0 ? "" : level + "/";
			if (!StringTools.startsWith(from, prefix) || !StringTools.startsWith(to, prefix))
				throw 'Port connection leaves its level "$level"';
			target.portConnections.push({id: read(relationship, "id"), fromInstance: from.substr(prefix.length),
				fromPort: read(relationship, "fromPort"), toInstance: to.substr(prefix.length),
				toPort: read(relationship, "toPort")});
		}
		for (tool in tools) sortLevels(tool);
		sortLevels(description);
		return record;
	}

	static function requireLevel(levels:Map<String, MachineLevelRecord>, path:String):MachineLevelRecord {
		var level = levels.get(path);
		if (level == null) throw 'Unknown assembly level "$path"';
		return level;
	}

	static function sortLevels(description:MachineAssemblyDescription):Void
		for (level in levels(description)) {
			level.machine.members.sort((a, b) -> Reflect.compare(a.occurrence, b.occurrence));
			level.machine.portConnections.sort((a, b) -> Reflect.compare(a.id, b.id));
		}

	public static function rebuildAssembly(root:Element, ?registry:ComponentRegistry):MachineAssembly
		return rebuild(describeAssembly(root), registry);

	static function rebuild(record:MachineDocumentRecord, ?registry:ComponentRegistry):MachineAssembly
		return switch record.kind {
			case Assembly: MachineAssembly.fromDescription(record.assembly, registry);
			case Effector(effector):
				machinekit.robotics.EndEffector.fromDescription({assembly: record.assembly, endEffector: effector}, registry);
			case EffectorSet(effector, changer, tools):
				machinekit.robotics.EndEffectorSet.fromDescription({base: {assembly: record.assembly, endEffector: effector},
					changer: changer, tools: tools}, registry);
		};

	static function put(relationship:cadkit.parametric.Relationship, name:String, value:String):Void
		relationship.setProperty(TypedProperty.text(PREFIX + name, value));

	static function read(relationship:cadkit.parametric.Relationship, name:String):String {
		var property = relationship.property(PREFIX + name);
		if (property == null) throw 'Port connection is missing "$name"';
		return cast property.value;
	}

	static function propertyTextRelationship(relationship:cadkit.parametric.Relationship,
			name:String):Null<String> {
		var property = relationship.property(PREFIX + name);
		return property == null ? null : cast property.value;
	}
}
