package machinekit.document;

import cadkit.parametric.AssemblyDocuments;
import cadkit.parametric.Document;
import cadkit.parametric.Element;
import cadkit.parametric.ElementReference;
import cadkit.parametric.PersistentReference;
import cadkit.parametric.TypedProperty;
import haxeon.wire.JsonWire;
import machinekit.assembly.MachineAssembly;
import machinekit.assembly.MachineAssemblyDescription;
import machinekit.assembly.MachineAssemblyDescription.AssemblySideRecord;
import machinekit.assembly.MachineAssemblyDescription.PortConnectionRecord;
import machinekit.assembly.MachineAssemblyDescription.MemberSource;
import machinekit.assembly.MachineAssemblyDescription.SavedValue;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentValue;
import machinekit.component.MachineKitComponents;
import machinekit.robotics.EndEffector;
import machinekit.robotics.EndEffectorSet;
import cadkit.parametric.Definition;
import cadkit.parametric.InstanceElement;
import materia.assembly.AssemblyDefinitionFlattener;

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

	static function writeAssembly(document:Document, assembly:MachineAssembly):Element {
		// The generated codec checks all members, including tools, before creating document objects.
		var description:MachineAssemblyDescription = JsonWire.decode(assembly.encode());
		MachineAssembly.fromDescription(description);
		for (existing in document.allElements())
			if (propertyText(existing, "cadkit.assembly.kind") == "root" &&
				propertyText(existing, "cadkit.assembly.id") == description.mechanical.id)
				removeToolInstances(document, existing);
		var recipes:Map<String, Definition> = [];
		var root = AssemblyDocuments.fromDefinition(document, machinekit.assembly.FrozenAssemblyDefinitions.thaw(description.mechanical),
			(scope, occurrence, component) -> {
				if (component == null) return null;
				var member = sourceFor(description.machine.members, scope, occurrence.id);
				if (member == null) return null;
				return recipeDefinition(document, member, recipes);
			});
		var side = description.machine;
		var noConnections:Array<PortConnectionRecord> = [];
		var noMembers:Array<machinekit.assembly.MachineAssemblyDescription.MemberRecord> = [];
		var saved:AssemblySideRecord = {
			members: noMembers, ports: side.ports, included: side.included,
			portConnections: noConnections, portExposures: side.portExposures,
			bomExtras: side.bomExtras, connectorExposures: side.connectorExposures,
			memberConnectors: side.memberConnectors, endEffector: side.endEffector,
			changer: side.changer, tools: side.tools, transmissions: side.transmissions, motors: side.motors, encoders: side.encoders
		};
		var toolRecords:Array<machinekit.assembly.MachineAssemblyDescription.ToolRecord> = [];
		if (side.tools != null) for (tool in side.tools) {
			var emptyToolConnections:Array<PortConnectionRecord> = [];
			toolRecords.push({id: tool.id, mechanical: tool.mechanical,
				machine: {members: tool.machine.members, ports: tool.machine.ports,
					included: tool.machine.included, portConnections: emptyToolConnections,
					portExposures: tool.machine.portExposures, bomExtras: tool.machine.bomExtras,
					connectorExposures: tool.machine.connectorExposures,
					memberConnectors: tool.machine.memberConnectors,
					endEffector: tool.machine.endEffector, transmissions: tool.machine.transmissions,
					motors: tool.machine.motors, encoders: tool.machine.encoders}});
		}
		saved.tools = toolRecords;
		root.setProperty(TypedProperty.text(PREFIX + "schemaVersion", Std.string(MachineAssembly.SCHEMA_VERSION)));
		root.setProperty(TypedProperty.text(PREFIX + "side", JsonWire.encode(saved)));
		var endpoints = new Map<String, Element>();
		for (element in document.allElements()) if (element.kind == "instance" && belongsToCadKit(element, root)) {
			var path = propertyText(element, "cadkit.assembly.path");
			var member = sourceFor(side.members, "", path);
			if (member != null) {
				element.setProperty(TypedProperty.text(PREFIX + "occurrence", member.occurrence));
				element.setProperty(TypedProperty.text("machinekit.occurrence", member.occurrence));
				endpoints.set(member.occurrence, element);
			}
		}
		for (connection in side.portConnections) {
			var source = endpoints.get(connection.fromInstance);
			var target = endpoints.get(connection.toInstance);
			if (source == null || target == null)
				throw 'Port connection "${connection.id}" references a missing member';
			var relationship = document.createRelationship(PORT_CONNECTION,
				new ElementReference(document.id, source.id), new ElementReference(document.id, target.id));
			put(relationship, "id", connection.id);
			put(relationship, "fromPort", connection.fromPort);
			put(relationship, "toPort", connection.toPort);
		}
		if (side.tools != null) for (tool in side.tools) {
			var toolEndpoints:Map<String, Element> = [];
			var flatTool = AssemblyDefinitionFlattener.flatten(machinekit.assembly.FrozenAssemblyDefinitions.thaw(tool.mechanical));
			for (member in tool.machine.members) {
				var definition = recipeDefinition(document, member, recipes);
				if (definition == null) throw 'Tool member "${tool.id}/${member.occurrence}" has no recipe';
				var instance = document.createInstance(tool.id + "/" + member.occurrence, definition);
				for (occurrence in flatTool.occurrences) if (occurrence.id == member.occurrence)
					instance.setPlacement(cadkit.parametric.PlacementFrames.fromAssemblyFrame(occurrence.initialPose));
				instance.setProperty(TypedProperty.elementReference(PREFIX + "owner",
					new ElementReference(document.id, root.id)));
				instance.setProperty(TypedProperty.text(PREFIX + "tool", tool.id));
				instance.setProperty(TypedProperty.text(PREFIX + "occurrence", member.occurrence));
				instance.setProperty(TypedProperty.text("machinekit.occurrence", tool.id + "/" + member.occurrence));
				toolEndpoints.set(member.occurrence, instance);
			}
			for (connection in tool.machine.portConnections) {
				var source = toolEndpoints.get(connection.fromInstance);
				var target = toolEndpoints.get(connection.toInstance);
				if (source == null || target == null)
					throw 'Tool port connection "${tool.id}/${connection.id}" references a missing member';
				var relationship = document.createRelationship(PORT_CONNECTION,
					new ElementReference(document.id, source.id), new ElementReference(document.id, target.id));
				put(relationship, "tool", tool.id);
				put(relationship, "id", connection.id);
				put(relationship, "fromPort", connection.fromPort);
				put(relationship, "toPort", connection.toPort);
			}
		}
		return root;
	}

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

	static function sourceFor(members:haxe.ds.ReadOnlyArray<machinekit.assembly.MachineAssemblyDescription.MemberRecord>,
			scope:String, local:String):Null<machinekit.assembly.MachineAssemblyDescription.MemberRecord> {
		var path = scope + local;
		for (member in members) if (member.occurrence == path) return member;
		return null;
	}

	static function recipeDefinition(document:Document,
			member:machinekit.assembly.MachineAssemblyDescription.MemberRecord,
			cache:Map<String, Definition>):Null<Definition> {
		var typeId:String;
		var values = new ComponentValues();
		switch member.source {
			case Code(_): return null;
			case Typed(id, saved):
				typeId = id;
				for (entry in saved) values.set(entry.name, switch entry.value {
					case SavedValue.Number(value): ComponentValue.Number(value);
					case SavedValue.Integer(value): ComponentValue.Integer(value);
					case SavedValue.Boolean(value): ComponentValue.Boolean(value);
					case SavedValue.Token(value): ComponentValue.Token(value);
					case SavedValue.Unset: ComponentValue.Unset;
				});
		}
		values.setToken("material", member.material);
		var type = MachineKitComponents.byId(typeId);
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

	public static function describeAssembly(root:Element):MachineAssemblyDescription {
		var version = propertyText(root, PREFIX + "schemaVersion");
		if (version != Std.string(MachineAssembly.SCHEMA_VERSION))
			throw 'schema v$version is unsupported; expected v${MachineAssembly.SCHEMA_VERSION}';
		var property = root.property(PREFIX + "side");
		if (property == null) throw "Assembly document has no MachineKit side record";
		var savedText:String = cast property.value;
		var side:AssemblySideRecord = JsonWire.decode(savedText);
		var endpoints = new Map<String, String>();
		var toolEndpoints:Map<String, {tool:String, occurrence:String}> = [];
		var toolElements:Map<String, Array<InstanceElement>> = [];
		var rebuiltMembers:Array<machinekit.assembly.MachineAssemblyDescription.MemberRecord> = [];
		var rebuiltPorts:Array<machinekit.assembly.MachineAssemblyDescription.PortRecord> = [];
		for (element in root.document.allElements()) {
			var path = element.property(PREFIX + "occurrence");
			if (path == null) continue;
			var toolId = propertyText(element, PREFIX + "tool");
			if (toolId != null && belongsToMachineKit(element, root)) {
				toolEndpoints.set(element.id.value, {tool: toolId, occurrence: cast path.value});
				var instances = toolElements.get(toolId);
				if (instances == null) {instances = []; toolElements.set(toolId, instances);}
				instances.push(cast element);
				continue;
			}
			if (!belongsToCadKit(element, root)) continue;
			var occurrence:String = cast path.value;
			endpoints.set(element.id.value, occurrence);
			var component = MachineKitRecipes.component(cast element);
			var recipe = component.componentType();
			if (recipe == null) throw 'Saved member "$occurrence" has no recipe';
			var values = component.values(), names = values.names();
			names.sort(Reflect.compare);
			rebuiltMembers.push({occurrence: occurrence, material: component.materialSpec(),
				source: MemberSource.Typed(recipe.id, [for (name in names) {name: name,
					value: switch values.get(name) {
						case ComponentValue.Number(value): SavedValue.Number(value);
						case ComponentValue.Integer(value): SavedValue.Integer(value);
						case ComponentValue.Boolean(value): SavedValue.Boolean(value);
						case ComponentValue.Token(value): SavedValue.Token(value);
						case ComponentValue.Unset: SavedValue.Unset;
						case null: throw 'Missing value "$name"';
					}}])});
			for (port in component.ports()) rebuiltPorts.push({occurrence: occurrence,
				name: port.name, kind: port.kind, role: port.role, iface: port.iface,
				required: port.required, connector: port.connector});
		}
		rebuiltMembers.sort((a, b) -> Reflect.compare(a.occurrence, b.occurrence));
		rebuiltPorts.sort((a, b) -> Reflect.compare(a.occurrence + "/" + a.name, b.occurrence + "/" + b.name));
		side.members = rebuiltMembers;
		side.ports = rebuiltPorts;
		var connections:Array<PortConnectionRecord> = [];
		var toolConnections:Map<String, Array<PortConnectionRecord>> = [];
		for (relationship in root.document.allRelationships()) if (relationship.typeName == PORT_CONNECTION) {
			var toolId = propertyTextRelationship(relationship, "tool");
			if (toolId != null) {
				var fromTool = toolEndpoints.get(relationship.source.elementId.value);
				var toTool = toolEndpoints.get(relationship.target.elementId.value);
				if (fromTool == null || toTool == null) continue;
				if (fromTool.tool != toolId || toTool.tool != toolId)
					throw 'Tool port connection crosses tool scopes: "$toolId"';
				var rows = toolConnections.get(toolId);
				if (rows == null) {rows = []; toolConnections.set(toolId, rows);}
				rows.push({id: read(relationship, "id"), fromInstance: fromTool.occurrence,
					fromPort: read(relationship, "fromPort"), toInstance: toTool.occurrence,
					toPort: read(relationship, "toPort")});
				continue;
			}
			var from = endpoints.get(relationship.source.elementId.value);
			var to = endpoints.get(relationship.target.elementId.value);
			if (from == null && to == null) continue;
			if (from == null || to == null) throw "Port connection crosses assembly documents";
			connections.push({id: read(relationship, "id"), fromInstance: from,
				fromPort: read(relationship, "fromPort"), toInstance: to,
				toPort: read(relationship, "toPort")});
		}
		connections.sort((a, b) -> Reflect.compare(a.id, b.id));
		side.portConnections = connections;
		if (side.tools != null) for (tool in side.tools) {
			var rebuiltToolMembers:Array<machinekit.assembly.MachineAssemblyDescription.MemberRecord> = [];
			var rebuiltToolPorts:Array<machinekit.assembly.MachineAssemblyDescription.PortRecord> = [];
			var instances = toolElements.get(tool.id);
			if (instances != null) for (instance in instances) {
				var occurrence = propertyText(instance, PREFIX + "occurrence");
				if (occurrence == null) throw "Tool instance is missing its occurrence id";
				var component = MachineKitRecipes.component(instance);
				var recipe = component.componentType();
				if (recipe == null) throw 'Saved tool member "$occurrence" has no recipe';
				var values = component.values(), names = values.names();
				names.sort(Reflect.compare);
				rebuiltToolMembers.push({occurrence: occurrence, material: component.materialSpec(),
					source: MemberSource.Typed(recipe.id, [for (name in names) {name: name,
						value: switch values.get(name) {
							case ComponentValue.Number(value): SavedValue.Number(value);
							case ComponentValue.Integer(value): SavedValue.Integer(value);
							case ComponentValue.Boolean(value): SavedValue.Boolean(value);
							case ComponentValue.Token(value): SavedValue.Token(value);
							case ComponentValue.Unset: SavedValue.Unset;
							case null: throw 'Missing value "$name"';
						}}])});
				for (port in component.ports()) rebuiltToolPorts.push({occurrence: occurrence,
					name: port.name, kind: port.kind, role: port.role, iface: port.iface,
					required: port.required, connector: port.connector});
			}
			rebuiltToolMembers.sort((a, b) -> Reflect.compare(a.occurrence, b.occurrence));
			rebuiltToolPorts.sort((a, b) -> Reflect.compare(a.occurrence + "/" + a.name, b.occurrence + "/" + b.name));
			tool.machine.members = rebuiltToolMembers;
			tool.machine.ports = rebuiltToolPorts;
			var rows = toolConnections.get(tool.id);
			if (rows == null) rows = [];
			rows.sort((a, b) -> Reflect.compare(a.id, b.id));
			tool.machine.portConnections = rows;
		}
		return {schemaVersion: MachineAssembly.SCHEMA_VERSION, mechanical: machinekit.assembly.FrozenAssemblyDefinitions.freeze(AssemblyDocuments.toDefinition(root)), machine: side};
	}

	public static function rebuildAssembly(root:Element):MachineAssembly {
		var description = describeAssembly(root);
		if (description.machine.changer != null) return EndEffectorSet.fromDescription(description);
		if (description.machine.endEffector != null) return EndEffector.fromDescription(description);
		return MachineAssembly.fromDescription(description);
	}

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
