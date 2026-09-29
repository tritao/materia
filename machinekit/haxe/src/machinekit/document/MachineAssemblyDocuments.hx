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

/** Stores mechanical data through CadKit and MachineKit facts beside it. */
class MachineAssemblyDocuments {
	public static inline var PORT_CONNECTION:String = "machinekit.port-connection";
	static inline var PREFIX:String = "machinekit.assembly.";

	public static function defineAssembly(document:Document, assembly:MachineAssembly):Element {
		// The generated codec checks all members, including tools, before creating document objects.
		var description:MachineAssemblyDescription = JsonWire.decode(assembly.encode());
		MachineAssembly.fromDescription(description);
		var root = AssemblyDocuments.fromDefinition(document, description.mechanical);
		var side = description.machine;
		var noConnections:Array<PortConnectionRecord> = [];
		var saved:AssemblySideRecord = {
			members: side.members, ports: side.ports, included: side.included,
			portBridges: side.portBridges, portConversions: side.portConversions,
			portConnections: noConnections, portExposures: side.portExposures,
			bomExtras: side.bomExtras, connectorExposures: side.connectorExposures,
			memberConnectors: side.memberConnectors, endEffector: side.endEffector,
			changer: side.changer, tools: side.tools
		};
		root.setProperty(TypedProperty.text(PREFIX + "side", JsonWire.encode(saved)));
		var endpoints = new Map<String, Element>();
		for (member in side.members) {
			var endpoint = document.createObject(member.occurrence);
			endpoint.setProperty(TypedProperty.elementReference(PREFIX + "owner",
				new ElementReference(document.id, root.id)));
			endpoint.setProperty(TypedProperty.text(PREFIX + "occurrence", member.occurrence));
			endpoints.set(member.occurrence, endpoint);
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
		return root;
	}

	public static function describeAssembly(root:Element):MachineAssemblyDescription {
		var property = root.property(PREFIX + "side");
		if (property == null) throw "Assembly document has no MachineKit side record";
		var savedText:String = cast property.value;
		var side:AssemblySideRecord = JsonWire.decode(savedText);
		var endpoints = new Map<String, String>();
		for (element in root.document.allElements()) {
			var owner = element.property(PREFIX + "owner");
			if (owner == null || owner.type != TypedProperty.TypeElementReference) continue;
			var reference:PersistentReference = cast owner.value;
			if (reference.documentId != root.document.id.value || reference.targetId != root.id.value) continue;
			var path = element.property(PREFIX + "occurrence");
			if (path != null) endpoints.set(element.id.value, cast path.value);
		}
		var connections:Array<PortConnectionRecord> = [];
		for (relationship in root.document.allRelationships()) if (relationship.typeName == PORT_CONNECTION) {
			var from = endpoints.get(relationship.source.elementId.value);
			var to = endpoints.get(relationship.target.elementId.value);
			if (from == null && to == null) continue;
			if (from == null || to == null) throw "Port connection crosses assembly documents";
			connections.push({id: read(relationship, "id"), fromInstance: from,
				fromPort: read(relationship, "fromPort"), toInstance: to,
				toPort: read(relationship, "toPort")});
		}
		side.portConnections = connections;
		return {mechanical: AssemblyDocuments.toDefinition(root), machine: side};
	}

	public static function rebuildAssembly(root:Element):MachineAssembly
		return MachineAssembly.fromDescription(describeAssembly(root));

	static function put(relationship:cadkit.parametric.Relationship, name:String, value:String):Void
		relationship.setProperty(TypedProperty.text(PREFIX + name, value));

	static function read(relationship:cadkit.parametric.Relationship, name:String):String {
		var property = relationship.property(PREFIX + name);
		if (property == null) throw 'Port connection is missing "$name"';
		return cast property.value;
	}
}
