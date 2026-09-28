package machinekit.assembly;

import cadkit.modeling.AssemblyModel;
import machinekit.component.Bom;
import machinekit.component.BomItem;
import machinekit.component.MachineComponent;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyDefinition.AssemblyJointLimits;
import materia.assembly.AssemblyDefinition.AssemblyVector;
import materia.assembly.AssemblyRecord.AssemblyFrame;

typedef MachineAssemblyComponent = { var id:String; var component:MachineComponent; }
typedef MachineAssemblyConnector = { var instanceId:String; var connectorName:String; }

private typedef AssemblyMember = {
	var id:String;
	var component:MachineComponent;
	var pose:AssemblyFrame;
}

private typedef AssemblyOperation = {
	var operation:String;
	var id:String;
	var jointKind:String;
	var parent:String;
	var parentConnector:String;
	var child:String;
	var childConnector:String;
	var axis:Null<AssemblyVector>;
	var value:Float;
	var limits:Null<AssemblyJointLimits>;
	var tolerance:Null<Float>;
	var source:Null<String>;
	var target:Null<String>;
	var ratio:Float;
	var offset:Float;
}

/** Reusable, prefixable assembly made from MachineComponents and named connector references. */
class MachineAssembly {
	final members:Array<AssemblyMember> = [];
	final externalConnectors:Array<{name:String, instanceId:String, connectorName:String}> = [];
	final operations:Array<AssemblyOperation> = [];
	final bomItems:Array<{item:BomItem, quantity:Int}> = [];

	public function new() {}

	public function addComponent(id:String, component:MachineComponent, ?pose:AssemblyFrame):Void {
		if (id == null || id.length == 0 || component == null) throw "Assembly component needs an id and component";
		for (member in members) if (member.id == id) throw 'Duplicate assembly component "$id"';
		members.push({id: id, component: component,
			pose: pose == null ? AssemblyFrames.identity() : copyFrame(pose)});
	}

	/** Include another assembly below a local namespace, optionally moving all of its parts. */
	public function include(id:String, assembly:MachineAssembly, ?pose:AssemblyFrame):Void {
		if (id == null || id.length == 0 || assembly == null) throw "Included assembly needs an id and assembly";
		for (member in assembly.members) {
			var localPose = pose == null ? member.pose : AssemblyFrames.compose(pose, member.pose);
			addComponent(join(id, member.id), member.component, localPose);
		}
		for (connector in assembly.memberConnectorFrames)
			memberConnectorFrames.push({instanceId: join(id, connector.instanceId), name: connector.name,
				frame: copyFrame(connector.frame)});
		for (connector in assembly.externalConnectors)
			exposeConnector(join(id, connector.name), join(id, connector.instanceId), connector.connectorName);
		for (operation in assembly.operations) {
			var copied:AssemblyOperation = {
				operation: operation.operation, id: operation.id, jointKind: operation.jointKind,
				parent: operation.parent, parentConnector: operation.parentConnector,
				child: operation.child, childConnector: operation.childConnector,
				axis: operation.axis, value: operation.value, limits: operation.limits,
				tolerance: operation.tolerance, source: operation.source, target: operation.target,
				ratio: operation.ratio, offset: operation.offset
			};
			switch copied.operation {
				case "mate", "mateOnAxis", "constrain", "constrainOnAxis":
					copied.parent = join(id, operation.parent);
					copied.child = join(id, operation.child);
				case "couple":
					copied.source = join(id, operation.source);
					copied.target = join(id, operation.target);
			}
			copied.id = join(id, operation.id);
			operations.push(copied);
		}
		for (entry in assembly.bomItems) addBomItem(entry.item, entry.quantity);
	}

	public function addMate(id:String, kind:String, parent:String, parentConnector:String,
			child:String, childConnector:String, value:Float = 0):Void
		operations.push(operation("mate", id, kind, parent, parentConnector, child, childConnector, value));

	public function addMateOnAxis(id:String, kind:String, parent:String, parentConnector:String,
			child:String, childConnector:String, axis:AssemblyVector, value:Float = 0,
			?limits:AssemblyJointLimits):Void
		operations.push(operation("mateOnAxis", id, kind, parent, parentConnector, child, childConnector,
			value, axis, limits));

	public function addConstraint(id:String, kind:String, parent:String, parentConnector:String,
			child:String, childConnector:String, ?tolerance:Float):Void {
		var entry = operation("constrain", id, kind, parent, parentConnector, child, childConnector);
		entry.tolerance = tolerance;
		operations.push(entry);
	}

	public function addConstraintOnAxis(id:String, kind:String, parent:String, parentConnector:String,
			child:String, childConnector:String, axis:AssemblyVector, ?tolerance:Float,
			?limits:AssemblyJointLimits):Void {
		var entry = operation("constrainOnAxis", id, kind, parent, parentConnector, child, childConnector,
			0, axis, limits);
		entry.tolerance = tolerance;
		operations.push(entry);
	}

	public function addCoupling(id:String, source:String, target:String, ratio:Float, offset:Float = 0):Void {
		var entry = operation("couple", id, "", "", "", "", "");
		entry.source = source;
		entry.target = target;
		entry.ratio = ratio;
		entry.offset = offset;
		operations.push(entry);
	}

	/** Add a calculated connector on one member, such as a screw seat above a housing face. */
	public function addMemberConnector(instanceId:String, name:String, frame:AssemblyFrame):Void {
		if (name == null || name.length == 0) throw "Assembly connector needs a name";
		memberConnectorFrames.push({instanceId: instanceId, name: name, frame: copyFrame(frame)});
	}

	final memberConnectorFrames:Array<{instanceId:String, name:String, frame:AssemblyFrame}> = [];

	/** Publish a stable assembly-level name that resolves to one member connector. */
	public function exposeConnector(name:String, instanceId:String, connectorName:String):Void {
		if (name == null || name.length == 0) throw "External assembly connector needs a name";
		for (existing in externalConnectors) if (existing.name == name)
			throw 'Duplicate external assembly connector "$name"';
		externalConnectors.push({name: name, instanceId: instanceId, connectorName: connectorName});
	}

	public function addBomItem(item:BomItem, quantity:Int = 1):Void {
		if (item == null || quantity <= 0) throw "Assembly BOM entry needs an item and positive quantity";
		bomItems.push({item: item, quantity: quantity});
	}

	/** Populate an existing model. All member and joint ids receive the supplied prefix. */
	public function addTo(model:AssemblyModel, prefix:String, ?pose:AssemblyFrame):Void {
		for (member in members) {
			var localPose = pose == null ? member.pose : AssemblyFrames.compose(pose, member.pose);
			member.component.addTo(model, join(prefix, member.id), localPose);
		}
		for (connector in memberConnectorFrames)
			model.connector(join(prefix, connector.instanceId), connector.name, connector.frame);
		for (operation in operations) {
			var id = join(prefix, operation.id);
			switch operation.operation {
				case "mate": model.mate(id, operation.jointKind, join(prefix, operation.parent),
					operation.parentConnector, join(prefix, operation.child), operation.childConnector, operation.value);
				case "mateOnAxis": model.mateOnAxis(id, operation.jointKind, join(prefix, operation.parent),
					operation.parentConnector, join(prefix, operation.child), operation.childConnector,
					operation.axis, operation.value, operation.limits);
				case "constrain": model.constrain(id, operation.jointKind, join(prefix, operation.parent),
					operation.parentConnector, join(prefix, operation.child), operation.childConnector, operation.tolerance);
				case "constrainOnAxis": model.constrainOnAxis(id, operation.jointKind, join(prefix, operation.parent),
					operation.parentConnector, join(prefix, operation.child), operation.childConnector,
					operation.axis, operation.tolerance, operation.limits);
				case "couple": model.couple(id, join(prefix, operation.source), join(prefix, operation.target),
					operation.ratio, operation.offset);
				default: throw 'Unknown assembly operation "${operation.operation}"';
			}
		}
	}

	/** Member ids and components in this assembly's local namespace. */
	public function components():Array<MachineAssemblyComponent>
		return [for (member in members) {id: member.id, component: member.component}];

	public function billOfMaterials():Bom {
		var result = new Bom();
		for (member in members) result.addComponent(member.component);
		for (entry in bomItems) result.add(entry.item, entry.quantity);
		return result;
	}

	public function connectorNames():Array<String>
		return [for (connector in externalConnectors) connector.name];

	public function connector(name:String, prefix:String = ""):MachineAssemblyConnector {
		for (connector in externalConnectors) if (connector.name == name)
			return {instanceId: join(prefix, connector.instanceId), connectorName: connector.connectorName};
		throw 'Missing assembly connector "$name"';
	}

	public static function join(prefix:String, id:String):String
		return prefix == null || prefix.length == 0 ? id : '$prefix/$id';

	static function operation(type:String, id:String, kind:String, parent:String, parentConnector:String,
			child:String, childConnector:String, value:Float = 0, ?axis:AssemblyVector,
			?limits:AssemblyJointLimits):AssemblyOperation
		return {operation: type, id: id, jointKind: kind, parent: parent, parentConnector: parentConnector,
			child: child, childConnector: childConnector, axis: axis, value: value, limits: limits,
			tolerance: null, source: null, target: null, ratio: 1, offset: 0};

	static function copyFrame(frame:AssemblyFrame):AssemblyFrame
		return {x: frame.x, y: frame.y, z: frame.z, qx: frame.qx, qy: frame.qy, qz: frame.qz, qw: frame.qw};
}
