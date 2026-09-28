package machinekit.assembly;

import cadkit.modeling.AssemblyModel;
import cadkit.modeling.AssemblyState;
import cadkit.modeling.Vector;
import machinekit.component.Bom;
import machinekit.component.BomItem;
import machinekit.component.MachineComponent;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyDefinition.AssemblyJointType;
import materia.assembly.AssemblyDefinition.AssemblyJointLimits;
import materia.assembly.AssemblyDefinition.AssemblyVector;
import materia.assembly.AssemblyRecord.AssemblyFrame;

typedef MachineAssemblyComponent = { var id:String; var component:MachineComponent; }
typedef MachineAssemblyConnector = { var instanceId:String; var connectorName:String; }
typedef MachineSubassembly = { var id:String; var assembly:MachineAssembly; }
typedef MachineAssemblyMassProperties = {
	var mass:Float;
	var centreOfMass:Vector;
	var unaccounted:Array<String>;
}

enum MachineAssemblyOperation {
	Mate(id:String, kind:AssemblyJointType, parent:MachineAssemblyConnector,
		child:MachineAssemblyConnector, value:Float, axis:Null<AssemblyVector>, limits:Null<AssemblyJointLimits>);
	Constrain(id:String, kind:AssemblyJointType, parent:MachineAssemblyConnector,
		child:MachineAssemblyConnector, axis:Null<AssemblyVector>, tolerance:Null<Float>, limits:Null<AssemblyJointLimits>);
	Couple(id:String, source:String, target:String, ratio:Float, offset:Float);

}

private typedef AssemblyMember = {
	var id:String;
	var component:MachineComponent;
	var pose:AssemblyFrame;
}

/** Reusable, prefixable assembly made from MachineComponents and named connector references. */
class MachineAssembly {
	final members:Array<AssemblyMember> = [];
	final included:Array<MachineSubassembly> = [];
	final externalConnectors:Array<{name:String, instanceId:String, connectorName:String}> = [];
	final operations:Array<MachineAssemblyOperation> = [];
	final bomItems:Array<{item:BomItem, quantity:Int}> = [];
	final memberConnectorFrames:Array<{instanceId:String, name:String, frame:AssemblyFrame}> = [];

	public function new() {}

	public function addComponent(id:String, component:MachineComponent, ?pose:AssemblyFrame):Void {
		if (id == null || id.length == 0 || component == null) throw "Assembly component needs an id and component";
		for (member in members) if (member.id == id) throw 'Duplicate assembly component "$id"';
		members.push({id: id, component: component,
			pose: pose == null ? AssemblyFrames.identity() : copyFrame(pose)});
	}

	/** Include another assembly below a local namespace, optionally moving all of its parts. */
	public function include(id:String, assembly:MachineAssembly, ?pose:AssemblyFrame):Void {
		if (id == null || id.length == 0 || assembly == null || assembly == this)
			throw "Included assembly needs a distinct id and assembly";
		for (entry in included) if (entry.id == id) throw 'Duplicate included assembly "$id"';
		assembly.validate();
		for (member in assembly.members) {
			var localPose = pose == null ? member.pose : AssemblyFrames.compose(pose, member.pose);
			addComponent(join(id, member.id), member.component, localPose);
		}
		for (connector in assembly.memberConnectorFrames)
			addMemberConnector(join(id, connector.instanceId), connector.name, connector.frame);
		for (connector in assembly.externalConnectors)
			exposeConnector(join(id, connector.name), join(id, connector.instanceId), connector.connectorName);
		for (operation in assembly.operations) addOperation(prefixed(operation, id));
		for (entry in assembly.bomItems) addBomItem(entry.item, entry.quantity);
		included.push({id: id, assembly: assembly});
	}

	public function addMate(id:String, kind:String, parent:String, parentConnector:String,
			child:String, childConnector:String, value:Float = 0):Void
		addOperation(Mate(id, cast kind, ref(parent, parentConnector), ref(child, childConnector), value, null, null));

	public function addMateOnAxis(id:String, kind:String, parent:String, parentConnector:String,
			child:String, childConnector:String, axis:AssemblyVector, value:Float = 0,
			?limits:AssemblyJointLimits):Void
		addOperation(Mate(id, cast kind, ref(parent, parentConnector), ref(child, childConnector), value, axis, limits));

	public function addConstraint(id:String, kind:String, parent:String, parentConnector:String,
			child:String, childConnector:String, ?tolerance:Float):Void
		addOperation(Constrain(id, cast kind, ref(parent, parentConnector), ref(child, childConnector), null, tolerance, null));

	public function addConstraintOnAxis(id:String, kind:String, parent:String, parentConnector:String,
			child:String, childConnector:String, axis:AssemblyVector, ?tolerance:Float,
			?limits:AssemblyJointLimits):Void
		addOperation(Constrain(id, cast kind, ref(parent, parentConnector), ref(child, childConnector), axis, tolerance, limits));

	public function addCoupling(id:String, source:String, target:String, ratio:Float, offset:Float = 0):Void
		addOperation(Couple(id, source, target, ratio, offset));

	/** Add a calculated connector on one member, such as a screw seat above a housing face. */
	public function addMemberConnector(instanceId:String, name:String, frame:AssemblyFrame):Void {
		if (name == null || name.length == 0) throw "Assembly connector needs a name";
		requireMember(instanceId);
		for (entry in memberConnectorFrames) if (entry.instanceId == instanceId && entry.name == name)
			throw 'Duplicate assembly connector "$instanceId/$name"';
		for (connector in requireMember(instanceId).connectors()) if (connector.name == name)
			throw 'Duplicate assembly connector "$instanceId/$name"';
		memberConnectorFrames.push({instanceId: instanceId, name: name, frame: copyFrame(frame)});
	}

	/** Publish a stable assembly-level name that resolves to one member connector. */
	public function exposeConnector(name:String, instanceId:String, connectorName:String):Void {
		if (name == null || name.length == 0) throw "External assembly connector needs a name";
		requireConnector(ref(instanceId, connectorName));
		for (existing in externalConnectors) if (existing.name == name)
			throw 'Duplicate external assembly connector "$name"';
		externalConnectors.push({name: name, instanceId: instanceId, connectorName: connectorName});
	}

	public function addBomItem(item:BomItem, quantity:Int = 1):Void {
		if (item == null || quantity <= 0) throw "Assembly BOM entry needs an item and positive quantity";
		bomItems.push({item: item, quantity: quantity});
	}

	/** Check relationships that depend on the full operation set. */
	public function validate():Void {
		var parents:Map<String, String> = [];
		var joints:Map<String, Bool> = [];
		for (op in operations) switch op {
			case Mate(id, _, parent, child, _, _, _):
				if (parents.exists(child.instanceId)) throw 'Assembly member "${child.instanceId}" has two parent joints';
				parents.set(child.instanceId, parent.instanceId);
				joints.set(id, true);
			case Constrain(id, _, _, _, _, _, _): joints.set(id, true);
			case Couple(_, _, _, _, _):
		}
		for (member in members) {
			var seen:Map<String, Bool> = [];
			var current = member.id;
			while (parents.exists(current)) {
				if (seen.exists(current)) throw 'Assembly mate cycle at "$current"';
				seen.set(current, true);
				current = parents.get(current);
			}
		}
		for (op in operations) switch op {
			case Couple(id, source, target, _, _):
				if (!joints.exists(source) || !joints.exists(target))
					throw 'Assembly coupling "$id" refers to a missing joint';
			case _:
		}
	}

	/** Populate an existing model. All member and joint ids receive the supplied prefix. */
	public function addTo(model:AssemblyModel, prefix:String, ?pose:AssemblyFrame):Void {
		validate();
		for (member in members) {
			var localPose = pose == null ? member.pose : AssemblyFrames.compose(pose, member.pose);
			member.component.addTo(model, join(prefix, member.id), localPose);
		}
		for (connector in memberConnectorFrames)
			model.connector(join(prefix, connector.instanceId), connector.name, connector.frame);
		for (op in operations) switch op {
			case Mate(id, kind, parent, child, value, axis, limits):
				if (axis == null) model.mate(join(prefix, id), kind, join(prefix, parent.instanceId),
					parent.connectorName, join(prefix, child.instanceId), child.connectorName, value);
				else model.mateOnAxis(join(prefix, id), kind, join(prefix, parent.instanceId),
					parent.connectorName, join(prefix, child.instanceId), child.connectorName, axis, value, limits);
			case Constrain(id, kind, parent, child, axis, tolerance, limits):
				if (axis == null) model.constrain(join(prefix, id), kind, join(prefix, parent.instanceId),
					parent.connectorName, join(prefix, child.instanceId), child.connectorName, tolerance);
				else model.constrainOnAxis(join(prefix, id), kind, join(prefix, parent.instanceId),
					parent.connectorName, join(prefix, child.instanceId), child.connectorName, axis, tolerance, limits);
			case Couple(id, source, target, ratio, offset):
				model.couple(join(prefix, id), join(prefix, source), join(prefix, target), ratio, offset);
		}
	}

	public function components():Array<MachineAssemblyComponent>
		return [for (member in members) {id: member.id, component: member.component}];

	public function subassemblies():Array<MachineSubassembly> return included.copy();

	public function billOfMaterials():Bom {
		var result = new Bom();
		for (member in members) result.addComponent(member.component);
		for (entry in bomItems) result.add(entry.item, entry.quantity);
		return result;
	}

	/** Sum posed component masses; separately report BOM extras with no mass model. */
	public function massProperties(?state:AssemblyState):MachineAssemblyMassProperties {
		var model = new AssemblyModel();
		addTo(model, "");
		var mass = 0.0, weightedX = 0.0, weightedY = 0.0, weightedZ = 0.0;
		for (member in members) {
			var properties = member.component.massProperties();
			var pose = state == null ? model.pose(member.id) : state.worldPose(member.id);
			var centre = properties.centreOfMass;
			var world = AssemblyFrames.transformPoint(pose, centre.x, centre.y, centre.z);
			mass += properties.mass;
			weightedX += properties.mass * world.x;
			weightedY += properties.mass * world.y;
			weightedZ += properties.mass * world.z;
		}
		var unaccounted:Array<String> = [];
		for (entry in bomItems) if (unaccounted.indexOf(entry.item.partNumber) < 0)
			unaccounted.push(entry.item.partNumber);
		return {mass: mass, centreOfMass: mass == 0 ? new Vector() :
			new Vector(weightedX / mass, weightedY / mass, weightedZ / mass), unaccounted: unaccounted};
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

	function addOperation(op:MachineAssemblyOperation):Void {
		var id = operationId(op);
		if (id == null || id.length == 0) throw "Assembly operation needs an id";
		for (existing in operations) if (operationId(existing) == id)
			throw 'Duplicate assembly operation "$id"';
		switch op {
			case Mate(_, kind, parent, child, _, _, _) | Constrain(_, kind, parent, child, _, _, _):
				if (kind != AssemblyJointType.Fixed && kind != AssemblyJointType.Revolute &&
					kind != AssemblyJointType.Continuous && kind != AssemblyJointType.Prismatic)
					throw 'Unsupported assembly joint "$kind"';
				requireConnector(parent);
				requireConnector(child);
			case Couple(_, source, target, _, _):
				if (source == null || source.length == 0 || target == null || target.length == 0)
					throw 'Assembly coupling "$id" needs source and target';
		}
		operations.push(op);
	}

	function requireMember(id:String):MachineComponent {
		for (member in members) if (member.id == id) return member.component;
		throw 'Unknown assembly member "$id"';
	}

	function requireConnector(reference:MachineAssemblyConnector):Void {
		var component = requireMember(reference.instanceId);
		if (reference.connectorName == null || reference.connectorName.length == 0)
			throw 'Unknown connector "${reference.instanceId}/${reference.connectorName}"';
		for (entry in memberConnectorFrames)
			if (entry.instanceId == reference.instanceId && entry.name == reference.connectorName) return;
		for (connector in component.connectors()) if (connector.name == reference.connectorName) return;
		throw 'Unknown connector "${reference.instanceId}/${reference.connectorName}"';
	}

	static function operationId(op:MachineAssemblyOperation):String return switch op {
		case Mate(id, _, _, _, _, _, _) | Constrain(id, _, _, _, _, _, _) | Couple(id, _, _, _, _): id;
	}

	static function ref(instanceId:String, connectorName:String):MachineAssemblyConnector
		return {instanceId: instanceId, connectorName: connectorName};

	static function prefixed(op:MachineAssemblyOperation, prefix:String):MachineAssemblyOperation return switch op {
		case Mate(id, kind, parent, child, value, axis, limits):
			Mate(join(prefix, id), kind, ref(join(prefix, parent.instanceId), parent.connectorName),
				ref(join(prefix, child.instanceId), child.connectorName), value, axis, limits);
		case Constrain(id, kind, parent, child, axis, tolerance, limits):
			Constrain(join(prefix, id), kind, ref(join(prefix, parent.instanceId), parent.connectorName),
				ref(join(prefix, child.instanceId), child.connectorName), axis, tolerance, limits);
		case Couple(id, source, target, ratio, offset):
			Couple(join(prefix, id), join(prefix, source), join(prefix, target), ratio, offset);
	}

	static function copyFrame(frame:AssemblyFrame):AssemblyFrame
		return {x: frame.x, y: frame.y, z: frame.z, qx: frame.qx, qy: frame.qy, qz: frame.qz, qw: frame.qw};
}
