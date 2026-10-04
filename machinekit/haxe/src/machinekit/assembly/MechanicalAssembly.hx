package machinekit.assembly;

import machinekit.component.MachineComponent;
import machinekit.assembly.MachineAssembly.MachineSubassembly;
import machinekit.assembly.MachineAssemblyDescription.ConnectorExposureRecord;
import machinekit.assembly.MachineAssemblyDescription.MemberConnectorRecord;
import materia.assembly.AssemblyDefinition.AssemblyJointCoupling;
import materia.assembly.AssemblyDefinition.AssemblyJointRole;
import materia.assembly.AssemblyDefinition.KinematicJoint;
import materia.assembly.AssemblyRecord.AssemblyFrame;

typedef LevelMember = {var id:String; var component:MachineComponent; var pose:AssemblyFrame;}

/**
 * One level's parts and how they are put together: its own members, the subassemblies it includes,
 * connectors it adds or publishes, and its joints and couplings. A reference to a member is a path
 * below this level, so a joint may attach a member of a subassembly (`arm/link1`).
 */
class MechanicalAssembly {
	public final members:Array<LevelMember> = [];
	public final subassemblies:Array<MachineSubassembly> = [];
	/** Members and subassemblies by local id, in the order they were added. */
	public final order:Array<String> = [];
	public final memberConnectors:Array<MemberConnectorRecord> = [];
	public final connectorExposures:Array<ConnectorExposureRecord> = [];
	public final joints:Array<KinematicJoint> = [];
	public final couplings:Array<AssemblyJointCoupling> = [];
	final memberById:Map<String, LevelMember> = [];
	final subassemblyById:Map<String, MachineSubassembly> = [];

	public function new() {}

	public function addMember(id:String, component:MachineComponent, pose:AssemblyFrame):Void {
		requireFreeName(id);
		var member = {id: id, component: component, pose: MachineAssembly.copyFrame(pose)};
		members.push(member);
		memberById.set(id, member);
		order.push(id);
	}

	public function addSubassembly(entry:MachineSubassembly):Void {
		requireFreeName(entry.id);
		subassemblies.push(entry);
		subassemblyById.set(entry.id, entry);
		order.push(entry.id);
	}

	function requireFreeName(id:String):Void {
		if (memberById.exists(id)) throw 'Duplicate assembly component "$id"';
		if (subassemblyById.exists(id)) throw 'Duplicate included assembly "$id"';
	}

	public function member(id:String):Null<LevelMember> return memberById.get(id);

	public function subassembly(id:String):Null<MachineSubassembly> return subassemblyById.get(id);

	public function requireSubassembly(id:String):MachineSubassembly {
		var entry = subassemblyById.get(id);
		if (entry == null) throw 'Missing included assembly "$id"';
		return entry;
	}

	public function coupling(id:String):Null<AssemblyJointCoupling> {
		for (entry in couplings) if (entry.id == id) return entry;
		return null;
	}

	/** Mate-tree faults: two parents, cycles, and couplings between missing joints. */
	public static function checkStructure(flat:FlatAssembly, result:Diagnostics):Void {
		var parents:Map<String, String> = [];
		var joints:Map<String, Bool> = [];
		for (joint in flat.definition.joints) {
			if (joint.role == AssemblyJointRole.Tree) {
				if (parents.exists(joint.child))
					result.error("assembly.multiple-parents", joint.child,
						'Assembly member "${joint.child}" has two parent joints');
				else parents.set(joint.child, joint.parent);
			}
			joints.set(joint.id, true);
		}
		var cycles:Map<String, Bool> = [];
		for (member in flat.members) {
			var seen:Map<String, Bool> = [];
			var current = member.id;
			while (parents.exists(current)) {
				if (seen.exists(current)) {
					if (!cycles.exists(current)) result.error("assembly.mate-cycle", current,
						'Assembly mate cycle at "$current"');
					cycles.set(current, true);
					break;
				}
				seen.set(current, true);
				current = parents.get(current);
			}
		}
		if (flat.definition.couplings != null) for (coupling in flat.definition.couplings)
			if (!joints.exists(coupling.source) || !joints.exists(coupling.target))
				result.error("assembly.missing-coupling-joint", coupling.id,
					'Assembly coupling "${coupling.id}" refers to a missing joint');
	}

	public static function copyJoint(joint:KinematicJoint, map:String->String):KinematicJoint {
		var copy:KinematicJoint = {id: map(joint.id), type: joint.type, role: joint.role,
			parent: map(joint.parent), parentConnector: joint.parentConnector,
			child: map(joint.child), childConnector: joint.childConnector,
			axis: {x: joint.axis.x, y: joint.axis.y, z: joint.axis.z},
			limits: {lower: joint.limits.lower, upper: joint.limits.upper,
				velocity: joint.limits.velocity, effort: joint.limits.effort, overtravel: joint.limits.overtravel,
				acceleration: joint.limits.acceleration},
			defaultValue: joint.defaultValue};
		if (joint.limits.assumptions != null && joint.limits.assumptions.length > 0)
			copy.limits.assumptions = [for (value in joint.limits.assumptions)
				{quantity: value.quantity, label: value.label}];
		if (joint.closureTolerance != null) copy.closureTolerance = joint.closureTolerance;
		return copy;
	}

	public static function copyCoupling(coupling:AssemblyJointCoupling, map:String->String):AssemblyJointCoupling {
		var copy:AssemblyJointCoupling = {id: map(coupling.id), source: map(coupling.source),
			target: map(coupling.target), ratio: coupling.ratio, offset: coupling.offset};
		if (coupling.efficiency != null) copy.efficiency = coupling.efficiency;
		if (coupling.stiffness != null) copy.stiffness = coupling.stiffness;
		if (coupling.backlash != null) copy.backlash = coupling.backlash;
		if (coupling.drag != null) copy.drag = coupling.drag;
		if (coupling.assumptions != null && coupling.assumptions.length > 0)
			copy.assumptions = [for (value in coupling.assumptions) {quantity: value.quantity, label: value.label}];
		if (coupling.assumed != null && coupling.assumed.length > 0) copy.assumed = [for (label in coupling.assumed) label];
		return copy;
	}
}
