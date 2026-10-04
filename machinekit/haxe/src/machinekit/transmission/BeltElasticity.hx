package machinekit.transmission;

import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinition.AssemblyElasticNetwork;
import materia.assembly.AssemblyDefinition.AssemblyElasticSpan;
import materia.assembly.AssemblyDefinition.AssemblyJointType;
import materia.assembly.AssemblyDefinition.AssemblyJointRole;
import machinekit.assembly.MachineAssemblyDescription.BeltPathRecord;
import machinekit.assembly.MachineAssemblyDescription.TransmissionRecord;
import machinekit.component.MachineComponent;
import cadkit.modeling.AssemblyState;

/** Compile one shaft belt's physical spans, regardless of how many motion records refer to it. */
class BeltElasticity {
	public static function build(belt:TimingBelt, path:BeltPathRecord, records:Array<TransmissionRecord>,
			mechanical:AssemblyDefinition, member:String->MachineComponent,
			actuators:Array<materia.assembly.AssemblyDefinition.AssemblyActuator>,
			?context:BeltPoseContext):AssemblyElasticNetwork {
		if (context == null) context = new BeltPoseContext(mechanical);
		context.reset();
		var definition = context.definition;
		var samples:Array<{joint:String, positions:Array<Float>, zero:Float}> = [];
		for (joint in definition.joints) if (joint.role == AssemblyJointRole.Tree && joint.type != AssemblyJointType.Fixed) {
			var positions = [joint.defaultValue + 0.001, joint.defaultValue - 0.001];
			for (source in mechanical.joints) if (source.id == joint.id) {
				if (source.limits.lower != null) positions.push(source.limits.lower);
				if (source.limits.upper != null) positions.push(source.limits.upper);
			}
			samples.push({joint: joint.id, positions: positions, zero: joint.defaultValue});
		}
		var state = context.state;
		var posed = BeltStretch.posedBelt(belt, path, state);
		if (Math.abs(posed.length - belt.length) > 1e-4)
			throw new TransmissionDesignError("Belt length does not match its wrap attachments");
		var free:Array<String> = [], owners:Array<String> = [];
		var stated:Null<Float> = null, statedCoupling:Null<String> = null;
		for (record in records) switch record.source {
			case BeltReduction(id, _, _) if (id == path.belt):
				owners.push(record.coupling);
				if (record.stiffness != null) {
					if (stated != null) throw new TransmissionDesignError("State one stiffness for a two-terminal belt, not one per output");
					stated = record.stiffness; statedCoupling = record.coupling;
				}
			case BeltIdler(id, pulley) if (id == path.belt): free.push(pulley); owners.push(record.coupling);
			case TimingBelt(id, _) if (id == path.belt):
				throw new TransmissionDesignError("A shaft reduction belt cannot also own a carriage coupling; use a separate carriage belt");
			case _:
		}
		var indexes:Array<Int> = [], joints:Array<String> = [], radii:Array<Float> = [];
		for (index in 0...path.wraps.length) {
			var reference = path.wraps[index];
			if (free.indexOf(reference.instanceId) >= 0) continue;
			var joint = rotaryJoint(definition, reference.instanceId);
			if (joint == null) continue; // A stationary-centre, freely turning unactuated wrap.
			var part = member(reference.instanceId);
			if (!Std.isOfType(part, TimingPulley)) throw new TransmissionDesignError("A loaded belt wrap needs a timing pulley");
			var pulley:TimingPulley = cast part;
			if (pulley.beltProfile != belt.beltProfile || Math.abs(pulley.pitchDiameter / 2 - belt.wraps()[index].radius) > 1e-5)
				throw new TransmissionDesignError("Loaded pulley tooth count or profile does not match its belt wrap");
			var sign = BeltStretch.axisSign(definition, state, path.belt, joint, reference.instanceId);
			indexes.push(index); joints.push(joint); radii.push(sign * belt.wraps()[index].radius * belt.wraps()[index].side);
		}
		if (indexes.length < 2) throw new TransmissionDesignError("A shaft belt needs at least two loaded rotary attachments");
		if (stated != null && indexes.length != 2)
			throw new TransmissionDesignError("Pairwise stiffness overrides cannot replace a shared multi-output belt; clear the override");
		var ea = TimingBelt.cordStiffnessPerMm(belt.beltProfile) * belt.width;
		var spans:Array<AssemblyElasticSpan> = [];
		for (at in 0...indexes.length) {
			var next = (at + 1) % indexes.length;
			var length = posed.freePaths(indexes[at], indexes[next])[0];
			spans.push({stiffness: BeltStretch.spanStiffness(ea, length), terms: [
				{joint: joints[at], coefficient: radii[at]}, {joint: joints[next], coefficient: -radii[next]}]});
		}
		// This shaft-loop model requires fixed free-path lengths; do not silently omit a
		// moving tensioner's or an eccentric shaft's displacement from the span energies.
		for (sample in samples) {
			for (position in sample.positions) {
				state.setJoint(sample.joint, position);
				var moved = BeltStretch.posedBelt(belt, path, state);
				for (at in 0...indexes.length) {
					var next = (at + 1) % indexes.length;
					if (Math.abs(moved.freePaths(indexes[at], indexes[next])[0] - posed.freePaths(indexes[at], indexes[next])[0]) > 1e-5)
						throw new TransmissionDesignError("Shaft belt has moving free-path geometry; fix its centres or model the tensioner before solving its elasticity");
				}
			}
			state.setJoint(sample.joint, sample.zero);
		}

		if (stated != null) {
			var source:Null<String> = null;
			for (coupling in mechanical.couplings) if (coupling.id == statedCoupling) source = coupling.source;
			var at = joints.indexOf(source);
			if (at < 0) throw new TransmissionDesignError("Stated belt stiffness needs its rotary leader");
			var paths = posed.freePaths(indexes[0], indexes[1]);
			var derived = BeltStretch.energy(ea, paths, radii[at], radii[at]) / 1000;
			for (span in spans) span.stiffness *= stated / derived;
		}
		var clearances:Array<materia.assembly.AssemblyDefinition.AssemblyElasticClearance> = [];
		for (record in records) switch record.source {
			case BeltReduction(id, driver, driven) if (id == path.belt):
				var drivenJoint = rotaryJoint(definition, driven), driverJoint = rotaryJoint(definition, driver);
				var drivenAt = joints.indexOf(drivenJoint), driverAt = joints.indexOf(driverJoint);
				if (drivenAt < 0 || driverAt < 0) throw new TransmissionDesignError("Reduction needs two loaded pulley contacts");
				var clearance = record.backlash == null ? TimingBelt.toothClearance(belt.beltProfile) / Math.abs(radii[drivenAt])
					: record.backlash * Math.abs(radii[driverAt] / radii[drivenAt]);
				var found = false;
				for (entry in clearances) if (entry.joint == drivenJoint) {
					if (Math.abs(entry.allowance - clearance) > 1e-9) throw new TransmissionDesignError("One pulley contact has conflicting tooth-clearance overrides");
					found = true;
				}
				if (!found) clearances.push({joint: drivenJoint, allowance: clearance});
			case _:
		}
		// A screw linked to the carriage by a lead-screw relation can still be a
		// loaded contact even when no extra belt motion record names that pulley.
		for (at in 0...joints.length) {
			var found = false;
			for (entry in clearances) if (entry.joint == joints[at]) found = true;
			if (!found) clearances.push({joint: joints[at],
				allowance: TimingBelt.toothClearance(belt.beltProfile) / Math.abs(radii[at])});
		}

		// Use the actuator's usable design torque, not the stepper's holding or overload torque.
		var peakDifference = 0.0;
		for (actuator in actuators) {
			var at = joints.indexOf(actuator.joint);
			if (at < 0) continue;
			var torque = actuator.maxEffort;
			if (actuator.gearRatio != null) torque *= actuator.gearRatio;
			if (actuator.gearEfficiency != null) torque *= actuator.gearEfficiency;
			peakDifference += torque * 1000 / Math.abs(radii[at]);
		}
		if (peakDifference > 0) belt.checkTension(peakDifference, belt.assumedPretension());
		var assumptions:Array<materia.assembly.AssemblyDefinition.QuantityAssumption> = [
			{quantity: "pretension", label: 'assumed ${belt.assumedPretension()} N; working limit ${belt.assumedWorkingTension()} N'}];
		if (stated == null) assumptions.push({quantity: "stiffness", label: "belt stiffness with pretension"});
		return {id: path.belt, couplings: owners, spans: spans, clearances: clearances,
			assumptions: assumptions};
	}

	/** Follow rigid attachments to the rotary shaft carrying this pulley. */
	public static function rotaryJoint(definition:AssemblyDefinition, pulley:String):Null<String> {
		var child = pulley;
		for (_ in 0...definition.joints.length) {
			var found:Null<materia.assembly.AssemblyDefinition.KinematicJoint> = null;
			for (joint in definition.joints) if (joint.child == child) found = joint;
			if (found == null) return null;
			if (found.type == AssemblyJointType.Continuous || found.type == AssemblyJointType.Revolute) return found.id;
			if (found.type != AssemblyJointType.Fixed) return null;
			child = found.parent;
		}
		return null;
	}
}
