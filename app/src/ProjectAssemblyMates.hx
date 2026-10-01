package app;

import cadkit.modeling.AssemblyMateSolver;
import cadkit.parametric.GeometricConnectors;
import cadkit.parametric.GeometricConnectors.GeometricCandidates;
import cadkit.parametric.GeometricConnectors.GeometricConnectorError;
import cadkit.parametric.GeometricConnectors.GeometricFeatureKind;
import haxeon.wire.JsonWire;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinition.AssemblyMate;
import materia.assembly.AssemblyDefinition.AssemblyStateRecord;
import materia.assembly.AssemblyDefinition.KinematicJoint;
import materia.assembly.AssemblyDefinitionCodec;
import materia.assembly.AssemblyRecord.AssemblyConnector;

/** A connector the editor added to a generated component (a face captured for a mate). */
@:wire typedef AssemblyOverlayConnector = {
	@:id(1) var component:String;
	@:id(2) var connector:AssemblyConnector;
}

/** The mates authored in the editor over a generated assembly, as saved in the project scene record. */
@:wire typedef AssemblyMateOverlayRecord = {
	@:id(1) var connectors:Array<AssemblyOverlayConnector>;
	@:id(2) var mates:Array<AssemblyMate>;
	/** Joints made from mates (see `ProjectAssemblyMates.withJoint`). */
	@:id(3) @:optional var joints:Array<KinematicJoint>;
}

/**
	Mates authored in the editor over a project's generated assembly (plan
	C4.5): the project process owns the definition, so the editor keeps its
	mates, and the face connectors they name, beside it and lays them over it
	(`effective`). Face connectors are captured from the face descriptors the
	project wrote into its artifact and framed again from them after every
	rebuild (`reframe`). Immutable: an edit makes a new overlay, so undo is a
	swap.
*/
class ProjectAssemblyMates {
	public final connectors:Array<AssemblyOverlayConnector>;
	public final mates:Array<AssemblyMate>;
	public final joints:Array<KinematicJoint>;

	public function new(?connectors:Array<AssemblyOverlayConnector>, ?mates:Array<AssemblyMate>, ?joints:Array<KinematicJoint>) {
		this.connectors = connectors == null ? [] : connectors.copy();
		this.mates = mates == null ? [] : mates.copy();
		this.joints = joints == null ? [] : joints.copy();
	}

	public static function empty():ProjectAssemblyMates
		return new ProjectAssemblyMates();

	public static function decode(text:Null<String>):ProjectAssemblyMates {
		if (text == null || text.length == 0) return empty();
		var record:AssemblyMateOverlayRecord = JsonWire.decode(text);
		return new ProjectAssemblyMates(record.connectors, record.mates, record.joints);
	}

	public function encode():String {
		var record:AssemblyMateOverlayRecord = {connectors: connectors, mates: mates};
		if (joints.length > 0) record.joints = joints;
		return JsonWire.encode(record);
	}

	public function isEmpty():Bool
		return mates.length == 0 && connectors.length == 0 && joints.length == 0;

	/** The generated definition with this overlay's connectors and mates added; validated, with the mates checked against their faces. */
	public function effective(generated:AssemblyDefinition):AssemblyDefinition {
		var copy:AssemblyDefinition = JsonWire.decode(JsonWire.encode(generated));
		for (added in connectors) {
			var found = false;
			for (component in copy.definitions) if (component.id == added.component) {
				for (existing in component.connectors)
					if (existing.name == added.connector.name)
						throw 'Component "${added.component}" already has a connector named "${added.connector.name}"';
				component.connectors.push(added.connector);
				found = true;
			}
			if (!found) throw 'The project no longer has component "${added.component}"';
		}
		for (joint in joints) copy.joints.push(joint);
		if (mates.length > 0) {
			var all = copy.mates == null ? [] : copy.mates.copy();
			for (mate in mates) all.push(mate);
			copy.mates = all;
		}
		AssemblyDefinitionCodec.validate(copy);
		GeometricConnectors.checkMates(copy);
		return copy;
	}

	/**
		This overlay with its face connectors framed again from the components' current face descriptors
		(`faceDescriptors` by component id). Throws `GeometricConnectorError` when a face is lost or ambiguous.
	*/
	public function reframe(generated:AssemblyDefinition, faceDescriptors:Map<String, String>):ProjectAssemblyMates {
		if (connectors.length == 0) return this;
		var framed = GeometricConnectors.reframe(effective(generated), (scope, component) -> {
			var descriptors = scope == "" ? faceDescriptors.get(component) : null;
			return descriptors == null ? [] : [GeometricCandidates.ofDescriptors(descriptors)];
		});
		var next:Array<AssemblyOverlayConnector> = [];
		for (added in connectors)
			for (component in framed.definitions) if (component.id == added.component)
				for (connector in component.connectors) if (connector.name == added.connector.name)
					next.push({component: added.component, connector: connector});
		return new ProjectAssemblyMates(next, mates, joints);
	}

	/**
		The name of the connector on face `faceIndex` of `component`, and the overlay that has it: an existing
		connector on that face is reused, else one is captured from `descriptors` as "face<index>".
	*/
	public function faceConnector(component:String, faceIndex:Int, descriptors:Null<String>):{name:String, overlay:ProjectAssemblyMates} {
		var name = "face" + faceIndex;
		for (added in connectors) if (added.component == component && added.connector.name == name) return {name: name, overlay: this};
		if (descriptors == null) throw 'Component "$component" has no face descriptors; rebuild the project to mate its faces';
		var connector = GeometricConnectors.captureDescribed(name, descriptors, faceIndex);
		var next = connectors.copy();
		next.push({component: component, connector: connector});
		return {name: name, overlay: new ProjectAssemblyMates(next, mates, joints)};
	}

	/** The feature face `faceIndex` offers a mate, from its component's `descriptors`; null when it offers none. */
	public static function describedFeature(descriptors:Null<String>, faceIndex:Int):Null<GeometricFeatureKind> {
		if (descriptors == null) return null;
		var records:Array<Dynamic> = haxe.Json.parse(descriptors);
		for (record in records) if (Reflect.field(record, "index") == faceIndex) {
			var feature:String = Reflect.field(record, "feature");
			return switch feature {
				case "plane": GeometricFeatureKind.Plane;
				case "axis": GeometricFeatureKind.Axis;
				case "sphere": GeometricFeatureKind.Sphere;
				case "circle": GeometricFeatureKind.Circle;
				case "line": GeometricFeatureKind.Line;
				default: null;
			};
		}
		return null;
	}

	public function withMate(mate:AssemblyMate):ProjectAssemblyMates {
		for (existing in mates) if (existing.id == mate.id) throw 'A mate named "${mate.id}" already exists';
		var next = mates.copy();
		next.push(mate);
		return new ProjectAssemblyMates(connectors, next, joints);
	}

	/** This overlay without mate `id`, and without the face connectors no remaining mate names. */
	public function withoutMate(id:String):ProjectAssemblyMates {
		var next = [for (mate in mates) if (mate.id != id) mate];
		if (next.length == mates.length) throw 'There is no mate named "$id"';
		return new ProjectAssemblyMates([for (added in connectors) if (usedBy(added, next, joints)) added], next, joints);
	}

	/**
		This overlay with `conversion`'s joint (and the connectors it needs) in place of the mates it replaces; the
		face connectors only those mates named go too.
	*/
	public function withJoint(conversion:cadkit.modeling.AssemblyMateJoints.AssemblyMateJointConversion):ProjectAssemblyMates {
		var remaining = [for (mate in mates) if (conversion.mates.indexOf(mate.id) < 0) mate];
		var nextJoints = joints.copy();
		nextJoints.push(conversion.joint);
		var kept = [for (added in connectors) if (usedBy(added, remaining, nextJoints)) added];
		kept.push({component: conversion.parentComponent, connector: conversion.parentConnector});
		kept.push({component: conversion.childComponent, connector: conversion.childConnector});
		return new ProjectAssemblyMates(kept, remaining, nextJoints);
	}

	/** A mate id not yet used, from `prefix`. */
	public function freshId(prefix:String):String {
		var index = mates.length + 1;
		while (true) {
			var candidate = prefix + index;
			var taken = false;
			for (mate in mates) if (mate.id == candidate) taken = true;
			if (!taken) return candidate;
			index++;
		}
	}

	/**
		Places the parts by the generated assembly's mates and this overlay's, starting from `state` (a configuration of
		`effective(generated)`); the result keeps every coordinate the mates did not move.
	*/
	public function solve(generated:AssemblyDefinition, state:AssemblyStateRecord):{result:AssemblyMateSolveResult, state:AssemblyStateRecord} {
		var definition = effective(generated);
		var result = AssemblyMateSolver.solve(definition, state);
		return {result: result, state: AssemblyMateSolver.merge(definition, state, result.rootPoses, result.jointCoordinates)};
	}

	/** Whether a remaining mate or joint names `added` (on any occurrence of its component). */
	static function usedBy(added:AssemblyOverlayConnector, remaining:Array<AssemblyMate>, joints:Array<KinematicJoint>):Bool {
		var name = added.connector.name;
		for (mate in remaining) if (mate.firstConnector == name || mate.secondConnector == name) return true;
		for (joint in joints) if (joint.parentConnector == name || joint.childConnector == name) return true;
		return false;
	}
}
