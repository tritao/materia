import cadkit.modeling.AssemblyState;
import haxe.Json;
import haxeon.Equality;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinition.AssemblyJointRole;
import materia.assembly.AssemblyDefinition.AssemblyJointType;
import materia.assembly.AssemblyDefinitionCodec;
import materia.assembly.AssemblyDefinitionFlattener;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Nested schema, compatibility input, and solver equivalence. */
class AssemblyNestingSmoke {
	public static function run():Void {
		var nested:AssemblyDefinition = {schemaVersion: AssemblyDefinitionCodec.VERSION, id: "nested-test",
			definitions: [{id: "root-part", connectors: [{name: "tip", frame: at(10)}]}],
			occurrences: [{id: "root", definition: "root-part", initialPose: at(100)},
				{id: "module", definition: "module-design", assembly: "module-design", initialPose: at(10)}],
			joints: [{id: "join", type: AssemblyJointType.Fixed, role: AssemblyJointRole.Tree,
				parent: "root", parentConnector: "tip", child: "module", childConnector: "base",
				axis: {x: 0, y: 1, z: 0}, limits: emptyLimits(), defaultValue: 0}],
			assemblies: [{id: "module-design", definitions: [{id: "part", connectors: [
				{name: "base", frame: AssemblyFrames.identity()}, {name: "tip", frame: at(5)}]}],
				occurrences: [{id: "body", definition: "part", initialPose: AssemblyFrames.identity()}],
				joints: [], exposedConnectors: [{name: "base", occurrence: "body", connector: "base"},
					{name: "tip", occurrence: "body", connector: "tip"}]}]};
		AssemblyDefinitionCodec.validate(nested);
		var flat = AssemblyDefinitionFlattener.flatten(nested);
		if (flat.occurrences.length != 2 || flat.occurrences[1].id != "module/body" ||
			flat.joints[0].child != "module/body") throw "Nested assembly did not flatten correctly";
		var handBuilt:AssemblyDefinition = {schemaVersion: AssemblyDefinitionCodec.VERSION, id: "nested-test",
			definitions: [{id: "root-part", connectors: [{name: "tip", frame: at(10)}]},
				{id: "module/part", connectors: [{name: "base", frame: AssemblyFrames.identity()},
					{name: "tip", frame: at(5)}]}],
			occurrences: [{id: "root", definition: "root-part", initialPose: at(100), includePath: ""},
				{id: "module/body", definition: "module/part", initialPose: at(10), includePath: "module"}],
			joints: [{id: "join", type: AssemblyJointType.Fixed, role: AssemblyJointRole.Tree,
				parent: "root", parentConnector: "tip", child: "module/body", childConnector: "base",
				axis: {x: 0, y: 1, z: 0}, limits: emptyLimits(), defaultValue: 0, includePath: ""}], couplings: []};
		if (!Equality.equals(flat, handBuilt)) throw "Nested and flat assembly definitions differ";
		var restored = AssemblyDefinitionCodec.decode(AssemblyDefinitionCodec.encode(nested));
		if (!Equality.equals(AssemblyDefinitionFlattener.flatten(restored), flat))
			throw "Nested assembly JSON round trip changed the flattened definition";
		var state = new AssemblyState(restored);
		if (Math.abs(state.worldPose("module/body").x - 110) > 1e-9)
			throw "Nested assembly solved to the wrong pose";
		var saved = state.record();
		var loadedState = AssemblyDefinitionCodec.decodeState(restored, AssemblyDefinitionCodec.encodeState(restored, saved));
		if (!Equality.equals(loadedState, saved)) throw "Nested assembly state JSON round trip changed the state";
		var deeper = AssemblyDefinitionCodec.decode(AssemblyDefinitionCodec.encode(nested));
		deeper.occurrences[1].definition = "wrapper";
		deeper.occurrences[1].assembly = "wrapper";
		deeper.assemblies.push({id: "wrapper", definitions: [],
			occurrences: [{id: "inner", definition: "module-design", assembly: "module-design",
				initialPose: AssemblyFrames.identity()}], joints: [],
			exposedConnectors: [{name: "base", occurrence: "inner", connector: "base"}]});
		AssemblyDefinitionCodec.validate(deeper);
		if (Math.abs(new AssemblyState(deeper).worldPose("module/inner/body").x - 110) > 1e-9)
			throw "Two-level assembly nesting solved to the wrong pose";
		var unjoined = AssemblyDefinitionCodec.decode(AssemblyDefinitionCodec.encode(nested));
		unjoined.joints = [];
		var groupPose:materia.assembly.AssemblyDefinition.AssemblyStateRecord = {
			schemaVersion: AssemblyDefinitionCodec.VERSION, definition: unjoined.id,
			jointCoordinates: [], rootPoses: [{occurrence: "module", pose: at(200)}]};
		AssemblyDefinitionCodec.validateState(unjoined, groupPose);
		if (Math.abs(new AssemblyState(unjoined, groupPose).worldPose("module/body").x - 200) > 1e-9)
			throw "Saved subassembly root pose did not reach its flattened member";
		var legacy:Dynamic = Json.parse(Json.stringify(handBuilt));
		Reflect.setField(legacy, "schemaVersion", 1);
		reject(() -> AssemblyDefinitionCodec.decode(Json.stringify(legacy)), "old JSON assembly definition");
		var oldState:Dynamic = Json.parse(Json.stringify(saved));
		Reflect.setField(oldState, "schemaVersion", 1);
		reject(() -> AssemblyDefinitionCodec.decodeState(restored, Json.stringify(oldState)), "old JSON assembly state");
		nested.assemblies[0].exposedConnectors[0].connector = "missing";
		reject(() -> AssemblyDefinitionCodec.validate(nested), "missing connector");
		nested.assemblies[0].exposedConnectors[0].connector = "base";
		nested.assemblies[0].occurrences[0].definition = "module-design";
		nested.assemblies[0].occurrences[0].assembly = "module-design";
		reject(() -> AssemblyDefinitionCodec.validate(nested), "cycle");
	}

	static function reject(action:Void->Void, message:String):Void {
		try action() catch (_:Dynamic) return;
		throw 'Expected invalid $message to be rejected';
	}

	static function at(x:Float):AssemblyFrame
		return {x: x, y: 0, z: 0, qx: 0, qy: 0, qz: 0, qw: 1};

	static function emptyLimits():materia.assembly.AssemblyDefinition.AssemblyJointLimits
		return {lower: null, upper: null, velocity: null, effort: null};
}
