import cadkit.modeling.AssemblyState;
import cadkit.parametric.AssemblyDocuments;
import cadkit.parametric.Document;
import cadkit.parametric.DocumentCodec;
import haxeon.Equality;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinition.AssemblyJointRole;
import materia.assembly.AssemblyDefinition.AssemblyJointType;
import materia.assembly.AssemblyDefinition.AssemblyJointCoupling;
import materia.assembly.AssemblyDefinitionCodec;
import materia.assembly.AssemblyFrames;

/** A joint that sums two leaders (a CoreXY motor follows both axes): validation, propagation and documents. */
class AssemblyCouplingSmoke {
	public static function run():Void {
		var terms:Array<AssemblyJointCoupling> = [{id: "m-x", source: "x", target: "m", ratio: 1, offset: 0},
			{id: "m-y", source: "y", target: "m", ratio: 1, offset: 0.5}];
		var definition = build(terms);
		AssemblyDefinitionCodec.validate(definition);
		var state = new AssemblyState(definition);
		state.setJoint("x", 10);
		check(state.joint("m") == 10.5, 'm sums its terms after x moves: ${state.joint("m")}');
		state.setJoint("y", 4);
		check(state.joint("m") == 14.5, 'm sums its terms after y moves: ${state.joint("m")}');
		state.forwardKinematics();
		throws(() -> state.setJoint("m", 1), "a joint with coupling terms is never set directly");
		// A single-leader coupling still reads and writes exactly as before.
		var single = build([{id: "m-x", source: "x", target: "m", ratio: 2, offset: 0}]);
		AssemblyDefinitionCodec.validate(single);
		var singleState = new AssemblyState(single);
		singleState.setJoint("x", 3);
		check(singleState.joint("m") == 6, "one term keeps ratio times leader");
		// The sum survives a document round trip.
		var document = new Document();
		var root = AssemblyDocuments.fromDefinition(document, definition);
		var reopened = DocumentCodec.decode(DocumentCodec.encode(document), false, false);
		var rebuilt = AssemblyDocuments.toDefinition(reopened.element(root.id));
		var saved = rebuilt.couplings;
		check(saved != null && saved.length == 2, "both terms are saved as relationships");
		reopened.close();
		document.close();
		// A leader appears once per target; terms never form a cycle; ratios are finite and non-zero.
		throws(() -> AssemblyDefinitionCodec.validate(build([terms[0], {id: "again", source: "x", target: "m", ratio: 2, offset: 0}])),
			"the same leader twice is rejected");
		throws(() -> AssemblyDefinitionCodec.validate(build([terms[0], {id: "m-x2", source: "y", target: "x", ratio: 1, offset: 0},
			{id: "x-m", source: "m", target: "y", ratio: 1, offset: 0}])), "a cycle through several terms is rejected");
		throws(() -> AssemblyDefinitionCodec.validate(build([terms[0], {id: "zero", source: "y", target: "m", ratio: 0, offset: 0}])),
			"a zero ratio is rejected");
	}

	static function build(couplings:Array<AssemblyJointCoupling>):AssemblyDefinition {
		var frame = AssemblyFrames.identity();
		var limits:materia.assembly.AssemblyDefinition.AssemblyJointLimits =
			{lower: -100.0, upper: 100.0, velocity: null, effort: null, overtravel: null, acceleration: null};
		function slide(id:String, parent:String, child:String, axis:{x:Float, y:Float, z:Float}):KinematicJoint
			return {id: id, type: AssemblyJointType.Prismatic, role: AssemblyJointRole.Tree, parent: parent,
				parentConnector: "c", child: child, childConnector: "c", axis: axis, limits: limits, defaultValue: 0};
		return {schemaVersion: AssemblyDefinitionCodec.VERSION, id: "sum",
			definitions: [{id: "part", connectors: [{name: "c", frame: frame}]}],
			occurrences: [for (id in ["root", "a", "b", "motor"]) {id: id, definition: "part", initialPose: frame}],
			joints: [slide("x", "root", "a", {x: 1, y: 0, z: 0}), slide("y", "a", "b", {x: 0, y: 1, z: 0}),
				{id: "m", type: AssemblyJointType.Revolute, role: AssemblyJointRole.Tree, parent: "root", parentConnector: "c",
					child: "motor", childConnector: "c", axis: {x: 0, y: 0, z: 1}, limits: limits, defaultValue: 0.5}],
			couplings: couplings};
	}

	static function check(condition:Bool, message:String):Void {
		if (!condition) throw message;
	}

	static function throws(action:() -> Void, message:String):Void {
		var threw = false;
		try action() catch (error:Dynamic) threw = true;
		if (!threw) throw message;
	}
}
