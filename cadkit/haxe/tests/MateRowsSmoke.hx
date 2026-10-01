import cadkit.modeling.AssemblyKinematics;
import cadkit.solve.JacobianCheck;
import kinematicskit.ClosureTask;
import kinematicskit.KinematicProblem;
import kinematicskit.KinematicSnapshot;
import kinematicskit.KinematicState;
import kinematicskit.LevenbergMarquardt;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinition.AssemblyComponentDefinition;
import materia.assembly.AssemblyDefinition.AssemblyJointRole;
import materia.assembly.AssemblyDefinition.KinematicJoint;
import materia.assembly.AssemblyDefinition.AssemblyJointType;
import materia.assembly.AssemblyDefinition.AssemblyMate;
import materia.assembly.AssemblyDefinition.AssemblyMateKind;
import materia.assembly.AssemblyDefinitionCodec;
import materia.assembly.AssemblyDefinitionFlattener;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/**
	Every mate kind compiles to closure rows that a solve can satisfy and that
	match central differences. The mated part hangs from ground on three slides
	and three hinges, so its pose is plain joint coordinates.
*/
class MateRowsSmoke {
	public static function run():Void {
		for (kind in [AssemblyMateKind.Coincident, AssemblyMateKind.Coaxial, AssemblyMateKind.Planar, AssemblyMateKind.Parallel,
			AssemblyMateKind.Perpendicular, AssemblyMateKind.Distance, AssemblyMateKind.Angle, AssemblyMateKind.Lock]) {
			var value:Null<Float> = switch kind {
				case AssemblyMateKind.Planar: 15.0;
				case AssemblyMateKind.Distance: 120.0;
				case AssemblyMateKind.Angle: 0.6;
				default: null;
			};
			solveAndCheck(kind, value);
		}
		checkCodec();
		checkNesting();
	}

	static function solveAndCheck(kind:AssemblyMateKind, value:Null<Float>):Void {
		var definition = floatingPart();
		var mate:AssemblyMate = {id: "mate", kind: kind, first: "ground", firstConnector: "g", second: "part", secondConnector: "p",
			axis: {x: 0, y: 0, z: 1}};
		if (value != null) mate.value = value;
		definition.mates = [mate];
		AssemblyDefinitionCodec.validate(definition);
		var model = AssemblyKinematics.compile(AssemblyDefinitionFlattener.flatten(definition), true).model;
		check(model.closureCount() == 1, '$kind compiles to one closure');
		var dofs = [for (dof in 0...model.dofCount()) dof];
		var problem = new KinematicProblem(model).setActiveDofs(dofs).add(new ClosureTask(model, 0, 1e-6, 1e-8));
		var seed = new KinematicState(model);
		var solution = LevenbergMarquardt.solve(problem, seed, 200, 1e-3, 1e-8, 100, null, true);
		check(solution.converged(), '$kind is satisfiable from a generic pose (${solution.status})');

		var snapshot = new KinematicSnapshot(model), rows = problem.rowCount(), width = problem.layout().width;
		var evaluate = (x:Array<Float>) -> {
			var trial = solution.state.copy();
			for (i in 0...dofs.length) trial.q[dofs[i]] = x[i];
			var residual = [for (_ in 0...rows) 0.0], jacobian = [for (_ in 0...rows * width) 0.0];
			problem.evaluate(trial, snapshot, residual, jacobian);
			return {residual: residual, jacobian: jacobian};
		};
		// Tasks return residual −value with Jacobian ∂value/∂q; checked where the rows are exact (at the solution).
		var result = JacobianCheck.compare(x -> [for (r in evaluate(x).residual) -r], x -> evaluate(x).jacobian,
			[for (dof in dofs) solution.state.q[dof]], 1e-5);
		if (!result.passed) throw '$kind mate rows: $result';
	}

	/** Ground, and a part reached through slides x, y, z and hinges z, y, x, starting away from any mate. */
	static function floatingPart():AssemblyDefinition {
		var links = ["sx", "sy", "sz", "rz", "ry", "part"];
		var definitions:Array<AssemblyComponentDefinition> = [{id: "ground", connectors: [{name: "a", frame: frame(0, 0, 0)}, {name: "g", frame: frame(100, 50, 25, 0.4)}]}];
		for (id in links)
			definitions.push({id: id, connectors: id == "part"
				? [{name: "a", frame: frame(0, 0, 0)}, {name: "b", frame: frame(0, 0, 0)}, {name: "p", frame: frame(30, 20, 10, -0.3)}]
				: [{name: "a", frame: frame(0, 0, 0)}, {name: "b", frame: frame(0, 0, 0)}]});
		var axes = [{x: 1.0, y: 0.0, z: 0.0}, {x: 0.0, y: 1.0, z: 0.0}, {x: 0.0, y: 0.0, z: 1.0}, {x: 0.0, y: 0.0, z: 1.0},
			{x: 0.0, y: 1.0, z: 0.0}, {x: 1.0, y: 0.0, z: 0.0}];
		var starts = [5.0, -3.0, 2.0, 0.2, -0.15, 0.3];
		var parents = ["ground", "sx", "sy", "sz", "rz", "ry"];
		var joints:Array<KinematicJoint> = [for (i in 0...links.length) {
			id: "j" + i, type: i < 3 ? AssemblyJointType.Prismatic : AssemblyJointType.Revolute, role: AssemblyJointRole.Tree,
			parent: parents[i], parentConnector: i == 0 ? "a" : "b", child: links[i], childConnector: "a", axis: axes[i],
			limits: {lower: null, upper: null, velocity: null, effort: null}, defaultValue: starts[i]
		}];
		return {schemaVersion: AssemblyDefinitionCodec.VERSION, id: "floating", lengthUnit: "mm", definitions: definitions,
			occurrences: [for (definition in definitions) {id: definition.id, definition: definition.id, initialPose: frame(0, 0, 0)}],
			joints: joints};
	}

	static function checkCodec():Void {
		var definition = floatingPart();
		definition.mates = [{id: "seat", kind: AssemblyMateKind.Distance, first: "ground", firstConnector: "g", second: "part",
			secondConnector: "p", axis: {x: 0, y: 0, z: 1}, value: 40}];
		definition.occurrences[0].grounded = true;
		var decoded = AssemblyDefinitionCodec.decode(AssemblyDefinitionCodec.encode(definition));
		var decodedMates = decoded.mates;
		check(decodedMates != null && decodedMates.length == 1 && decodedMates[0].value == 40.0 && decoded.occurrences[0].grounded == true,
			"mates and grounded occurrences survive the codec");
		var invalid:Array<{label:String, change:AssemblyMate->Void}> = [
			{label: "a distance mate without a value", change: m -> { m.value = null; }},
			{label: "a negative distance", change: m -> { m.value = -1; }},
			{label: "a missing connector", change: m -> { m.secondConnector = "nowhere"; }},
			{label: "a mate of a part with itself", change: m -> { m.second = "ground"; }},
			{label: "a mate id that is also a joint id", change: m -> { m.id = "j0"; }},
		];
		for (entry in invalid) {
			var copy = AssemblyDefinitionCodec.decode(AssemblyDefinitionCodec.encode(definition));
			var copyMates = copy.mates;
			if (copyMates != null) entry.change(copyMates[0]);
			var threw = false;
			try AssemblyDefinitionCodec.validate(copy) catch (_:Dynamic) threw = true;
			check(threw, '${entry.label} is refused');
		}
	}

	/** A mate inside a nested assembly is scoped like its joints, and its connectors resolve through the nesting. */
	static function checkNesting():Void {
		var nested:AssemblyDefinition = {schemaVersion: AssemblyDefinitionCodec.VERSION, id: "nested", lengthUnit: "mm",
			definitions: [{id: "plate", connectors: [{name: "top", frame: frame(0, 0, 5)}]}],
			occurrences: [{id: "base", definition: "plate", initialPose: frame(0, 0, 0)},
				{id: "module", definition: "pair", assembly: "pair", initialPose: frame(50, 0, 0)}],
			joints: [],
			assemblies: [{id: "pair", definitions: [{id: "block", connectors: [{name: "face", frame: frame(0, 0, 0)}]}],
				occurrences: [{id: "left", definition: "block", initialPose: frame(0, 0, 0)},
					{id: "right", definition: "block", initialPose: frame(10, 0, 0)}],
				joints: [],
				mates: [{id: "flush", kind: AssemblyMateKind.Planar, first: "left", firstConnector: "face", second: "right",
					secondConnector: "face", axis: {x: 0, y: 0, z: 1}}],
				exposedConnectors: [{name: "face", occurrence: "left", connector: "face"}]}],
			mates: [{id: "seat", kind: AssemblyMateKind.Coincident, first: "base", firstConnector: "top", second: "module",
				secondConnector: "face", axis: {x: 0, y: 0, z: 1}}]};
		AssemblyDefinitionCodec.validate(nested);
		var flat = AssemblyDefinitionFlattener.flatten(nested);
		var flatMates = flat.mates == null ? [] : flat.mates;
		var mates = [for (mate in flatMates) mate.id + ":" + mate.first + "/" + mate.firstConnector + "-" + mate.second + "/" + mate.secondConnector];
		mates.sort(Reflect.compare);
		check(mates.join(" ") == "module/flush:module/left/face-module/right/face seat:base/top-module/left/face",
			'nested mates are scoped and resolved: $mates');
	}

	static function frame(x:Float, y:Float, z:Float, ?angle:Float = 0.0):AssemblyFrame
		return {x: x, y: y, z: z, qx: 0, qy: 0, qz: Math.sin(angle / 2), qw: Math.cos(angle / 2)};

	static function check(value:Bool, label:String):Void {
		if (!value) throw label;
	}
}
