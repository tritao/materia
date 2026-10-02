import machinekit.assembly.AssemblyPreview;
import haxe.io.Bytes;
import cadkit.modeling.AssemblyModel;
import cadkit.modeling.AssemblyState;
import cadkit.modeling.Location;
import cadkit.modeling.Part;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.robotics.EndEffectorControls;
import machinekit.welding.WeldingPowerSource;
import machinekit.welding.WeldingTorch;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.project.SceneArtifact;

/** Materia project entrypoint for the robot welding cell. */
class RobotWelderPreview {
	public static inline var ASSEMBLY_ID:String = "robot-welder";

	/**
	 * Geometry, joints and initial pose of the cell: the arm with its torch, the feeder, the power
	 * source and gas cylinder, and the table with its weldment. The scene carries no `robotTools`
	 * yet: the torch has no runtime tool kind until W2.
	 */
	public static function cell():Bytes {
		var cell = new WeldingCell();
		return SceneArtifact.encode(AssemblyPreview.scene(cell, ASSEMBLY_ID));
	}
}

/** A pose of the arm: its six joint values, in radians. */
typedef ArmPose = Array<Float>;

/**
 * Geometry builds, the services reach the torch, the torch points down at the table, its neck
 * clears the arm, and the arm reaches the weldment's seams with the torch on their bisector.
 */
class RobotWelderChecks {
	static final JOINTS = ["arm/j1", "arm/j2", "arm/j3", "arm/j4", "arm/j5", "arm/j6"];

	static function near(actual:Float, expected:Float, message:String, tolerance:Float = 1e-6):Void {
		if (!(Math.abs(actual - expected) <= tolerance))
			throw '$message: expected $expected, got $actual';
	}

	public static function run():Void {
		var scene = SceneArtifact.decode(RobotWelderPreview.cell());
		var definition = scene.assemblyDefinition;
		var cell = new WeldingCell();
		if (definition == null || definition.occurrences.length != cell.components().length)
			throw "Robot welder preview has the wrong number of occurrences";
		for (part in scene.parts) if (part.volume == null || part.volume <= 0 || part.inertia == null)
			throw 'Robot welder part "${part.id}" has no mass properties';
		if (scene.robotTools != null && scene.robotTools.length > 0) throw "The torch has no runtime tool kind before W2";

		checkServices(cell);

		var model = new AssemblyModel("mm");
		cell.addTo(model, "");
		var state = new AssemblyState(model.definition(RobotWelderPreview.ASSEMBLY_ID));
		var ready = [for (spec in cell.arm.specs) spec.initial];
		pose(state, ready);
		checkReadyPose(cell, state);
		var seams = checkReach(cell, state, ready);
		checkClearance(cell, state, ready, seams);

		var bom = cell.billOfMaterials().lines();
		var parts = [for (line in bom) line.partNumber];
		for (prefix in ["WELD-TORCH-MIG-", "WIRE-FEEDER-", "WELD-SOURCE-", "GAS-CYLINDER-", "HOSE-GAS-", "CABLE-WELD-", "HOSEPACK-"])
			if ([for (number in parts) if (StringTools.startsWith(number, prefix)) number].length == 0)
				throw 'The bill of materials should list $prefix equipment, got $parts';
		var mass = cell.massProperties().mass;
		Sys.println('robot welder: ${scene.parts.length} definitions, ${definition.occurrences.length} occurrences, ' +
			'${bom.length} BOM lines, ${Math.round(mass * 10) / 10} kg');
	}

	/** The torch's inlets are all supplied from the power source, the cylinder or the controller. */
	static function checkServices(cell:WeldingCell):Void {
		var arm = cell.arm;
		// Both the arm alone and the cell supply every required inlet, or throw.
		arm.validate();
		cell.validate();
		function chain(instance:String, port:String):String return cell.upstreamChain(instance, port).join(" < ");
		var power = chain("arm/tool/torch", "power");
		if (power.indexOf("source/weldPositive") < 0) throw 'Torch power should come from the power source, got $power';
		var gas = chain("arm/tool/torch", "gas");
		if (gas.indexOf("cylinder/gas") < 0) throw 'Torch gas should come from the cylinder, got $gas';
		if (!cell.upstream("arm/tool/torch", "control").external) throw "Torch control should come from the controller";
		if (!cell.upstream("source", "mains").external) throw "The power source should run from the wall";
		if (cell.port("mains").instanceId != "source") throw "The cell should expose the power source's mains";
		var supplies = [for (entry in cell.components()) if (Std.isOfType(entry.component, WeldingPowerSource)) entry.id];
		if (supplies.join(",") != "source") throw 'Expected one power source, got $supplies';

		// The effector derives its arc channel from the torch, to drive the arc through the feeder and source.
		var controls = EndEffectorControls.derive(arm.tool, "tool");
		if (controls.arcChannel() != "tool/torch.arc" || controls.arcs[0].tcpConnector != "tcp")
			throw 'The torch should derive an arc channel on tcp, got ${controls.arcs}';
		if (controls.vacuumChannel() != null) throw "The welding tool has no vacuum control";
	}

	static function checkReadyPose(cell:WeldingCell, state:AssemblyState):Void {
		var tcp = state.worldConnector("arm/tool/torch", "tcp");
		var wire = AssemblyFrames.transformVector(tcp, 0, 0, 1);
		var flange = state.worldPose("arm/toolFlange");
		// The wire leaves the torch along the neck's bend, 45 degrees off the flange axis, which points down.
		var flangeAxis = AssemblyFrames.transformVector(flange, 0, 0, 1);
		near(flangeAxis.z, -1, "tool axis points down in the ready pose", 1e-6);
		near(wire.x * flangeAxis.x + wire.y * flangeAxis.y + wire.z * flangeAxis.z, Math.cos(Math.PI / 4),
			"the wire leaves the torch 45 degrees off the flange axis", 1e-9);
		if (!(wire.z < -0.7)) throw 'The torch should point down toward the table, its wire is ${wire.x}, ${wire.y}, ${wire.z}';
		// The tip hangs a torch length below the flange, over the table.
		var reach = WeldingTorch.BEND_Z + WeldingTorch.NECK_LENGTH + WeldingTorch.NOZZLE_LENGTH;
		if (!(flange.z - tcp.z > 150 && flange.z - tcp.z < reach))
			throw 'The wire tip should hang below the flange, got ${Math.round(flange.z - tcp.z)} mm';
		if (!(tcp.z > WeldingCell.TABLE_TOP - 1)) throw "The wire tip should clear the table at the ready pose";
		Sys.println('robot welder: wire tip at ${Math.round(tcp.x)}, ${Math.round(tcp.y)}, ${Math.round(tcp.z)} mm, ' +
			'wire ${round2(wire.x)}, ${round2(wire.y)}, ${round2(wire.z)} in the ready pose');
	}

	/**
	 * Every seam is within reach, at its start, middle and end, with the wire on the bisector of the
	 * plate and the upright (45 degrees to both faces, into the corner). Returns the poses found.
	 */
	static function checkReach(cell:WeldingCell, state:AssemblyState, ready:ArmPose):Array<ArmPose> {
		var poses:Array<ArmPose> = [];
		var limits = [for (spec in cell.arm.specs) {lower: spec.lower, upper: spec.upper}];
		for (seam in WeldingCell.seams()) {
			// The upright's face looks away from its centre line, so the wire leans into the corner.
			var lean = seam.start.y < WeldingCell.WORK_Y ? 1.0 : -1.0;
			var direction = {x: 0.0, y: lean * Math.sqrt(0.5), z: -Math.sqrt(0.5)};
			for (fraction in [0.0, 0.5, 1.0]) {
				var target = {x: seam.start.x + fraction * (seam.stop.x - seam.start.x),
					y: seam.start.y + fraction * (seam.stop.y - seam.start.y),
					z: seam.start.z + fraction * (seam.stop.z - seam.start.z)};
				var found:Null<Array<Float>> = null;
				for (seed in seeds(ready)) if (found == null) found = ArmIk.solve(state, JOINTS, limits, target, direction, seed);
				if (found == null) throw 'The ${seam.id} seam at ${target.x}, ${target.y} is out of reach of the torch';
				poses.push(found);
			}
		}
		pose(state, ready);
		return poses;
	}

	/**
	 * At the ready pose and at each pose that reaches a seam, the torch (neck, nozzle and breakaway
	 * mount) clears every other part of the arm and the feeder, and the table, equipment and weldment;
	 * and the arm and feeder clear the table, equipment and weldment too.
	 */
	static function checkClearance(cell:WeldingCell, state:AssemblyState, ready:ArmPose, seams:Array<ArmPose>):Void {
		var moving = [for (entry in cell.components()) if (StringTools.startsWith(entry.id, "arm/")) entry.id];
		moving.push("feeder");
		var fixed = ["table", "source", "cylinder", "fixtureNear", "fixtureFar", "basePlate", "upright"];
		for (angles in [ready].concat(seams)) {
			pose(state, angles);
			for (id in moving) {
				// The tool's plate meets the flange it is bolted to, and the arm's own links meet at their joints.
				if (id != "arm/tool/torch") for (other in fixed) checkApart(cell, state, id, other);
				if (id != "arm/tool/torch" && !StringTools.startsWith(id, "arm/tool/")) checkApart(cell, state, "arm/tool/torch", id);
			}
			for (other in fixed) checkApart(cell, state, "arm/tool/torch", other);
		}
		pose(state, ready);
	}

	static function checkApart(cell:WeldingCell, state:AssemblyState, a:String, b:String):Void {
		var first = posed(cell, state, a), second = posed(cell, state, b);
		var common = first.intersect(second);
		var volume = common.volume();
		common.close();
		first.close();
		second.close();
		if (volume > 1e-3) throw '$a hits $b: ${Math.round(volume)} mm³';
	}

	/** Starting poses for the solver: the ready pose, and the arm leaning further out and in, tool kept down. */
	static function seeds(ready:ArmPose):Array<ArmPose> {
		var result = [ready];
		for (shoulder in [0.4, 0.8, 1.2]) for (elbow in [-1.4, -0.9, -0.4]) for (turn in [0.0, 0.5, -0.5])
			result.push([turn, shoulder, elbow, 0, Math.PI - (shoulder - elbow), 0]);
		return result;
	}

	static function pose(state:AssemblyState, angles:ArmPose):Void {
		for (i in 0...JOINTS.length) state.setJoint(JOINTS[i], angles[i]);
		state.forwardKinematics();
	}

	static function round2(value:Float):Float return Math.round(value * 100) / 100;

	static function posed(cell:WeldingCell, state:AssemblyState, id:String):Part {
		for (entry in cell.components()) if (entry.id == id) {
			var frame:AssemblyFrame = state.worldPose(id);
			var x = AssemblyFrames.transformVector(frame, 1, 0, 0), z = AssemblyFrames.transformVector(frame, 0, 0, 1);
			var local = entry.component.geometry(ComponentDetail.Preview);
			var placed = local.placed(new Location(new Plane(new Vector(frame.x, frame.y, frame.z), new Vector(x.x, x.y, x.z),
				new Vector(z.x, z.y, z.z))));
			local.close();
			return placed;
		}
		throw 'Robot welder has no member "$id"';
	}
}

/**
 * A damped least-squares inverse kinematics solver for the cell's arm, by finite differences on its
 * forward kinematics: it brings the wire tip to a point with the wire along a direction, from a
 * seed pose, within the joint limits. Enough to ask whether a seam can be reached; a runtime planner
 * does this properly.
 */
class ArmIk {
	/** Weight of one unit of direction error against one millimetre of position error. */
	static inline var DIRECTION_WEIGHT:Float = 200;

	/** The joint values that put the tip on `target` with the wire along `direction`, or null. */
	public static function solve(state:AssemblyState, joints:Array<String>, limits:Array<{lower:Float, upper:Float}>,
			target:{x:Float, y:Float, z:Float}, direction:{x:Float, y:Float, z:Float}, seed:Array<Float>):Null<Array<Float>> {
		var q = [for (j in 0...joints.length) Math.min(limits[j].upper, Math.max(limits[j].lower, seed[j]))];
		var residual = evaluate(state, joints, q, target, direction);
		var damping = 1.0;
		for (iteration in 0...120) {
			if (norm(residual) < 0.05) return q;
			var jacobian:Array<Array<Float>> = [];
			for (j in 0...joints.length) {
				var step = q.copy();
				var h = q[j] + 1e-4 <= limits[j].upper ? 1e-4 : -1e-4;
				step[j] += h;
				var moved = evaluate(state, joints, step, target, direction);
				jacobian.push([for (k in 0...6) (moved[k] - residual[k]) / h]);
			}
			// (J^T J + damping I) delta = -J^T r, with jacobian[j][k] = d residual k / d joint j.
			var normal:Array<Array<Float>> = [];
			for (a in 0...joints.length) {
				var row:Array<Float> = [];
				for (b in 0...joints.length) {
					var sum = a == b ? damping : 0.0;
					for (k in 0...6) sum += jacobian[a][k] * jacobian[b][k];
					row.push(sum);
				}
				var rhs = 0.0;
				for (k in 0...6) rhs -= jacobian[a][k] * residual[k];
				row.push(rhs);
				normal.push(row);
			}
			var delta = gauss(normal);
			var next = [for (j in 0...joints.length) Math.min(limits[j].upper, Math.max(limits[j].lower, q[j] + delta[j]))];
			var nextResidual = evaluate(state, joints, next, target, direction);
			if (norm(nextResidual) < norm(residual)) {
				q = next;
				residual = nextResidual;
				damping = Math.max(1e-6, damping * 0.5);
			} else damping *= 4;
		}
		return norm(residual) < 0.5 ? q : null;
	}

	static function evaluate(state:AssemblyState, joints:Array<String>, q:Array<Float>, target:{x:Float, y:Float, z:Float},
			direction:{x:Float, y:Float, z:Float}):Array<Float> {
		for (i in 0...joints.length) state.setJoint(joints[i], q[i]);
		state.forwardKinematics();
		var tcp = state.worldConnector("arm/tool/torch", "tcp");
		var wire = AssemblyFrames.transformVector(tcp, 0, 0, 1);
		return [tcp.x - target.x, tcp.y - target.y, tcp.z - target.z, DIRECTION_WEIGHT * (wire.x - direction.x),
			DIRECTION_WEIGHT * (wire.y - direction.y), DIRECTION_WEIGHT * (wire.z - direction.z)];
	}

	static function norm(values:Array<Float>):Float {
		var sum = 0.0;
		for (value in values) sum += value * value;
		return Math.sqrt(sum);
	}

	/** Solves an augmented n x (n+1) system by elimination with partial pivoting. */
	static function gauss(matrix:Array<Array<Float>>):Array<Float> {
		var n = matrix.length;
		for (column in 0...n) {
			var pivot = column;
			for (row in column + 1...n) if (Math.abs(matrix[row][column]) > Math.abs(matrix[pivot][column])) pivot = row;
			var swap = matrix[column];
			matrix[column] = matrix[pivot];
			matrix[pivot] = swap;
			for (row in column + 1...n) {
				var factor = matrix[row][column] / matrix[column][column];
				for (k in column...n + 1) matrix[row][k] -= factor * matrix[column][k];
			}
		}
		var result = [for (i in 0...n) 0.0];
		var i = n - 1;
		while (i >= 0) {
			var sum = matrix[i][n];
			for (k in i + 1...n) sum -= matrix[i][k] * result[k];
			result[i] = sum / matrix[i][i];
			i--;
		}
		return result;
	}
}

/** Standalone check of the robot welder example. */
function main():Void RobotWelderChecks.run();
