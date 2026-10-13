package machinekit.pressbrake;

import NativeKitSim;
import cadkit.modeling.Location;
import cadkit.sheetmetal.ManufacturedSheetState;
import cadkit.sheetmetal.Sheet;
import cadkit.sheetmetal.SheetAngles;
import cadkit.sheetmetal.SheetDiagnosis;
import cadkit.sheetmetal.SheetEvaluation;
import cadkit.sheetmetal.process.AirBending;
import cadkit.sheetmetal.process.ProcessTooling;
import machinekit.pressbrake.planning.BendOrientation;
import machinekit.pressbrake.planning.ManufacturedSheetMotion;
import machinekit.pressbrake.planning.PressSheetPose;
import machinekit.pressbrake.planning.PressStationGeometry;
import machinekit.pressbrake.planning.SheetBodyGeometry;
import materia.project.MaterialDef.MaterialMechanical;
import nativekit.sim.MotionType;
import nativekit.sim.SimFixedJoint;
import nativekit.sim.SimInertialProperties;
import nativekit.sim.SimObject;
import nativekit.sim.SimPose;
import nativekit.sim.SimSession;
import nativekit.sim.SimShape;
import robotkit.spatial.Quat;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;

/** Physical developed stock: free rigid bodies outside the press, measured-depth motion inside it. */
class SimulatedSheet {
	public final root:SimObject;
	public final mass:Float;
	final session:SimSession;
	final sheet:Sheet;
	final density:Float;
	final mechanical:MaterialMechanical;
	final bodies:Array<SheetBody> = [];
	final joints:Array<SimFixedJoint> = [];
	final reference:Transform3;
	var actual:ManufacturedSheetState;
	var stroke:Null<SheetStroke> = null;
	var disposed:Bool = false;

	/** Prepare stock at first tool contact using the canonical CAD-to-physics placement. */
	public static function atContact(session:SimSession, sheet:Sheet, initial:ManufacturedSheetState,
			bend:Int, tooling:ProcessTooling, station:PressStationGeometry, orientation:BendOrientation,
			pressFrame:Transform3, density:Float, maximumError:Float = 0.05,
			?actualMechanical:MaterialMechanical):SimulatedSheet {
		var pose = PressSheetPose.manufactured(sheet, initial, bend, tooling, station, orientation);
		return new SimulatedSheet(session, sheet, initial, pressFrame.compose(transform(pose.placement)),
			density, maximumError, actualMechanical);
	}

	/** Density is explicit kg/m³. The optional actual lot changes recovery, never the authored blank. */
	public function new(session:SimSession, sheet:Sheet, initial:SheetAngles, placement:Transform3,
			density:Float, maximumError:Float = 0.05, ?actualMechanical:MaterialMechanical) {
		if (!Math.isFinite(density) || density <= 0)
			throw new SheetDiagnosis("press.stock-density", "Physical stock needs positive density in kg/m³");
		this.session = session; this.sheet = sheet.copy(); this.density = density;
		var selected = actualMechanical == null ? sheet.material.mechanical : actualMechanical;
		if (selected == null) throw new SheetDiagnosis("sheet.no-mechanical-data", "Physical stock needs strength and elastic modulus");
		if (!Math.isFinite(selected.uts) || !Math.isFinite(selected.yield) || !Math.isFinite(selected.youngs)
			|| selected.yield <= 0 || selected.uts < selected.yield || selected.youngs <= selected.yield)
			throw new SheetDiagnosis("sheet.invalid-mechanical-data", "Actual stock needs positive yield, tensile strength above yield and modulus above yield");
		mechanical = {uts: selected.uts, yield: selected.yield, youngs: selected.youngs};
		actual = new ManufacturedSheetState(initial);
		var evaluation = this.sheet.evaluateManufactured(actual);
		var total = 0.0;
		// Body frames and collision error use the same physical resolution on every rebuild.
		var geometry = SheetBodyGeometry.of(this.sheet, Math.PI, maximumError);
		var placements = [for (part in geometry) placement.compose(transform(part.frame(evaluation)))];
		session.stop(); editTopology(() -> {
			for (i in 0...geometry.length) {
				var part = geometry[i], object = createBody(part, placements[i]);
				bodies.push(new SheetBody(part, object));
			}
		});
		for (part in geometry) total += part.properties.mass(density * 1e-9);
		root = bodies[0].object;
		reference = observed(root);
		mass = total;
		connect();
	}

	public function state():ManufacturedSheetState return new ManufacturedSheetState(actual);
	public function wallObject(id:Int):SimObject {
		for (body in bodies) if (body.geometry.wall == id) return body.object;
		throw new SheetDiagnosis("sheet.missing-wall", "Physical sheet has no wall body", ["wall:" + id]);
	}
	/** Measured world placement of the authored wall origin, in metres. */
	public function wallFrame(id:Int):Transform3 {
		ensureLive();
		for (body in bodies) if (body.geometry.wall == id) {
			var center = body.geometry.properties.centerOfMass;
			return observed(body.object).compose(new Transform3(
				new Vec3(-center.x * 0.001, -center.y * 0.001, -center.z * 0.001), Quat.identity()));
		}
		throw new SheetDiagnosis("sheet.missing-wall", "Physical sheet has no wall frame", ["wall:" + id]);
	}

	/** The caller owns the clock and reads ram depth; this transfers motion ownership to the press carrier. */
	public function beginStroke(bend:Int, tooling:ProcessTooling, station:PressStationGeometry,
			orientation:BendOrientation, pressFrame:Transform3, carrier:nksim_body):Void {
		ensureLive();
		if (stroke != null) throw new SheetDiagnosis("press.sheet-stroke", "Sheet already has an active stroke");
		if (actual.angle(bend) != 0) throw new SheetDiagnosis("press.sheet-rebend", "A new stroke requires an unbent active strip", ["bend:" + bend]);
		var motion = new ManufacturedSheetMotion(sheet, actual, bend, tooling, station, orientation);
		var first = motion.atDepth(motion.contactDepth);
		session.stop(); disconnect();
		stroke = new SheetStroke(motion, pressFrame, carrier);
		drive(first);
	}

	/** Sample measured ram depth before the next shared physics tick; no elapsed-time trajectory is used. */
	public function driveAtDepth(depth:Float):Void {
		ensureLive(); var active = stroke;
		if (active == null) throw new SheetDiagnosis("press.sheet-stroke", "Sheet has no active stroke");
		drive(active.motion.atDepth(depth));
	}

	function drive(pose:PressSheetPose):Void {
		var active = stroke;
		if (active == null) throw new SheetDiagnosis("press.sheet-stroke", "Sheet has no active stroke");
		var desired:Array<Transform3> = [];
		for (body in bodies) desired.push(active.pressFrame.compose(transform(pose.placement.compose(body.geometry.frame(pose.evaluation)))));
		var rootPose = desired[0], carrierPose = fromPose(session.bodyPose(active.carrier));
		session.holdObject(root, active.carrier, toPose(carrierPose.inverse().compose(rootPose)));
		for (i in 1...bodies.length) session.holdObjectFromObject(bodies[i].object, root,
			toPose(rootPose.inverse().compose(desired[i])));
		actual.set(active.motion.bend, pose.evaluation.bend(active.motion.bend).angle);
	}

	/** Signed angle from native wall normals and the measured wall-A bend axis. */
	public function measuredAngle(id:Int):Float {
		ensureLive(); var bend = sheet.bend(id);
		var a = observed(wallObject(bend.wallA)), b = observed(wallObject(bend.wallB));
		var normalA = a.rotation.rotate(new Vec3(0, 0, 1)), normalB = b.rotation.rotate(new Vec3(0, 0, 1));
		var edge = Sheet.edgeFrame(bend.edgeA.span).plane.yDirection;
		var axis = a.rotation.rotate(new Vec3(edge.x, edge.y, edge.z));
		return Math.atan2(-axis.dot(normalA.cross(normalB)), normalA.dot(normalB));
	}

	/** Spring back the measured loaded angle, rebuild child reference frames once, then release the free part. */
	public function release():Float {
		ensureLive(); var active = stroke;
		if (active == null) throw new SheetDiagnosis("press.sheet-stroke", "Sheet has no active stroke");
		var angle = measuredAngle(active.motion.bend), bend = sheet.bend(active.motion.bend);
		var length = sheet.evaluate(sheet.flat()).bend(bend.id).allowance;
		var offset = length / Math.abs(bend.angle) - bend.radius;
		var released = 0.0;
		if (angle != 0) {
			var radius = length / Math.abs(angle) - offset;
			// The plastic relation tends to zero residual curvature at its elastic boundary.
			if (radius * mechanical.yield / (mechanical.youngs * sheet.thickness()) < 0.5) {
				var finalRadius = AirBending.unloadRadius(radius, sheet.thickness(), mechanical);
				released = (angle > 0 ? 1 : -1) * length / (finalRadius + offset);
			}
		}
		var next = new ManufacturedSheetState(actual).set(bend.id, released);
		var evaluation = sheet.evaluateManufactured(next);
		var rootFrame = transform(bodies[0].geometry.frame(evaluation));
		session.stop(); editTopology(() -> {
			for (i in 1...bodies.length) {
				var body = bodies[i];
				if (session.objectCarrier(body.object).rawValue() != 0) session.releaseObject(body.object);
				body.remove();
				var local = rootFrame.inverse().compose(transform(body.geometry.frame(evaluation)));
				body.object = createBody(body.geometry, reference.compose(local));
				joints.push(session.createFixedJoint(root, body.object));
			}
		});
		session.releaseObject(root); actual = next; stroke = null;
		return released;
	}

	public function dispose():Void {
		if (disposed) return;
		session.stop(); disconnect();
		for (body in bodies) if (body.present() && session.objectCarrier(body.object).rawValue() != 0)
			session.releaseObject(body.object);
		editTopology(() -> { for (body in bodies) body.remove(); });
		disposed = true;
	}

	function connect():Void {
		editTopology(() -> { for (i in 1...bodies.length) joints.push(session.createFixedJoint(root, bodies[i].object)); });
	}
	function disconnect():Void {
		editTopology(() -> { for (joint in joints) joint.dispose(); joints.resize(0); });
	}
	function editTopology(edit:Void->Void):Void {
		session.beginTopologyUpdate();
		try { edit(); }
		catch (failure:Dynamic) {
			try { session.endTopologyUpdate(); }
			catch (cleanup:Dynamic) {
				throw new SheetDiagnosis("press.sheet-topology", "Topology edit failed: " + Std.string(failure)
					+ "; closing the batch also failed: " + Std.string(cleanup));
			}
			throw failure;
		}
		session.endTopologyUpdate();
	}
	function createBody(geometry:SheetBodyGeometry, placement:Transform3):SimObject {
		var vertices = [for (piece in geometry.pieces()) [for (point in piece.vertices())
			for (coordinate in [point.x, point.y, point.z]) coordinate * 0.001]];
		var tensor = geometry.properties.inertia.scaled(density * 1e-15);
		var inertial = new SimInertialProperties([0, 0, 0],
			[tensor.xx, tensor.xy, tensor.xz, tensor.xy, tensor.yy, tensor.yz, tensor.xz, tensor.yz, tensor.zz]);
		return session.createObject(MotionType.Dynamic, SimShape.compoundConvex(vertices), toPose(placement),
			geometry.properties.mass(density * 1e-9), null, null, inertial);
	}
	function observed(object:SimObject):Transform3 return fromPose(session.objectPose(object));
	function ensureLive():Void { if (disposed) throw new SheetDiagnosis("press.sheet-disposed", "Physical sheet has been disposed"); }
	static function transform(location:Location):Transform3 {
		var p = location.plane, x = p.xDirection, y = p.yDirection, z = p.normal;
		return new Transform3(new Vec3(p.origin.x * 0.001, p.origin.y * 0.001, p.origin.z * 0.001),
			Quat.fromRotationMatrix([x.x, x.y, x.z, y.x, y.y, y.z, z.x, z.y, z.z]));
	}
	static function toPose(value:Transform3):SimPose {
		var p = value.translation, q = value.rotation;
		return new SimPose(p.x, p.y, p.z, q.x, q.y, q.z, q.w);
	}
	static function fromPose(value:SimPose):Transform3
		return new Transform3(new Vec3(value.x, value.y, value.z), new Quat(value.qx, value.qy, value.qz, value.qw));
}

private class SheetBody {
	public final geometry:SheetBodyGeometry;
	var current:Null<SimObject>;
	public var object(get, set):SimObject;
	public function new(geometry:SheetBodyGeometry, object:SimObject) { this.geometry = geometry; current = object; }
	function get_object():SimObject {
		var value = current;
		if (value == null) throw new SheetDiagnosis("press.sheet-body", "Physical patch has no live body", [geometry.id]);
		return value;
	}
	function set_object(value:SimObject):SimObject { current = value; return value; }
	public function present():Bool return current != null;
	public function remove():Void {
		var value = current;
		if (value != null) { value.dispose(); current = null; }
	}
}
private class SheetStroke {
	public final motion:ManufacturedSheetMotion;
	public final pressFrame:Transform3;
	public final carrier:nksim_body;
	public function new(motion:ManufacturedSheetMotion, pressFrame:Transform3, carrier:nksim_body) {
		this.motion = motion; this.pressFrame = pressFrame; this.carrier = carrier;
	}
}
