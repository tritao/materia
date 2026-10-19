package materia.tooling.panel;

import materia.assembly.AssemblyCodec;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.tooling.panel.PanelJoints.PanelJoint;
import materia.tooling.panel.PanelJoints.PanelJointPair;

typedef PanelPlacement = {
	var panel:PanelPlan;
	var pose:AssemblyFrame;
	var withdrawal:{x:Float, y:Float, z:Float};
}

typedef PanelAssemblyRecord = {
	var id:String;
	var members:Array<PanelPlacement>;
	var joints:Array<PanelJoint>;
}

/** A geometric observation supplied by machinekit.tooling, never inferred from datums. */
typedef PanelBlockingRelation = {
	var movingId:String;
	var blockerId:String;
}

/** Neutral reference checks and graph ordering. All blocking relations come from 3D sweeps. */
class PanelAssemblyPlan {
	public static function pair(pair:PanelJointPair, ?pose:AssemblyFrame):PanelAssemblyRecord {
		var base = pose == null ? F.identity() : F.compose(F.identity(), pose),
			direction = F.transformVector(base, 0, 0, 1);
		return {
			id: pair.joint.id,
			joints: [pair.joint],
			members: [
				{
					panel: pair.fixed,
					pose: base,
					withdrawal: {
						x: direction.x,
						y: direction.y,
						z: direction.z
					}
				},
				{
					panel: pair.moving,
					pose: PanelJoints.movingPose(pair.joint, base),
					withdrawal: {
						x: direction.x,
						y: direction.y,
						z: direction.z
					}
				}
			]
		};
	}

	public static function validate(plan:PanelAssemblyRecord):Void {
		PanelCodec.identifier(plan.id);
		if (plan.members == null || plan.members.length == 0 || plan.members.length > 128 || plan.joints == null || plan.joints.length > 256)
			throw "Invalid panel assembly count";
		var members = new Map<String, PanelPlacement>();
		for (m in plan.members) {
			PanelCodec.snapshot(m.panel);
			frame(m.pose);
			if (members.exists(m.panel.id))
				throw "Duplicate panel occurrence";
			var v = m.withdrawal;
			if (v == null
				|| !Math.isFinite(v.x)
				|| !Math.isFinite(v.y)
				|| !Math.isFinite(v.z)
				|| Math.abs(v.x * v.x + v.y * v.y + v.z * v.z - 1) > 1e-8)
				throw "Invalid withdrawal vector";
			members.set(m.panel.id, m);
		}
		var ids = new Map<String, Bool>();
		for (j in plan.joints) {
			PanelCodec.identifier(j.id);
			var a = members.get(j.fixedId), b = members.get(j.movingId);
			if (a == null || b == null || a == b || ids.exists(j.id))
				throw "Invalid joint references";
			ids.set(j.id, true);
			frame(j.fixedFrame);
			frame(j.movingFrame);
			if (!Math.isFinite(j.clearance) || j.clearance < 0)
				throw "Invalid panel joint clearance";
			PanelJoints.validateClearance(j.clearance, b.panel.stock.thicknessVariation, j.liningAllowance, j.machiningAllowance);
			if (j.fits == null)
				throw "Missing joint mating dimensions";
			for (fit in j.fits) {
				var operations = a.panel.operations.filter(o -> o.id == fit.fixedOperationId);
				if (operations.length != 1)
					throw "Unknown joint mating operation";
				var bounds = PanelGeometry.bounds(operations[0].contour);
				if (Math.abs(bounds.width - fit.reliefDiameter - fit.slotWidth) > 1e-7
					|| Math.abs(bounds.height - fit.reliefDiameter - fit.slotThickness) > 1e-7
					|| Math.abs(fit.tabThickness - b.panel.stock.measuredThickness) > 1e-7
					|| Math.abs(fit.slotWidth - fit.tabWidth - j.clearance) > 1e-7
					|| Math.abs(fit.slotThickness - fit.tabThickness - j.clearance) > 1e-7)
					throw "Joint slot/tab dimensions disagree: " + j.id;
				var outline = PanelContours.outside(b.panel);
				var section = PanelContours.line({x: fit.tabCenter - fit.tabWidth, y: fit.tabSectionY}, {
					x: fit.tabCenter + fit.tabWidth,
					y: fit.tabSectionY
				});
				var hits:Array<Float> = [];
				for (edge in outline)
					for (hit in PanelGeometry.intersections(section, edge))
						if (hits.filter(x -> Math.abs(x - hit.x) < 1e-7).length == 0)
							hits.push(hit.x);
				hits.sort(Reflect.compare);
				if (hits.length != 2
					|| Math.abs(hits[0] - fit.tabCenter + fit.tabWidth / 2) > 1e-7
					|| Math.abs(hits[1] - fit.tabCenter - fit.tabWidth / 2) > 1e-7)
					throw "Joint tab geometry disagrees: " + j.id;
			}
			var seat = j.seat;
			if (seat != null) {
				var operations = a.panel.operations.filter(o -> o.id == seat.operationId);
				if (operations.length != 1)
					throw "Unknown joint seat operation";
				var bounds = PanelGeometry.bounds(operations[0].contour);
				var width = seat.acrossX ? bounds.width : bounds.height;
				var length = seat.acrossX ? bounds.height : bounds.width;
				var movingBounds = PanelGeometry.bounds(PanelContours.outside(b.panel));
				if ((seat.thicknessFraction != 0.5 && seat.thicknessFraction != 1)
					|| Math.abs(width - seat.reliefDiameter - seat.openingWidth) > 1e-7
					|| Math.abs(length - seat.reliefDiameter - seat.openingLength) > 1e-7
					|| Math.abs(seat.memberWidth - b.panel.stock.measuredThickness * seat.thicknessFraction) > 1e-7
					|| Math.abs(seat.memberLength - movingBounds.width) > 1e-7
					|| Math.abs(seat.openingWidth - seat.memberWidth - seat.widthClearance) > 1e-7
					|| Math.abs(seat.openingLength - seat.memberLength - seat.lengthClearance) > 1e-7
					|| Math.abs(seat.widthClearance - j.clearance) > 1e-7)
					throw "Joint seat dimensions disagree: " + j.id;
			}
			if (j.pins == null)
				throw "Missing joint pin dimensions";
			for (pin in j.pins) {
				var fixedOps = a.panel.operations.filter(o -> o.id == pin.operationId);
				var movingOps = b.panel.operations.filter(o -> o.id == pin.operationId);
				if (fixedOps.length != 1 || movingOps.length != 1)
					throw "Unknown joint pin operation";
				var bounds = PanelGeometry.bounds(fixedOps[0].contour);
				var cx = bounds.x + bounds.width / 2,
					cy = bounds.y + bounds.height / 2;
				var expectedFixed = PanelContours.circle(cx, cy, pin.openingDiameter / 2);
				var expectedMoving = pin.travel == 0 ? expectedFixed : PanelContours.placed(PanelContours.obround(0, 0, pin.openingDiameter + pin.travel,
					pin.openingDiameter), pin.angle, cx, cy);
				if (pin.pinDiameter <= 0
					|| pin.travel < 0
					|| Math.abs(pin.openingDiameter - pin.pinDiameter - j.clearance) > 1e-7
					|| materia.tooling.ToolingCodec.canonical(expectedFixed) != materia.tooling.ToolingCodec.canonical(fixedOps[0].contour)
					|| materia.tooling.ToolingCodec.canonical(expectedMoving) != materia.tooling.ToolingCodec.canonical(movingOps[0].contour))
					throw "Joint pin dimensions disagree: " + j.id;
			}
			if (j.fits.length == 0 && seat == null && j.pins.length == 0)
				throw "Missing joint mating dimensions";
			var expected = PanelJoints.movingPose(j, a.pose),
				delta = F.compose(F.inverse(expected), b.pose);
			if (Math.abs(delta.x) + Math.abs(delta.y) + Math.abs(delta.z) > 1e-6
				|| Math.abs(delta.qx) + Math.abs(delta.qy) + Math.abs(delta.qz) > 1e-8)
				throw 'Misaligned or reversed joint ${j.id}';
		}
	}

	/** A -> B means B must be removed before A. Deterministic Kahn ordering. */
	public static function removalOrder(plan:PanelAssemblyRecord, blocking:Array<PanelBlockingRelation>):Array<String> {
		validate(plan);
		return topological([for (m in plan.members) m.panel.id], blocking);
	}

	public static function topological(sourceIds:Array<String>, blocking:Array<PanelBlockingRelation>):Array<String> {
		var ids = sourceIds.copy();
		ids.sort(Reflect.compare);
		if (blocking == null || blocking.length > ids.length * ids.length)
			throw "Invalid blocking graph";
		for (e in blocking)
			if (ids.indexOf(e.movingId) < 0 || ids.indexOf(e.blockerId) < 0)
				throw "Unknown blocking graph member";
		var done = new Map<String, Bool>(), order:Array<String> = [];
		while (order.length < ids.length) {
			var progress = false;
			for (id in ids)
				if (!done.exists(id)) {
					var ready = true;
					for (e in blocking)
						if (e.movingId == id && !done.exists(e.blockerId))
							ready = false;
					if (ready) {
						done.set(id, true);
						order.push(id);
						progress = true;
					}
				}
			if (!progress) {
				// A remaining node may merely depend on a cycle. Only mutually
				// reachable nodes belong in the cycle diagnostic.
				function reaches(start:String, target:String):Bool {
					var visited = new Map<String, Bool>();
					var pending = [start];
					while (pending.length > 0) {
						var current = pending.pop();
						if (visited.exists(current))
							continue;
						visited.set(current, true);
						for (edge in blocking)
							if (edge.movingId == current) {
								if (edge.blockerId == target)
									return true;
								pending.push(edge.blockerId);
							}
					}
					return false;
				}
				var cyclic = [for (id in ids) if (!done.exists(id) && reaches(id, id)) id];
				var edges = [
					for (edge in blocking)
						if (cyclic.indexOf(edge.movingId) >= 0 && cyclic.indexOf(edge.blockerId) >= 0 && reaches(edge.blockerId, edge.movingId)) edge.movingId
							+ " -> "
							+ edge.blockerId];
				edges.sort(Reflect.compare);
				throw "Cyclic panel blocking graph: " + cyclic.join(", ") + "; edges: " + edges.join(", ");
			}
		}
		return order;
	}

	public static function assemblyOrder(plan:PanelAssemblyRecord, blocking:Array<PanelBlockingRelation>):Array<String> {
		var order = removalOrder(plan, blocking);
		order.reverse();
		return order;
	}

	static function frame(p:AssemblyFrame):Void {
		AssemblyCodec.validateFrame(p);
		if (Math.abs(p.qx * p.qx + p.qy * p.qy + p.qz * p.qz + p.qw * p.qw - 1) > 1e-8)
			throw "Panel pose needs a unit quaternion";
	}
}
