package machinekit.milling;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.assembly.Diagnostics;
import machinekit.assembly.PosedParts;
import machinekit.structural.FoundationIntegrity;
import machinekit.component.ComponentValues;
import machinekit.component.Solids;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Continuous conservative structural sweeps. Y carries the whole X axis, and
 * table members sweep the Cartesian product of both strokes against the frame.
 * Intended nut/thread engagement is checked by layout, not a solid box around the nut.
 * No claim of strength, braking distance or fastener qualification is made here.
 */
class MillXYStageIntegrity {
	public static function check(axis:MillXYStage, frame:MillFrame):Diagnostics {
		if (axis == null || frame == null || axis.layout.frame != frame.layout) throw "XY audit needs matching datums";
		var owned:Array<Part> = [], cache = new PosedParts(), out:Diagnostics;
		try out = run(axis, frame, owned, cache) catch (e:Dynamic) {
			dispose(owned);
			cache.close();
			throw e;
		}
		dispose(owned);
		cache.close();
		return out;
	}

	public static function tableMember(id:String):Bool return id == "table" || StringTools.startsWith(id, "riser") || StringTools.startsWith(id, "blockX") || id == "nutX" || id == "nutBracketX";

	public static function yFixed(id:String):Bool return !tableMember(id)
		&& id != "saddle" && id != "nutY" && id != "nutBracketY" && !StringTools.startsWith(id, "blockY") && id.indexOf("X") < 0;

	static function sweep(axis:MillXYStage, id:String, dx:Float, dy:Float, owned:Array<Part>, cache:PosedParts):Part {
		var l = axis.layout;
		if (id == "table") {
			// Split the swept ribbed section to preserve the central drive corridor.
			var pieces = [Part.box(400 + dx, l.tableDepth + dy, MillXYStageLayout.TABLE_HEIGHT)];
			for (side in [-1, 1]) pieces.push(Part.box(400 + dx, 34 + dy, MillXYStageLayout.RISER_HEIGHT)
				.translated(new Vector(0, side * l.xRailSpacing / 2, -MillXYStageLayout.RISER_HEIGHT)));
			var local = Solids.union(pieces); owned.push(local);
			var moved = local.translated(new Vector(0, l.centreY, l.tableBottom)); owned.push(moved); return moved;
		}
		if (id == "saddle") {
			// Extrude the saddle section over the full Y interval.
			var saddle = new MillCrossSaddle(axis.saddle.values().setNumber("depth", l.saddleDepth + dy).setBoolean("motorPocket", false).setBoolean("ribbed",false));
			var part = cache.posed(saddle, axis.zeroPose(id));
			owned.push(part);
			return part;
		}
		var b = cache.box(axis.component(id), axis.zeroPose(id));
		var box = Part.box(b.maxX - b.minX + dx, b.maxY - b.minY + dy, b.maxZ - b.minZ);
		owned.push(box);
		var part = box.translated(new Vector((b.minX + b.maxX) / 2,(b.minY + b.maxY) / 2, b.minZ));
		owned.push(part);
		return part;
	}

	static function run(axis:MillXYStage, frame:MillFrame, owned:Array<Part>, cache:PosedParts):Diagnostics {
		var out = new Diagnostics(), l = axis.layout, components = axis.components();
		var fixedY = [for (m in components) if (yFixed(m.id) && m.id != "screwY" && m.id != "motorShaftY" && m.id != "couplingY") m];
		var fixedX = [for (m in components) if (!tableMember(m.id) &&(m.id.indexOf("X") >= 0
			|| m.id == "saddle") && m.id != "screwX" && m.id != "motorShaftX" && m.id != "couplingX") m];
		// X moving parts against their stationary saddle and drive.
		for (m in components) if (tableMember(m.id)) {
			var moving = sweep(axis, m.id, 2 * l.xHalfTravel, 0, owned, cache);
			for (other in fixedX) {
				var fixed = cache.posed(other.component, axis.zeroPose(other.id));
				owned.push(fixed);
				noOverlap(out, moving, fixed, 'X sweep ${m.id}/${other.id}');
				fixed.close();
			}
			moving.close();
		}
		var poses:Map<String, AssemblyFrame> = frame.solvedPoses();
		var frameParts:Array<{id:String,part:Part}> = [];
		for (member in frame.components()) {
			var part = cache.posed(member.component, poses.get(member.id)); owned.push(part);
			frameParts.push({id: member.id, part: part});
		}
		// Stationary Y hardware is now inside the casting: prove the bays and shaft tunnel, too.
		for (m in components) if (yFixed(m.id)) {
			var part = cache.posed(m.component, axis.zeroPose(m.id)); owned.push(part);
			for (other in frameParts) {
				// Nominal threads intentionally engage their matching steel hatch anchor.
				if (expectedHatchThread(m.id, other.id)) continue;
				noOverlap(out, part, other.part, 'stationary Y ${m.id}/frame/${other.id}');
			}
			part.close();
		}

		for (m in components) if (!yFixed(m.id)) {
			var moving = sweep(axis, m.id, tableMember(m.id) ? 2 * l.xHalfTravel : 0, 2 * l.yHalfTravel, owned, cache);
			for (other in fixedY) {
				var fixed = cache.posed(other.component, axis.zeroPose(other.id));
				owned.push(fixed);
				noOverlap(out, moving, fixed, 'XY sweep ${m.id}/${other.id}');
				fixed.close();
			}
			for (other in frameParts) noOverlap(out, moving, other.part, 'XY sweep ${m.id}/frame/${other.id}');
			if (m.id == "saddle") {
				var shaft = cache.posed(l.yScrew, axis.zeroPose("screwY"));
				owned.push(shaft);
				noOverlap(out, moving, shaft, "Y shaft clears the complete saddle sweep");
				shaft.close();
			}
			moving.close();
		}
		// Remove the rear hatch and its screws before extracting the motor along +Y.
		var withdrawal = l.yServiceRear - l.yMotorFace + l.motor.rating.shaftLength + l.clearance;
		var extraction = [Part.cylinderAlongY(l.motor.rating.bodyDiameter / 2, l.yMotorFace,
			l.yMotorFace + l.motor.rating.bodyLength + withdrawal, 0, l.yScrewZ),
			Part.cylinderAlongY(l.motor.rating.shaftDiameter / 2, l.yMotorFace - l.motor.rating.shaftLength,
				l.yMotorFace + withdrawal, 0, l.yScrewZ)];
		for (part in extraction) {
			owned.push(part);
			for (other in frameParts) noOverlap(out, part, other.part, 'rear motor extraction/frame/${other.id}');
			for (other in components) if (yFixed(other.id) && other.id != "motorY" && other.id != "motorShaftY" &&
				other.id != "motorCoverY" && !StringTools.startsWith(other.id, "rearCoverBoltY")) {
				var fixed = cache.posed(other.component, axis.zeroPose(other.id)); owned.push(fixed);
				noOverlap(out, part, fixed, 'rear motor extraction/${other.id}'); fixed.close();
			}
		}
		if (frame.zAxis != null) {
			var zAxis = new MillZAxis(frame.zAxis);
			var zParts:Array<{id:String,part:Part}> = [];
			for (member in zAxis.components()) {
				var pose = zAxis.zeroPose(member.id), part:Part;
				if (MillZAxis.isTranslating(member.id)) {
					var b = cache.box(member.component, pose);
					var local = Part.box(b.maxX - b.minX, b.maxY - b.minY, b.maxZ - b.minZ - zAxis.layout.lower); owned.push(local);
					part = local.translated(new Vector((b.minX + b.maxX)/2, (b.minY + b.maxY)/2, b.minZ + zAxis.layout.lower));
				} else part = cache.posed(member.component, pose);
				owned.push(part); zParts.push({id:member.id,part:part});
			}
			for (member in components) {
				var part = yFixed(member.id) ? cache.posed(member.component, axis.zeroPose(member.id)) :
					sweep(axis, member.id, tableMember(member.id) ? 2*l.xHalfTravel : 0, 2*l.yHalfTravel, owned, cache);
				owned.push(part);
				for (other in zParts) noOverlap(out, part, other.part, 'XYZ structural sweep ${member.id}/zAxis/${other.id}');
				part.close();
			}
		}
		if (frame.zAxis != null && l.tableTop + l.clearance > frame.zAxis.headBottom + frame.zAxis.lower) out.error(
			"mill-xy.z-envelope",
			"table/head",
			"Table enters the lower Z head envelope"
		);
		var mounts = FoundationIntegrity.checkMounts(frame,
			[{name: "Y fixed bearing pedestal", parent: "base/driveYFixed"}, {
			name: "Y floating bearing pedestal",
			parent: "base/driveYFloating"
		}
		]);
		for (item in mounts.items) out.add(item.severity, item.code, item.subject, item.message);
		return out;
	}

	static function expectedHatchThread(bolt:String, insert:String):Bool {
		for (end in ["Front", "Rear"]) for (side in [-1, 1]) for (name in ["Lower", "Upper"]) {
			var prefix = end == "Front" ? "frontCoverBoltY" : "rearCoverBoltY";
			if (bolt == '$prefix$side$name' && insert == '${end == "Rear" ? "column" : "base"}/hatch$end${side < 0 ? "Left" : "Right"}${name}') return true;
		}
		return false;
	}

	static function noOverlap(out:Diagnostics, a:Part, b:Part, subject:String):Void {
		if (PosedParts.apart(PosedParts.boxOf(a), PosedParts.boxOf(b))) return;
		var common = a.intersect(b), volume:Float;
		try volume = common.volume() catch (e:Dynamic) {
			common.close();
			throw e;
		}
		common.close();
		if (volume > 1e-5) out.error("mill-xy.full-stroke-collision", subject, '$subject intersects ($volume mm3)');
	}

	static function dispose(parts:Array<Part>):Void for (p in parts) if (!p.shape.isClosed()) p.close();
}
