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

	static function tableMember(id:String):Bool return id == "table" || StringTools.startsWith(id, "riser") || StringTools.startsWith(id, "blockX") || id == "nutX" || id == "nutBracketX";

	static function yFixed(id:String):Bool return !tableMember(id)
		&& id != "saddle" && id != "nutY" && id != "nutBracketY" && !StringTools.startsWith(id, "blockY") && id.indexOf("X") < 0;

	static function sweep(axis:MillXYStage, id:String, dx:Float, dy:Float, owned:Array<Part>, cache:PosedParts):Part {
		var l = axis.layout;
		if (id == "table") {
			// Split the swept ribbed section to preserve the central drive corridor.
			var pieces = [Part.box(400 + dx, 200 + dy, MillXYStageLayout.TABLE_HEIGHT)];
			for (side in [-1, 1]) pieces.push(Part.box(400 + dx, 34 + dy, MillXYStageLayout.RISER_HEIGHT)
				.translated(new Vector(0, side * l.xRailSpacing / 2, -MillXYStageLayout.RISER_HEIGHT)));
			var local = Solids.union(pieces); owned.push(local);
			var moved = local.translated(new Vector(0, l.centreY, l.tableBottom)); owned.push(moved); return moved;
		}
		if (id == "saddle") {
			// Extrude the saddle section and its through tunnel over the full Y interval.
			var saddle = new MillCrossSaddle(new ComponentValues().setNumber("depth", l.saddleDepth + dy).setBoolean("motorPocket", false));
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
		for (m in components) if (!yFixed(m.id)) {
			var moving = sweep(axis, m.id, tableMember(m.id) ? 2 * l.xHalfTravel : 0, 2 * l.yHalfTravel, owned, cache);
			for (other in fixedY) {
				var fixed = cache.posed(other.component, axis.zeroPose(other.id));
				owned.push(fixed);
				noOverlap(out, moving, fixed, 'XY sweep ${m.id}/${other.id}');
				fixed.close();
			}
			for (other in frame.components()) {
				var fixed = cache.posed(other.component, poses.get(other.id));
				owned.push(fixed);
				noOverlap(out, moving, fixed, 'XY sweep ${m.id}/frame/${other.id}');
				fixed.close();
			}
			if (m.id == "saddle") {
				var shaft = cache.posed(l.yScrew, axis.zeroPose("screwY"));
				owned.push(shaft);
				noOverlap(out, moving, shaft, "Y shaft clears the complete saddle tunnel sweep");
				shaft.close();
			}
			moving.close();
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
