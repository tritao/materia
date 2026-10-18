package machinekit.milling;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.assembly.Diagnostics;
import machinekit.assembly.PosedParts;
import machinekit.structural.FoundationIntegrity;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Conservative CAD swept envelopes for all translating Z structural parts.
 * Checks the entire interval against castings, inserts and stationary Z mounts.
 * Thread engagement, guide contact and spindle/work contact are separate checks;
 * this is a geometric audit, not a load-capacity or machine safety assessment.
 */
class MillZAxisIntegrity {
	public static function check(axis:MillZAxis, frame:MillFrame):Diagnostics {
		if (axis == null || frame == null || axis.layout.frame != frame.layout)
			throw "Z clearance audit needs matching frame and axis datums";
		var owned:Array<Part> = [];
		var result:Diagnostics;
		try result = run(axis, frame, owned) catch (error:Dynamic) { close(owned); throw error; }
		close(owned);
		return result;
	}
	static function run(axis:MillZAxis, frame:MillFrame, owned:Array<Part>):Diagnostics {
		var out = new Diagnostics(), l = axis.layout, f = l.frame;
		var poses:Map<String, AssemblyFrame> = axis.solvedPoses();
		// Swept boxes enclose every moving structural part throughout the
		// complete stroke. No discrete sampling is used for the CAD clearance proof.
		var travel = -l.lower, face = l.headCentre - l.nut.bodyLength / 2 - l.nut.flangeThickness;
		var shelfDepth = l.screwY - l.carriageBack + l.bracketWidth / 2;
		var swept = [
			box(owned, l.carriageWidth, l.carriageThickness, l.carriageHeight + travel, 0, l.carriageBack - l.carriageThickness / 2, l.headCentre - l.carriageHeight / 2 + l.lower),
			box(owned, l.bracketWidth, shelfDepth, l.nutPlateThickness + MillZCarriage.GUSSET_HEIGHT + travel, 0, l.carriageBack + shelfDepth / 2, face - l.nutPlateThickness - MillZCarriage.GUSSET_HEIGHT + l.lower),
			box(owned, l.nut.flangeDiameter, l.nut.flangeDiameter, l.nut.bodyLength + l.nut.flangeThickness + travel, 0, l.screwY, face + l.lower),
			headSweep(axis, owned)];
		for (side in [-1, 1]) for (span in [-l.blockSpan, l.blockSpan]) swept.push(
			box(owned, l.block.spec.blockWidth, l.block.spec.blockHeight - l.block.spec.railHeight, l.block.spec.blockLength + travel,
				side * f.zRailSpacing / 2, (l.carriageBack + f.columnFront - l.block.spec.railHeight) / 2,
				l.headCentre + span - l.block.spec.blockLength / 2 + l.lower));
		for (member in axis.components()) if (StringTools.startsWith(member.id, "headBolt")) {
			var local = member.component.geometry(); owned.push(local);
			var part = PosedParts.moved(local, poses.get(member.id)); owned.push(part);
			var b = PosedParts.boxOf(part);
			swept.push(box(owned, b.maxX-b.minX, b.maxY-b.minY, b.maxZ-b.minZ+travel,
				(b.minX+b.maxX)/2, (b.minY+b.maxY)/2, b.minZ+l.lower));
		}
		var framePoses:Map<String, AssemblyFrame> = frame.solvedPoses();
		for (member in frame.components()) {
			var local = member.component.geometry(); owned.push(local);
			var fixed = PosedParts.moved(local, framePoses.get(member.id)); owned.push(fixed); local.close();
			for (sweep in swept) noOverlap(out, sweep, fixed, 'full-stroke clearance to ${member.id}');
			if (member.id == "column/casting" && swept[1].distance(fixed) < l.clearance - 1e-6)
				out.error("mill-z.channel-clearance", member.id, "Central nut shelf does not retain its specified casting clearance over full travel");
			fixed.close();
		}
		for (id in ["fixedMount", "floatingMount", "fixedBearing", "floatingBearing", "motorStand", "couplingGuard", "motor", "railLeft", "railRight"]) {
			var part = axis.components().filter(m -> m.id == id)[0].component.geometry();
			owned.push(part);
			var fixed = PosedParts.moved(part, poses.get(id)); owned.push(fixed); part.close();
			for (sweep in swept) noOverlap(out, sweep, fixed, 'full-stroke clearance to $id'); fixed.close();
		}
		var mounts = FoundationIntegrity.checkMounts(frame, [for (end in ["Lower", "Upper"]) for (side in ["Left", "Right"])
			{name: 'Z $end bearing plate $side', parent: 'column/drive$end$side'}]);
		for (item in mounts.items) out.add(item.severity, item.code, item.subject, item.message);
		return out;
	}
	static function box(owned:Array<Part>, w:Float, d:Float, h:Float, x:Float, y:Float, z:Float):Part {
		var local = Part.box(w, d, h); owned.push(local);
		var moved = local.translated(new Vector(x, y, z)); owned.push(moved); local.close(); return moved;
	}
	static function headSweep(axis:MillZAxis, owned:Array<Part>):Part {
		var head = axis.head.geometry(); owned.push(head);
		var bounds = head.shape.bounds(), low = bounds.get_min(), high = bounds.get_max();
		var pose = axis.zeroPose("head"), l = axis.layout;
		return box(owned, high.get_x()-low.get_x(), high.get_y()-low.get_y(), high.get_z()-low.get_z()-l.lower,
			pose.x+(low.get_x()+high.get_x())/2, pose.y+(low.get_y()+high.get_y())/2, pose.z+low.get_z()+l.lower);
	}
	static function close(owned:Array<Part>):Void {
		for (part in owned) if (!part.shape.isClosed()) part.close();
	}

	static function noOverlap(out:Diagnostics, a:Part, b:Part, subject:String):Void {
		var common = a.intersect(b), volume:Float;
		try volume = common.volume() catch (error:Dynamic) { common.close(); throw error; }
		common.close();
		if (volume > 1e-5) out.error("mill-z.full-stroke-collision", subject, 'Swept envelope intersects $subject ($volume mm3)');
	}
}
