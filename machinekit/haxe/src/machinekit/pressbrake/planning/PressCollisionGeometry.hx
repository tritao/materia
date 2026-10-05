package machinekit.pressbrake.planning;

import cadkit.modeling.Vector;
import cadkit.sheetmetal.Sheet;
import cadkit.sheetmetal.SheetDiagnosis;
import machinekit.pressbrake.ToolRow;
import machinekit.pressbrake.ToolContactPrism;

/** Rebuild material collision geometry at each evaluated stroke pose, in press-frame mm. */
class PressCollisionGeometry {
	public static function sheet(source:Sheet, pose:PressSheetPose, maximumError:Float = 0.01):Array<PressCollisionBody> {
		var result:Array<PressCollisionBody> = [];
		for (wall in source.walls()) {
			var profile = SheetPrismProfile.of("wall:" + wall.id, wall.outline, source.thickness(), maximumError);
			result.push(new PressCollisionBody(SheetWall(wall.id),
				[for (piece in profile.pieces()) piece.placed(pose.wall(wall.id))], profile.curveError));
		}
		for (bend in source.bends()) {
			var profile = BendPrismProfile.of(bend, pose.evaluation.bend(bend.id), maximumError);
			result.push(new PressCollisionBody(SheetBend(bend.id),
				[for (piece in profile.pieces()) piece.placed(pose.placement)], profile.error));
		}
		return result;
	}

	/** All installed rows participate, including adjacent stations, gaps and end-horn reliefs. */
	public static function tools(press:PressGeometry, ramTravel:Float,
			maximumError:Float = 0.01):Array<PressCollisionBody> {
		travel(ramTravel, press.ramTravel, "ram");
		if (!Math.isFinite(maximumError) || maximumError <= 0)
			throw new SheetDiagnosis("press.collision-error", "Tool collision error must be finite positive mm");
		var result:Array<PressCollisionBody> = [];
		for (station in press.installedTools.stations()) {
			var contact = press.station(station.id), offset = station.offset - press.bedLength / 2;
			row(result, station.punches, offset, contact.punchLineY, contact.openTipHeight - ramTravel,
				station.id, true, maximumError);
			row(result, station.dies, offset, contact.punchLineY, contact.dieTopHeight,
				station.id, false, maximumError);
		}
		return result;
	}
	static function row(result:Array<PressCollisionBody>, row:ToolRow, offset:Float, y:Float, z:Float,
			station:String, punch:Bool, error:Float):Void {
		var placed = row.placements(offset);
		for (i in 0...placed.length) {
			var segment = placed[i], pieces:Array<ConvexPrism> = [];
			var label = (punch ? "punch:" : "die:") + station + ":segment:" + i;
			for (prism in segment.segment.collisionPrisms(error)) {
				var polygon = [for (point in prism.section.points())
					new Vector(segment.start + prism.start, y + point.y, z + point.z)];
				pieces.push(new ConvexPrism(label + ":piece:" + pieces.length, polygon, new Vector(prism.end - prism.start, 0, 0)));
			}
			result.push(new PressCollisionBody(punch ? Punch(station, i) : Die(station, i), pieces, error));
		}
	}
	/** Analytic sections share the installed row placements and retain every groove and horn. */
	public static function toolContacts(press:PressGeometry, ramTravel:Float):Array<PressToolContact> {
		travel(ramTravel, press.ramTravel, "ram");
		var result:Array<PressToolContact> = [];
		for (station in press.installedTools.stations()) {
			var contact = press.station(station.id), offset = station.offset - press.bedLength / 2;
			contactRow(result, station.punches, offset, contact.punchLineY, contact.openTipHeight - ramTravel, station.id, true);
			contactRow(result, station.dies, offset, contact.punchLineY, contact.dieTopHeight, station.id, false);
		}
		return result;
	}
	static function contactRow(result:Array<PressToolContact>, row:ToolRow, offset:Float, y:Float, z:Float,
			station:String, punch:Bool):Void {
		var placed = row.placements(offset);
		for (i in 0...placed.length) {
			var segment = placed[i], prisms:Array<ToolContactPrism> = [];
			for (prism in segment.segment.contactPrisms()) prisms.push(prism.placed(segment.start, y, z));
			result.push(new PressToolContact(punch ? Punch(station, i) : Die(station, i), prisms));
		}
	}

	/** Assembly-derived machine boxes and open frame volumes, including backgauge carriers. */
	public static function machineMembers(press:PressGeometry, ramTravel:Float, gaugeX:Float, gaugeR:Float):Array<PressCollisionBody> {
		travel(ramTravel, press.ramTravel, "ram"); travel(gaugeX, press.backgaugeX, "backgauge-X");
		travel(gaugeR, press.backgaugeR, "backgauge-R");
		var result = [box(MachineMember("ram"), "ram", press.ramEnvelope, new Vector(0, 0, -ramTravel))];
		var fingers = press.fingers(), contacts = press.fingerContacts();
		if (fingers.length != contacts.length)
			throw new SheetDiagnosis("press.collision-fingers", "Finger envelopes and physical contact datums disagree");
		for (i in 0...fingers.length)
			result.push(box(MachineMember(contacts[i].id), contacts[i].id, fingers[i], new Vector(0, gaugeX, gaugeR)));
		for (member in press.machineMembers()) result.push(member.at(ramTravel, gaugeX, gaugeR));
		return result;
	}
	public static function box(owner:PressCollisionOwner, id:String, envelope:PressEnvelope, shift:Vector):PressCollisionBody {
		var x = envelope.minX + shift.x, y = envelope.minY + shift.y, z = envelope.minZ + shift.z;
		var highX = envelope.maxX + shift.x, highY = envelope.maxY + shift.y;
		return new PressCollisionBody(owner, [new ConvexPrism(id, [new Vector(x, y, z), new Vector(highX, y, z),
			new Vector(highX, highY, z), new Vector(x, highY, z)], new Vector(0, 0, envelope.maxZ - envelope.minZ))], 0);
	}
	static function travel(value:Float, range:PressTravel, member:String):Void {
		if (!Math.isFinite(value) || value < range.lower - 1e-8 || value > range.upper + 1e-8)
			throw new SheetDiagnosis("press.collision-travel", "Collision pose exceeds nominal travel", [member]);
	}
}
