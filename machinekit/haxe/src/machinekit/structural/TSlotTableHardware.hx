package machinekit.structural;

import machinekit.standard.SocketHeadCapScrew;
import machinekit.standard.MetricScrewSpec;

/** Nominal dimensional hardware specification, not a supplier SKU or load rating.
 * Nuts are end-loaded rectangular slot plates: insert before closing the frame.
 * Threads are semantic, consistent with MachineKit's standard fasteners. */
class TSlotTableHardware {
	public final boltSize:String;
	public final bracketThickness:Float;
	public final nutWidth:Float;
	public final nutLength:Float;
	public final nutThickness:Float;
	public final footThread:String;
	public final footEngagement:Float;
	public static inline final TIP_CLEARANCE:Float = 0.2;

	public function new(boltSize:String, bracketThickness:Float, nutWidth:Float, nutLength:Float,
			nutThickness:Float, footThread:String, footEngagement:Float) {
		for (v in [bracketThickness, nutWidth, nutLength, nutThickness, footEngagement])
			if (!Math.isFinite(v) || v <= 0) throw "Table hardware dimensions must be finite and positive";
		SocketHeadCapScrew.catalog().get(boltSize);
		SocketHeadCapScrew.catalog().get(footThread);
		this.boltSize = boltSize; this.bracketThickness = bracketThickness;
		this.nutWidth = nutWidth; this.nutLength = nutLength; this.nutThickness = nutThickness;
		this.footThread = footThread; this.footEngagement = footEngagement;
	}

	public static function forProfile(profile:TSlotExtrusion):TSlotTableHardware {
		return switch profile.designation {
			case "HFS5-2020": new TSlotTableHardware("M4", 4.7, 9, 12, 1.6, "M5", 10);
			case "HFS5-4040": new TSlotTableHardware("M5", 6.2, 9, 20, 1.6, "M8", 16);
			default: throw 'No table hardware specification for ${profile.designation}';
		};
	}

	public function screwSpec():MetricScrewSpec return SocketHeadCapScrew.catalog().get(boltSize);
	public function footSpec():MetricScrewSpec return SocketHeadCapScrew.catalog().get(footThread);

	/** Installed nut bears against the slot retaining lip; clearance remains behind it. */
	public function nutDepth(profile:TSlotExtrusion):Float
		return profile.modeledSlotDepth() - profile.headDepth;

	/** Select a stocked nominal length; reject instead of silently using a special-length bolt. */
	public function bracketScrewLength(profile:TSlotExtrusion):Float {
		var minimum = bracketThickness + nutDepth(profile) + nutThickness;
		var maximum = bracketThickness + profile.modeledSlotDepth() - TIP_CLEARANCE;
		for (length in screwLengths()) if (length >= minimum - 1e-8 && length <= maximum + 1e-8) return length;
		throw "No standard screw length spans bracket and slot nut without bottoming";
	}

	public function topScrewLength(profile:TSlotExtrusion, thickness:Float):Float {
		var best = 0.0;
		for (length in screwLengths())
			if (length <= thickness + profile.modeledSlotDepth() - TIP_CLEARANCE - screwSpec().headHeight && length >= profile.modeledSlotDepth() + 2)
				best = length;
		if (best == 0) throw "Top is too thin for a recessed screw and remaining plate material";
		return best;
	}

	public function validate(profile:TSlotExtrusion):Void {
		var screw = screwSpec(), s = profile.size, t = bracketThickness;
		var accessRadius = screw.headDiameter / 2 + 1;
		if (screw.diameter >= profile.slotWidth || nutWidth <= profile.slotWidth || nutWidth >= profile.tWidth ||
				nutWidth <= screw.diameter + 2 || nutThickness >= profile.headDepth ||
				nutDepth(profile) + nutThickness > profile.modeledSlotDepth() - TIP_CLEARANCE + 1e-8 ||
				footEngagement < 1.5 * footSpec().diameter || nutLength >= s || nutLength <= screw.diameter + 2 ||
				s / 2 - accessRadius <= t || accessRadius * 2 >= s * 0.7 ||
				footSpec().tapDrill >= s - 2 * profile.modeledSlotDepth())
			throw "Table hardware does not fit the profile, bracket or driver access";
		bracketScrewLength(profile);
	}

	static function screwLengths():Array<Float> return [6,8,10,12,16,20,25,30,35,40,45,50,60,70,80];
}
