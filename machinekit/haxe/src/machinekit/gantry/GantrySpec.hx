package machinekit.gantry;

import machinekit.motion.LeadScrewThread;
import machinekit.motion.LeadScrewThread.LeadScrewThreadFamily;
import machinekit.transmission.TimingBeltProfile;
import machinekit.transmission.ValueBasis;

/** Mechanical drive choice: ratios are derived from these physical parts. */
enum GantryDrive {
	Screw(thread:LeadScrewThread);
	Belt(profile:TimingBeltProfile, teeth:Int, width:Float);
	Rack(moduleSize:Float, pinionTeeth:Int, width:Float);
}

/** Optional rotary head; the translational assembly exposes the flange without one. */
enum GantryHead {
	None;
	C;
	CA;
}

/** Travel is in millimetres. Frame and guide dimensions describe parts, not virtual joints. */
class GantrySpec {
	public final travelX:Float;
	public final travelY:Float;
	public final travelZ:Float;
	public final driveX:GantryDrive;
	public final driveY:GantryDrive;
	public final driveZ:GantryDrive;
	public final dualY:Bool;
	public final railProfile:String;
	public final motorFrame:Int;
	public final frameProfile:String;
	public final beamProfile:String;
	public final tableMounted:Bool;
	/** Extra frame length ahead of Y travel, clearing an overhanging tool's vertical sweep. */
	public final frontExtension:Float;
	public final head:GantryHead;
	/** Allowed side-to-side displacement; a stated design assumption until measured. */
	public final racking:Float;
	public final rackingBasis:ValueBasis;
	public final supplyVoltage:Float;
	public final microsteps:Int;
	/** Provenance for the authored machine layout and electrical settings. */
	public final assumedFields:Array<String>;

	public function new(travelX:Float = 1500, travelY:Float = 1000, travelZ:Float = 500,
			?driveX:GantryDrive, ?driveY:GantryDrive, ?driveZ:GantryDrive, dualY:Bool = true,
			railProfile:String = "MGN12C", motorFrame:Int = 23,
			frameProfile:String = "HFS5-4040", beamProfile:String = "HFS5-4040",
			tableMounted:Bool = false, head:GantryHead = None, racking:Float = 0.5,
			supplyVoltage:Float = 24, microsteps:Int = 16, frontExtension:Float = 0) {
		for (value in [travelX, travelY, travelZ, racking, supplyVoltage])
			if (!Math.isFinite(value) || value <= 0) throw "Gantry dimensions and electrical limits must be finite and positive";
		if (microsteps <= 0) throw "Gantry microsteps must be positive";
		if (!Math.isFinite(frontExtension) || frontExtension < 0) throw "Gantry front extension must be finite and non-negative";
		this.travelX = travelX; this.travelY = travelY; this.travelZ = travelZ;
		this.driveX = driveX == null ? Belt(TimingBeltProfile.GT2, 20, 9) : driveX;
		this.driveY = driveY == null ? Belt(TimingBeltProfile.GT2, 20, 9) : driveY;
		this.driveZ = driveZ == null ? Screw(new LeadScrewThread(MetricTrapezoidal, 12, 3)) : driveZ;
		this.dualY = dualY;
		this.railProfile = railProfile; this.motorFrame = motorFrame;
		this.frameProfile = frameProfile; this.beamProfile = beamProfile;
		this.tableMounted = tableMounted; this.head = head;
		this.frontExtension = frontExtension;
		this.racking = racking; rackingBasis = ValueBasis.Assumed;
		this.supplyVoltage = supplyVoltage; this.microsteps = microsteps;
		assumedFields = ["travelX", "travelY", "travelZ", "driveX", "driveY", "driveZ", "dualY",
			"railProfile", "motorFrame", "frameProfile", "beamProfile", "tableMounted", "head",
			"racking", "supplyVoltage", "microsteps", "frontExtension"];
	}
}
