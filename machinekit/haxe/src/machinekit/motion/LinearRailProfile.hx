package machinekit.motion;

/**
 * One published load rating of a carriage, as its source states it: basic dynamic (C) and static (C0) load ratings
 * in N, and static rated moments in N·m where the source gives them. Sizing applies its own static safety factor
 * to `staticLoad`; no derating is folded in here.
 */
typedef LinearRailRating = {
	/** Who publishes it, e.g. "Vallder", "HIWIN G99TE24-2410". */
	var source:String;
	var reference:String;
	/** Basic dynamic load rating C, N, at `lifeBasisKm`. */
	var dynamicLoad:Float;
	/** Basic static load rating C0, N. */
	var staticLoad:Float;
	/** Rated travel life the dynamic rating refers to, km (50 for ball guides to HIWIN's convention). */
	var lifeBasisKm:Float;
	/** Static rated moments about the rail axis (roll), across it (pitch) and the mounting normal (yaw), N·m. */
	@:optional var rollMoment:Float;
	@:optional var pitchMoment:Float;
	@:optional var yawMoment:Float;
	/** False when the source only reproduces another's figures (resellers copying a manufacturer edition). */
	var original:Bool;
}

/** Counterbored rail mounting hole from the rail table: through bore d, counterbore D and depth h, in mm. */
typedef LinearRailBore = {
	var through:Float;
	var counterbore:Float;
	var depth:Float;
}

/**
 * A profile-rail carriage and the rail it runs on, in mm and kg. `designation` names the carriage (HGH15CA, MGN12C)
 * and `rail` the rail (HGR15R, MGNR12R); every carriage on one rail repeats that rail's fields.
 */
typedef LinearRailProfileSpec = {
	var designation:String;
	var rail:String;
	var family:String;
	var railWidth:Float;
	var railHeight:Float;
	var blockWidth:Float;
	var blockHeight:Float;
	var blockLength:Float;
	/** Spacing of the carriage's mounting holes along the rail (HIWIN's C). */
	var blockHoleSpacingAlong:Float;
	/** Spacing of the carriage's mounting holes across the rail (HIWIN's B). */
	var blockHoleSpacingAcross:Float;
	/** Thread of the carriage's tapped mounting holes, e.g. "M4". */
	var blockMountThread:String;
	/** Full thread depth of those holes, where the table gives it. */
	@:optional var blockMountThreadDepth:Float;
	var railHolePitch:Float;
	var railEndMargin:Float;
	/** Rail mounting bolt from the rail table, size and length, e.g. "M4x16". */
	var railMountScrew:String;
	var railBore:LinearRailBore;
	/** Datasheet masses: one carriage, and the rail per metre of length. */
	var blockMassKg:Float;
	var railMassKgPerMetre:Float;
	/** Published ratings; the first is the one sizing uses, that of the supplier the part is bought from. */
	var ratings:Array<LinearRailRating>;
	/** Assumed procurement target, not a measured or simulated preload. */
	@:optional var preloadTarget:String;
}
