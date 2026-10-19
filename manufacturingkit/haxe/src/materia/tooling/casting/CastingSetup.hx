package materia.tooling.casting;

import materia.assembly.AssemblyRecord.AssemblyFrame;

enum abstract CoreStrategy(String) to String {
	var SacrificialEPS = "sacrificial-eps";
	var ReusableFormer = "reusable-former";
	var Unsupported = "unsupported";
}

/** Directions are expressed in mold coordinates, never source coordinates. */
typedef CastingFeatureStrategy = {
	var featureId:String;
	var strategy:CoreStrategy;
	/** Required for reusable formers. EPS may use null for breakup removal. */
	var withdrawal:Null<{var x:Float; var y:Float; var z:Float;}>;
}

/** Mold +Z is upward, gravity is -Z and the open fill face faces +Z.
 * The setup is tied to the exact input fingerprint, not merely its revision number.
 */
typedef CastingSetup = {
	var id:String;
	var revision:Int;
	var sourceFingerprint:String;
	var sourceToMold:AssemblyFrame;
	/** Exactly one strategy per ordinary void; insert recesses are handled by fixtures. */
	var features:Array<CastingFeatureStrategy>;
}
