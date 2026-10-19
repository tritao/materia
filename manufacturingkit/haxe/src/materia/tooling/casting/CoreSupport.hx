package materia.tooling.casting;

import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.tooling.casting.MoldPlan.MoldBox;

enum abstract CoreSupportMode(String) to String {
	var Auto = "auto";
	var Bridge = "bridge";
	var BedSocket = "bed-socket";
}

enum abstract CoreSupportKind(String) to String {
	var Bridge = "bridge";
	var BedSocket = "bed-socket";
}

enum abstract SupportHardwareKind(String) to String {
	var LocatingPin = "locating-pin";
	var ClampBolt = "clamp-bolt";
	var FixtureScrew = "fixture-screw";
	var FixturePin = "fixture-pin";
	var BoreMask = "bore-mask";
}

typedef CoreSupportOptions = {
	var mode:CoreSupportMode;
	var embedDepth:Float;
	var fitClearance:Float;
	var adhesiveThickness:Float;
}

typedef CoreCutting = {
	var block:MoldBox;
	var stockToMold:AssemblyFrame;
	var wireDirection:{x:Float, y:Float, z:Float};
	var entryFace:{
		origin:{x:Float, y:Float, z:Float},
		normal:{x:Float, y:Float, z:Float}
	};
	var exitFace:{
		origin:{x:Float, y:Float, z:Float},
		normal:{x:Float, y:Float, z:Float}
	};
}

typedef CoreFoot = {
	/** Finished EPS foot, separate from the main foam block; two glue lines are explicit. */
	var foam:MoldBox;

	var cutting:CoreCutting;
}

typedef SupportHardware = {
	var id:String;
	var kind:SupportHardwareKind;
	var pose:AssemblyFrame;
	var diameter:Float;
	var length:Float;
}

typedef CoreSupport = {
	var kind:CoreSupportKind;
	var adhesiveSpec:String;
	var adhesiveThickness:Float;
	var fitClearance:Float;
	var embedDepth:Float;
	var bondAreaMm2:Float;
	var socketOperationId:Null<String>;
	var foot:Null<CoreFoot>;
	var panelIds:Array<String>;

	/** Permanently bonded carrier and bridge move as one reusable fixture. */
	var bondedPanelIds:Array<String>;

	var hardware:Array<SupportHardware>;

	/** Bridge pads still require external clamps to the wall tops. */
	var padClosure:Null<materia.tooling.panel.PanelJoints.PanelClosure>;
}
