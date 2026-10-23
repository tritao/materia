package materia.tooling.casting;

import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.tooling.casting.CastingMoldPlan.MoldBox;

enum abstract CoreSupportKind(String) to String {
	var Bridge = "bridge";
	var RoutedSocket = "routed-socket";
	var RegisteredFlange = "registered-flange";
}

enum abstract SupportHardwareKind(String) to String {
	var LocatingPin = "locating-pin";
	var ClampBolt = "clamp-bolt";
	var FixtureScrew = "fixture-screw";
	var FixturePin = "fixture-pin";
	var BoreMask = "bore-mask";
	var BorePlug = "bore-plug";
}

typedef CoreCutting = {
	/** Finished foam in stock coordinates; the block is only its cutting blank. */
	var solid:materia.casting.CastingRegion;
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
	var bridgeBondAreaMm2:Float;
	/** Geometric upper limit for clamp bearing on a registered EPS flange. */
	var clampBearingAreaMm2:Float;
	var socketOperationId:Null<String>;
	/** Exact wall-operation trace retained when an access former becomes EPS. */
	var registrationProfiles:Array<materia.tooling.casting.CastingMoldPlan.MoldVolumeProfile>;
	var foot:Null<CoreFoot>;
	var panelIds:Array<String>;

	/** Permanently bonded carrier and bridge move as one reusable fixture. */
	var bondedPanelIds:Array<String>;

	var hardware:Array<SupportHardware>;

	/** Bridge pads still require external clamps to the wall tops. */
	var padClosure:Null<materia.tooling.panel.PanelJoints.PanelClosure>;
}
