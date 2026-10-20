package materia.tooling.casting;

import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.tooling.ToolingDocument;
import materia.tooling.panel.PanelAssemblyPlan.PanelAssemblyRecord;
import materia.tooling.panel.PanelAssemblyPlan.PanelPlacement;
import materia.tooling.casting.CoreSupport.CoreSupportOptions;

typedef MoldBox = materia.casting.Box3;

enum abstract PenetrationClass(String) to String {
	var OutsideCasting = "outside-casting";
	var InsertSealed = "insert-sealed";
	var CoreSealed = "core-sealed";
}

typedef PenetrationPlan = {
	var panelId:String;
	var operationId:String;
	var kind:PenetrationClass;
	var insertId:Null<String>;
	@:optional var sourceHoleId:String;
	var coreId:Null<String>;
	var sealMargin:Float;
	var sealMaterial:String;
	var sealingFace:Null<AssemblyFrame>;
	var plug:Null<{
		diameter:Float,
		length:Float,
		materialSpec:String
	}>;
}

enum abstract CoreRemoval(String) to String {
	var BreakoutThroughFill = "breakout-through-fill";
	var BreakoutThroughBed = "breakout-through-bed";
}

typedef CorePlan = {
	var id:String;
	var featureId:String;
	var cavity:MoldBox;
	var foam:MoldBox;
	var materialSpec:String;
	var coveringThickness:Float;
	var removal:CoreRemoval;

	/** Stock is local [0,width] x [0,depth] x [0,height], no kerf compensation. */
	var cutting:materia.tooling.casting.CoreSupport.CoreCutting;

	var support:CoreSupport;
	var restraintEvidence:Array<String>;
}

enum abstract FixtureThreadProtection(String) to String {
	var FixingScrewAndResinSidePlug = "fixing-screw-and-resin-side-plug";
}

typedef InsertDatums = {
	var primarySupportPoints:Array<{
		x:Float,
		y:Float,
		z:Float
	}>;
	var secondaryAxis:{x:Float, y:Float, z:Float};
	var tertiaryAxis:{x:Float, y:Float, z:Float};
}

typedef InsertFixturePlan = {
	var insertId:String;
	var body:MoldBox;
	var mountingFace:AssemblyFrame;
	var panelId:String;
	var fixings:Array<materia.tooling.casting.CastingMoldPlan.InsertFixtureFixing>;
	var boreMasks:Array<materia.tooling.casting.CastingMoldPlan.BoreMaskProtection>;
	var threadProtection:FixtureThreadProtection;
	var finishingAllowance:Null<Float>;
	var datumEvidence:Array<String>;
	var datums:InsertDatums;
	var precision:materia.tooling.casting.CastingMoldPlan.PrecisionInterface;
}

/** Versioned generated review record. Source document is the reproducible authority. */
typedef MoldPlan = {
	var id:String;
	var document:ToolingDocument;
	var documentFingerprint:String;
	var cavity:MoldBox;
	var panels:PanelAssemblyRecord;
	var supportOptions:CoreSupportOptions;
	var supportPanels:Array<PanelPlacement>;
	var changeRequests:Array<materia.tooling.casting.CastingMoldPlan.DesignChangeRequest>;
	var cores:Array<CorePlan>;
	var fixtures:Array<InsertFixturePlan>;
	var penetrations:Array<PenetrationPlan>;
	var validation:Array<materia.tooling.ToolingValidation>;
	var fabrication:materia.tooling.ToolingValidation.FabricationReadiness;
}
