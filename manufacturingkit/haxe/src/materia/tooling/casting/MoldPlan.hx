package materia.tooling.casting;

import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.tooling.ToolingDocument;
import materia.tooling.panel.PanelAssemblyPlan.PanelAssemblyRecord;
import materia.tooling.panel.PanelAssemblyPlan.PanelPlacement;
import materia.tooling.casting.CoreSupport.CoreSupportOptions;

typedef MoldBox = {var x:Float;var y:Float;var z:Float;var width:Float;var depth:Float;var height:Float;}
enum abstract PenetrationClass(String) to String {var OutsideCasting="outside-casting";var InsertSealed="insert-sealed";var CoreSealed="core-sealed";}
typedef PenetrationPlan = {
    var panelId:String;var operationId:String;var kind:PenetrationClass;
    var insertId:Null<String>;var coreId:Null<String>;var sealMargin:Float;var sealMaterial:String;
}
enum abstract CoreRemoval(String) to String {var BreakoutThroughFill="breakout-through-fill";var BreakoutThroughBed="breakout-through-bed";}
typedef CorePlan = {
    var id:String;var featureId:String;var cavity:MoldBox;var foam:MoldBox;
    var materialSpec:String;var coveringThickness:Float;var removal:CoreRemoval;
    /** Stock is local [0,width] x [0,depth] x [0,height], no kerf compensation. */
    var cutting:{block:MoldBox,stockToMold:AssemblyFrame,wireDirection:{x:Float,y:Float,z:Float}};
    var support:CoreSupport;
    var restraintEvidence:Array<String>;
}
enum abstract FixtureThreadProtection(String) to String {var FixingScrewAndResinSidePlug="fixing-screw-and-resin-side-plug";}
typedef InsertFixturePlan = {
    var insertId:String;var body:MoldBox;var mountingFace:AssemblyFrame;
    var panelId:String;var operationId:String;var fixingDiameter:Float;var threadSpec:String;
    var threadProtection:FixtureThreadProtection;var finishingAllowance:Null<Float>;
    var datumEvidence:Array<String>;
}
/** Versioned generated review record. Source document is the reproducible authority. */
typedef MoldPlan = {
    var id:String;var document:ToolingDocument;var documentFingerprint:String;
    var cavity:MoldBox;var panels:PanelAssemblyRecord;
    var supportOptions:CoreSupportOptions;var supportPanels:Array<PanelPlacement>;
    var cores:Array<CorePlan>;var fixtures:Array<InsertFixturePlan>;var penetrations:Array<PenetrationPlan>;
    var pending:Array<String>;
}
