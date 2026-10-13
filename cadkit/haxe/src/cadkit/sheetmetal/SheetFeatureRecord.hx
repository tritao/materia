package cadkit.sheetmetal;

import materia.project.MaterialDef.MaterialKFactorRow;
import materia.project.MaterialDef.MaterialMechanical;
import cadkit.sheetmetal.process.ProcessRuleRecord;

/** Typed document payloads; JSON validation ends here before constructing sheet features. */
@:wire typedef SheetBaseRecord = {
	@:id(1) var source:Int;
	@:id(2) var thickness:Float;
	@:id(3) var side:SheetSide;
	@:id(4) var material:SheetStockRecord;
	@:id(5) var identities:SheetIdentityRecord;
}

@:wire typedef SheetIdentityRecord = {
	@:id(1) var nextWall:Int;
	@:id(2) var nextBend:Int;
}

@:wire typedef SheetStockRecord = {
	@:id(1) var id:String;
	@:id(2) var rule:Null<SheetSavedBendRule>;
	@:id(3) var mechanical:Null<MaterialMechanical>;
	@:id(4) var defaultRadiusRatio:Null<Float>;
	@:id(5) var minRadiusRatio:Null<Float>;
	@:id(6) var grainAngle:Null<Float>;
}

@:wire enum SheetSavedBendRule {
	@:id(1) KFactor(rows:Array<MaterialKFactorRow>);
	@:id(2) Process(setup:ProcessRuleRecord);
}

@:wire typedef SheetFlangeRecord = {
	@:id(1) var source:Int;
	@:id(2) var angle:Float;
	@:id(3) var length:Float;
	@:id(4) var radius:Null<Float>;
	@:id(5) var lengthSpec:LengthSpec;
	@:id(6) var position:FlangePosition;
	@:id(7) var offset:Float;
	@:id(8) var gap1:Float;
	@:id(9) var gap2:Float;
	@:id(10) var extend1:Float;
	@:id(11) var extend2:Float;
	@:id(12) var walls:Array<Int>;
	@:id(13) var bends:Array<Int>;
	@:id(14) var faces:Array<SheetFaceRecord>;
	@:id(15) var reliefType:ReliefType;
	@:id(16) var reliefSizing:ReliefSizing;
	@:id(17) var reliefWidth:Float;
	@:id(18) var reliefDepth:Float;
}

@:wire typedef SheetFaceRecord = {
	@:id(1) var surface:String;
	@:id(2) var curve:String;
	@:id(3) var x:Float;
	@:id(4) var y:Float;
	@:id(5) var z:Float;
	@:id(6) var dx:Float;
	@:id(7) var dy:Float;
	@:id(8) var dz:Float;
	@:id(9) var measure:Float;
	@:id(10) var name:Null<String>;
	@:id(11) var naming:Null<Int>;
}

@:wire typedef SheetHemRecord = {
	@:id(1) var source:Int;
	@:id(2) var length:Float;
	@:id(3) var spec:HemSpec;
	@:id(4) var side:SheetSide;
	@:id(5) var walls:Array<Int>;
	@:id(6) var bends:Array<Int>;
	@:id(7) var faces:Array<SheetFaceRecord>;
}

@:wire typedef SheetRipRecord = {
	@:id(1) var source:Int;
	@:id(2) var bend:Int;
}

@:wire typedef SheetCutRecord = {
	@:id(1) var source:Int;
	@:id(2) var profile:Int;
	@:id(3) var wall:Int;
	@:id(4) var side:SheetSide;
}

@:wire typedef SheetCornerRecord = {
	@:id(1) var source:Int;
	@:id(2) var firstWall:Int;
	@:id(3) var firstEdge:String;
	@:id(4) var secondWall:Int;
	@:id(5) var secondEdge:String;
	@:id(6) var join:CornerJoin;
	@:id(7) var gap:Float;
	@:id(8) var overlap:Float;
	@:id(9) var relief:CornerRelief;
	@:id(10) var reliefSize:Float;
}
