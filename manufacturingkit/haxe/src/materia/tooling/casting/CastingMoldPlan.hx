package materia.tooling.casting;

import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.tooling.ToolingDocument;
import materia.tooling.panel.PanelAssemblyPlan.PanelAssemblyRecord;
import materia.tooling.panel.PanelAssemblyPlan.PanelPlacement;
import materia.tooling.casting.MoldPlan.MoldBox;
import materia.tooling.casting.CoreSupport.SupportHardware;

enum abstract MoldPartMaterial(String) to String {
	var EPSStock = "eps";
	var SteelStock = "steel";
	var FormerStock = "former";
}

typedef MoldVolumeProfile = {
	var panelId:String;
	var operationId:String;
	var contour:Array<materia.tooling.panel.PanelPlan.PanelSegment>;
	var pose:materia.assembly.AssemblyRecord.AssemblyFrame;
	var depth:Float;
	var subtract:Array<MoldBox>;
}

typedef MoldVolumeCut = {
	var operation:materia.tooling.panel.PanelPlan.PanelOperation;
	var pose:AssemblyFrame;
}

typedef MoldVolume = {
	var id:String;
	var material:MoldPartMaterial;
	var pieces:Array<MoldBox>;
	var profiles:Array<MoldVolumeProfile>;
	var cuts:Array<MoldVolumeCut>;
}

enum abstract FeatureToolingKind(String) to String {
	var EPSFeature = "eps";
	var FormerFeature = "former";
	var InsertFeature = "insert";
	var UnsupportedFeature = "unsupported";
}

typedef FeatureTooling = {
	var featureId:String;
	var kind:FeatureToolingKind;
	var memberIds:Array<String>;
}

typedef CastingMoldCore = {
	var id:String;
	var featureId:String;
	var cavity:MoldBox;
	var foamLayers:Array<MoldBox>;
	var coveringThickness:Float;
	var adhesiveThickness:Float;
	var carrierId:String;
	var bridgeId:String;
	var bondAreaMm2:Float;
	var bridgeBondAreaMm2:Float;
}

/** One source core intent, with all foam pieces created around retained supports. */
typedef CastingCoreUnit = {
	var id:String;
	var parentFeatureId:String;
	var featureIds:Array<String>;
	var coreIds:Array<String>;
}

typedef MoldFormer = {
	var volume:MoldVolume;
	var featureIds:Array<String>;
	var withdrawal:{x:Float, y:Float, z:Float};
	var draftDegrees:Float;
}

enum abstract MoldOpeningKind(String) to String {
	var ExteriorOpening = "outside-casting";
	var ToolingOpening = "tooling-sealed";
}

typedef MoldOpening = {
	var panelId:String;
	var operationId:String;
	var kind:MoldOpeningKind;
	var sealingIds:Array<String>;
	var sealSpec:String;
}

typedef PrecisionInterface = {
	var insertId:String;
	var jigReference:Null<String>;
	var masterReference:Null<String>;
	var masterFlatnessMm:Null<Float>;
	var releaseSpec:Null<String>;
	var holeProtectionReference:Null<String>;
	var evidence:Array<String>;
}

enum abstract CompactionMethod(String) to String {
	var Undeclared = "undeclared";
	var VibrationTable = "vibration-table";
	var ExternalVibrators = "external-vibrators";
	var InternalPoker = "internal-poker";
	var NoCompaction = "none";
}

typedef CompactionSpec = {
	var method:CompactionMethod;
	var bedInterfaceReference:Null<String>;
	var frequencyHz:Null<Float>;
	var amplitudeMm:Null<Float>;
	var durationSeconds:Null<Float>;
	var evidence:Array<String>;
}

enum abstract AnchorageKind(String) to String {
	var Pending = "pending";
	var Studs = "studs";
	var Undercut = "undercut";
	var BondedKeys = "bonded-keys";
}

typedef InsertAnchorage = {
	var insertId:String;
	var kind:AnchorageKind;
	var geometry:Array<MoldBox>;
	var pulloutN:Null<Float>;
	var shearN:Null<Float>;
	var evidence:Array<String>;
}

enum abstract DesignChangeKind(String) to String {
	var LiftingPoints = "lifting-points";
	var ReleaseAlternative = "release-alternative";
	var InterfaceAllowance = "interface-allowance";
	var InsertFixtureHole = "insert-fixture-hole";
}

typedef DesignChangeRequest = {
	var id:String;
	var kind:DesignChangeKind;
	var sourceIds:Array<String>;
	var reason:String;
	var blockingChecks:Array<String>;
}

typedef GroutedJoint = {
	var id:String;
	var insertIds:Array<String>;
	var gapMm:Null<Float>;
	var jackingReference:Null<String>;
	var boltSleeveReference:Null<String>;
	var cureBeforeTorqueEvidence:Array<String>;
}

enum abstract OverlapRule(String) to String {
	var ReservedThenPriority = "inserts-eps-datums-then-priority-id";
}

enum abstract LiftingMethod(String) to String {
	var CastInPoints = "cast-in-points";
	var ExternalCradle = "external-cradle";
}

typedef LiftingQualification = {
	var method:LiftingMethod;
	var equipmentReference:String;
	var capacityKg:Float;
	var verifiedMassKg:Float;
	var centerOfMass:{x:Float, y:Float, z:Float};
	var positionToleranceMm:Float;
	var attachmentIds:Array<String>;
	var verificationReference:String;
}

typedef HandlingPlan = {
	var requiresFlip:Bool;
	var moldToService:AssemblyFrame;
	var castingMassKg:Float;
	var centerOfMassMoldMm:{x:Float, y:Float, z:Float};
}

typedef CastingMoldOptions = {
	var groutedJoints:Array<GroutedJoint>;
	var precision:Array<PrecisionInterface>;
	var compaction:CompactionSpec;
	var anchorage:Array<InsertAnchorage>;
	var serviceOrientation:AssemblyFrame;
	var overlapRule:OverlapRule;
	var liftingEvidence:Array<LiftingQualification>;
}

typedef BoreMaskProtection = {
	var sourceHoleId:String;
	var hardwareId:String;
	var plugLength:Null<Float>;
	var farEndId:Null<String>;
	var withdrawal:{x:Float, y:Float, z:Float};
}

typedef InsertFixtureFixing = {
	var sourceHoleId:String;
	var hardwareId:String;
	var panelId:String;
	var operationId:String;
	var entry:AssemblyFrame;
	var engagement:Float;
	var threadSpec:String;
}

typedef MoldInsertFixture = {
	var insertId:String;
	var location:DatumFixtureLocation.DatumLocationPlan;
	var datums:materia.tooling.casting.MoldPlan.InsertDatums;
	var datum:MoldVolume;
	var mountingFace:AssemblyFrame;
	var withdrawal:{x:Float, y:Float, z:Float};
	var attachmentEvidence:Array<String>;
	var lateralLocation:{primaryId:String, secondaryId:String, baselineMm:Float};
	var fixings:Array<InsertFixtureFixing>;
	var penetrations:Array<materia.tooling.casting.MoldPlan.PenetrationPlan>;
	var boreMasks:Array<BoreMaskProtection>;
	var assemblyClearance:{x:Float, y:Float, z:Float};
}

typedef CastingMoldPlan = {
	var id:String;
	var document:ToolingDocument;
	var sourceFingerprint:String;
	var options:CastingMoldOptions;
	var handling:HandlingPlan;
	var envelope:MoldBox;
	var cavity:MoldBox;
	var panels:PanelAssemblyRecord;
	var supportPanels:Array<PanelPlacement>;
	var cores:Array<CastingMoldCore>;
	var coreUnits:Array<CastingCoreUnit>;
	var formers:Array<MoldFormer>;
	var fixtures:Array<MoldInsertFixture>;
	var features:Array<FeatureTooling>;
	var openings:Array<MoldOpening>;
	var hardware:Array<SupportHardware>;
	var changeRequests:Array<DesignChangeRequest>;
	var validation:Array<materia.tooling.ToolingValidation>;
	var fabrication:materia.tooling.ToolingValidation.FabricationReadiness;
}
