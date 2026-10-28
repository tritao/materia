package materia.tooling;

import materia.tooling.panel.PanelStockSpec;
import materia.tooling.casting.CoreStockSpec;

/** Initial manufacturing inputs; panel joints and load models are added with their planners.
 * All lengths are mm. Evidence references record provenance, not automatic qualification.
 */
enum abstract CoreWireAxis(String) to String {
  var X = "x";
  var Y = "y";
  var Z = "z";
}

/** Coplanar steel can use the real shell face; raised faces still need datums. */
enum abstract FixtureReceiver(String) to String {
  var IndependentDatums = "independent-datums";
  var PanelWhenCoplanar = "panel-when-coplanar";
}

class FixtureReceivers {
  public static function decode(token:String):FixtureReceiver return switch token {
    case "independent-datums":
      FixtureReceiver.IndependentDatums;
    case "panel-when-coplanar":
      FixtureReceiver.PanelWhenCoplanar;
    default:
      throw "Unknown fixture receiver: " + token;
  };
}

typedef ToolingHardware = {
  var fixtureReceiver:FixtureReceiver;
  var plugDepth:Float;
  var plugMaterial:String;
  var sealMargin:Float;
  var sealMaterial:String;
  var minimumBridgeWidth:Float;
  var locatingPinDiameter:Float;
  var supportScrewDiameter:Float;
  var supportClearanceDiameter:Float;
  var secondaryPinTravel:Float;
  var padDepth:Float;
  var labelOffset:Float;
  var datumThickness:Float;
  var datumPostWidth:Float;
  var formerFlangeThickness:Float;
  var formerFlangeMargin:Float;
  var fixturePinDiameter:Float;
  var fixturePinEmbed:Float;
  var fixturePinSpacing:Float;
  var fixtureSlotToolDiameter:Float;
}

typedef CastingReviewLimits = {
  var minimumCornerRadiusMm:Null<Float>;
  var minimumInsertCoverMm:Null<Float>;
  var minimumInsertEdgeMm:Null<Float>;
  var minimumFixingAccessMm:Null<Float>;
  var minimumLiftingPoints:Null<Int>;
  var removalClearanceMm:Null<Float>;
}

typedef ToolingRecipe = {
  var former:materia.tooling.casting.FormerConstruction.FormerRecipe;
  var featureRules:Array<materia.tooling.casting.CastingStrategies.CastingStrategyRule>;
  var designReview:CastingReviewLimits;
  var id:String;
  var revision:Int;
  var panel:PanelStockSpec;
  var core:CoreStockSpec;
  var jointClearance:Float;

  /** Total extra opening consumed by coatings and machining uncertainty. */
  var jointLiningAllowance:Float;

  var jointMachiningAllowance:Float;
  var hardware:ToolingHardware;
  var coreWireAxis:CoreWireAxis;
  var adhesiveLineThickness:Float;
  var mixDensityKgM3:Float;
  var steelDensityKgM3:Float;
  var handlingLoadFactor:Float;
  var linerThickness:Float;
  var routerDiameter:Float;
  var minimumLigament:Float;
  var resinSpec:String;
  var releaseSystem:materia.tooling.ReleaseSystem;
  var adhesiveSpec:String;
  var processEvidence:Array<String>;
}
