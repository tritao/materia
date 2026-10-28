package materia.casting;

import materia.casting.InsertManufacturing.InsertHoleRoute;
import materia.casting.CastingInput;
import materia.casting.CastingInput.CastingInputs as I;

/** Manufacturing error allocations, separate from the numeric hole design limits.
 * In-plane magnitudes refer to the insert XY datum origin; the angular term
 * acts over each hole's actual radial distance from that origin.
 * Unresolved values identify missing accuracy inputs, never thread-policy limits.
 */
typedef HoleClearanceBudget = {
  var boltDiameterMm:Float;
  var clearanceDiameterMm:CastingInput;
  var distanceFromDatumMm:Float;
  var placementOffsetMm:CastingInput;
  var placementAngleRad:CastingInput;
  var assemblyAdjustmentMm:CastingInput;
  var cureBowMm:CastingInput;
  var machiningErrorMm:CastingInput;
  var pitchAccumulationMm:CastingInput;
  var requiredMarginMm:CastingInput;
}

typedef HoleClearanceAssessment = {
  var beforeCastingMarginMm:Null<Float>;
  var transferMarginMm:Null<Float>;
  var missingInputs:Array<String>;
  var missingTransferInputs:Array<String>;
}

enum HoleRouteDecision {
  UsePreCast(marginMm:Float);
  UseTransfer(marginMm:Float);
  ProposeTransfer(missingInputs:Array<String>);
  AwaitInputs(missingInputs:Array<String>);
  ChangeDesign(reason:String);
}

class HoleClearanceBudgets {
  public static function snapshot(raw:Dynamic):HoleClearanceBudget {
    CastingFeatureCodec.fields(
      raw,
      [
        "boltDiameterMm",
        "clearanceDiameterMm",
        "distanceFromDatumMm",
        "placementOffsetMm",
        "placementAngleRad",
        "assemblyAdjustmentMm",
        "cureBowMm",
        "machiningErrorMm",
        "pitchAccumulationMm",
        "requiredMarginMm"
      ]
    );
    function nonnegative(name:String):Float {
      var value = CastingRecordCodec.number(raw, name);
      if (value < 0) throw "Negative hole clearance budget: " + name;
      return value;
    }
    function input(name:String):CastingInput {
      var result = I.number(Reflect.field(raw, name));
      switch result {
        case Known(value):
          if (value < 0) throw "Negative hole clearance budget: " + name;
        case Unresolved(_):
      }
      return result;
    }
    var result:HoleClearanceBudget = {
      boltDiameterMm: nonnegative("boltDiameterMm"),
      clearanceDiameterMm: input("clearanceDiameterMm"),
      distanceFromDatumMm: nonnegative("distanceFromDatumMm"),
      placementOffsetMm: input("placementOffsetMm"),
      placementAngleRad: input("placementAngleRad"),
      assemblyAdjustmentMm: input("assemblyAdjustmentMm"),
      cureBowMm: input("cureBowMm"),
      machiningErrorMm: input("machiningErrorMm"),
      pitchAccumulationMm: input("pitchAccumulationMm"),
      requiredMarginMm: input("requiredMarginMm")
    };
    var diameter = result.clearanceDiameterMm;
    var invalidDiameter = switch diameter {
      case Known(value):
        value <= 0;
      case Unresolved(_):
        false;
    };
    if (result.boltDiameterMm <= 0 || invalidDiameter) throw "Hole clearance budget diameters must be positive";
    var angle = result.placementAngleRad;
    var invalidAngle = switch angle {
      case Known(value):
        value > Math.PI;
      case Unresolved(_):
        false;
    };
    if (invalidAngle) throw "Hole placement angle exceeds pi radians";
    return result;
  }

  public static function record(value:HoleClearanceBudget):Dynamic return {
    boltDiameterMm: value.boltDiameterMm,
    distanceFromDatumMm: value.distanceFromDatumMm,
    clearanceDiameterMm: I.record(value.clearanceDiameterMm),
    placementOffsetMm: I.record(value.placementOffsetMm),
    placementAngleRad: I.record(value.placementAngleRad),
    assemblyAdjustmentMm: I.record(value.assemblyAdjustmentMm),
    cureBowMm: I.record(value.cureBowMm),
    machiningErrorMm: I.record(value.machiningErrorMm),
    pitchAccumulationMm: I.record(value.pitchAccumulationMm),
    requiredMarginMm: I.record(value.requiredMarginMm)
  };

  public static function copy(value:HoleClearanceBudget):HoleClearanceBudget return snapshot(record(value));

  /** Worst-case vector magnitudes add; no statistical independence is assumed.
	 * Rotation contributes the chord displacement from the actual datum lever arm.
	 * Transfer drilling removes casting placement, adjustment, bow and pitch error;
	 * its drill/guide error and the required clearance margin still apply.
	 */
  public static function assess(raw:HoleClearanceBudget):HoleClearanceAssessment {
    var budget = copy(raw);
    var missing = I.missing([
      budget.clearanceDiameterMm,
      budget.placementOffsetMm,
      budget.placementAngleRad,
      budget.assemblyAdjustmentMm,
      budget.cureBowMm,
      budget.machiningErrorMm,
      budget.pitchAccumulationMm,
      budget.requiredMarginMm
    ]);
    var transferMissing = I.missing([budget.clearanceDiameterMm, budget.machiningErrorMm, budget.requiredMarginMm]);
    var transfer:Null<Float> = transferMissing.length > 0 ? null : (I.require(budget.clearanceDiameterMm)
      - budget.boltDiameterMm) / 2 - I.require(budget.machiningErrorMm) - I.require(budget.requiredMarginMm);
    var before:Null<Float> = null;
    if (missing.length == 0) {
      var available:Float = cast transfer;
      before = available - I.require(budget.placementOffsetMm)
        - 2 * budget.distanceFromDatumMm * Math.sin(I.require(budget.placementAngleRad) / 2)
        - I.require(budget.assemblyAdjustmentMm) - I.require(budget.cureBowMm) - I.require(budget.pitchAccumulationMm);
    }
    return {
      beforeCastingMarginMm: before,
      transferMarginMm: transfer,
      missingInputs: missing,
      missingTransferInputs: transferMissing
    };
  }

  public static function choose(raw:HoleClearanceBudget, permitted:Array<InsertHoleRoute>):HoleRouteDecision {
    if (permitted == null) throw "Missing permitted hole routes";
    var seen:Array<InsertHoleRoute> = [];
    for (route in permitted) {
      if (route != InsertHoleRoute.BeforeCasting
        && route != InsertHoleRoute.TransferAfterCasting) throw "Unknown permitted hole route";
      if (seen.indexOf(route) >= 0) throw "Duplicate permitted hole route";
      seen.push(route);
    }
    var result = assess(raw), before = result.beforeCastingMarginMm, transfer = result.transferMarginMm;
    var allowBefore = permitted.indexOf(InsertHoleRoute.BeforeCasting) >= 0;
    var allowTransfer = permitted.indexOf(InsertHoleRoute.TransferAfterCasting) >= 0;
    if (allowBefore && before != null && before >= 0) return UsePreCast(before);
    if (allowTransfer) {
      if (transfer == null) return ProposeTransfer(result.missingTransferInputs);
      if (transfer >= 0) return UseTransfer(transfer);
    }
    // Every omitted pre-cast term is nonnegative. A negative transfer margin
    // therefore also proves the pre-cast route cannot close.
    if (allowBefore && before == null && (transfer == null || transfer >= 0)) return AwaitInputs(result.missingInputs);
    return ChangeDesign("No permitted hole route closes the bolt-clearance budget");
  }
}
