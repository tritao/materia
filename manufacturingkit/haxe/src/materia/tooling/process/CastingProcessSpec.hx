package materia.tooling.process;

import materia.casting.CastingInput;
import materia.casting.CastingInput.CastingTextInput;

/** The supplier specifies a full section limit or a poured-layer limit.
 * No thermal conductivity, kinetics or heat-transfer data are inferred. */
enum CompoundThicknessLimit {
  Unresolved(reason:String);
  Section(maximumMm:Float);
  Layer(maximumMm:Float);
}

typedef CompoundDatasheet = {
  var reference:CastingTextInput;
  var thickness:CompoundThicknessLimit;
  var maximumMassPerLiftKg:CastingInput;
}

typedef CastabilityLimits = {
  var maximumAggregateSizeMm:Null<Float>;
  var minimumPassageAggregateRatio:Null<Float>;
  var minimumWallAggregateRatio:Float;
  var steelAggregateMarginMm:Float;
  var steelSpacingAggregateRatio:Float;
  var evidence:Array<String>;
}

/** In-memory process choices; shop records belong to the delivered package. */
typedef CastingProcessSpec = {
  var id:String;
  var revision:Int;
  var recipeFingerprint:String;
  var pourRateMm3PerSecond:Null<Float>;
  var maximumAirEscapeWidthMm:Null<Float>;
  var mixComposition:Null<PourPlanning.MixComposition>;
  var mixer:Null<PourPlanning.PourMixerSpec>;
  var castability:CastabilityLimits;
  var compound:CompoundDatasheet;
}

/** Value checks and explicit copies for in-memory process choices. */
class ProcessSpecs {
  static function positive(value:Float):Float {
    if (!Math.isFinite(value) || value <= 0) throw "Invalid casting process number";
    return value;
  }
  static function wallRatio(value:Float):Float {
    if (value == null
      || !Math.isFinite(value) || value < 1 || value > 1e6) throw "Invalid minimum wall/aggregate ratio";
    return value;
  }
  static function optional(value:Null<Float>, minimum:Float, maximum:Float):Null<Float> {
    if (value != null
      && (!Math.isFinite(value) || value < minimum || value > maximum)) throw "Invalid casting process number";
    return value;
  }
  public static function thicknessRecord(value:CompoundThicknessLimit):Dynamic return switch value {
    case Unresolved(reason):
      {kind: "unresolved", reason: reason};
    case Section(maximum):
      {kind: "section", maximumMm: maximum};
    case Layer(maximum):
      {kind: "layer", maximumMm: maximum};
  };
  public static function copy(input:CastingProcessSpec):CastingProcessSpec {
    materia.tooling.ToolingIds.validate(input.id);
    if (input.revision < 1 || input.revision > 100000000
      || !~/^[a-f0-9]{64}$/.match(input.recipeFingerprint)) throw "Invalid process recipe fingerprint/revision";
    var compound = input.compound;
    var reference = materia.casting
      .CastingInput.CastingInputs.text(materia.casting.CastingInput.CastingInputs.textRecord(compound.reference));
    var mass = materia.casting.CastingInput
      .CastingInputs.number(materia.casting.CastingInput.CastingInputs.record(compound.maximumMassPerLiftKg));
    switch mass {
      case Known(value):
        positive(value);
      case Unresolved(_):
    }
    var thickness:CompoundThicknessLimit = switch compound.thickness {
      case Unresolved(reason):
        materia.casting.CastingInput.CastingInputs.text({kind: "unresolved", reason: reason});
        CompoundThicknessLimit.Unresolved(reason);
      case Section(value):
        CompoundThicknessLimit.Section(positive(value));
      case Layer(value):
        CompoundThicknessLimit.Layer(positive(value));
    };
    var limits = input.castability;
    var evidence = limits.evidence.copy();
    if (evidence.length > 512) throw "Casting process array exceeds limit";
    for (value in evidence) {
      if (value == null || StringTools.trim(value).length == 0 || value.length > 2048
        || materia.assembly.AssemblyCodec.containsNul(value)) throw "Invalid casting process evidence";
    }
    return {
      id: input.id,
      revision: input.revision,
      recipeFingerprint: input.recipeFingerprint,
      pourRateMm3PerSecond: optional(
        input.pourRateMm3PerSecond,
        1e-12,
        1e15
      ),
      maximumAirEscapeWidthMm: optional(
        input.maximumAirEscapeWidthMm,
        1e-9,
        1e6
      ),
      mixComposition: input.mixComposition == null ? null : PourPlanning.copyComposition(input.mixComposition),
      mixer : input.mixer == null ? null : PourPlanning.copyMixer(input.mixer),
      castability : {
        maximumAggregateSizeMm : optional(
          limits.maximumAggregateSizeMm,
          1e-9,
          1e6
        ),
        minimumPassageAggregateRatio : optional(
          limits.minimumPassageAggregateRatio,
          1,
          1e6
        ),
        minimumWallAggregateRatio : wallRatio(limits.minimumWallAggregateRatio),
        steelAggregateMarginMm : positive(limits.steelAggregateMarginMm),
        steelSpacingAggregateRatio : wallRatio(limits.steelSpacingAggregateRatio),
        evidence : evidence
      },
      compound : {
        reference : reference,
        thickness : thickness,
        maximumMassPerLiftKg : mass
      }
    };
  }
  static function inputNumber(raw:Dynamic, key:String):Null<Float> {
    var value:Dynamic = Reflect.field(raw, key);
    if (value == null) return null;
    if ((!Std.isOfType(value,
      Float) && !Std.isOfType(value, Int)) || !Math.isFinite(value)) throw "Invalid casting process number";
    return value;
  }
  public static function fromRecord(raw:Dynamic):CastingProcessSpec {
    materia.casting.CastingFeatureCodec.fields(
      raw,
      [
        "id",
        "revision",
        "recipeFingerprint",
        "pourRateMm3PerSecond",
        "maximumAirEscapeWidthMm",
        "mixComposition",
        "mixer",
        "castability",
        "compound"
      ]
    );
    materia.casting.CastingFeatureCodec.fields(
      Reflect.field(raw, "castability"),
      [
        "maximumAggregateSizeMm",
        "minimumPassageAggregateRatio",
        "minimumWallAggregateRatio",
        "steelAggregateMarginMm",
        "steelSpacingAggregateRatio",
        "evidence"
      ]
    );
    var c:Dynamic = Reflect.field(raw, "compound");
    materia.casting.CastingFeatureCodec.fields(c, ["reference", "thickness", "maximumMassPerLiftKg"]);
    var t:Dynamic = Reflect.field(c, "thickness");
    var thickness:CompoundThicknessLimit = switch materia.casting.CastingRecordCodec.string(t, "kind") {
      case "unresolved":
        materia.casting.CastingFeatureCodec.fields(t, ["kind", "reason"]);
        CompoundThicknessLimit.Unresolved(materia.casting.CastingRecordCodec.string(t, "reason"));
      case "section":
        materia.casting.CastingFeatureCodec.fields(t, ["kind", "maximumMm"]);
        CompoundThicknessLimit.Section(materia.casting.CastingRecordCodec.number(t, "maximumMm"));
      case "layer":
        materia.casting.CastingFeatureCodec.fields(t, ["kind", "maximumMm"]);
        CompoundThicknessLimit.Layer(materia.casting.CastingRecordCodec.number(t, "maximumMm"));
      default:
        throw "Unknown compound thickness limit";
    };
    var limits:Dynamic = Reflect.field(raw, "castability");
    var evidence:Array<String> = [];
    for (value in materia.casting.CastingRecordCodec.array(limits, "evidence")) {
      if (!Std.isOfType(value, String)) throw "Invalid casting process evidence";
      var text:String = cast value;
      evidence.push(text);
    }
    var result:CastingProcessSpec = {
      id: materia.casting.CastingRecordCodec.string(raw, "id"),
      revision: materia.casting.CastingRecordCodec.integer(
        raw,
        "revision"
      ),
      recipeFingerprint: materia.casting.CastingRecordCodec.string(
        raw,
        "recipeFingerprint"
      ),
      pourRateMm3PerSecond: inputNumber(
        raw,
        "pourRateMm3PerSecond"
      ),
      maximumAirEscapeWidthMm: inputNumber(
        raw,
        "maximumAirEscapeWidthMm"
      ),
      mixComposition: Reflect.field(
        raw,
        "mixComposition"
      ) == null ? null : PourPlanning.copyComposition(Reflect.field(
        raw,
        "mixComposition"
      )),
      mixer : Reflect.field(
        raw,
        "mixer"
      ) == null ? null : PourPlanning.copyMixer(Reflect.field(
        raw,
        "mixer"
      )),
      castability : {
        maximumAggregateSizeMm : inputNumber(
          limits,
          "maximumAggregateSizeMm"
        ),
        minimumWallAggregateRatio : materia.casting.CastingRecordCodec.number(
          Reflect.field(
            raw,
            "castability"
          ),
          "minimumWallAggregateRatio"
        ),
        steelAggregateMarginMm : materia.casting.CastingRecordCodec.number(
          limits,
          "steelAggregateMarginMm"
        ),
        steelSpacingAggregateRatio : materia.casting.CastingRecordCodec.number(
          limits,
          "steelSpacingAggregateRatio"
        ),
        minimumPassageAggregateRatio : inputNumber(
          limits,
          "minimumPassageAggregateRatio"
        ),
        evidence : evidence
      },
      compound : {
        reference : materia.casting.CastingInput.CastingInputs.text(Reflect.field(
          c,
          "reference"
        )),
        thickness : thickness,
        maximumMassPerLiftKg : materia.casting.CastingInput.CastingInputs.number(Reflect.field(
          c,
          "maximumMassPerLiftKg"
        ))
      }
    };

    return copy(result);
  }
}
