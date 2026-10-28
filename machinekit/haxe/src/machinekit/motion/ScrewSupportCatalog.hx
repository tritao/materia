package machinekit.motion;

import machinekit.assembly.HzMotionCatalog;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/**
 * One maker's evidence for a support row: where its table is, its SKU and weight, its pinned CAD (and the pose that
 * puts that CAD in the support frame), the bolt-hole diameter of its foot or flange, and measured seats: the bolt
 * counterbore above the base and the retaining-ring groove along the shaft (support frame), where reviewed.
 */
typedef ScrewSupportSource = {
  var maker:String;
  var reference:String;
  @:optional var sku:String;
  @:optional var massKg:Float;
  @:optional var cadSha256:String;
  /** Which prepared CAD library holds it: `vallder` or `hzmotion`. */
  @:optional var cadLibrary:String;
  @:optional var cadPose:AssemblyFrame;
  var holeDiameter:Float;
  @:optional var boltSeat:Float;
  @:optional var ringSeatZ:Float;
  /** The CAD's leading solids that are the unit itself, where it also models a loose part (an as-supplied ring). */
  @:optional var unitSolids:Int;
}

/**
 * A support designation's interface, in mm: journal bore, shaft height and the foot or flange it bolts by, the
 * bearing it carries, and the makers who publish it, the first being the one it is bought from. Makers agree on
 * every field here; where one differed it would be its own row.
 */
typedef ScrewSupportRow = {
  var model:String;
  var bore:Float;
  /** Foot-mounted (BK, BF, EF) rather than flange-mounted (FK, FF). */
  var base:Bool;
  var shaftHeight:Float;
  var width:Float;
  var height:Float;
  var length:Float;
  /** Bolt spacing across (foot) or pitch-circle diameter (flange), and along the shaft for four-bolt feet. */
  var pitch:Float;
  var longitudinalPitch:Float;
  var counterbore:Float;
  var bearing:String;
  var sources:Array<ScrewSupportSource>;
}

/** BK/BF/EF/FK/FF supports from the Vallder tables and pinned CAD, with HZMotion's reviewed drawings and STEP files. */
class ScrewSupportCatalog {
  static var cache:Null<Map<String, ScrewSupportRow>>;

  public static function models():Array<String> return VallderSupportData.models().concat(["EF12"]);

  public static function get(model:String):ScrewSupportRow {
    if (cache == null) {
      cache = [];
      for (name in models()) cache.set(name, build(name));
    }
    var row = cache.get(model);
    if (row == null) throw "Support needs interface review: " + model;
    return row;
  }

  /**
	 * The usual bearing of each size: an angular-contact pair (70xx/72xx AC) at fixed ends, a shielded deep-groove
	 * bearing at floating ends, by journal bore. HZMotion's drawings name the same bearings for BF12, BF15 and FK12.
	 */
  static function bearing(fixed:Bool, bore:Float):String return fixed ? switch Math.round(bore) {
    case 10:
      "7000AC";
    case 12:
      "7001AC";
    case 15:
      "7002AC";
    case 17:
      "7203AC";
    case 20:
      "7204AC";
    case 25:
      "7205AC";
    case _:
      throw 'No fixed-end bearing for a $bore mm journal';
  }
  :switch Math.round(bore) {
    case 8:
      "608ZZ";
    case 10:
      "6000ZZ";
    case 15:
      "6002ZZ";
    case 17:
      "6203ZZ";
    case 20:
      "6204ZZ";
    case 25:
      "6205ZZ";
    case _:
      throw 'No floating-end bearing for a $bore mm journal';
  };

  static function build(model:String):ScrewSupportRow {
    var hzmotion = hzmotionSource(model);
    if (model == "EF12") {
      var s = HzMotionCatalog.get(model), holes = s.baseHoles;
      return {
        model: model,
        bore: s.shaftBore,
        base: true,
        shaftHeight: s.shaftHeight,
        width: s.width,
        height: s.rearHeight,
        length: s.length,
        pitch: Math.abs(holes[0].x - holes[1].x),
        longitudinalPitch: 0,
        counterbore: holes[0].diameter,
        bearing: s.bearing,
        sources: [hzmotion]
      };
    }
    var v = VallderSupportData.get(model), fixed = StringTools.startsWith(model, "BK") || StringTools.startsWith(
      model,
      "FK"
    );
    var sources:Array<ScrewSupportSource> = [{
      maker: "Vallder",
      reference: "https://vallder.com/",
      sku: v.sku,
      massKg: v.massKg,
      cadSha256: v.sha256,
      cadLibrary: "vallder",
      cadPose: AssemblyFrames.identity(),
      holeDiameter: v.clearance
    }];
    if (hzmotion != null) sources.push(hzmotion);
    return {
      model: model,
      bore: v.bore,
      base: v.base,
      shaftHeight: v.shaftHeight,
      width: v.width,
      height: v.height,
      length: v.length,
      pitch: v.pitch,
      longitudinalPitch: v.longitudinalPitch,
      counterbore: v.counterbore,
      bearing: hzmotion != null ? HzMotionCatalog.get(model).bearing : bearing(fixed, v.bore),
      sources : sources
    };
  }

  /**
	 * HZMotion's evidence for a plain support it publishes. Its drawings and STEP files use the foot frame (base at
	 * y = 0, front face at z = 0); the support frame has the shaft at the origin and is centred along it.
	 */
  static function hzmotionSource(model:String):Null<ScrewSupportSource> {
    if (["BF12", "BF15", "FK12", "EF12"].indexOf(model) < 0) return null;
    var s = HzMotionCatalog.get(model), base = s.family != "FK";
    var toSupport = base ? AssemblyFrames.translation(0, -s.shaftHeight, -s.length / 2) : AssemblyFrames.identity();
    var source:ScrewSupportSource = {
      maker: "HZMotion",
      reference: HzMotionCatalog.sourceUrl(model),
      holeDiameter: s.baseHoles.length > 0 ? s.baseHoles[0].diameter : 4.5
    };
    if (HzMotionCatalog.cadModels().indexOf(model) >= 0) {
      var asset = HzMotionCatalog.cadAsset(model), t = asset.translation;
      source.cadSha256 = asset.sha256;
      source.cadLibrary = "hzmotion";
      source.cadPose = AssemblyFrames.compose(
        toSupport,
        AssemblyFrames.fromRotationMatrix(t[0], t[1], t[2], asset.rotation)
      );
      if (asset.interfaces != null && asset.interfaces.boltSeatY != null) source.boltSeat = asset.interfaces.boltSeatY;
      // The DIN 471 ring's groove, measured on the pinned BF STEP files (foot frame z 14 and 14.5). The STEP also
      // models the ring loose, as supplied, as its third solid: the ring is its own part in an assembly.
      if (model == "BF12") source.ringSeatZ = 14 - s.length / 2;
      if (model == "BF15") source.ringSeatZ = 14.5 - s.length / 2;
      if (model == "BF12" || model == "BF15") source.unitSolids = 2;
    }
    return source;
  }
}
