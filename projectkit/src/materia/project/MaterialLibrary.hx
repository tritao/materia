package materia.project;

import materia.project.MaterialDef;
import materia.project.Appearance;

/**
 * Built-in material definitions shared by generators, the BOM, and the editor.
 *
 * Visuals are metallic-roughness PBR, as SceneKit and glTF read them: `baseColor` is sRGB. A bare metal has
 * metallic 1 and its base colour is its reflectance at normal incidence (steel about 0.56 linear, sRGB 0.78;
 * aluminium about 0.91 linear), so metal reads as metal under the viewport look's studio room. Coatings that
 * hide the metal (black oxide, paint) are dark or dielectric instead.
 */
class MaterialLibrary {
  static function material(id:String, name:String, color:Array<Float>, metallic:Float,
      roughness:Float, density:Float, spec:String):MaterialDef return {
    id: id, name: name,
    visual: {baseColor: color, metallic: metallic, roughness: roughness},
    physical: {density: density, spec: spec}
  };

  public static function all():Array<MaterialDef> return [
    material("neutral", "Neutral", [0.7, 0.7, 0.7], 0.0, 0.65, 1000, "unspecified"),
    material("painted", "Painted steel", [0.24, 0.34, 0.43], 0.0, 0.48, 7850, "painted steel"),
    material("machined-steel", "Machined steel", [0.77, 0.78, 0.79], 1.0, 0.34, 7850, "steel"),
    material("steel-c45", "C45 steel", [0.77, 0.78, 0.79], 1.0, 0.34, 7850, "steel C45"),
    material("steel-8", "Class 8 steel", [0.77, 0.78, 0.79], 1.0, 0.34, 7850, "steel 8"),
    material("steel-8-8", "Class 8.8 steel", [0.77, 0.78, 0.79], 1.0, 0.34, 7850, "steel 8.8"),
    material("steel-10-9", "Class 10.9 steel", [0.77, 0.78, 0.79], 1.0, 0.34, 7850, "steel 10.9"),
    material("stainless-a2-70", "A2-70 stainless steel", [0.79, 0.79, 0.78], 1.0, 0.32, 7900, "stainless A2-70"),
    material("steel-12-9", "Class 12.9 steel", [0.18, 0.19, 0.20], 0.55, 0.52, 7850, "steel 12.9"),
    material("black-oxide", "Black oxide steel", [0.18, 0.19, 0.20], 0.55, 0.52, 7850, "steel 12.9"),
    material("bearing-steel", "Bearing steel", [0.79, 0.80, 0.81], 1.0, 0.30, 7810, "bearing steel"),
    material("aluminium", "Anodized aluminium 6061", [0.86, 0.87, 0.88], 1.0, 0.42, 2700, "aluminium 6061"),
    material("rubber", "Rubber", [0.10, 0.10, 0.11], 0.0, 0.90, 1100, "rubber"),
    material("nitrile-rubber", "Nitrile rubber", [0.10, 0.10, 0.11], 0.0, 0.90, 1100, "nitrile rubber NBR"),
    // Nominal planning density; print-lot properties must be supplied for production validation.
    material("pa12-nylon", "PA12 nylon (nominal)", [0.68, 0.69, 0.71], 0.0, 0.85, 1000, "PA12 nylon"),
    material("plastic", "Engineering plastic", [0.28, 0.30, 0.32], 0.0, 0.55, 1200, "plastic"),
    // Housings of purchased electrical devices (DIN-rail modules, drives, controller cabinets): what is seen and what
    // the BOM names. Their mass comes from an assumed effective density, not this density (machinekit EffectiveDensities).
    material("device-housing", "Device housing, light grey", [0.80, 0.80, 0.78], 0.0, 0.55, 1200, "device housing"),
    material("device-housing-dark", "Device housing, dark grey", [0.20, 0.21, 0.22], 0.0, 0.50, 1200, "device housing dark"),
    polycarbonate(),
    material("polyurethane", "Polyurethane", [0.26, 0.34, 0.37], 0.0, 0.65, 1200, "polyurethane PU"),
    routerMaterial("plywood-birch", "Birch plywood", 680, 0.12, 0.08, 0.15, 0.25),
    // Assumed layout density; lot-specific permeability is supplied by the spoilboard recipe.
    routerMaterial("mdf", "MDF (nominal)", 750, 0.10, 0.08, 0.15, 0.25),
    routerMaterial("acrylic", "PMMA acrylic (assumed)", 1180, 0.30, 0.05, 0.10, 0.18),
    toolingEPS(),
    toolingBoard(),
    mineralCast(),
    material("carbide", "Cemented carbide", [0.70, 0.71, 0.72], 1.0, 0.35, 14500, "carbide"),
    material("cast-iron", "Cast iron", [0.58, 0.59, 0.60], 1.0, 0.55, 7200, "cast iron"),
    material("bronze", "Bronze", [0.80, 0.60, 0.38], 1.0, 0.38, 8800, "bronze"),
    material("brass", "Brass", [0.92, 0.82, 0.55], 1.0, 0.34, 8500, "brass"),
    material("spring-steel", "Spring steel", [0.72, 0.73, 0.75], 1.0, 0.38, 7850, "spring steel"),
    sheetSteel(), sheetAluminium(),
    enclosureMaterial("compact-laminate", "Compact laminate", 1400),
    enclosureMaterial("grp", "Glass reinforced plastic", 1800),
    enclosureMaterial("pp", "Polypropylene", 900),
    enclosureMaterial("hdpe", "HDPE", 950),
    enclosureMaterial("uhmw-pe", "UHMW polyethylene", 930),
    enclosureMaterial("fr-acm", "FR aluminium composite panel", 1800),
    enclosureMaterial("al-honeycomb", "Aluminium honeycomb panel", 200),
    enclosureMaterial("insulated-sandwich", "Insulated sandwich panel", 100),
    enclosureMaterial("stainless-sheet", "Stainless sheet", 7900),
    enclosureMaterial("panel-adhesive", "Panel adhesive", 1200)
  ];

  static function routerMaterial(id:String,name:String,density:Float,energy:Float,low:Float,nominal:Float,high:Float):MaterialDef {
    var spec=id=="mdf"?"MDF":id=="plywood-birch"?"birch plywood":"PMMA acrylic";
    var result=material(id,name,[0.70,0.55,0.34],0,0.72,density,spec);
    result.routerCutting={assumed:true,specificEnergyJPerMm3:energy,chipLoads:[
      {minDiameterMm:3,maxDiameterMm:6,minChipMm:low,nominalChipMm:nominal,maxChipMm:high},
      {minDiameterMm:6.000001,maxDiameterMm:12,minChipMm:low*1.5,nominalChipMm:nominal*1.5,maxChipMm:high*1.5}]};
    return result;
  }

  /** Clear viewing sheet; visual opacity does not change physical material properties. */
  static function polycarbonate():MaterialDef {
    var result = material("polycarbonate", "Polycarbonate", [0.58, 0.74, 0.79], 0, 0.2, 1200, "polycarbonate");
    result.visual.alpha = 0.18;
    return result;
  }

  /** Planning densities only. IDs imply no chemical, fire or impact qualification. */
  static function enclosureMaterial(id:String, name:String, density:Float):MaterialDef {
    var result = material(id, name + " (assumed)", [0.65, 0.68, 0.70], 0, 0.6, density, name);
    result.physical.propertiesAssumed = true;
    return result;
  }

  /** Display/BOM assumptions only; supplier lot data belongs in tooling stock records. */
  static function toolingEPS():MaterialDef {
    var result=material("tooling-eps","EPS tooling (nominal)",[0.93,0.94,0.95],0,0.95,20,"EPS tooling");
    result.physical.propertiesAssumed=true;return result;
  }
  static function toolingBoard():MaterialDef {
    var result=material("tooling-coated-board","Coated particleboard (nominal)",[0.85,0.83,0.73],0,0.65,650,"coated particleboard");
    result.physical.propertiesAssumed=true;return result;
  }

  /** Assumed epoxy-granite bulk inputs, for checks that explicitly consume them.
      eta=0.02 uses an assumed cast-iron reference eta=0.0025 (8x); not a modal result. */
  static function mineralCast():MaterialDef {
    var result=material("mineral-cast","Mineral cast",[0.16,0.17,0.18],0.0,0.95,2400,"mineral cast");
    result.physical.youngsModulusPa=35e9;
    result.physical.dampingLossFactor=0.02;
    result.physical.thermalExpansionPerK=14e-6;
    result.physical.thermalConductivityWPerMK=2;
    result.physical.propertiesAssumed=true;
    return result;
  }

  /** Plan material A. Mechanical values and development defaults are assumed, not lot certificates. */
  static function sheetSteel():MaterialDef {
    var result = material("mild-steel", "Mild steel sheet", [0.77, 0.78, 0.79], 1.0, 0.34, 7850, "mild steel sheet");
    result.mechanical = {uts: 400, yield: 250, youngs: 205000};
    result.sheet = {kFactorTable: [{ratio: 0, k: 0.30}, {ratio: 1, k: 0.38}, {ratio: 3, k: 0.45}],
      defaultRadiusRatio: 1, minRadiusRatio: 0.5, grain: true};
    return result;
  }

  /** Plan material B. Mechanical values and development defaults are assumed, not lot certificates. */
  static function sheetAluminium():MaterialDef {
    var result = material("aluminium-5052-h32", "5052-H32 aluminium sheet", [0.88, 0.89, 0.90], 1.0, 0.38, 2680, "aluminium 5052-H32 sheet");
    result.mechanical = {uts: 228, yield: 193, youngs: 70000};
    result.sheet = {kFactorTable: [{ratio: 0, k: 0.33}, {ratio: 1, k: 0.40}, {ratio: 3, k: 0.45}],
      defaultRadiusRatio: 1, minRadiusRatio: 1, grain: true};
    return result;
  }

  public static function sheetMaterials():Array<MaterialDef> {
    return [for (item in all()) if (item.sheet != null) item];
  }

  public static function get(id:String):Null<MaterialDef> {
    for (item in all()) if (item.id == id) return item;
    return null;
  }

  public static function require(id:String):MaterialDef {
    var result = get(id);
    if (result == null) throw 'Unknown material "$id"';
    return result;
  }

  public static function specs():Array<String> {
    var result:Array<String> = [];
    for (item in all()) if (result.indexOf(item.physical.spec) < 0) result.push(item.physical.spec);
    return result;
  }

  public static function validateCustom(items:Array<MaterialDef>):Void {
    if (items == null || items.length > 1000) throw "Invalid custom material library";
    var seen = new Map<String, Bool>();
    for (item in items) {
      if (item == null || item.id == null || StringTools.trim(item.id).length == 0 ||
          get(item.id) != null || seen.exists(item.id) || item.name == null ||
          StringTools.trim(item.name).length == 0 || item.visual == null ||
          item.visual.baseColor == null || item.visual.baseColor.length != 3 ||
          item.physical == null || !Math.isFinite(item.physical.density) ||
          item.physical.density <= 0 || item.physical.spec == null ||
          StringTools.trim(item.physical.spec).length == 0 ||
          !Math.isFinite(item.visual.metallic) || item.visual.metallic < 0 || item.visual.metallic > 1 ||
          !Math.isFinite(item.visual.roughness) || item.visual.roughness < 0 || item.visual.roughness > 1)
        throw "Invalid custom material";
      for (channel in item.visual.baseColor)
        if (!Math.isFinite(channel) || channel < 0 || channel > 1)
          throw "Invalid custom material color";
      var emissive = item.visual.emissive;
      if (emissive != null) {
        if (emissive.length != 3) throw "Invalid custom material emissive color";
        for (channel in emissive)
          if (!Math.isFinite(channel) || channel < 0 || channel > 1) throw "Invalid custom material emissive color";
      }
      var alpha = item.visual.alpha;
      if (alpha != null && (!Math.isFinite(alpha) || alpha < 0 || alpha > 1))
        throw "Invalid custom material alpha";
      validateSheetProperties(item);
      seen.set(item.id, true);
    }
  }

  /** Validate optional structural and development data without assigning defaults. */
  public static function validateSheetProperties(item:MaterialDef):Void {
    var mechanical = item.mechanical;
    if (mechanical != null && (!Math.isFinite(mechanical.uts) || mechanical.uts <= 0
        || !Math.isFinite(mechanical.yield) || mechanical.yield <= 0 || mechanical.yield > mechanical.uts
        || !Math.isFinite(mechanical.youngs) || mechanical.youngs <= mechanical.yield))
      throw "Invalid material mechanical properties";
    var sheet = item.sheet;
    if (sheet == null) return;
    if (!Math.isFinite(sheet.defaultRadiusRatio) || !Math.isFinite(sheet.minRadiusRatio)
        || sheet.minRadiusRatio < 0 || sheet.defaultRadiusRatio < sheet.minRadiusRatio
        || sheet.kFactorTable == null || sheet.kFactorTable.length == 0)
      throw "Invalid material sheet properties";
    var previous = -1.0;
    for (row in sheet.kFactorTable) {
      if (!Math.isFinite(row.ratio) || row.ratio < 0 || row.ratio <= previous
          || !Math.isFinite(row.k) || row.k < 0 || row.k > 0.5)
        throw "Invalid material K-factor table";
      previous = row.ratio;
    }
  }

  public static function fromSpec(spec:Null<String>):String {
    if (spec == null) return "painted";
    for (item in all()) if (item.physical.spec == spec) return item.id;
    throw 'Unknown material specification "$spec"';
  }

  public static function appearance(id:String):Appearance {
    var item = require(id);
    return {finish: item.id, metallic: item.visual.metallic, roughness: item.visual.roughness};
  }
}
