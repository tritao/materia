package materia.project;

import materia.project.MaterialDef;
import materia.project.Appearance;

/** Built-in material definitions shared by generators, the BOM, and the editor. */
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
    material("machined-steel", "Machined steel", [0.62, 0.64, 0.67], 0.8, 0.34, 7850, "steel"),
    material("steel-c45", "C45 steel", [0.62, 0.64, 0.67], 0.8, 0.34, 7850, "steel C45"),
    material("steel-8", "Class 8 steel", [0.62, 0.64, 0.67], 0.8, 0.34, 7850, "steel 8"),
    material("steel-8-8", "Class 8.8 steel", [0.62, 0.64, 0.67], 0.8, 0.34, 7850, "steel 8.8"),
    material("steel-12-9", "Class 12.9 steel", [0.18, 0.19, 0.20], 0.55, 0.52, 7850, "steel 12.9"),
    material("black-oxide", "Black oxide steel", [0.18, 0.19, 0.20], 0.55, 0.52, 7850, "steel 12.9"),
    material("bearing-steel", "Bearing steel", [0.58, 0.61, 0.64], 0.78, 0.30, 7810, "bearing steel"),
    material("aluminium", "Anodized aluminium 6061", [0.68, 0.70, 0.72], 0.72, 0.42, 2700, "aluminium 6061"),
    material("rubber", "Rubber", [0.10, 0.10, 0.11], 0.0, 0.90, 1100, "rubber"),
    material("plywood-birch", "Birch plywood", [0.70, 0.55, 0.34], 0.0, 0.72, 680, "birch plywood"),
    material("cast-iron", "Cast iron", [0.35, 0.37, 0.39], 0.60, 0.55, 7200, "cast iron"),
    material("bronze", "Bronze", [0.55, 0.38, 0.20], 0.70, 0.38, 8800, "bronze"),
    material("spring-steel", "Spring steel", [0.56, 0.58, 0.60], 0.75, 0.38, 7850, "spring steel")
  ];

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
      seen.set(item.id, true);
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
