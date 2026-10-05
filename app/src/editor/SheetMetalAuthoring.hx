package app.editor;

import cadkit.sheetmetal.BendRule;
import cadkit.sheetmetal.KFactorTable;
import cadkit.sheetmetal.SheetMaterial;

/** Unsaved form input; a stock becomes typed only after explicit material and K-factor input. */
class SheetMetalAuthoring {
  public var materialId:String = "";
  public var thickness:String = "1";
  public var kFactor:String = "";
  public var radius:String = "1";
  public var length:String = "40";
  public var angleDegrees:String = "90";
  public function new() {}
  public static function number(value:String, label:String):Float {
    var text = StringTools.trim(value);
    var index = 0;
    if (text.charAt(index) == "+" || text.charAt(index) == "-") index++;
    var digits = 0;
    while (digit(text, index)) { index++; digits++; }
    if (text.charAt(index) == ".") {
      index++;
      while (digit(text, index)) { index++; digits++; }
    }
    if (digits == 0) throw label + " requires a finite number";
    if (text.charAt(index) == "e" || text.charAt(index) == "E") {
      index++;
      if (text.charAt(index) == "+" || text.charAt(index) == "-") index++;
      var exponentDigits = 0;
      while (digit(text, index)) { index++; exponentDigits++; }
      if (exponentDigits == 0) throw label + " requires a finite number";
    }
    var result = Std.parseFloat(text);
    if (index != text.length || !Math.isFinite(result)) throw label + " requires a finite number";
    return result;
  }
  static function digit(text:String, index:Int):Bool {
    var code = text.charCodeAt(index);
    return code != null && code >= 48 && code <= 57;
  }
  public function stock():SheetMaterial {
    if (StringTools.trim(materialId).length == 0) throw "Choose a material identity";
    return new SheetMaterial(materialId, number(thickness, "Thickness"),
      BendRule.KFactor(new KFactorTable([0], [number(kFactor, "ANSI K factor")])));
  }
}
