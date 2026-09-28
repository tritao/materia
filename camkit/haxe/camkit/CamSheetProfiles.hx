package camkit;

import cnckit.ir.CncPoint;
import materia.sheet.SheetPartPlacement;
import materia.sheet.SheetCutPlan;
import cadkit.units.LengthUnits;

/** Rectangular MachineKit sheet blanks as CNC profile inputs. */
class CamSheetProfiles {
  public static function fromPlan(plan:SheetCutPlan, placementId:String,
      ?surfaceZMetres:Float = 0.0):CamContour {
    if (plan == null) throw "CAM needs a sheet cut plan";
    for (item in plan.placements)
      if (item.id == placementId)
        return placement(item, plan.lengthUnit, surfaceZMetres);
    throw 'Unknown CAM sheet placement "$placementId"';
  }

  public static function placement(value:SheetPartPlacement,
      unit:String, ?surfaceZMetres:Float = 0.0):CamContour {
    if (value == null || value.width <= 0.0 || value.height <= 0.0 ||
        !Math.isFinite(surfaceZMetres))
      throw "CAM sheet placement needs positive dimensions and a finite surface";
    var factor = LengthUnits.factorToMillimetres(unit);
    if (factor == null) throw 'Unsupported sheet unit "$unit"';
    var scale = factor * 0.001;
    var x = value.x * scale, y = value.y * scale;
    var width = value.width * scale, height = value.height * scale;
    return new CamContour([
      new CncPoint(x, y, surfaceZMetres),
      new CncPoint(x + width, y, surfaceZMetres),
      new CncPoint(x + width, y + height, surfaceZMetres),
      new CncPoint(x, y + height, surfaceZMetres)
    ]);
  }
}
