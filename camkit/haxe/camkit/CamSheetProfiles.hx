package camkit;

import toolpathkit.path.Point3;
import materia.sheet.SheetPartPlacement;
import materia.sheet.SheetCutPlan;
import cadkit.units.LengthUnits;

/** Rectangular MachineKit sheet blanks as CNC profile inputs. */
class CamSheetProfiles {
  /** SR10 exact part boundaries are mm; CAM placement and cutting coordinates are metres. */
  public static function fromDxf(part:cadkit.drawing.DxfContours.DxfPart,surfaceZMetres:Float=0.0,
      chordToleranceMetres:Float=0.00005):{outer:CamContour,holes:Array<CamContour>} {
    if(part==null||!Math.isFinite(surfaceZMetres)||!Math.isFinite(chordToleranceMetres)||chordToleranceMetres<=0)
      throw "DXF CAM needs a part, finite surface and positive chord tolerance";
    function contour(loop:Array<cadkit.sheetmetal.Segment>):CamContour {
      var boundary:Array<cadkit.sheetmetal.Segment> = [],points:Array<Point3> = [];
      for(s in loop) {
        var count=s.isLine()?1:Std.int(Math.max(1,Math.ceil(Math.abs(s.sweep)/Math.PI)));
        for(i in 0...count) {
          var segment=s.isLine()?cadkit.sheetmetal.Segment.line(new cadkit.sheetmetal.Point2(s.start.x*0.001,s.start.y*0.001),
            new cadkit.sheetmetal.Point2(s.end.x*0.001,s.end.y*0.001)):
            cadkit.sheetmetal.Segment.arc(new cadkit.sheetmetal.Point2(s.arcCenter().x*0.001,s.arcCenter().y*0.001),
              s.radius*0.001,s.startAngle+s.sweep*i/count,s.sweep/count);
          boundary.push(segment);
          var steps=1;
          if(!segment.isLine()) {
            var angle=2*Math.acos(Math.max(-1,Math.min(1,1-chordToleranceMetres/segment.radius)));
            steps=Std.int(Math.max(2,Math.ceil(Math.abs(segment.sweep)/angle)));
          }
          if(steps>1000000||points.length+steps>1000000)throw "DXF CAM contour exceeds sampling limit";
          for(j in 0...steps){var p=segment.point(j/steps);points.push(new Point3(p.x,p.y,surfaceZMetres));}
        }
      }
      var result=new CamContour(points,true);result.exactBoundary=boundary;result.chordTolerance=chordToleranceMetres;return result;
    }
    return {outer:contour(part.outer.segments),holes:[for(hole in part.holes)contour(hole.segments)]};
  }

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
      new Point3(x, y, surfaceZMetres),
      new Point3(x + width, y, surfaceZMetres),
      new Point3(x + width, y + height, surfaceZMetres),
      new Point3(x, y + height, surfaceZMetres)
    ]);
  }
}
