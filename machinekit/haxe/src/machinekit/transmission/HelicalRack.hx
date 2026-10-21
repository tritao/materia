package machinekit.transmission;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import materia.assembly.AssemblyFrames as F;

/** Assumed catalogue-class normal-module helical rack. Dimensions mm; SI ratings unless stated.
 * Geometry is a nominal layout envelope, not manufacturing or supplier geometry. */
class HelicalRack extends AssumedCatalogComponent {
  /** Lies along its length (+Z) with its teeth (+Y) up. */
  override public function presentationView():Null<machinekit.component.ComponentView>
    return machinekit.component.ComponentView.lengthAlongZ();

  static var recipe:Null<ComponentType>;
  static function inputs():Array<ComponentParameter> return [
    P.length("normalModule", 1.5),
    P.count("teeth", 20),
    P.length("faceWidth", 20),
    P.quantity("helixAngle", Math.acos(3 * Math.PI / 10), "rad"),
    P.quantity("pressureAngle", 0.3490658504, "rad"),
    P.choice("hand", [ "right", "left" ], "left"),
    P.length("barHeight", 12),
    P.length("backlash", 0.02),
    P.quantity("meshStiffness", 50000, "N/mm"),
    P.scalar("efficiency", 0.95, 0, 1)
  ];
  public function new(?v:ComponentValues) {
    super("helical-rack", "normal-module helical rack", "steel", inputs(), v);
    positive(["normalModule", "faceWidth", "helixAngle", "pressureAngle", "barHeight", "meshStiffness", "efficiency"]);
    require(
      parameterCount("teeth") >= 1 && n("helixAngle") < Math.PI / 4 && SpurGear.validPressureAngle(n("pressureAngle")),
      "unsupported tooth count/angles"
    );
    require(n("backlash") < Math.PI * transverseModule() / 4, "backlash leaves insufficient tooth material");
    addConnector("pitch", Pitch, Solids.axial(n("faceWidth") / 2, 0, 0));
    addConnector("end", Mount, Solids.axial(n("faceWidth") / 2, 0, length()));
  }
  public static function recipeType():ComponentType {
    if (recipe == null) recipe = new ComponentType(
      "machinekit.transmission.helical-rack",
      inputs(),
      v -> new HelicalRack(v)
    );
    return recipe;
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function geometry(detail:ComponentDetail = Preview):Part {
    // Bound OCCT wire validation cost at full-sheet lengths. Whole-pitch pieces
    // meet on square faces with identical tooth phase; their volumes do not overlap.
    var total=parameterCount("teeth");
    if(total>80)return Solids.building([],parts->{
      var prototype=new HelicalRack(values().setInteger("teeth",80)).geometry(detail);parts.push(prototype);
      var copies:Array<Part> =[];
      var offset=0;
      while(offset<total) {
        var count=Std.int(Math.min(80,total-offset));
        var piece=count==80?prototype:new HelicalRack(values().setInteger("teeth",count)).geometry(detail);
        if(piece!=prototype)parts.push(piece);
        var placed=piece.translated(new Vector(0,0,offset*Math.PI*transverseModule()));parts.push(placed);copies.push(placed);
        offset+=count;
      }
      return new Part(cadkit.Shape.compound([for(part in copies)part.shape]));
    });
    // Skew a transverse rack profile, then trim ends square for whole-pitch joints.
    var p = Math.PI * transverseModule(), add = n("normalModule"),
      ded = 1.25 * add, bottom = -ded - n("barHeight"), slope = Math.tan(transversePressureAngle());
    var count = parameterCount("teeth"), extra = Math.ceil(n("faceWidth") * Math.tan(n("helixAngle")) / p) + 1;
    var sections:Array<cadkit.modeling.Curve> = [];
    try {
      for (side in 0...2) {
        var x = side * n("faceWidth"),
          shift =(parameterToken("hand") == "right" ? 1 : -1) * x * Math.tan(n("helixAngle"));
        var pts = [new Vector(x, bottom, -extra * p + shift), new Vector(x, -ded, -extra * p + shift)];
        for (k in - extra...count + extra) {
          var center =(k + 0.5) * p + shift, half = p / 4 - n("backlash") / 2;
          pts.push(new Vector(x, -ded, center - half - ded * slope));
          pts.push(new Vector(x, add, center - half + add * slope));
          pts.push(new Vector(x, add, center + half - add * slope));
          pts.push(new Vector(x, -ded, center + half + ded * slope));
        }
        pts.push(new Vector(x, -ded,(count + extra) * p + shift));
        pts.push(new Vector(x, bottom,(count + extra) * p + shift));
        sections.push(cadkit.modeling.Curve.polyline(pts, true));
      }
      var raw = Part.loft(sections, true);
      for (section in sections) section.close();
      sections = [];
      var clip = Part.box(n("faceWidth"), add - bottom, length(), Min, Min, Min), placed = clip.translated(new Vector(
        0,
        bottom,
        0
      ));
      clip.close();
      var result:Part = null;
      try {
        result = raw.intersect(placed);
        raw.close();
        placed.close();
      } catch (error:Dynamic) {
        raw.close();
        placed.close();
        throw error;
      }
      return Solids.named(result, "square-ended-helical-rack");
    } catch (error:Dynamic) {
      for (section in sections) section.close();
      throw error;
    }

  }
  public function transverseModule():Float return n("normalModule") / Math.cos(n("helixAngle"));
  public function transversePressureAngle():Float return Math.atan(Math.tan(n("pressureAngle"))
    / Math.cos(n("helixAngle")));
  public function length():Float return parameterCount("teeth") * Math.PI * transverseModule();
  /** rad/mm for pinion rotation driven by rack travel, matching Rack.relation. */
  public function relation(pinion:HelicalPinion, alignment:Float = 1):TransmissionRelation {
    for (field in["normalModule", "helixAngle", "pressureAngle"]) require(
      Math.abs(n(field) - pinion.n(field)) < 1e-10,
      "rack and pinion differ in " + field
    );
    require(parameterToken("hand") != pinion.parameterToken("hand"), "rack and pinion need opposite hands");
    require(Math.isFinite(alignment) && Math.abs(Math.abs(alignment) - 1) < 1e-10, "alignment must be +1 or -1");
    var result = new TransmissionRelation(
      alignment * 2 / pinion.pitchDiameter(),
      n("efficiency"),
      n("meshStiffness"),
      n("backlash") + pinion.n("backlash")
    );
    for (field in["efficiency", "stiffness", "backlash"]) result.setBasis(
      field,
      Assumed,
      "unverified helical rack " + field
    );
    return result;
  }

}
