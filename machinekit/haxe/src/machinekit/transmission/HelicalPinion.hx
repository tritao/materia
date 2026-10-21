package machinekit.transmission;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import materia.assembly.AssemblyFrames as F;

/** Assumed catalogue-class normal-module helical pinion. Dimensions mm; SI ratings unless stated.
 * Geometry is a nominal layout envelope, not manufacturing or supplier geometry. */
class HelicalPinion extends AssumedCatalogComponent {
  static var recipe:Null<ComponentType>;
  static function inputs():Array<ComponentParameter> return [
    P.length("normalModule", 1.5),
    P.count("teeth", 20),
    P.length("faceWidth", 20),
    P.quantity("helixAngle", Math.acos(3 * Math.PI / 10), "rad"),
    P.quantity("pressureAngle", 0.3490658504, "rad"),
    P.choice("hand", [ "right", "left" ], "right"),
    P.length("bore", 14),
    P.length("backlash", 0.02)
  ];
  public function new(?v:ComponentValues) {
    super("helical-pinion", "normal-module helical pinion", "steel", inputs(), v);
    positive(["normalModule", "faceWidth", "helixAngle", "pressureAngle"]);
    require(
      n("helixAngle") < Math.PI / 4 && SpurGear.validPressureAngle(n("pressureAngle")),
      "unsupported helix/pressure angle"
    );
    require(parameterCount("teeth")>=3 && parameterCount("teeth")/Math.pow(Math.cos(n("helixAngle")),3)>=2/Math.pow(Math.sin(n("pressureAngle")),2),
      "pinion virtual tooth count requires undercut; increase teeth or pressure angle");
    require(
      n("bore") < rootDiameter() && n("backlash") < Math.PI * transverseModule() / 4,
      "bore/backlash leaves insufficient tooth material"
    );
    addConnector("axis", Axis, Solids.axial(0, 0, n("faceWidth") / 2));
    addConnector("pitch", Pitch, Solids.axial(0, -pitchDiameter() / 2, n("faceWidth") / 2));
  }
  public static function recipeType():ComponentType {
    if (recipe == null) recipe = new ComponentType(
      "machinekit.transmission.helical-pinion",
      inputs(),
      v -> new HelicalPinion(v)
    );
    return recipe;
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function geometry(detail:ComponentDetail = Preview):Part {
    if(detail==Envelope) return Solids.named(Part.cylinder(pitchDiameter()/2+n("normalModule"),n("faceWidth")),"pinion-envelope");
    var sections:Array<cadkit.modeling.Curve> = [];
    try {
      // A single tooth has a unique root edge. Lofting the complete periodic
      // outline lets OCCT re-index equivalent wire starts and alter the helix.
      // A smooth loft through the twisted sections makes each flank one helical face.
      var wire = toothOutline().close();
      var count = 7;
      try {
        for (i in 0...count) {
          var z = n("faceWidth") * i /(count - 1), a = handSign() * 2 * z * Math.tan(n("helixAngle")) / pitchDiameter();
          sections.push(wire.placed(ToothOutline.at(z, a)));
        }
      } catch (error:Dynamic) {
        wire.close();
        throw error;
      }
      wire.close();
      var tooth = Part.loft(sections, false);
      for (section in sections) section.close();
      sections = [];
      var body=Solids.building([tooth],owned->{
        var pieces:Array<Part> = [Solids.named(Part.cylinder(rootDiameter()/2,n("faceWidth")),"pinion-core")];owned.push(pieces[0]);
        for(i in 0...parameterCount("teeth")) {
          var angle=2*Math.PI*i/parameterCount("teeth");
          var placed=tooth.placed(new cadkit.modeling.Location(new cadkit.modeling.Plane(new Vector(0,0,0),
            new Vector(Math.cos(angle),Math.sin(angle),0),new Vector(0,0,1))));
          placed=Solids.named(placed,"tooth-"+i);pieces.push(placed);owned.push(placed);
        }
        return Solids.union(pieces);
      });
      return n("bore") == 0 ? body : Solids.cut(
        body,
        [Solids.named(
          Part.cylinderSpan(
            n("bore") / 2,
            -1,
            n("faceWidth") + 1
          ),
          "shaft-bore"
        )]
      );
    } catch (error:Dynamic) {
      for (section in sections) section.close();
      throw error;
    }

  }
  public function handSign():Int return parameterToken("hand") == "right" ? 1 : -1;
  public function transverseModule():Float return n("normalModule") / Math.cos(n("helixAngle"));
  public function transversePressureAngle():Float return Math.atan(Math.tan(n("pressureAngle"))
    / Math.cos(n("helixAngle")));
  public function pitchDiameter():Float return transverseModule() * parameterCount("teeth");
  public function rootDiameter():Float return pitchDiameter() - 2.5 * n("normalModule");
  public function travelPerRevolution():Float return Math.PI * pitchDiameter();
  /** Signed axial force for a signed tangential force; bearing reactions oppose this. */
  public function axialThrust(tangentialForce:Float):Float {
    require(Math.isFinite(tangentialForce), "invalid tangential force");
    return handSign() * tangentialForce * Math.tan(n("helixAngle"));
  }
  public function separatingForce(tangentialForce:Float):Float return Math.abs(tangentialForce)
    * Math.tan(transversePressureAngle());
  public function overlapRatio():Float return n("faceWidth") * Math.sin(n("helixAngle")) /(Math.PI * n("normalModule"));
  /** Transverse pinion/rack contact ratio for unshifted full-depth teeth. */
  public function transverseContactRatio():Float {
    var r = pitchDiameter() / 2, a = transversePressureAngle(), ra = r + n("normalModule"), rb = r * Math.cos(a);
    return(Math.sqrt(ra * ra
      - rb * rb) - r * Math.sin(a) + n("normalModule") / Math.sin(a)) /(Math.PI * transverseModule() * Math.cos(a));
  }
  public function contactRatio():Float return transverseContactRatio() + overlapRatio();
  /**
   * Tooth 0's transverse outline: spline flanks through the involute, a tip arc on the outside circle, and a
   * root closed by chords inside the core it is united with.
   */
  function toothOutline():ToothOutline {
    var r = pitchDiameter() / 2,
      a = transversePressureAngle(), rb = r * Math.cos(a), rf = rootDiameter() / 2, ra = r + n("normalModule");
    var start = Math.max(rb, rf), half =(Math.PI * transverseModule() / 2 - n("backlash")) /(2 * r),
      invPitch = involute(r, rb);
    var rootAngle = half + invPitch - involute(start, rb);
    var right = [for (j in 0...7) {
      var rr = start +(ra - start) * j / 6, aa = -half + involute(rr, rb) - invPitch;
      new Vector(rr * Math.cos(aa), rr * Math.sin(aa));
    }];
    var left = [for (j in 0...7) {
      var rr = ra -(ra - start) * j / 6, aa = half - involute(rr, rb) + invPitch;
      new Vector(rr * Math.cos(aa), rr * Math.sin(aa));
    }];
    var result = new ToothOutline();
    if (rf < start - 1e-10) {
      var rootA = new Vector(rf * Math.cos(-rootAngle), rf * Math.sin(-rootAngle));
      var rootB = new Vector(rf * Math.cos(rootAngle), rf * Math.sin(rootAngle));
      result.line(rootA, right[0]).spline(right).arc(right[6], new Vector(ra, 0), left[0]).spline(left)
        .line(left[6], rootB).line(rootB, rootA);
    } else
      result.spline(right).arc(right[6], new Vector(ra, 0), left[0]).spline(left).line(left[6], right[0]);
    return result;
  }
  static function involute(r:Float, rb:Float):Float {
    var t = Math.sqrt(Math.max(0, r * r /(rb * rb) - 1));
    return t - Math.atan(t);
  }

}
