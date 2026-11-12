package machinekit.motion;
import cadkit.modeling.*;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import machinekit.motion.SfuFamilyData.SfuNutReference;

/** Shared SFU flange family; published per-model dimensions, exact hole clearances and flats.
 * Root diameter is an explicit engineering input, not inferred from nominal size/lead.
 */
class SfuNut extends LeadScrewNut {
  static final recipe = new machinekit.cache.Once<ComponentType>();
  public final spec:SfuNutReference;
  public function new(model:String = "SFU1605", raceRootDiameter:Float = 0) {
    var s = SfuFamilyData.nut(model);
    super(
      new LeadScrewThread(Ball, s.diameter, s.pitch, 1, RightHand, raceRootDiameter),
      6,
      true,
      BallNut,
      {
        bodyDiameter: s.pilot,
        bodyLength: s.length,
        flangeDiameter: s.flange,
        flangeThickness: s.thickness,
        flangeOffset: s.offset,
        boltCircleDiameter: s.pcd,
        mountScrew: s.mount
      }
    );
    addFacet(EngineeringMetadata.assumed(""));
    spec = s;
    setMaterial("steel");
  }
  override public function boltPattern():Array<{x:Float, y:Float}> return [for (a in [45, 90, 135, 225, 270, 315]) {
    x:boltCircleDiameter / 2 * Math.cos(a * Math.PI / 180), y:boltCircleDiameter / 2 * Math.sin(a * Math.PI / 180)
  }
  ];
  override public function record(b:cadkit.parametric.recording.DocumentBuilder, detail:ComponentDetail):Null<cadkit.parametric.Feature> {
    var nut = super.record(b, detail), s = spec, up = Vector.Z(), axis = new Vector(0, 0, 0);
    if (nut == null) throw "An SFU nut records its lead-screw nut";
    var tools:Array<cadkit.parametric.Feature> = [
      b.cutAll(b.cylinderAlong(axis, up, s.pilot / 2 + 0.1, s.offset + s.thickness, s.length + 0.1),
        [b.cylinderAlong(axis, up, s.barrel / 2, s.offset + s.thickness - 0.1, s.length + 0.2)], ["inside"])
    ];
    var labels = ["tail"];
    for (k => side in [-1, 1]) {
      var x = side * (s.flats / 2 + s.flange / 2);
      tools.push(b.boxBetween(x - s.flange / 2, x + s.flange / 2, -s.flange, s.flange, s.offset - 0.1, s.offset + s.thickness + 0.1));
      labels.push('flange flat ${k + 1}');
    }
    if (detail != Envelope) for (k => p in boltPattern()) {
      tools.push(b.cylinderAlong(new Vector(p.x, p.y, 0), up, s.hole / 2, s.offset - 0.1, s.offset + s.thickness + 0.1));
      labels.push('flange bolt ${k + 1}');
    }
    return b.label(b.cutAll(nut, tools, labels), "Tail, flats and flange bolts");
  }

  public static function familyRecipeType():ComponentType {
    return recipe.get(() -> new ComponentType(
      "machinekit.motion.sfu-nut",
      [P.choice("model", SfuFamilyData.nutModels(), "SFU1605"), P.length("raceRootDiameter", 13)],
      v -> new SfuNut(v.token("model"), v.number("raceRootDiameter"))
    ).numbered(v -> v.token("model") + "-NUT-R" + Dimension.format(v.number("raceRootDiameter")))
      .procured(_ -> Buy(null, null, null)).requiringInputs(["model", "raceRootDiameter"]));
  }
  override public function componentType():Null<ComponentType> return familyRecipeType();
  override public function values():ComponentValues return new ComponentValues().setToken("model",
    spec.model).setNumber(
      "raceRootDiameter",
      thread.rootDiameter()
    ).setToken(
      "material",
      materialSpec()
    );
}
