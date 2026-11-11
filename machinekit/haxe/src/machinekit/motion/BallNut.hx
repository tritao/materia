package machinekit.motion;

import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentRecipeSupport;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.Solids;
import materia.assembly.AssemblyFrames;

/** Generic SFU1605 flanged nut. Dimensions, preload drag and zero reversal clearance are assumed;
 * they describe a preloaded nut, not an accuracy guarantee for an unpreloaded vendor assembly.
 */
class BallNut extends LeadScrewNut {
  public final preloaded:Bool;

  public function new(preloaded:Bool = true) {
    super(
      BallScrew.race(),
      6,
      true,
      preloaded ? PreloadedBallNut : BallNut,
      {
        bodyDiameter : 28,
        bodyLength : 35,
        flangeDiameter : 48,
        flangeThickness : 10,
        boltCircleDiameter : 38,
        mountScrew : "M5"
      }
    );
    addFacet(machinekit.component.EngineeringMetadata.assumed(""));
    this.preloaded = preloaded;
    setMaterial("steel");
    addConnector(
      "lubricator",
      Mount,
      AssemblyFrames.alongY(0, flangeDiameter / 2, bodyLength + flangeThickness / 2, 0, 1, 0)
    );
    var holes:Array<machinekit.design.DesignInterfaces.HoleInterface> = [], i = 0;
    for (p in boltPattern()) holes.push({
      id: 'flange${i++}',
      frame: AssemblyFrames.translation(p.x, p.y, bodyLength),
      radius: 2.75,
      depth: flangeThickness,
      minWall: 0.5,
      minClearance: 0.3,
      thread: null,
      counterbore: null,
      approach: null
    });
    addFacet(new machinekit.design.DesignInterfaces(
      holes,
      [{
        id: "grease",
        kind: LubricationPort,
        frame: AssemblyFrames.fromRotationMatrix(
          0,
          flangeDiameter / 2,
          bodyLength + flangeThickness / 2,
          [1, 0, 0, 0, 0, 1, 0, -1, 0]
        )
      }],
      []
    ));
  }

  /** Assumed exterior details; bore, flange face and bolt pattern retain their catalog datums. */
  override public function geometry(detail:ComponentDetail = Preview):Part {
    var body = super.geometry(detail);
    var z = bodyLength + flangeThickness / 2;
    // Radial fitting on the flange between the two upper mounting holes.
    var fitting = Solids.union([
      Solids.named(Part.cylinderAlongY(2.8, 22.5, 25, 0, z), "grease-fitting-base"),
      Solids.named(Part.cylinderAlongY(1.6, 24.8, 27, 0, z), "grease-fitting-nipple")
    ]);
    body = Solids.union([body, fitting]);
    if (detail == Envelope) return body;
    var tools:Array<Part> = [];
    // Two shallow annular seal seats; the screw bore stays at its original diameter.
    for (k => end in [0.0, bodyLength + flangeThickness - 0.45]) {
      var ring = Solids.cut(
        Solids.named(Part.cylinderSpan(11.2, end - 0.01, end + 0.46), "seal-seat" + (k + 1) + "-outer"),
        [Solids.named(Part.cylinderSpan(9.2, end - 0.1, end + 0.6), "seal-seat" + (k + 1) + "-inner")]
      );
      tools.push(ring);
    }
    // Flange flats leave all six mount holes and their face positions intact.
    for (k => side in [-1, 1]) {
      var box = Part.box(10, 60, flangeThickness + 0.2);
      var placed = box.translated(new Vector(side * 27.5, 0, bodyLength - 0.1));
      box.close();
      tools.push(Solids.named(placed, "flange-flat" + (k + 1)));
    }
    // Relief at the barrel/flange shoulder and a visible mouth in the grease nipple.
    tools.push(Solids.cut(
      Solids.named(Part.cylinderSpan(14.1, bodyLength - 0.8, bodyLength), "shoulder-relief-outer"),
      [Solids.named(Part.cylinderSpan(13.6, bodyLength - 0.9, bodyLength + 0.1), "shoulder-relief-inner")]
    ));
    tools.push(Solids.named(Part.cylinderAlongY(0.65, 25.8, 27.1, 0, z), "nipple-mouth"));
    return Solids.cut(body, tools);
  }

  static final ballRecipe = new machinekit.cache.Once<ComponentType>();
  public static function ballRecipeType():ComponentType {
    return ballRecipe.get(() -> new ComponentType(
      "machinekit.motion.ball-nut",
      [ComponentRecipeSupport.flag("preloaded", true)],
      v -> new BallNut(v.boolean("preloaded"))
    ).numbered(v -> v.boolean("preloaded") ? "SFU1605-NUT-PRELOAD" : "SFU1605-NUT-NO-PRELOAD")
      .procured(_ -> Buy(null, null, null)).requiring(v -> [
        machinekit.component.Procurement.ProcurementCodec.requirement("screwDiameter", "mm", null, null, 16),
        machinekit.component.Procurement.ProcurementCodec.requirement("lead", "mm", null, null, 5),
        machinekit.component.Procurement.ProcurementCodec.choice("preloaded", v.boolean("preloaded") ? "true" : "false")
      ]));
  }
  override public function componentType():Null<ComponentType> return ballRecipeType();
  override public function values():ComponentValues return new ComponentValues().setBoolean("preloaded",
    preloaded).setToken(
      "material",
      materialSpec()
    );
}
