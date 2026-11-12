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
  /** Radius (mm) from the screw axis to the grease nipple's tip: the fitting stands on the flange's rim toward the nut's +Y. */
  public static inline final GREASE_FITTING_TIP = 27.0;

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
  /** The nut's furthest reach from the screw axis (mm), on its +Y side: the flange's rim or the grease fitting's tip. */
  public function radialReach():Float return Math.max(flangeDiameter / 2, GREASE_FITTING_TIP);

  override public function record(b:cadkit.parametric.recording.DocumentBuilder, detail:ComponentDetail):Null<cadkit.parametric.Feature> {
    var nut = super.record(b, detail), z = bodyLength + flangeThickness / 2;
    if (nut == null) throw "A ball nut records its lead-screw nut";
    var up = Vector.Z(), across = Vector.Y();
    function post(r:Float, z0:Float, z1:Float):cadkit.parametric.Feature return b.cylinderAlong(new Vector(0, 0, 0), up, r, z0, z1);
    function ring(outer:Float, inner:Float, z0:Float, z1:Float, slack:Float):cadkit.parametric.Feature
      return b.cutAll(post(outer, z0, z1), [post(inner, z0 - slack, z1 + slack)], ["inside"]);
    // Radial fitting on the flange between the two upper mounting holes.
    var body = b.label(b.unionAll([
      nut,
      b.label(b.cylinderAlong(new Vector(0, 0, z), across, 2.8, 22.5, 25), "Grease fitting base"),
      b.label(b.cylinderAlong(new Vector(0, 0, z), across, 1.6, 24.8, GREASE_FITTING_TIP), "Grease fitting nipple")
    ]), "Nut with grease fitting");
    if (detail == Envelope) return body;
    var tools:Array<cadkit.parametric.Feature> = [], labels:Array<String> = [];
    // Two shallow annular seal seats; the screw bore stays at its original diameter.
    for (k => end in [0.0, bodyLength + flangeThickness - 0.45]) {
      tools.push(ring(11.2, 9.2, end - 0.01, end + 0.46, 0.09));
      labels.push('seal seat ${k + 1}');
    }
    // Flange flats leave all six mount holes and their face positions intact.
    for (k => side in [-1, 1]) {
      var x = side * 27.5;
      tools.push(b.boxBetween(x - 5, x + 5, -30, 30, bodyLength - 0.1, bodyLength + flangeThickness + 0.1));
      labels.push('flange flat ${k + 1}');
    }
    // Relief at the barrel/flange shoulder and a visible mouth in the grease nipple.
    tools.push(ring(14.1, 13.6, bodyLength - 0.8, bodyLength, 0.1));
    labels.push("shoulder relief");
    tools.push(b.cylinderAlong(new Vector(0, 0, z), across, 0.65, 25.8, GREASE_FITTING_TIP + 0.1));
    labels.push("nipple mouth");
    return b.label(b.cutAll(body, tools, labels), "Seal seats, flats and shoulder relief");
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
