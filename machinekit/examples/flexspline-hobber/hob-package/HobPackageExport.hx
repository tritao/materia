import gearkit.ConjugateRack;
import gearkit.GearSpec;
import gearkit.HobDesign;
import gearkit.Point2;
import gearkit.ToothProfile;

/** Writes the supplier hob package data for the reference flexspline: the exact
    generating rack and hob design that FlexsplineHobber cuts with, plus the target
    gear. hob_package.py turns the JSON into the data sheets, DXF and CSV.
    Usage: hl main.hl <output.json> */
class HobPackageExport {
  // FlexsplineHobber defaults (FlexsplineHobber.hx): GearSpec(0.3, 200, 8, 20 deg),
  // HobDesign(rack, 1 start, 32 mm OD, 12 gashes), Hob.fromDesign(.., 40 mm long, 16 mm bore).
  static inline final MODULE = 0.3;
  static inline final TEETH = 200;
  static inline final FACE = 8.0;
  static inline final PRESSURE_DEG = 20.0;
  static inline final STARTS = 1;
  static inline final OUTSIDE = 32.0;
  static inline final GASHES = 12;
  static inline final LENGTH = 40.0;
  static inline final BORE = 16.0;
  // HOB_GEOMETRY.md defaults for the detailed hob.
  static inline final TIP_RELIEF_DEG = 3.0;
  static inline final LAND_FRACTION = 0.6;

  static function main() {
    var args = Sys.args();
    if (args.length != 1) throw "usage: hl main.hl <output.json>";
    var spec = new GearSpec(MODULE, TEETH, FACE, PRESSURE_DEG * Math.PI / 180);
    var rack = ConjugateRack.of(new ToothProfile(spec), 256);
    var hob = new HobDesign(rack, STARTS, OUTSIDE, GASHES);
    var region = rack.cutterRegion(hob.rootHeight);
    var tipHeight = -rack.cornerSamples()[0].y;
    var data = {
      source: "materia machinekit/examples/flexspline-hobber (gearkit ConjugateRack + HobDesign)",
      units: "mm, degrees",
      gear: {
        module: spec.module,
        teeth: spec.teeth,
        faceWidth: spec.faceWidth,
        pressureAngleDeg: PRESSURE_DEG,
        profileShift: spec.profileShift,
        pitchDiameter: spec.pitchDiameter(),
        baseDiameter: 2 * spec.baseRadius(),
        tipDiameter: 2 * spec.tipRadius(),
        rootDiameter: 2 * spec.rootRadius(),
        pitchToothThickness: spec.pitchToothThickness(),
        rootFilletRadius: spec.cutterTipRadius,
        tipFilletRadius: spec.tipFilletRadius
      },
      hob: {
        starts: hob.starts,
        hand: "right",
        outsideDiameter: hob.outsideDiameter,
        pitchDiameter: hob.pitchDiameter,
        leadAngleDeg: hob.leadAngle * 180 / Math.PI,
        axialPitch: hob.axialPitch,
        lead: hob.lead,
        normalPitch: Math.PI * MODULE,
        gashes: hob.gashes,
        rakeDeg: hob.rake * 180 / Math.PI,
        length: LENGTH,
        bore: BORE,
        tipReliefDeg: TIP_RELIEF_DEG,
        landFraction: LAND_FRACTION,
        addendum: tipHeight,
        dedendum: hob.rootHeight,
        toothDepth: tipHeight + hob.rootHeight,
        tipRadius: spec.cutterTipRadius
      },
      // Closed normal-section cutter tooth: x across the tooth (centred), y = height
      // above the pitch line of the generated gear (hob tip is the most negative y).
      rackTooth: [for (p in region) [p.x, p.y]],
      rackRootHasFold: rack.hasRootFold
    };
    sys.io.File.saveContent(args[0], haxe.Json.stringify(data, null, "  "));
    Sys.println('wrote ${args[0]}: ${region.length} rack points');
  }
}
