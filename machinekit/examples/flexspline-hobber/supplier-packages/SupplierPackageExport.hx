import gearkit.ConjugateRack;
import gearkit.GearSpec;
import gearkit.HobDesign;
import gearkit.ToothProfile;
import camkit.HobbingJob;
import camkit.HobbingSettings;

/** Writes the data behind the supplier packages for the reference flexspline: the
    exact generating rack and hob design that FlexsplineHobber cuts with, the target
    gear, and the workholding (cup and expanding mandrel) derived from the same gear.
    hob_package.py and mandrel_package.py turn the JSON into data sheets and drawings.
    Usage: hl main.hl <output.json> */
class SupplierPackageExport {
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
  // Workholding, as FlexsplineHobber builds it: cup wall 0.6 mm under the root,
  // 0.05 mm diametral loading clearance, 0.3 mm radial rim-support land, sleeve
  // max(16, face + 8) long from the seat, mandrel 2 mm above the sleeve end.
  static inline final WALL = 0.6;
  static inline final CLEARANCE = 0.05;
  static inline final RIM_LAND = 0.3;
  // Work datum (tooth band) z = 363 mm in the machine; table face FoundationLayout.TABLE_FACE = 228 mm.
  static inline final WORK_DATUM = 363.0;
  static inline final TABLE_FACE = 228.0;
  // FlexsplineCupBlank defaults: blank OD 62 over the band, 40 mm tall, 2 mm closed end.
  static inline final CUP_BLANK_OUTSIDE = 62.0;
  static inline final CUP_HEIGHT = 40.0;
  static inline final CUP_END = 2.0;
  // A swivel mechanism range (PARTS.md SwivelHead, layout check range) for the keep-out sweep.
  static inline final SWIVEL_DEG = 15.0;

  static function main() {
    var args = Sys.args();
    if (args.length != 1) throw "usage: hl main.hl <output.json>";
    var spec = new GearSpec(MODULE, TEETH, FACE, PRESSURE_DEG * Math.PI / 180);
    var rack = ConjugateRack.of(new ToothProfile(spec), 256);
    var hob = new HobDesign(rack, STARTS, OUTSIDE, GASHES);
    var region = rack.cutterRegion(hob.rootHeight);
    var tipHeight = -rack.cornerSamples()[0].y;
    var bore = 2 * (spec.rootRadius() - WALL);
    var sleeveLength = Math.max(16, FACE + 8);
    // The hob's axial travel comes from the same CAM job the hobber runs: the pass
    // Z range (centre height, seat = face bottom = 0) includes the engagement
    // overrun and axial clearance. The blank is the physical cup blank.
    var settings = new HobbingSettings();
    settings.blankOuterRadius = CUP_BLANK_OUTSIDE / 2;
    var job = new HobbingJob(spec, hob, LENGTH, settings);
    var zMin = Math.POSITIVE_INFINITY, zMax = Math.NEGATIVE_INFINITY;
    for (p in job.passes) {
      zMin = Math.min(zMin, Math.min(p.zStart, p.zEnd));
      zMax = Math.max(zMax, Math.max(p.zStart, p.zEnd));
    }
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
      workholding: {
        cupBore: bore,
        cupWall: WALL,
        cupBodyOutside: 2 * spec.rootRadius(),
        cupBlankOutside: CUP_BLANK_OUTSIDE,
        cupHeight: CUP_HEIGHT,
        cupEnd: CUP_END,
        toothBand: FACE,
        sleeveOutside: bore - CLEARANCE,
        sleeveLength: sleeveLength,
        stopDiameter: bore + 2 * RIM_LAND,
        protrusionAboveSeat: sleeveLength + 2,
        seatAboveTableFace: WORK_DATUM - TABLE_FACE,
        swivelRangeDeg: SWIVEL_DEG,
        hobCentreZMin: zMin,
        hobCentreZMax: zMax,
        camClearance: settings.clearance,
        camFeedPerRev: settings.feedPerRevolution
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
