package machinekit.hobbing;

import machinekit.component.BoltedFace;
import machinekit.component.BoltedFace.PinnedFace;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Real outboard assembly: made housing and thermally floating bearing sleeve.
 * Body frame +Z follows the arbor toward the tool; offsets below are mm in it.
 * Functional tolerance/thermal figures are assumed project requirements. */
class HobberOutboardLayout {
  public static inline final WIDTH = 60.0;
  public static inline final BODY_LENGTH = 30.0;
  public static inline final SLEEVE_DIAMETER = 40.0;
  public static inline final SLEEVE_FROM = 3.0;
  public static inline final SLEEVE_TO = 27.0;
  public static inline final FRONT_STOP = 2.0;
  public static inline final REAR_STOP = 28.0;
  public static inline final PAIR_FROM = 6.0;
  public static inline final PAIR_TO = 24.0;
  public static inline final GLAND_FROM = 24.0;
  public static inline final GLAND_TO = 27.0;
  public static inline final GLAND_DIAMETER = 36.0;
  public static inline final GLAND_BORE = 28.0;
  public static inline final CAP_BORE = 30.0;
  public static inline final CAP_THICKNESS = 3.0;
  public static inline final SPACER_LENGTH = 5.0;
  public static inline final SPACER_DIAMETER = 20.0;
  public static inline final NUT_LENGTH = 6.0;
  public static inline final ARBOR_HEIGHT = 180.0;
  public static inline final JOURNAL_LENGTH = 29.0;
  public static inline final INDICATED_RADIAL = 0.002;
  public static inline final SEAT_RADIAL = 0.001;
  public static inline final BEARING_RADIAL = 0.0015;
  public static inline final THERMAL_RADIAL = 0.0005;
  public static inline final RADIAL_BUDGET = 0.005;
  public static inline final ASSEMBLY_AXIAL = 0.2;
  public static inline final TEMPERATURE_RISE = 30.0;
  public static inline final STEEL_EXPANSION = 12e-6;

  public static function bodyPose():AssemblyFrame return F.fromRotationMatrix(0, -85, 0, HobberSwivelLayout.SPINDLE_ROTATION);
  public static function capHoles():Array<machinekit.component.BoltedFace.BoltHole> return [
    for (x in [-23.0, 23.0]) for (y in [-12.0, 12.0]) {x: x, y: y}
  ];
  public static function capFrame(into:Float):AssemblyFrame return BoltedFace.frameOf({x: 0.0, y: 0.0, z: BODY_LENGTH},
    {x: 0, y: 0, z: into}, {x: 1, y: 0, z: 0});
  public static function housingMountFrame():AssemblyFrame return BoltedFace.frameOf({x: -30.0, y: 0.0, z: 15.0},
    {x: 1, y: 0, z: 0}, {x: 0, y: 0, z: 1});
  public static function carrierMountFrame():AssemblyFrame return BoltedFace.frameOf({x: 30.0, y: -100.0, z: 0.0},
    {x: 1, y: 0, z: 0}, {x: 0, y: 1, z: 0});
  public static function carrierHoles():Array<machinekit.component.BoltedFace.BoltHole> return [
    for (y in [-8.0, 8.0]) for (z in [-22.0, 22.0]) {x: y, y: z}
  ];
  public static function housingPins():PinnedFace return PinnedFace.atPoints("carrier-pins", housingMountFrame(),
    [for (y in [-22.0, 22.0]) {x: -30.0, y: y, z: 15.0}], 4, 6, false);
  public static function carrierPins():PinnedFace return PinnedFace.atPoints("housing-pins", carrierMountFrame(),
    [for (z in [-22.0, 22.0]) {x: 30.0, y: -100.0, z: z}], 4, 6, false);
}
