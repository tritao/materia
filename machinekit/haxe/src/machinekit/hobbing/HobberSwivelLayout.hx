package machinekit.hobbing;

import cadkit.modeling.Part;
import machinekit.assembly.PosedParts;
import machinekit.motion.ErobActuatorPart;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/**
 * Where the A axis sits (work frame, mm): the eRob 110 actuator's output face looks toward the hob along -x and stands 11 mm proud of the Z
 * saddle plate; its Ø85 pilot (16 mm tall, `ErobActuatorData.PILOT_HEIGHT`) goes through an output bore in a 5 mm web at the plate's front, and
 * the Ø110 housing stands in an 8 mm pocket cut into the plate from behind, its front face (with the 16 M4 holes) against the web. The 13 mm
 * plate is the 5 mm web and the 8 mm minimum pocket depth. A turned adapter ring bridges the 24 mm between the output face and the back of the
 * swivel plate. The axis passes through the hob centre line at y = 0 (the hob is 20 mm off it, as before) at `HobberZLayout.SEAT_Z`.
 */
class HobberSwivelLayout {
  /** Output flange face, x. */
  public static inline final OUTPUT_X = 124.0;
  /** The Z saddle plate's front face and back (block) face, x. */
  public static inline final PLATE_FRONT = 135.0;
  public static inline final PLATE_BACK = 148.0;
  /** The swivel plate's back face, where the adapter ring ends. */
  public static inline final PLATE_X = 100.0;
  /** Swing either side of the middle position. */
  public static inline final RANGE:Float = Math.PI / 12;
  /** Adapter ring: outside and inside diameter, mm. The inside leaves the output flange more than half its footprint to bear on. */
  public static inline final RING_OUTSIDE = 110.0;
  public static inline final RING_INSIDE = 40.0;
  /**
   * Adapter-to-plate screws: four M6 on this pitch circle, in the ring's plate end, at 45 deg to the plate's axes. They have to stand in the strip |z| 24..40 of the
   * plate's front: the Y scale's housing lies on it along y at z 0, and the Y guide blocks (z 43 to 77, on the rails at +-60) run over everything beyond z 40.
   * Four M6 12.9 clamp the ring with ~440 N m of friction at this radius (mu 0.15, 70 % proof preload) against the 100:1 eRob's 204 N m peak and 369 N m momentary output; the ring was first drawn with eight M5, whose heads fell under the scale and the guide blocks.
   */
  public static inline final PLATE_PCD = 95.0;
  public static inline final PLATE_SCREWS = 4;
  public static inline final PLATE_PHASE:Float = Math.PI / 4;
  /** Rotation that takes the actuator's CAD +Z to the work frame's -x (the old swivel head used the same). */
  public static final ROTATION:Array<Float> = [0.0, 0, -1, 1, 0, 0, 0, -1, 0];

  /** How far the output face stands in front of the saddle plate's front face (its bore is 2.2 mm wider in radius than the pilot). */
  public static inline final RUNNING_GAP:Float = PLATE_FRONT - OUTPUT_X;
  /** Depth of the pocket from the plate's back face to the housing's front face. */
  public static inline final POCKET_DEPTH:Float = PLATE_BACK - (OUTPUT_X - ErobActuatorPart.SEAT_Z);

  /** Rotation of the spindle cartridge and the hob's line in the work frame (the arbor, outboard support and hob use the same). */
  public static final SPINDLE_ROTATION:Array<Float> = [-1.0, 0, 0, 0, 0, -1, 0, -1, 0];
  /** Where the cartridge's housing datum stands across the machine (work y), at X = 0 and the swivel's middle position. */
  public static inline final SPINDLE_Y = 240.0;

  /** The spindle cartridge's housing frame in the work frame at X = 0, Y = 0, Z = 0 and the swivel's middle position. */
  public static function spindlePose():AssemblyFrame return F.fromRotationMatrix(0, SPINDLE_Y, HobberZLayout.SEAT_Z, SPINDLE_ROTATION);

  /** The actuator's CAD frame in the work frame. */
  public static function pose():AssemblyFrame return F.fromRotationMatrix(OUTPUT_X, 0, HobberZLayout.SEAT_Z, ROTATION);

  /** The pocket, output bore and screw clearance holes the Z saddle needs, in the saddle's frame (the work frame lowered by `SEAT_Z`). */
  public static function housingCutout():Part {
    var tool = new ErobActuatorPart().mountingCutout(POCKET_DEPTH, RUNNING_GAP, 0.4, 0, ErobActuatorPart.PILOT_BORE_CLEARANCE);
    var placed = PosedParts.moved(tool, F.fromRotationMatrix(OUTPUT_X, 0, 0, ROTATION));
    tool.close();
    return placed;
  }
}
