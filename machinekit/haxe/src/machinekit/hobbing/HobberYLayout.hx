package machinekit.hobbing;

/**
 * Y drive screw. Y travel is only +-10 mm (5 mm overtravel), so the SFU1605 is short, on the standard 51 / 11 mm ends.
 * Measured from the fixed end face: NUT_STATION is the nut body centre, the 35 mm body spans +-17.5 and the 10 mm flange
 * follows towards the floating end. At the limits (+-15) the body starts at 100-15-17.5 = 67.5 (16.5 mm from the 51 mm
 * shoulder) and the flange ends at 100+15+17.5+10 = 142.5 (10.5 mm from the BF12 housing face at 170-17 = 153).
 * The fixed end, motor and Y drive frame stay where they were; only the far end of the screw is 80 mm shorter.
 *
 * With the nut on a `NutBlock` (flange outward on the floating side, block behind it on the motor side) the flange's outer
 * face is 45 mm from the nut part's start, which sits 17.5 mm before NUT_STATION: at +15 it ends at 100+15+27.5 = 142.5.
 * The BF12 housing face is at 170-17 = 153, so 10.5 mm clear. The next shorter 10 mm step, 160, leaves 143 - 142.5 =
 * 0.5 mm, under the 2 mm running clearance, so 170 is the shortest standard length; the nut would have to move 8 mm
 * towards the motor to use 160.
 */
class HobberYLayout {
  public static inline final SCREW_LENGTH = 170.0;
  public static inline final NUT_STATION = 100.0;
  /** Y axis (z 508) above the top of the nut seat bar on the Y saddle (z 458 + 20). */
  public static inline final NUT_BLOCK_HEIGHT = 30.0;
}
