package machinekit.hobbing;

/**
 * Y drive screw. Y travel is only +-10 mm (5 mm overtravel), so the SFU1605 is short, on the standard 51 / 11 mm ends.
 * Measured from the fixed end face: NUT_STATION is the nut body centre, the 35 mm body spans +-17.5 and the 10 mm flange
 * follows towards the floating end. At the limits (+-15) the body starts at 100-15-17.5 = 67.5 (16.5 mm from the 51 mm
 * shoulder) and the flange ends at 100+15+17.5+10 = 142.5 (10.5 mm from the BF12 housing face at 170-17 = 153).
 * The fixed end, motor and Y drive frame stay where they were; only the far end of the screw is 80 mm shorter.
 */
class HobberYLayout {
  public static inline final SCREW_LENGTH = 170.0;
  public static inline final NUT_STATION = 100.0;
}
