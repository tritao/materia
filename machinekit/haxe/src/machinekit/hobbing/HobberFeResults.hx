package machinekit.hobbing;

/**
 * Stored results of the finite element checks of the hobber's structure (`examples/flexspline-hobber/fem/`, C3D10 tetrahedra, gmsh + scipy; the full table is
 * `fem/results/hobber-fem-results.json`, re-run with `fem/run_fem.sh`). The routine checks read these numbers; `HobberFrameChecks.spindleDrive` throws when the cradle's
 * geometry no longer matches `CRADLE_DESIGN` (the FE reference is then stale) or when its hand model and the FE disagree by more than 25 %.
 */
class HobberFeResults {
  /** The cradle geometry the FE below was run on (`HobberSaddle.cradleSignature`). */
  public static final CRADLE_DESIGN = "wall 8, rib 6 from y 192 windowed y 209..232 z 95..160, floor 12/8 from y 125, foot 18, bolts -30,135 30,135 -30,232 30,232, rest x -44..36 y 145..200 top 81.2, x -56..31";

  /**
   * First mode of the servo group on the cradle and spindle seat, Hz (C3D10 FE, cradle size 8 mm, seat 16 mm, the seat held at its four M8 and two dowels on the saddle plate, the servo's
   * flange on its four M6 patches): preloaded floor joint = bolt rings and dowels tied with the rest of the interface closed in the normal direction (first value) / open (second);
   * unpreloaded = bolt springs and dowels with the contact closed / open; bonded. `FACE_FLANGE` couples the whole flange (upper bracket), `NO_REST` is the same cradle without the rest
   * block's support (what the floor alone gives), `JACK_SCREW` a pair of M8 screws (600 N/um) instead of the shim pack. The first four rows are the design's gates.
   */
  public static final CRADLE_PRELOADED_HZ = [775.0, 750.0];
  public static final CRADLE_UNPRELOADED_HZ = [743.0, 575.0];
  public static final CRADLE_BONDED_HZ = 810.0;
  public static final CRADLE_FACE_FLANGE_PRELOADED_HZ = [800.0, 773.0];
  public static final CRADLE_NO_REST_PRELOADED_HZ = [352.0, 339.0];
  public static final CRADLE_JACK_SCREW_PRELOADED_HZ = [708.0, 689.0];

  /** Shim pack and contact of the rest block, series stiffness N/mm (assumed; the FE sensitivity to a jack-screw pair at 6e5 is in the table). */
  public static final REST_PACK_N_PER_MM = 3.0e6;

  /** XC10 mm C3D10: ground deck +4 ribs +4 replicated guide pads, four80 mm
   * corner sole contact patches spanning380x180 mm,
   * pads restrained in z (LM0 boundary conditions). Bolts-only interface gives
   * a conservative operating point without adhesive stiffness credit. See
   * fem/results/carriage.json.
   * Replication E=4 GPa, bolt patches and closed preload contact are ASSUMED. */
  public static final CARRIAGE_SOLE_TILT_NM_PER_RAD = 17076760.469294;
  public static final CARRIAGE_FRONT_TIE_N_PER_UM = 43.633364;

  /**
   * Constants of the cradle hand model (`HobberFrameChecks.spindleDrive`), fitted to the FE of 15 cradle variants (wall 8..15 mm, rib solid / windowed / absent,
   * rest block of three widths and stiffnesses, floor bolts at two rows; `fem/results/cradle-variants.json`):
   * base rotation about x (rocking, no rest block), about y (lateral roll), rib participation in the section, and the seat's local stiffness under the rest block, in series with the shim pack.
   */
  public static final BASE_ROCKING_N_MM_PER_RAD = 1.31e9;
  public static final BASE_ROLL_N_MM_PER_RAD = 2.23e8;
  public static final RIB_PARTICIPATION = 0.19;
  public static final SEAT_UNDER_REST_N_PER_MM = 3.5e5;

  /** Z saddle (13 mm plate, head as a rigid 36.7 kg mass on the eRob seat: upper bounds, the eRob's bearing and housing compliance are not in the model): first three modes, Hz, supports at the block bolt patches / whole block footprints. */
  public static final SADDLE_MODES_BOLTS_HZ = [121.0, 154.4, 359.7];
  public static final SADDLE_MODES_BLOCKS_HZ = [125.1, 161.1, 384.2];
  /** Seat tilt stiffness about y, kN m/rad (bolt patches, block footprints), and the hob tip displacement per 100 N of cutting force. */
  public static final SADDLE_TILT_KNM_PER_RAD = [346.6, 380.5];
  public static final SADDLE_TIP_UM_PER_100N = 5.68;
}
