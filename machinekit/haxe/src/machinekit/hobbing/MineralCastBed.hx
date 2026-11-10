package machinekit.hobbing;

/** Hobber layout on the shared typed foundation model. */
class MineralCastBed extends machinekit.structural.MineralFoundation {

  public static inline final PLINTH_HEIGHT = 800.0;
  public static inline final TABLE_SEAT_HEIGHT = 608.0;
  public static inline final TABLE_WELL_FLOOR_HEIGHT = 598.0;
  public static inline final WORK_WELL_TOP = 810.0;
  public static inline final WORK_WELL_WIDTH = 398.0;
  public static inline final WORK_WELL_DEPTH = 350.0;
  public static inline final WORK_WELL_CENTRE_X = -45.0;
  public static inline final DRIVE_CHANNEL_FRONT_CLEARANCE = 12.5;
  public static inline final DRIVE_CHANNEL_FLOOR = 720.0;
  public static inline final DRIVE_CHANNEL_WIDTH = 110.0;
  public static inline final CHIP_DRAIN_DEPTH = 80.0;
  public static inline final CHIP_DRAIN_WIDTH = 180.0;
  public static inline final CHIP_TRAY_WELL_CLEARANCE_MM = 5.0;
  public static inline final RAIL_SEAT_HEIGHT = 790.0;
  public static inline final DRIVE_SEAT_HEIGHT = 710.0;
  public static inline final LIFT_SEAT_HEIGHT = 770.0;
  public static inline final LIFT_CENTRE_Y = 310.0;
  public static inline final BED_RAIL_INSERT_THICKNESS = 20.0;
  public static inline final BED_RAIL_SEAT_WIDTH = 50.0;
  public static inline final BED_RAIL_BEAM_WIDTH = 110.0;
  public static inline final FOOT_FLOOR = 40.0;
  public static inline final FOOT_ROOF = 160.0;
  public static inline final FOOT_SUPPORT_TOP = 230.0;
  public static inline final FOOT_ACCESS_WIDTH = 140.0;
  public static inline final FOOT_BOSS_WIDTH = 190.0;
  public static inline final FOOT_STEM_CLEARANCE = 28.0;
  public static inline final FOOT_PLATE_THICKNESS = 15.0;
  public static inline final FOOT_PLATE_WIDTH = 110.0;
  public static inline final PLINTH_CORE_FLOOR = 85.0;
  public static inline final PLINTH_CORE_WEB = 35.0;
  public static inline final CABLE_DUCT_WIDTH = 30.0;
  public static inline final CABLE_DUCT_HEIGHT = 40.0;
  public static inline final CABLE_DUCT_CENTRE_Y = 300.0;
  public static inline final CABLE_DUCT_FLOOR = 500.0;
  public static inline final DRIVE_CAP_UNDERSIDE = 660.0;
  public static inline final DRIVE_RIB_HALF_WIDTH = 15.0;
  public static inline final FIXTURE_THREAD_MM = 6.0;
  public static inline final TABLE_INSERT_WIDTH = 280.0;
  public static inline final TABLE_INSERT_BORE = 40.0;
  public static inline final TABLE_INSERT_THICKNESS = 20.0;

  /** Unverified exterior study choices; published bore/thread dimensions live in the catalogue variant. */
  public static inline final TRAY_SOCKET_OUTSIDE_DIAMETER = 8.0;
  public static inline final TRAY_SOCKET_RELIEF_DIAMETER = 6.2;
  public static inline final TRAY_SOCKET_TAIL_WIDTH = 11.0;
  public static inline final TRAY_SOCKET_TAIL_THICKNESS = 2.2;
  public static inline final TRAY_SOCKET_TAIL_WAVE = 1.4;
  public static inline final TRAY_SOCKET_TAIL_BLEND = 0.5;
  public static inline final TRAY_SOCKET_POSITION_TOLERANCE = 0.5;
  // Approved 5 mm mix design: a 10 mm steel-to-ledge land. The toe
  // clamps remain on the same finished plane; side/end toe overlap is
  // 11/9.5 mm respectively, with grip qualification still a physical input.
  public static inline final SOURCE_STEEL_COVER_MM = 10.0;
  public static inline final TRAY_SOCKET_EDGE_INSET = machinekit.enclosure.TraySeat.WIDTH
    + MineralCastBed.TRAY_SOCKET_OUTSIDE_DIAMETER / 2 + MineralCastBed.SOURCE_STEEL_COVER_MM;
  public static inline final TRAY_SOCKET_END_INSET:Float = machinekit.enclosure.TraySeat.WIDTH + (MineralCastBed.TRAY_SOCKET_TAIL_WIDTH
    > MineralCastBed.TRAY_SOCKET_OUTSIDE_DIAMETER ? MineralCastBed.TRAY_SOCKET_TAIL_WIDTH : MineralCastBed.TRAY_SOCKET_OUTSIDE_DIAMETER) / 2 + MineralCastBed.SOURCE_STEEL_COVER_MM;

  public function new() {
    var source = HobberFoundationSizing.resolve("bed");
    super(source.body, source.placements, null, null, source.liftingPoints);
  }
}
