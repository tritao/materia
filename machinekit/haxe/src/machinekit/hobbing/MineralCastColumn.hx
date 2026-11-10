package machinekit.hobbing;

/** Hobber layout on the shared typed foundation model. */
class MineralCastColumn extends machinekit.structural.MineralFoundation {

  public static inline final COLUMN_SOLE_THICKNESS = 20.0;

  public static inline final COLUMN_SPINE_HALF_WIDTH = 60.0;
  public static inline final COLUMN_HEAD_WIDTH = 240.0;
  /** The recess on the drive's side opens this far past the widest drive support (the pads, 80 wide, centred on the screw), mm. */
  public static inline final DRIVE_RECESS_CLEARANCE = 3.0;
  public static inline final COLUMN_HEAD_DEPTH = 100.0;
  public static inline final COLUMN_HEAD_CENTRE_Y = 60.0;

  public static inline final RAIL_SEAT_WIDTH = 30.0;
  public static inline final RAIL_SEAT_LENGTH = 470.0;
  public static inline final RAIL_INSERT_THICKNESS = 20.0;
  public static inline final RAIL_NATIVE_Y = 90.0;
  public static inline final RAIL_CENTRE_Z = 245.0;

  public function new() {
    var source = HobberFoundationSizing.resolve("column");
    super(source.body, source.placements, HobberLayout.get().columnSolePads()[0].id, null, source.liftingPoints);
    // Keep the insert at its purpose datum; split a large anchor pattern by
    // its physical sides so the scene does not bury twenty parts in one folder.
    for(row in insertLocations())if(row.purpose=="lifting-mount") {
      var anchors=machinekit.tooling.InsertAnchorGeometry.components(row.part.insertSpec());
      if(anchors.length>15)for(anchor in anchors)assignGroup(row.id+"-"+anchor.role,
        purposeGroup(row.purpose)+"/"+row.id+"/"+(anchor.pose.x<0?"Left anchors":"Right anchors"));
    }
  }
}
