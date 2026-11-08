package machinekit.hobbing;

/** Hobber layout on the shared typed foundation model. */
class MineralCastColumn extends machinekit.structural.MineralFoundation {
  public function new() {
    var source = HobberFoundationSizing.resolve("column");
    super(source.body, source.placements, FoundationLayout.columnSolePads()[0].id, null, source.liftingPoints);
    // Keep the insert at its purpose datum; split a large anchor pattern by
    // its physical sides so the scene does not bury twenty parts in one folder.
    for(row in insertLocations())if(row.purpose=="lifting-mount") {
      var anchors=machinekit.tooling.InsertAnchorGeometry.components(row.part.insertSpec());
      if(anchors.length>15)for(anchor in anchors)assignGroup(row.id+"-"+anchor.role,
        purposeGroup(row.purpose)+"/"+row.id+"/"+(anchor.pose.x<0?"Left anchors":"Right anchors"));
    }
  }
}
