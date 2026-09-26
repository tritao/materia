package cadbridge;

import bimkit.BimDocument;
import bimkit.BimSchema;
import cadkit.parametric.ElementId;
import robotkit.spatial.Vec3;
import robotkit.spatial.Quat;
import robotkit.spatial.Transform3;
import robotkit.spatial.FrameTree3;
import robotkit.spatial.FrameTransform3;

/**
 * Registers a BimKit document's spatial aggregation hierarchy
 * (project -> site -> building -> storey, `bimkit.BimSchema.Aggregates`
 * edges) as `FrameTree3` edges, named `"bim:" + elementId.value`. Project/
 * site/building objects carry no geometry of their own (they are generic
 * classified CadKit objects, not features), so their edges are identity; a
 * storey with a base Level gets a Z-only translation from that level's
 * elevation, converted from BimKit's millimeters into RobotKit's meters.
 */
class BimFrameBridge {
  public static function frameName(elementId:ElementId):String return "bim:" + elementId.value;

  public static function registerHierarchy(bim:BimDocument, tree:FrameTree3):Void {
    if (bim == null || tree == null) throw "BIM frame bridge requires a document and a frame tree";
    for (relationship in bim.cad.allRelationships()) {
      if (relationship.typeName != BimSchema.Aggregates) continue;
      if (relationship.source.documentId.value != bim.cad.id.value || relationship.target.documentId.value != bim.cad.id.value)
        continue;
      var parentId = relationship.source.elementId;
      var childId = relationship.target.elementId;
      tree.add(new FrameTransform3(frameName(parentId), frameName(childId), storeyTransform(bim, childId)));
    }
  }

  static function storeyTransform(bim:BimDocument, elementId:ElementId):Transform3 {
    var element = bim.cad.element(elementId);
    var classification = element.property("bim.class");
    if (classification == null || classification.value != BimSchema.Storey) return Transform3.identity();
    var base = bim.storeyBaseLevel(elementId);
    if (base == null) return Transform3.identity();
    var elevationMm = bim.cad.levelElevation(base);
    return new Transform3(new Vec3(0.0, 0.0, elevationMm / 1000.0), Quat.identity());
  }
}
