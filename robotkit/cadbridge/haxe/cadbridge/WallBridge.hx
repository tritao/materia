package cadbridge;

import CadKit;
import bimkit.BimDocument;
import cadkit.Face;
import cadkit.parametric.ElementId;
import robotkit.spatial.Vec3;
import robotkit.work.Provenance;
import robotkit.work.SourceKind;
import robotkit.work.WorkSurface;
import robotkit.work.WorkSurfaceId;

/**
 * Converts a BimKit wall's side face into a WorkSurface. Hosted
 * windows/doors already cut through-holes into that face
 * (bimkit.BimDocument.rebuildWall boolean-cuts the wall body with every
 * hosted opening), so they surface as FaceBridge exclusions automatically:
 * one inner wire, and so one exclusion, per opening. BimKit's own
 * convention is millimeter dimensions (see bimkit.BimSchema quantities),
 * so geometry is scaled by 1/1000 into RobotKit's meters.
 */
class WallBridge {
  public static function wallToWorkSurface(bim:BimDocument, wallId:ElementId, id:WorkSurfaceId,
      frameId:String, ?sideNormal:Vec3):WorkSurface {
    if (bim == null || wallId == null) throw "Wall bridge requires a document and a wall id";
    var target = sideNormal == null ? new Vec3(0.0, 1.0, 0.0) : sideNormal.normalized();
    var shape = bim.cad.element(wallId).shape();
    var faces = shape.faces();
    var chosen:Null<Face> = null;
    var bestDot = -2.0;
    for (i in 0...faces.count()) {
      var candidate = faces.at(i);
      if (candidate.surfaceKind() == CadKit.SurfaceKind.Plane) {
        var normalValue = candidate.normal();
        var normal = new Vec3(normalValue.get_x(), normalValue.get_y(), normalValue.get_z()).normalized();
        var dot = normal.dot(target);
        if (dot > bestDot) {
          if (chosen != null) chosen.close();
          chosen = candidate;
          bestDot = dot;
        } else {
          candidate.close();
        }
      } else {
        candidate.close();
      }
    }
    if (chosen == null || bestDot < 0.99) {
      if (chosen != null) chosen.close();
      throw "Wall bridge could not find a side face matching the requested normal";
    }
    var provenance = new Provenance(wallId.value, SourceKind.Design);
    var surface = FaceBridge.toWorkSurface(chosen, id, frameId, provenance, null, 0.001);
    chosen.close();
    return surface;
  }
}
