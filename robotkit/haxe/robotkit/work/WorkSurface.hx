package robotkit.work;

import robotkit.spatial.Transform3;
import robotkit.spatial.FrameTransform3;

/**
 * A planar region of design/observed/work geometry to run a surface process
 * over. `frameId` is the frame this surface is registered against (e.g. a
 * BIM wall's frame or a robot's world frame); `surfaceFrameId` names this
 * surface's own local plane frame (+Z outward normal), and `frame_T_surface`
 * is the `frameId -> surfaceFrameId` edge a caller registers into a
 * `FrameTree3` (see `frameEdge()`). `boundary`/`exclusions`, and every
 * `Toolpath` a generator builds over this surface, are expressed directly
 * in `surfaceFrameId`.
 */
class WorkSurface {
  public final id:WorkSurfaceId;
  public final frameId:String;
  public final surfaceFrameId:String;
  public final frame_T_surface:Transform3;
  public final boundary:Polygon2;
  public final exclusions:Array<Polygon2>;
  public final tolerance:Float;
  public final materialTag:String;
  public final provenance:Provenance;

  public function new(id:WorkSurfaceId, frameId:String, frame_T_surface:Transform3, boundary:Polygon2,
      ?exclusions:Array<Polygon2>, ?tolerance:Float = 0.002, ?materialTag:String = "",
      ?provenance:Provenance, ?surfaceFrameId:String) {
    if (id == null || id.length == 0) throw "WorkSurface requires a non-empty id";
    if (frameId == null || frameId.length == 0) throw "WorkSurface requires a non-empty frame id";
    if (frame_T_surface == null) throw "WorkSurface requires a frame_T_surface transform";
    if (boundary == null) throw "WorkSurface requires a boundary polygon";
    if (!Math.isFinite(tolerance) || tolerance < 0.0) throw "WorkSurface tolerance must be finite and non-negative";
    this.id = id;
    this.frameId = frameId;
    this.surfaceFrameId = surfaceFrameId == null ? id : surfaceFrameId;
    this.frame_T_surface = frame_T_surface;
    this.boundary = boundary;
    this.exclusions = exclusions == null ? [] : exclusions.copy();
    this.tolerance = tolerance;
    this.materialTag = materialTag;
    this.provenance = provenance == null ? new Provenance(id, SourceKind.Design) : provenance;
  }

  /** The `frameId -> surfaceFrameId` registration edge, ready for `FrameTree3.add`. */
  public function frameEdge():FrameTransform3 return new FrameTransform3(frameId, surfaceFrameId, frame_T_surface);
}
