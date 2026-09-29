package visionkit;

import VisionKitNative;

/** Materia camera convention: +X forward, +Y left, +Z up. */
class CameraModel {
  public final width:Int;
  public final height:Int;
  public final fx:Float;
  public final fy:Float;
  public final cx:Float;
  public final cy:Float;
  public final distortionModel:Int;
  public final k1:Float;
  public final k2:Float;
  public final p1:Float;
  public final p2:Float;
  public final k3:Float;

  public function new(width:Int, height:Int, fx:Float, fy:Float, cx:Float,
      cy:Float, distortionModel:Int = 0, k1:Float = 0, k2:Float = 0,
      p1:Float = 0, p2:Float = 0, k3:Float = 0) {
    if (width <= 0 || height <= 0 || fx <= 0 || fy <= 0 ||
        (distortionModel != 0 && distortionModel != 1))
      throw "Invalid camera model";
    this.width = width; this.height = height;
    this.fx = fx; this.fy = fy; this.cx = cx; this.cy = cy;
    this.distortionModel = distortionModel;
    this.k1 = k1; this.k2 = k2; this.p1 = p1; this.p2 = p2; this.k3 = k3;
  }

  public function native():vk_camera_model {
    var n = new vk_camera_model();
    n.set_struct_size(vk_camera_model.size());
    n.set_width(width); n.set_height(height);
    n.set_fx(fx); n.set_fy(fy); n.set_cx(cx); n.set_cy(cy);
    n.set_distortion_model(distortionModel);
    n.set_k1(k1); n.set_k2(k2); n.set_p1(p1); n.set_p2(p2); n.set_k3(k3);
    return n;
  }

  public static function fromNative(n:vk_camera_model):CameraModel {
    return new CameraModel(n.get_width(), n.get_height(), n.get_fx(), n.get_fy(),
      n.get_cx(), n.get_cy(), n.get_distortion_model(), n.get_k1(), n.get_k2(),
      n.get_p1(), n.get_p2(), n.get_k3());
  }

  /** Projects metre points in the Materia camera frame into pixels. */
  public function project(points:Array<{x:Float, y:Float, z:Float}>):Array<{x:Float, y:Float}> {
    if (points == null || points.length == 0) throw "Expected camera points";
    var input:Array<vk_point3> = [];
    for (p in points) {
      var n = new vk_point3(); n.set_x(p.x); n.set_y(p.y); n.set_z(p.z);
      input.push(n);
    }
    var result = VisionKitNative.vk_project_points(native(), input);
    if (result.status != 0)
      throw 'Projection failed with VisionKit error ${result.status}';
    return [for (p in result.out_pixels) {x: p.get_x(), y: p.get_y()}];
  }

  /** Returns unit rays in the Materia camera frame. */
  public function unproject(pixels:Array<{x:Float, y:Float}>):Array<{x:Float, y:Float, z:Float}> {
    if (pixels == null || pixels.length == 0) throw "Expected pixels";
    var input:Array<vk_pixel> = [];
    for (p in pixels) {
      var n = new vk_pixel(); n.set_x(p.x); n.set_y(p.y);
      input.push(n);
    }
    var result = VisionKitNative.vk_unproject_points(native(), input);
    if (result.status != 0)
      throw 'Unprojection failed with VisionKit error ${result.status}';
    return [for (p in result.out_rays) {x: p.get_x(), y: p.get_y(), z: p.get_z()}];
  }
}
