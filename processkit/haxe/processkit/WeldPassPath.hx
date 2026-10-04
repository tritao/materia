package processkit;

import processkit.skill.WeldPlan;
import processkit.skill.WeldPlan.WeldSegment;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;

/** Offsets a CAD seam chain toward its faces and intersects neighbouring offset lines at their shared corner. */
class WeldPassPath {
  public static function offset(segments:Array<WeldSegment>, normals:Array<Array<Vec3>>, faceA:Float, faceB:Float):Array<WeldSegment> {
    if (segments == null || segments.length == 0 || normals == null || normals.length != segments.length)
      throw "A pass offset needs a seam path and its face normals";
    if (!(Math.isFinite(faceA) && Math.isFinite(faceB) && faceA >= 0 && faceB >= 0)) throw "Invalid pass offset";
    if (faceA == 0 && faceB == 0) return segments;
    var starts:Array<Vec3> = [], stops:Array<Vec3> = [];
    for (index in 0...segments.length) {
      var faces = normals[index];
      if (faces == null || faces.length != 2) throw "A pass offset needs two face normals per seam";
      var shift = faces[0].scale(faceA).add(faces[1].scale(faceB));
      starts.push(segments[index].start.translation.add(shift));
      stops.push(segments[index].stop.translation.add(shift));
    }
    for (index in 1...segments.length) join(segments, starts, stops, index - 1, index);
    if (segments.length > 1 && segments[0].start.translation.sub(segments[segments.length - 1].stop.translation).norm() <= WeldPlan.JOIN)
      join(segments, starts, stops, segments.length - 1, 0);
    var result:Array<WeldSegment> = [];
    for (index in 0...segments.length) {
      var segment = segments[index];
      var direction = segment.stop.translation.sub(segment.start.translation).normalized();
      if (!(stops[index].sub(starts[index]).dot(direction) > WeldPlan.JOIN))
        throw 'Pass offset consumes seam "${segment.name}"';
      result.push(new WeldSegment(new Transform3(starts[index], segment.start.rotation),
        new Transform3(stops[index], segment.stop.rotation), segment.name, segment.open));
    }
    return result;
  }

  static function join(segments:Array<WeldSegment>, starts:Array<Vec3>, stops:Array<Vec3>, previous:Int, next:Int):Void {
    var a = segments[previous], b = segments[next];
    if (a.stop.translation.sub(b.start.translation).norm() > WeldPlan.JOIN) throw "CAD seam chain has a gap";
    var first = a.stop.translation.sub(a.start.translation).normalized();
    var second = b.stop.translation.sub(b.start.translation).normalized();
    var delta = starts[next].sub(stops[previous]);
    var cosine = first.dot(second), denominator = 1 - cosine * cosine;
    if (denominator < 1e-10) {
      if (delta.norm() > WeldPlan.JOIN) throw 'Parallel offset seams "${a.name}" and "${b.name}" do not join';
      starts[next] = stops[previous];
      return;
    }
    var alongFirst = (delta.dot(first) - cosine * delta.dot(second)) / denominator;
    var alongSecond = (cosine * delta.dot(first) - delta.dot(second)) / denominator;
    var onFirst = stops[previous].add(first.scale(alongFirst));
    var onSecond = starts[next].add(second.scale(alongSecond));
    if (onFirst.sub(onSecond).norm() > WeldPlan.JOIN) throw 'Offset seams "${a.name}" and "${b.name}" are skew';
    var bound = Math.min(a.length(), b.length()) * 0.45;
    if (Math.abs(alongFirst) > bound || Math.abs(alongSecond) > bound)
      throw 'Pass offset consumes the corner between "${a.name}" and "${b.name}"';
    var corner = onFirst.add(onSecond).scale(0.5);
    stops[previous] = corner;
    starts[next] = corner;
  }
}
