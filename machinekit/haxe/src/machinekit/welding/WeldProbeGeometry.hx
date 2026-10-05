package machinekit.welding;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import cadkit.modeling.Location;
import cadkit.modeling.Plane;
import machinekit.assembly.MachineAssembly;
import machinekit.component.ComponentDetail;
import machinekit.welding.WeldSeams.SeamSolid;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** A polygonal CAD face available to touch sensing, expressed in the weldment reference frame. */
class WeldProbeFace {
  public final member:String;
  public final face:String;
  public final target:Bool;
  public final normal:Vector;
  public final centre:Vector;
  public final chords:Array<Array<Vector>>;
  public final u:Vector;
  public final v:Vector;

  public function new(member:String, face:String, target:Bool, normal:Vector, centre:Vector, chords:Array<Array<Vector>>) {
    this.member = member; this.face = face; this.target = target;
    this.normal = normal.normalized(); this.centre = centre; this.chords = chords;
    u = this.normal.cross(Math.abs(this.normal.z) < 0.9 ? Vector.Z() : Vector.X()).normalized();
    v = this.normal.cross(u).normalized();
  }

  public function name():String return '$member:$face';
  public function offset():Float return normal.dot(centre);

  /** Even-odd contours include all holes. A positive inset keeps probes off every boundary. */
  public function contains(point:Vector, inset:Float = 0):Bool {
    if (point == null || !Math.isFinite(inset) || inset < 0) throw "Probe-face containment needs a point and finite nonnegative inset";
    if (Math.abs(normal.dot(point) - offset()) > 1e-5) return false;
    var x = point.dot(u), y = point.dot(v), inside = false;
    var nearest = Math.POSITIVE_INFINITY;
    for (chord in chords) {
      var a = chord[0], b = chord[1];
      var x0 = a.dot(u), y0 = a.dot(v), x1 = b.dot(u), y1 = b.dot(v);
      if ((y0 > y) != (y1 > y) && x0 + (y - y0) * (x1 - x0) / (y1 - y0) > x) inside = !inside;
      var edge = b.subtract(a);
      var square = edge.dot(edge);
      var fraction = square > 0 ? Math.max(0.0, Math.min(1.0, point.subtract(a).dot(edge) / square)) : 0.0;
      nearest = Math.min(nearest, point.subtract(a.add(edge.scale(fraction))).length());
    }
    return inside && nearest >= inset;
  }

  /** Conservatively contain a complete rectangular uncertainty patch, including holes and concavities. */
  public function containsRegion(point:Vector, halfU:Float, halfV:Float, inset:Float = 0):Bool {
    if (!Math.isFinite(halfU) || halfU < 0 || !Math.isFinite(halfV) || halfV < 0 || !Math.isFinite(inset) || inset < 0)
      throw "Probe-region extents must be finite and nonnegative";
    if (!contains(point, inset)) return false;
    var extentU = halfU + inset, extentV = halfV + inset;
    for (su in [-1, 1]) for (sv in [-1, 1])
      if (!contains(point.add(u.scale(su * extentU)).add(v.scale(sv * extentV)))) return false;
    // A hole or concave boundary can enter the patch even if all four corners are material.
    for (chord in chords) {
      var a = chord[0].subtract(point), b = chord[1].subtract(point);
      var lo = 0.0, hi = 1.0;
      var starts = [a.dot(u), a.dot(v)], ends = [b.dot(u), b.dot(v)], extents = [extentU, extentV];
      var crossing = true;
      for (axis in 0...2) {
        var delta = ends[axis] - starts[axis];
        if (Math.abs(delta) < 1e-12) {
          if (Math.abs(starts[axis]) > extents[axis]) crossing = false;
        } else {
          var first = (-extents[axis] - starts[axis]) / delta, last = (extents[axis] - starts[axis]) / delta;
          lo = Math.max(lo, Math.min(first, last)); hi = Math.min(hi, Math.max(first, last));
        }
      }
      if (crossing && lo <= hi) return false;
    }
    return true;
  }

  /** A finite search lattice comes from face bounds, never authored weldment coordinates. */
  public function samples(divisions:Int = 9, inset:Float = 1):Array<Vector> {
    if (divisions < 1 || divisions > 65 || !Math.isFinite(inset) || inset < 0)
      throw "Probe-face samples need bounded divisions and a finite nonnegative inset";
    var lowU = Math.POSITIVE_INFINITY, highU = Math.NEGATIVE_INFINITY;
    var lowV = Math.POSITIVE_INFINITY, highV = Math.NEGATIVE_INFINITY;
    for (chord in chords) for (point in chord) {
      lowU = Math.min(lowU, point.dot(u)); highU = Math.max(highU, point.dot(u));
      lowV = Math.min(lowV, point.dot(v)); highV = Math.max(highV, point.dot(v));
    }
    var points:Array<Vector> = [];
    if (chords.length == 0) return points;
    for (i in 0...divisions) for (j in 0...divisions) {
      var point = u.scale(lowU + (highU - lowU) * (i + 0.5) / divisions)
        .add(v.scale(lowV + (highV - lowV) * (j + 0.5) / divisions)).add(normal.scale(offset()));
      if (contains(point, inset)) points.push(point);
    }
    return points;
  }
}

/** CAD-only contact patches and nominal visibility; runtime owns reach, motion and observations. */
class WeldProbeGeometry {
  public final faces:Array<WeldProbeFace>;
  public function new(faces:Array<WeldProbeFace>) this.faces = faces.copy();

  /** Borrowed parts already posed in a common reference frame. Curved boundaries are deliberately unsupported. */
  public static function of(parts:Map<String, Part>, targets:Array<String>):WeldProbeGeometry {
    var faces:Array<WeldProbeFace> = [];
    for (member in parts.keys()) {
      var solid = SeamSolid.of({id: member, part: parts.get(member)});
      for (face in solid.faces) {
        if (!face.planar) throw 'Contact probe geometry requires polygonal solids: "$member" has a curved face';
        var polygonal = true;
        for (loop in face.loops) for (edge in loop) if (!solid.edges[edge].line) polygonal = false;
        if (!polygonal) throw 'Contact probe geometry requires polygonal solids: "$member" has a curved boundary';
        if (face.chords.length > 0)
          faces.push(new WeldProbeFace(member, face.name, targets.indexOf(member) >= 0, face.normal, face.centre, face.chords));
      }
    }
    faces.sort((a, b) -> Reflect.compare(a.name(), b.name()));
    return new WeldProbeGeometry(faces);
  }

  /** Collect actual B-rep faces in the weldment frame, with optional table/clamp obstruction geometry. */
  public static function findIn(assembly:MachineAssembly, weldment:Weldment, poses:Map<String, AssemblyFrame>,
      ?obstacles:Array<String>):WeldProbeGeometry {
    var reference = poses.get(weldment.reference);
    if (reference == null) throw 'Probe geometry has no weldment reference "${weldment.reference}"';
    var inverse = AssemblyFrames.inverse(reference);
    var members = weldment.members.copy();
    if (obstacles != null) for (id in obstacles) if (members.indexOf(id) < 0) members.push(id);
    var parts:Map<String, Part> = new Map();
    try {
      for (entry in assembly.components()) if (members.indexOf(entry.id) >= 0 && entry.component.hasGeometry()) {
        var pose = poses.get(entry.id);
        if (pose == null) throw 'Probe geometry has no pose for "${entry.id}"';
        var relative = AssemblyFrames.compose(inverse, pose);
        var x = AssemblyFrames.transformVector(relative, 1, 0, 0), z = AssemblyFrames.transformVector(relative, 0, 0, 1);
        var local = entry.component.geometry(ComponentDetail.Preview);
        var placed:Part;
        try placed = local.placed(new Location(new Plane(new Vector(relative.x, relative.y, relative.z),
          new Vector(x.x, x.y, x.z), new Vector(z.x, z.y, z.z)))) catch (error:Dynamic) {
          local.close(); throw error;
        }
        local.close(); parts.set(entry.id, placed);
      }
      for (member in members) if (!parts.exists(member)) throw 'Probe geometry has no solid for "$member"';
      var result = of(parts, weldment.members);
      for (part in parts) part.close();
      return result;
    } catch (error:Dynamic) {
      for (part in parts) part.close();
      throw error;
    }
  }

  /** The outward approach segment may not cross another CAD face or a mating surface. */
  public function exposed(face:WeldProbeFace, point:Vector, approach:Float, inset:Float = 1):Bool {
    if (!Math.isFinite(approach) || !(approach > 0)) throw "Probe visibility needs a positive finite approach distance";
    if (!face.target || !face.contains(point, inset)) return false;
    for (other in faces) if (other != face) {
      var alignment = other.normal.dot(face.normal);
      if (Math.abs(alignment) < 1e-9) continue;
      var depth = (other.offset() - other.normal.dot(point)) / alignment;
      if (depth < -1e-5 || depth > approach) continue;
      if (depth <= 1e-5 && alignment > -0.9) continue;
      if (other.contains(point.add(face.normal.scale(depth)))) return false;
    }
    return true;
  }
}
