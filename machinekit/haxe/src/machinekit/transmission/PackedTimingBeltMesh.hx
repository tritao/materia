package machinekit.transmission;

import haxe.io.Bytes;
import machinekit.transmission.TimingBelt.BeltPoint;

/** Reusable display mesh storage. SceneKit copies these streams when publishing geometry. */
class PackedTimingBeltMesh {
  public var positions(default, null):Bytes = Bytes.alloc(0);
  public var normals(default, null):Bytes = Bytes.alloc(0);
  public var indices(default, null):Bytes = Bytes.alloc(0);
  public var vertexCount(default, null):Int = 0;
  public var indexCount(default, null):Int = 0;
  public final minimum:Array<Float> = [0.0, 0.0, 0.0];
  public final maximum:Array<Float> = [0.0, 0.0, 0.0];
  final distances:Array<Float> = [];
  final samplePoint = new BeltPoint(0.0, 0.0, 0.0, 0.0);
  var prior:Array<Float> = [for (_ in 0...12) 0.0];
  var next:Array<Float> = [for (_ in 0...12) 0.0];
  var packedPrior:Bytes = Bytes.alloc(48);
  var packedNext:Bytes = Bytes.alloc(48);
  final planarNormals:Bytes = Bytes.alloc(96);

  public function new() {
    // Four copies of each unit Z normal, ready for one whole-quad copy.
    for (corner in 0...4) {
      planarNormals.setFloat(corner * 12 + 8, 1.0);
      planarNormals.setFloat(48 + corner * 12 + 8, -1.0);
    }
  }

  /** Same tooth profile and face winding as TimingBeltMesh, without per-tooth temporary arrays. */
  public function update(belt:TimingBelt, phase:Float, toothSide:Int, thickness:Float,
      scale:Float, center:Array<Float>):Void {
    if (!Math.isFinite(phase) || (toothSide != 1 && toothSide != -1)) throw "Invalid belt mesh phase or tooth side";
    if (!Math.isFinite(thickness) || thickness <= 0.0) throw "Invalid belt mesh thickness";
    if (belt.teeth > 200000) throw "Belt display mesh has too many teeth";
    var pitch = belt.length / belt.teeth;
    // The teeth repeat every pitch; keep long-running simulation phases bounded.
    phase = (phase / belt.pitch - Math.floor(phase / belt.pitch)) * pitch;
    distances.resize(0);
    distances.push(0.0);
    for (tooth in -1...belt.teeth + 1) for (sample in 0...5) {
      var fraction = switch sample { case 0: 0.0; case 1: 0.2; case 2: 0.35; case 3: 0.65; default: 0.8; };
      var distance = phase + (tooth + fraction) * pitch;
      if (distance > 1e-8 && distance < belt.length - 1e-8) distances.push(distance);
    }
    distances.push(belt.length);
    var capacity = (distances.length - 1) * 16;
    if (positions.length < capacity * 12) {
      positions = Bytes.alloc(capacity * 12);
      normals = Bytes.alloc(capacity * 12);
    }
    vertexCount = 0; indexCount = 0;
    for (axis in 0...3) { minimum[axis] = Math.POSITIVE_INFINITY; maximum[axis] = Math.NEGATIVE_INFINITY; }
    ring(belt, distances[0], phase, pitch, thickness, toothSide, prior, packedPrior, scale, center);
    for (sample in 1...distances.length) {
      ring(belt, distances[sample], phase, pitch, thickness, toothSide, next, packedNext, scale, center);
      for (side in 0...4) face(side, toothSide);
      var swap = prior; prior = next; next = swap;
      var packedSwap = packedPrior; packedPrior = packedNext; packedNext = packedSwap;
    }
    // GeometryData requires an exact-sized index stream. Reuse it while the seam topology is unchanged.
    if (indices.length != indexCount * 4) {
      indices = Bytes.alloc(indexCount * 4);
      for (quad in 0...Std.int(vertexCount / 4)) for (corner in 0...6) {
        var offset = switch corner { case 0, 3: 0; case 1: 1; case 2, 4: 2; default: 3; };
        indices.setInt32((quad * 6 + corner) * 4, quad * 4 + offset);
      }
    }
  }

  function ring(belt:TimingBelt, distance:Float, phase:Float, pitch:Float,
      thickness:Float, toothSide:Int, target:Array<Float>, packed:Bytes,
      scale:Float, center:Array<Float>):Void {
    var p = belt.pointAt(distance, samplePoint);
    var fraction = (distance - phase) / pitch;
    fraction -= Math.floor(fraction);
    var height = fraction < 0.2 || fraction > 0.8 ? 0.0 :
      fraction < 0.35 ? (fraction - 0.2) / 0.15 : fraction <= 0.65 ? 1.0 : (0.8 - fraction) / 0.15;
    var back = -thickness * 0.35, front = thickness * (0.1 + 0.55 * height);
    var nx = -p.dy * toothSide, ny = p.dx * toothSide;
    target[0] = target[9] = p.x + nx * back;
    target[1] = target[10] = p.y + ny * back;
    target[3] = target[6] = p.x + nx * front;
    target[4] = target[7] = p.y + ny * front;
    target[2] = target[5] = 0.0;
    target[8] = target[11] = belt.width;
    // Each ring is shared by its neighbouring faces. Transform and convert it
    // once, then copy the exact float32 bytes into each face's vertex stream.
    for (vertex in 0...4) for (axis in 0...3) {
      var coordinate = vertex * 3 + axis;
      var value = target[coordinate] * scale - center[axis];
      packed.setFloat(coordinate * 4, value);
      if (value < minimum[axis]) minimum[axis] = value;
      if (value > maximum[axis]) maximum[axis] = value;
    }
  }

  function face(side:Int, toothSide:Int):Void {
    var a = side * 3, b = ((side + 1) % 4) * 3;
    // Original quad: prior[a], next[a], next[b], prior[b], reversed for the inner tooth side.
    var first = toothSide == 1 ? b : a;
    var third = toothSide == 1 ? a : b;
    var ux = next[first] - prior[first], uy = next[first + 1] - prior[first + 1];
    var byte = vertexCount * 12;
    if (side == 0 || side == 2) {
      // Both edges lie in one XY plane: only the cross product's Z component survives.
      var vx = next[third] - prior[first], vy = next[third + 1] - prior[first + 1];
      var nz = ux * vy - uy * vx;
      if (Math.abs(nz) <= 1e-12) return;
      normals.blit(byte, planarNormals, nz > 0.0 ? 0 : 48, 48);
    } else {
      // Extrusion edges share XY coordinates; their cross product has no Z component.
      var vz = next[third + 2] - prior[first + 2];
      var nx = uy * vz, ny = -ux * vz;
      var length = Math.sqrt(nx * nx + ny * ny);
      if (length <= 1e-12) return;
      normals.setFloat(byte, nx / length);
      normals.setFloat(byte + 4, ny / length);
      normals.setFloat(byte + 8, 0.0);
      normals.blit(byte + 12, normals, byte, 12);
      normals.blit(byte + 24, normals, byte, 24);
    }
    // Preserve the original winding without branching for each of the four corners.
    positions.blit(byte, packedPrior, first * 4, 12);
    positions.blit(byte + 12, packedNext, first * 4, 12);
    positions.blit(byte + 24, packedNext, third * 4, 12);
    positions.blit(byte + 36, packedPrior, third * 4, 12);
    vertexCount += 4;
    indexCount += 6;
  }
}
