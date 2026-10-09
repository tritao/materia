import machinekit.transmission.TimingBelt;
import machinekit.transmission.TimingBelt.BeltWrap;
import machinekit.transmission.TimingBelt.BeltPoint;
import machinekit.transmission.TimingBeltProfile;
import machinekit.transmission.TimingBeltMesh;
import machinekit.transmission.PackedTimingBeltMesh;

/** Compare the streaming animator with the authored mesh, including seam phases and winding. */
class PackedTimingBeltMeshTests {
  static function check(value:Bool, message:String):Void if (!value) throw message;
  static function near(actual:Float, expected:Float, message:String):Void
    check(Math.abs(actual - expected) < 1e-5 * Math.max(1.0, Math.abs(expected)), '$message: $actual != $expected');

  public static function main():Int {
    var mesh = new PackedTimingBeltMesh();
    var center = [0.1, -0.2, 0.3];
    for (route in [
      [new BeltWrap(0, 0, 10), new BeltWrap(200, 0, 10)],
      [new BeltWrap(0, 0, 10), new BeltWrap(200, 0, 15), new BeltWrap(100, 150, 8)],
      [new BeltWrap(-66, 0, 6, 1), new BeltWrap(88, -12, 6, -1),
        new BeltWrap(100, -90, 6, 1), new BeltWrap(100, 90, 6, 1), new BeltWrap(-66, 90, 6, 1)]
    ]) for (width in [9.0, 1e-9]) {
      // A very thin extrusion exercises faces near the degenerate-area cutoff.
      var belt = new TimingBelt(TimingBeltProfile.GT2, width, route);
      var target = new BeltPoint(0.0, 0.0, 0.0, 0.0);
      var snapshot = belt.pointAt(0.0), originalX = snapshot.x, originalY = snapshot.y;
      for (distance in [-belt.length - 0.3, -0.1, 0.0, 0.4, belt.length / 2, belt.length, belt.length * 3 + 0.7]) {
        var expected = belt.pointAt(distance);
        check(belt.pointAt(distance, target) == target, "sampling returns caller-owned storage");
        near(target.x, expected.x, "reused sample x"); near(target.y, expected.y, "reused sample y");
        near(target.dx, expected.dx, "reused sample tangent x"); near(target.dy, expected.dy, "reused sample tangent y");
      }
      check(snapshot != target && snapshot.x == originalX && snapshot.y == originalY,
        "ordinary samples remain independent snapshots");

      for (side in [-1, 1]) for (phase in [-3.17, 0.0, 1e-9, 0.4, 0.7, 1.3, 1.6, 2.0 - 1e-9, 2.0, 5.17]) {
        var reference = TimingBeltMesh.build(belt, phase, side, belt.thickness);
        mesh.update(belt, phase, side, belt.thickness, 0.001, center);
        check(mesh.vertexCount * 3 == reference.positions.length && mesh.indexCount == reference.indices.length,
          'packed tooth topology matches at phase $phase, side $side');
        for (i in 0...reference.positions.length) {
          var value = reference.positions[i] * 0.001 - center[i % 3];
          near(mesh.positions.getFloat(i * 4), value, "packed position");
          near(mesh.normals.getFloat(i * 4), reference.normals[i], "packed normal and winding");
          check(value >= mesh.minimum[i % 3] - 1e-9 && value <= mesh.maximum[i % 3] + 1e-9,
            "bounds enclose every vertex");
        }
        for (i in 0...reference.indices.length) check(mesh.indices.getInt32(i * 4) == reference.indices[i], "packed triangle index");
        var positions = mesh.positions, normals = mesh.normals, indices = mesh.indices;
        mesh.update(belt, phase, side, belt.thickness, 0.001, center);
        check(mesh.positions == positions && mesh.normals == normals && mesh.indices == indices,
          "repeated updates retain their backing buffers");
      }
    }
    Sys.println("Packed timing belt mesh tests passed");
    return 0;
  }
}
