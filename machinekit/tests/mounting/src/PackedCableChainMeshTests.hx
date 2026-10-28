import machinekit.motion.CableChain;
import machinekit.motion.PackedCableChainMesh;
import materia.assembly.AssemblyFrames as F;

class PackedCableChainMeshTests {
  static var checks = 0;
  static function check(b:Bool, s:String):Void {
    checks++;
    if (!b) throw s;
  }
  static function near(a:Float, b:Float, s:String):Void check(Math.abs(a - b) < 1e-4, s + ": " + a + " vs " + b);
  public static function main():Void {
    var chain = new CableChain(), mesh = new PackedCableChainMesh(), positions = mesh.positions, normals = mesh.normals;
    for (position in [0.0, 1.0, 700.0, 1400.0, 2100.0, 2799.0, 2800.0]) {
      mesh.update(chain, position, 1, [0.0, 0.0, 0.0]);
      check(mesh.positions == positions && mesh.normals == normals, "vertex and normal storage reused");
      var volume = 0.0;
      for (i in 0...Std.int(mesh.indexCount / 3)) {
        var a = mesh.indices.getInt32(i * 12) * 12,
          b = mesh.indices.getInt32(i * 12 + 4) * 12, c = mesh.indices.getInt32(i * 12 + 8) * 12;
        var ax = positions.getFloat(a), ay = positions.getFloat(a + 4), az = positions.getFloat(a + 8);
        var bx = positions.getFloat(b), by = positions.getFloat(b + 4), bz = positions.getFloat(b + 8);
        var cx = positions.getFloat(c), cy = positions.getFloat(c + 4), cz = positions.getFloat(c + 8);
        volume += (ax * (by * cz - bz * cy) + ay * (bz * cx - bx * cz) + az * (bx * cy - by * cx)) / 6;
      }
      check(volume > 0, "closed hollow mesh has outward winding");
      var body = chain.geometryAt(position);
      try {
        var bounds = body.shape.bounds(), lo = bounds.get_min(), hi = bounds.get_max();
        near(mesh.minimum[0], lo.get_x(), "native X min");
        near(mesh.maximum[0], hi.get_x(), "native X max");
        near(mesh.minimum[1], lo.get_y(), "native Y min");
        near(mesh.maximum[1], hi.get_y(), "native Y max");
        near(mesh.minimum[2], lo.get_z(), "native Z min");
        near(mesh.maximum[2], hi.get_z(), "native Z max");
        // 32 chordal bend facets approximate the native loft; account for that discretization.
        check(Math.abs(volume / body.volume() - 1) < 0.015, "native hollow volume agrees within bend discretization");
      } catch (e:Dynamic) {
        body.close();
        throw e;
      }
      body.close();
      var path = chain.path(position), end = chain.movingFrame(position);
      near(path[path.length - 1].y, end.y, "moving centreline end follows source connector");
      near(path[path.length - 1].z, end.z, "moving centreline height follows source connector");
      var frame = F.fromRotationMatrix(100, 200, 300, [0, 0, 1, 1, 0, 0, 0, 1, 0]);
      mesh.update(chain, position, 0.001, [0.1, 0.2, 0.3], frame);
      for (i in 0...mesh.vertexCount) for (axis in 0...3) check(
        Math.isFinite(mesh.positions.getFloat(i * 12 + axis * 4)),
        "transformed mesh finite"
      );
    }
    var routed = new machinekit.gantry.GantryCableChain(chain, "host", "carrier", "y");
    var registry = machinekit.component.MachineKitComponents.standard();
    var rebuilt = registry.byId(machinekit.gantry.GantryCableChain.recipeType().id).create(routed.values());
    check(Std.isOfType(rebuilt, machinekit.gantry.GantryCableChain), "routed chain recipe survives reload");
    var copy:machinekit.gantry.GantryCableChain = cast rebuilt;
    check(
      copy.fixedHost == "host" && copy.movingHost == "carrier" && copy.axis == "y",
      "end bindings survive recipe reload"
    );
    Sys.println('PackedCableChainMeshTests: $checks assertions passed');
  }
}
