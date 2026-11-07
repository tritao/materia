import haxe.crypto.Sha256;
import machinekit.cache.Codecs;
import machinekit.cache.Memos;
import machinekit.component.ComponentDisplayMesh;
import machinekit.component.ComponentValues;
import machinekit.hobbing.Hob;

/** Builds one hob display mesh and prints its digest and time: run it in separate processes against one cache directory. */
class HobMeshCacheProbe {
  public static function main():Void {
    for (module in [
      "machinekit.hobbing.HobMesh",
      "machinekit.hobbing.HobSolid",
      "machinekit.hobbing.Hob",
      "machinekit.cache.Memo"
    ]) Sys.println("identity " + module + " " + machinekit.cache.CodeIdentity.of(module));
    Sys.println("native " + machinekit.cache.Memos.nativeIdentity());
    if (Sys.getEnv("PROBE_IDENTITIES_ONLY") == "1") return;
    var hob = new Hob(new ComponentValues().setBoolean("renderTeeth", true).setNumber(
      "height",
      Std.parseFloat(Sys.getEnv("PROBE_HEIGHT") ?? "40")
    ));
    var facet = ComponentDisplayMesh.of(hob);
    if (facet == null) throw "no display mesh facet";
    var began = Sys.time();
    var mesh = facet.build();
    var seconds = Sys.time() - began;
    var again = facet.build();
    var bytes = Codecs.displayMesh(mesh);
    if (Codecs.displayMesh(again).length != bytes.length) throw "second build differs";
    Sys.println("probe digest=" + Sha256.encode(Std.string(bytes.length)
      + ":" + hex(bytes)) + " vertices=" + mesh.positions.length / 3 + " seconds=" + Math.round(seconds * 1000) / 1000);
    var negativeZeros = 0;
    for (list in [mesh.positions, mesh.normals]) for (value in list) if (value == 0 && 1 / value < 0) negativeZeros++;
    Sys.println("probe negative-zeros="
      + negativeZeros + " of " + (mesh.positions.length + mesh.normals.length) + " values");
    for (line in Memos.report()) if (line.indexOf("hob-display-mesh") == 0) Sys.println(line);
  }

  static function hex(bytes:haxe.io.Bytes):String {
    var buf = new StringBuf();
    for (i in 0...bytes.length) buf.add(StringTools.hex(bytes.get(i), 2));
    return buf.toString();
  }
}
