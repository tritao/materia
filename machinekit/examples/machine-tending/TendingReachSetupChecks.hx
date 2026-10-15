import haxe.crypto.Sha256;
import materia.project.SceneArtifact;
import sys.FileSystem;
import sys.io.File;

/** Exercise cache integrity and code invalidation with real resolved scene data. */
class TendingReachSetupChecks {
  static var assertions = 0;
  static function require(value:Bool, message:String):Void {
    assertions++;
    if (!value) throw message;
  }
  static function digest(bytes:haxe.io.Bytes):String
    return haxe.crypto.Base64.encode(Sha256.make(bytes));

  public static function run():Void {
    var inputs = new TendingReachInputs(null);
    require(TendingReachLayout.fingerprint(TendingCyclePlan.bearingBlocks(), inputs) ==
      TendingReachLayout.decode(TendingReachCertificate.JSON).fingerprint, "direct scenes preserve the exact original fingerprint");
    var physical = cadbridge.AssemblyPhysicalPartView.fromSceneArtifact(inputs.millScene);
    require(physical.parts.length == inputs.millScene.parts.length, "direct preview data resolves material properties without decoding");
    var root = "machinekit/examples/machine-tending/build/setup-check-" + Std.string(Math.random()).substr(2);
    var path = root + "/nested/mill.scene", code = root + "/code";
    var builds = 0;
    function build():materia.project.SceneArtifact.SceneArtifactData { builds++; return inputs.millScene; }
    var cold = @:privateAccess TendingGeometryCache.artifact(path, build);
    require(builds == 1 && FileSystem.exists(path), "cold artifact is built and stored");
    var warm = @:privateAccess TendingGeometryCache.artifact(path, build);
    require(builds == 1 && digest(cold.bytes) == digest(warm.bytes), "warm artifact skips construction and preserves exact bytes");
    require(digest(SceneArtifact.encode(warm.data)) == digest(cold.bytes), "cached decode preserves the canonical artifact");
    File.saveContent(path, "truncated");
    var repaired = @:privateAccess TendingGeometryCache.artifact(path, build);
    require(builds == 2 && digest(repaired.bytes) == digest(cold.bytes), "truncated cache rebuilds from source");
    var corrupt = File.getBytes(path); corrupt.set(40, corrupt.get(40) ^ 1); File.saveBytes(path, corrupt);
    repaired = @:privateAccess TendingGeometryCache.artifact(path, build);
    require(builds == 3 && digest(repaired.bytes) == digest(cold.bytes), "same-length corrupted cache rebuilds from source");
    var bypass = @:privateAccess TendingGeometryCache.artifact(null, build);
    require(builds == 4 && digest(bypass.bytes) == digest(cold.bytes), "disabled cache builds directly");
    File.saveContent(code, "aaaa");
    var first = @:privateAccess TendingGeometryCache.contentKey([code]);
    require(first == (@:privateAccess TendingGeometryCache.contentKey([code])), "unchanged code keeps its cache identity");
    File.saveContent(code, "bbbb");
    require(first != (@:privateAccess TendingGeometryCache.contentKey([code])), "same-length code changes invalidate the cache");
    var unavailable = @:privateAccess TendingGeometryCache.artifact(code + "/mill.scene", build);
    require(builds == 5 && digest(unavailable.bytes) == digest(cold.bytes) && File.getContent(code) == "bbbb",
      "unwritable cache storage falls back without damaging the blocking file");
    FileSystem.deleteFile(path); FileSystem.deleteFile(code); FileSystem.deleteDirectory(root + "/nested");
    FileSystem.deleteDirectory(root);
    Sys.println('Tending setup checks passed ($assertions assertions)');
  }
}
