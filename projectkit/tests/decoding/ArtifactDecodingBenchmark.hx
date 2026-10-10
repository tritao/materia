import haxe.io.Bytes;
import materia.project.SceneArtifact;

/** Isolates binary decoding from disk I/O, compilation, geometry preparation and rendering. */
class ArtifactDecodingBenchmark {
  static var source:Bytes;
  static function main():Void {
    source = sys.io.File.getBytes(Sys.args()[0]);
    for (borrow in [false, true]) for (round in 0...5) {
      var allocated = hl.Gc.totalAllocated(), start = Sys.time();
      var decoded = borrow ? SceneArtifact.decodeView(source) : SceneArtifact.decode(source);
      var decodeSeconds = Sys.time() - start, decodeBytes = hl.Gc.totalAllocated() - allocated;
      start = Sys.time();
      for (part in decoded.parts) @:privateAccess SceneArtifact.validatePart(part, true);
      Sys.println(haxe.Json.stringify({borrow: borrow, round: round, sourceBytes: source.length, parts: decoded.parts.length,
        decodeMilliseconds: decodeSeconds * 1000, allocatedBytes: decodeBytes,
        meshValidationMilliseconds: (Sys.time() - start) * 1000}));
    }
  }
}
