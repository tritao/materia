import haxe.io.Bytes;
import materia.project.SceneArtifact;

/** Supply an artifact outside the timed region, then measure the browser's real decoder. */
class ArtifactDecodingBrowserBenchmark {
  static var source:Bytes;
  static function main():Int return 42;
  @:expose public static function initialize(length:Int):Void source = Bytes.alloc(length);
  @:expose public static function setWord(offset:Int, value:Int):Void source.setInt32(offset, value);
  @:expose public static function setByte(offset:Int, value:Int):Void source.set(offset, value);
  @:expose public static function decode(rounds:Int):Int {
    var count = 0;
    for (_ in 0...rounds) count += SceneArtifact.decodeView(source).parts.length;
    return count;
  }
}
