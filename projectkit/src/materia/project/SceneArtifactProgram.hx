package materia.project;

import haxe.io.Bytes;
import haxe.crypto.Sha256;
import materia.project.SceneArtifact.SceneArtifactMachining;

/** Program dependencies stay outside the bounded geometry metadata section. */
class SceneArtifactProgram {
  public static inline final MAX_BYTES = 64000000;

  public static function validate(job:SceneArtifactMachining):Void {
    if ((job.program == null) == (job.programFile == null))
      throw "Machining job needs exactly one of program and programFile";
    if (job.programFile != null) {
      var path = job.programFile;
      if (path.length == 0 || path.length > 1024 || path.indexOf("\\") >= 0 || path.indexOf(":") >= 0)
        throw "Invalid relative machining program path";
      for (i in 0...path.length) if (path.charCodeAt(i) < 32) throw "Invalid relative machining program path";
      for (segment in path.split("/")) if (segment == "" || segment == "." || segment == "..")
        throw "Invalid relative machining program path";
      if (job.programSha256 == null || !~/^[a-f0-9]{64}$/.match(job.programSha256))
        throw "External machining program needs its SHA-256";
    } else if (job.programSha256 != null) throw "Inline program cannot declare an external digest";
  }

  public static function digest(bytes:Bytes):String {
    var hash = Sha256.make(bytes), result = new StringBuf();
    var digits="0123456789abcdef";
    for (i in 0...hash.length) { var value=hash.get(i); result.add(digits.charAt(value>>4)); result.add(digits.charAt(value&15)); }
    return result.toString();
  }

  public static function resolve(job:SceneArtifactMachining, ?read:String->Bytes):String {
    validate(job);
    if (job.program != null) return job.program;
    if (read == null) throw "No reader for external machining program";
    var bytes = read(job.programFile);
    if (bytes == null || bytes.length > MAX_BYTES) throw "Machining program is missing or too large";
    if (digest(bytes) != job.programSha256) throw "Machining program SHA-256 mismatch";
    return bytes.toString();
  }
}
