package tests;

import app.MateriaProjectRunner;

/** Real child exit statuses must retain diagnostics and distinguish a signal from a compiler error. */
class ProjectProcessTests {
  public static function run():Void {
    if (Sys.systemName() == "Windows") return;
    expect("printf 'compiler diagnostic' >&2; exit 7", "exit 7", "compiler diagnostic");
    expect("kill -TERM $$", "terminated by SIGTERM (signal 15)", null);
    expect("kill -KILL $$", "terminated by SIGKILL (signal 9)", null);
    Sys.println("Project child process diagnostics passed");
  }

  static function expect(script:String, status:String, diagnostic:Null<String>):Void {
    var failure:Null<String> = null;
    try MateriaProjectRunner.runCommand("/bin/sh", ["-c", script], "Project compiler")
    catch (error:Dynamic) failure = Std.string(error);
    if (failure == null || failure.indexOf(status) < 0 ||
        (diagnostic != null && failure.indexOf(diagnostic) < 0))
      throw 'Incorrect child process diagnostic: $failure';
  }
}
