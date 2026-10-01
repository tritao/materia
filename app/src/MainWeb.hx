package app;

import FontFamily;
import app.Main.ReferenceEditorApp;
import haxe.CallStack;
import nativekit.ui.host.BrowserUiHost;
import nativekit.ui.host.BrowserUiHostOptions;
import nativekit.ui.host.BrowserUiHostOptions.BrowserUiFontAsset;
import nativekit.ui.host.BrowserUiHostSession;
import nativekit.ui.host.DesktopUiHostContext;
import nativekit.ui.host.UiHostSession.UiHostLifecycle;
import nativekit.ui.theme.Theme;

/**
 * Browser entry point for the reference editor. The page calls the exposed `configure`, then `main` once, then
 * `frame` from each requestAnimationFrame tick; the editor itself is the one `Main.open` hosts on the desktop.
 */
class MainWeb {
  static var width = 1280;
  static var height = 800;
  static var darkTheme = false;
  static var session:Null<BrowserUiHostSession>;
  static var editor:Null<ReferenceEditorApp>;

  /** Sets the initial canvas size in CSS pixels and the theme (0 = light, 1 = dark) before `main`. */
  @:expose public static function configure(canvasWidth:Int, canvasHeight:Int, theme:Int):Int {
    if (session != null || canvasWidth <= 0 || canvasHeight <= 0) return 1;
    width = canvasWidth;
    height = canvasHeight;
    darkTheme = theme == 1;
    return 0;
  }

  @:expose public static function main():Int {
    try {
      var options = new BrowserUiHostOptions();
      options.title = "Materia";
      options.width = width;
      options.height = height;
      options.fonts = [
        new BrowserUiFontAsset("IBMPlexSans-Regular", "assets/IBMPlexSans-Regular.ttf",
          "/assets/IBMPlexSans-Regular.ttf", FontFamily.Default),
        new BrowserUiFontAsset("NotoEmoji-Regular", "assets/NotoEmoji-Regular.ttf",
          "/assets/NotoEmoji-Regular.ttf", FontFamily.Emoji)
      ];
      var started = BrowserUiHost.start(options, function(context) {
        var host:DesktopUiHostContext = cast context;
        var created = new ReferenceEditorApp(host.fonts, null, darkTheme ? Theme.dark() : Theme.light(), null, host);
        if (created.preferences.showStartPage) created.showStartPage();
        editor = created;
        return created;
      });
      session = started;
      return started.state == UiHostLifecycle.Failed ? 2 : 0;
    } catch (error:Dynamic) {
      record(error);
      return 1;
    }
  }

  /** Advances the host and the editor; returns 1 while running, 0 once stopped, and a negative value on failure. */
  @:expose public static function frame(time:Float):Int {
    var active = session;
    if (active == null) return 0;
    try {
      var advanced = active.advance(time);
      if (advanced < 0) {
        var failure = active.error;
        Sys.println("materia: " + (failure == null ? "the UI host failed" : failure.toString() + "\n" + failure.stack));
        return advanced;
      }
      if (editor != null) editor.tick();
      return advanced;
    } catch (error:Dynamic) {
      record(error);
      return -1;
    }
  }

  /** Failures go to the page's console; `main` and `frame` also report them through their result. */
  static function record(error:Dynamic):Void {
    Sys.println("materia: " + Std.string(error) + "\n" + CallStack.toString(CallStack.exceptionStack(true)));
  }
}
