package tests;

import app.editor.ExampleBrowser;
import app.editor.ExampleCatalog;
import app.Main.ReferenceEditorApp;
import haxeon.ui.FontCollection;
import haxeon.ui.LayoutFrame;
import haxeon.ui.core.RenderNode;
import sys.FileSystem;

@:access(app.Main.ReferenceEditorApp)
class StartPageTests {
  static function check(value:Bool, message:String):Void {
    if (!value) throw message;
  }

  public static function main():Int {
    try {
      run();
      Sys.println("Start-page categories, variant search and navigation passed");
      return 0;
    } catch (error:Dynamic) {
      Sys.println('Start-page tests failed: $error');
      return 1;
    }
  }

  static function run():Void {
    var rootPath = Sys.getCwd();
    while (!FileSystem.exists(rootPath + "/app/src/Main.hx")) {
      while (StringTools.endsWith(rootPath, "/")) rootPath = rootPath.substr(0, rootPath.length - 1);
      var parent = haxe.io.Path.directory(rootPath);
      check(parent != rootPath && parent.length > 0, "repository root exists");
      rootPath = parent;
    }
    Sys.setCwd(rootPath);
    var all = ExampleCatalog.entries;
    var groups = ExampleBrowser.families(all);
    var seen:Map<String, Bool> = new Map();
    for (family in groups) {
      check(ExampleBrowser.categories.indexOf(family.category) > 0, "each family has a category");
      for (entry in family.examples) {
        check(!seen.exists(entry.id), "variants appear in exactly one family");
        seen.set(entry.id, true);
      }
    }
    for (entry in all) check(seen.exists(entry.id), "all catalogue entries remain reachable");
    var browser = new ExampleBrowser();
    browser.selectCategory("Welding");
    check(browser.filtered(all).length == 4, "nine welding examples form four families");
    browser.search("  WOVEN seam  ");
    var matches = browser.filtered(all);
    check(matches.length == 1 && matches[0].id == "robot-welding", "search finds variants regardless of case");
    check(matches[0].examples.length == 5, "search preserves the family's five variants");
    browser.selectCategory("Machining");
    check(browser.filtered(all).length == 0, "search and category compose");
    browser.search("");
    check(browser.filtered(all).length == 2, "bench mill variants share a card");
    browser.selectedFamily = "bench-mills";
    browser.selectCategory("All");
    check(browser.selectedFamily == null, "category navigation leaves a family detail");

    // A partial installation still groups only the variants that are actually available.
    var variant = ExampleCatalog.find("robot-welder-weave");
    check(variant != null, "woven seam exists");
    var partial = browser.filtered([cast variant]);
    check(partial.length == 1 && partial[0].title == "Robot welding" && partial[0].examples.length == 1,
      "a family works even when its primary example is absent");

    var directory = "build/start-page-test";
    FileSystem.createDirectory(directory);
    var fonts = FontCollection.create();
    var font:Null<String> = null;
    for (candidate in ["haxeon/packages/ui/vendor/skribidi/example/data/IBMPlexSans-Regular.ttf",
      "../../haxeon/packages/ui/vendor/skribidi/example/data/IBMPlexSans-Regular.ttf",
      "../../../haxeon/packages/ui/vendor/skribidi/example/data/IBMPlexSans-Regular.ttf"])
      if (FileSystem.exists(candidate)) font = candidate;
    check(font != null, "test font exists");
    fonts.add(cast font);
    var editor = new ReferenceEditorApp(fonts, directory + "/workspace.json");
    editor.commands.execute("start.show");
    var frame = new LayoutFrame(1600, 1000);
    var root = editor.submit(frame);
    var startRoot:RenderNode = cast find(root, "start-content");
    check(startRoot != null && find(startRoot, "search-field") != null, "search is present");
    click(editor, frame, "start-category:Welding");
    check(editor.startExamples.category == "Welding", "category button updates navigation");
    check(find(editor.submit(frame), "card-button:Robot welding") != null, "family card is present");
    var preview:RenderNode = cast find(editor.submit(frame), "start-preview:robot-welding");
    check(preview != null && preview.resolved != null && preview.resolved.width > 0 && preview.resolved.height > 0,
      "preview illustrations have paintable dimensions");
    click(editor, frame, "card-button:Robot welding");
    root = editor.submit(frame);
    for (id in ["robot-welder", "robot-welder-seam", "robot-welder-post", "robot-welder-weave", "robot-welder-multipass"])
      check(find(root, "start-open-example:" + id) != null, "detail offers " + id);
    click(editor, frame, "start-examples-back");
    check(editor.startExamples.selectedFamily == null && editor.startExamples.category == "Welding", "Back preserves category");
    editor.startExamples.search("nothing matches 12345");
    editor.invalidateView();
    check(find(editor.submit(frame), "start-clear-filters") != null, "empty search offers recovery");
    click(editor, frame, "start-clear-filters");
    check(editor.startExamples.query == "" && editor.startExamples.category == "All", "clear restores all families");
    var narrow = editor.submit(new LayoutFrame(1024, 768));
    check(find(narrow, "start-category:Welding") != null && find(narrow, "start-category:People & simulation") != null,
      "all categories remain available in a narrower window");
    editor.dispose();
    fonts.dispose();
  }

  static function find(node:RenderNode, key:String):Null<RenderNode> {
    if (node.styleKey == key) return node;
    for (child in node.children) {
      var found = find(child, key);
      if (found != null) return found;
    }
    return null;
  }

  static function click(editor:ReferenceEditorApp, frame:LayoutFrame, key:String):Void {
    var node:RenderNode = cast find(editor.submit(frame), key);
    check(node != null && node.resolved != null, "control exists: " + key);
    var bounds = node.resolved.clippedViewportBounds();
    check(bounds.width > 0 && bounds.height > 0, "control is visible: " + key);
    var x = bounds.x + bounds.width / 2, y = bounds.y + bounds.height / 2;
    editor.ui.pointerDown(x, y, 0);
    editor.ui.pointerUp(x, y, 0);
    editor.submit(frame);
  }
}
