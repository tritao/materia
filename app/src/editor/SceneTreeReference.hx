package app.editor;

import app.EditorSceneTree;
import haxe.io.Path;
import haxeon.ui.widgets.overlays.MenuItem;

/** Snapshot of an occurrence or group, independent of later selection, filtering and renaming. */
class SceneTreeReference {
  public final name:String;
  public final key:String;
  public final scene:String;
  public final path:Array<String>;
  public final project:Null<String>;
  public final variant:Null<String>;

  function new(name:String, key:String, scene:String, path:Array<String>, project:Null<String>, variant:Null<String>) {
    this.name = name;
    this.key = key;
    this.scene = scene;
    this.path = path;
    this.project = project;
    this.variant = variant;
  }

  public static function capture(tree:EditorSceneTree, key:String, ?project:String, ?variant:String):SceneTreeReference {
    var file = project;
    if (file != null) {
      file = Path.normalize(file);
      var root = Path.normalize(Sys.getCwd());
      if (!StringTools.endsWith(root, "/")) root += "/";
      if (StringTools.startsWith(file, root)) file = file.substr(root.length);
    }
    return new SceneTreeReference(tree.itemName(key), key, tree.itemName(tree.assemblyKey()),
      [for (ancestor in tree.objectPath(key)) tree.itemName(ancestor)], file, variant);
  }

  public function text():String {
    var lines = ["Scene: " + singleLine(scene)];
    if (project != null) lines.push("Project: " + singleLine(project));
    if (variant != null) lines.push("Variant: " + singleLine(variant));
    lines.push("Path: " + [for (name in path) singleLine(name)].join(" / "));
    lines.push("Key: " + singleLine(key));
    return lines.join("\n");
  }

  /** Read-only actions remain available while playback or document edits block mutation. */
  public static function copyItems(reference:SceneTreeReference, copy:String->Void):Array<MenuItem> return [
    new MenuItem("copy-reference", "Copy reference", () -> copy(reference.text())),
    new MenuItem("copy-name", "Copy name", () -> copy(reference.name))
  ];

  static function singleLine(value:String):String
    return StringTools.replace(StringTools.replace(value, "\r", "\\r"), "\n", "\\n");
}
