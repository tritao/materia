package tests;

import app.Main.ReferenceEditorApp;
import haxeon.ui.FontCollection;
import haxeon.ui.LayoutFrame;
import sys.FileSystem;
import sys.io.File;

/** Project extension state is prepared by the owner, before tracked Inspector builds. */
@:access(app.Main.ReferenceEditorApp) @:access(app.ProjectDocumentSession)
class ProjectUiFrameTests {
  static function check(value:Bool, message:String):Void if (!value) throw message;

  public static function run():Void {
    var directory = "build/project-ui-frame-tests";
    FileSystem.createDirectory(directory);
    var manifest = FileSystem.fullPath(directory) + "/project.json";
    File.saveContent(manifest, '{"uiExtension":{"kind":"unsupported"}}');
    var fonts = FontCollection.create();
    fonts.add(cast TestPaths.font());
    var editor = new ReferenceEditorApp(fonts, directory + "/workspace.json");
    try {
      editor.workspace.close("start");
      editor.workspace.activate("inspector");
      var frame = new LayoutFrame(1320, 900);
      editor.submit(frame);
      // No geometry generation is needed: even assigning the null extension in the old
      // lazy panel builder failed the Observable write guard inside its tracked scope.
      editor.session.projectReference = manifest;
      editor.submit(frame);
      check(editor.projectUiReference == manifest, "project extension reference is prepared before building");
      check(editor.projectUiError == "Unsupported project UI extension", "extension errors become panel state");
      editor.submit(frame);

      var plain = FileSystem.fullPath(directory) + "/plain.json";
      File.saveContent(plain, '{}');
      editor.session.projectReference = plain;
      editor.submit(frame);
      check(editor.projectUiError == null && editor.projectUiExtension == null, "replacement clears extension error state");
      editor.session.projectReference = null;
      editor.submit(frame);
      check(editor.projectUiReference == null, "closing the project clears its prepared extension reference");
    } catch (error:Dynamic) {
      editor.dispose();
      fonts.dispose();
      throw error;
    }
    editor.dispose();
    fonts.dispose();
    Sys.println("Project UI frame preparation tests passed");
  }
}
