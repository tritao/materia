package app.editor;

import app.Main.ReferenceEditorApp;
import nativekit.ui.core.Command;
import nativekit.ui.core.Shortcut;
import nativekit.ui.core.UiKey;
import nativekit.ui.core.UiModifier;

/** Shared document history and file commands. */
@:access(app.Main.ReferenceEditorApp)
class EditorDocumentCommands {
  public static function install(app:ReferenceEditorApp):Void {
    app.commands.register(new Command("editor.undo", "Undo", function() {
      if (app.scene.hasActiveSketchEdit()) app.scene.cancelSelectedSketchEdit();
      app.runSceneEdit("Could not undo", function() app.session.document.undo());
      if (app.session.scriptOwnership != null) app.refreshScriptMaterialization("Override undone");
    }, new Shortcut(UiKey.Z, UiModifier.Control), function() return !app.documents.blocked() && app.session.document.canUndo));
    app.commands.register(new Command("editor.redo", "Redo", function() {
      app.runSceneEdit("Could not redo", function() app.session.document.redo());
      if (app.session.scriptOwnership != null) app.refreshScriptMaterialization("Override redone");
    }, new Shortcut(UiKey.Z, UiModifier.Control | UiModifier.Shift), function() return !app.documents.blocked() && app.session.document.canRedo));
    app.commands.register(new Command("editor.new", "New", function() app.documents.requestNew(),
      new Shortcut(78, UiModifier.Control), function() return !app.documents.blocked()));
    app.commands.register(new Command("editor.open", "Open", function() app.documents.requestOpen(),
      new Shortcut(79, UiModifier.Control), function() return !app.documents.blocked()));
    app.commands.register(new Command("editor.save", "Save", function() app.documents.save(),
      new Shortcut(UiKey.S, UiModifier.Control), function() return !app.documents.blocked()));
    app.commands.register(new Command("editor.save-as", "Save As", function() app.documents.save(true),
      new Shortcut(UiKey.S, UiModifier.Control | UiModifier.Shift), function() return !app.documents.blocked()));
  }
}
