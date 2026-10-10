package app;

import haxe.Json;
import sys.io.File;

/** Document workflow shared by commands, native close requests and confirmation UI. */
class SceneDocumentController {
  public final session:ProjectDocumentSession;
  final choosePath:Bool->Null<String>->(Null<String>->Null<String>->Void)->Void;
  final changed:Void->Void;
  final commitPendingEdit:Void->Void;
  final cancelPendingEdit:Void->Void;
  var pending:Null<Void->Void> = null;
  var trustedOpen:Null<String> = null;
  public var trustReference(default, null):Null<String> = null;
  public var choosing(default, null):Bool = false;
  public var error(default, null):Null<String> = null;
  /** Set by the host while something outside the controller, such as a background project load, owns the document. */
  public var busy:Null<Void->Bool> = null;
  /** Hosted artifact opening can report progress before replacing the current document. */
  public var openArtifact:Null<String->Void> = null;

  public function new(session:ProjectDocumentSession,
      choosePath:Bool->Null<String>->(Null<String>->Null<String>->Void)->Void, changed:Void->Void,
      ?commitPendingEdit:Void->Void, ?cancelPendingEdit:Void->Void) {
    this.session = session;
    this.choosePath = choosePath;
    this.changed = changed;
    this.commitPendingEdit = commitPendingEdit == null ? function() {} : commitPendingEdit;
    this.cancelPendingEdit = cancelPendingEdit == null ? function() {} : cancelPendingEdit;
  }

  public function needsConfirmation():Bool return pending != null;
  public function needsTrustConfirmation():Bool return trustedOpen != null;
  public function blocked():Bool return choosing || pending != null || trustedOpen != null || error != null ||
    (busy != null && busy());

  public function requestNew():Void interrupt(function() {
    try session.newDocument() catch (failure:Dynamic) { fail(failure); return; }
    changed();
  });

  public function requestOpen():Void interrupt(function() {
    choose(false, openAccepted);
  });

  /** Opens a known path (for example a recent file) with the same prompts as File > Open. */
  public function requestOpenPath(path:String):Void interrupt(function() openAccepted(path));

  /** Runs a document-replacing action after the unsaved-change prompt, reporting failures as errors. */
  public function requestRun(action:Void->Void):Void interrupt(function() {
    try action() catch (failure:Dynamic) { fail(failure); return; }
    changed();
  });

  function openAccepted(path:String):Void {
    try {
      if (ProjectSourceLoader.isPrebuilt(path)) {
        if (openArtifact != null) openArtifact(path); else session.open(path);
        changed();
        return;
      }
      var root:Dynamic = Json.parse(File.getContent(path));
      var project = SceneCodec.decodeProjectRoot(root);
      var script = project == null ? SceneCodec.decodeScriptRoot(root) : null;
      if (project != null && ProjectSourceLoader.isPrebuilt(Reflect.field(project, "reference"))) {
        session.open(path);
        changed();
        return;
      }
      if (project != null || script != null) {
        trustedOpen = path;
        trustReference = project != null ? Reflect.field(project, "reference") :
          Reflect.field(script, "reference");
        changed();
        return;
      }
      session.open(path);
    } catch (failure:Dynamic) { fail(failure); return; }
    changed();
  }

  public function resolveTrust(allow:Bool):Void {
    var path = trustedOpen;
    if (path == null) return;
    trustedOpen = null;
    trustReference = null;
    if (allow) {
      try session.open(path) catch (failure:Dynamic) { fail(failure); return; }
    }
    changed();
  }

  public function requestClose(close:Void->Void):Void interrupt(close);

  public function save(as:Bool = false):Void {
    if (!blocked()) {
      commitPendingEdit();
      saveThen(as, null);
    }
  }

  public function resolve(choice:String):Void {
    var action = pending;
    if (action == null) return;
    pending = null;
    changed();
    switch (choice) {
      case "discard": action();
      case "save": saveThen(false, action);
      default: // Cancel preserves the current scene and history.
    }
  }

  public function dismissError():Void { error = null; changed(); }

  function protect(action:Void->Void):Void {
    if (blocked()) return;
    if (session.isDirty()) { pending = action; changed(); }
    else action();
  }

  function interrupt(action:Void->Void):Void {
    if (blocked()) return;
    cancelPendingEdit();
    protect(action);
  }

  function saveThen(as:Bool, after:Null<Void->Void>):Void {
    var write = function(path:String) {
      try session.save(path) catch (failure:Dynamic) { fail(failure); return; }
      changed();
      if (after != null) after();
    };
    var path = session.path;
    if (!as && path != null) write(path);
    else choose(true, write);
  }

  function choose(save:Bool, accepted:String->Void):Void {
    choosing = true;
    changed();
    try choosePath(save, session.path, function(path, message) {
      choosing = false;
      if (message != null) { fail(message); return; }
      changed();
      if (path != null) accepted(path);
    }) catch (failure:Dynamic) { choosing = false; fail(failure); }
  }

  function fail(failure:Dynamic):Void {
    error = Std.string(failure);
    changed();
  }
}
