package app;

/** Document workflow shared by commands, native close requests and confirmation UI. */
class SceneDocumentController {
  public final session:ProjectDocumentSession;
  final choosePath:Bool->Null<String>->(Null<String>->Null<String>->Void)->Void;
  final changed:Void->Void;
  final commitPendingEdit:Void->Void;
  final cancelPendingEdit:Void->Void;
  var pending:Null<Void->Void> = null;
  public var choosing(default, null):Bool = false;
  public var error(default, null):Null<String> = null;

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
  public function blocked():Bool return choosing || pending != null || error != null;

  public function requestNew():Void interrupt(function() {
    try session.newDocument() catch (failure:Dynamic) { fail(failure); return; }
    changed();
  });

  public function requestOpen():Void interrupt(function() {
    choose(false, function(path) {
      try session.open(path) catch (failure:Dynamic) { fail(failure); return; }
      changed();
    });
  });

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
