package app;

import haxeon.platform.NativeKitError;
import haxeon.platform.async.Outcome;
import haxeon.platform.resource.Resource;

import haxeon.ui.host.DesktopUiHostContext;
import nativekit.ffi.NativeKit;
import nativekit.ffi.NativeKitTypes;
import haxeon.platform.event.PlatformEvent;
import haxe.io.Bytes;

/**
 * The native file chooser. Documents are files the editor reads and writes by path: a desktop chooser gives a local
 * path, and in the browser a picked file or save target is kept in the editor's own storage (BrowserFiles).
 */
class SceneFileDialogs {
  final host:DesktopUiHostContext;
  /** Cancels and stops observing the chooser in flight; the ticket tells a stale completion from the current one. */
  var stopPending:Null<Void -> Void> = null;
  var ticket = 0;

  public function new(host:DesktopUiHostContext) this.host = host;

  public function choose(save:Bool, current:Null<String>, complete:Null<String> -> Null<String> -> Void):Void {
    chooseResource(save, save ? "Save scene" : "Open scene", save ? "Untitled.materia.json" : null, current, complete);
  }

  public function chooseExport(title:String,
    suggestedName:String, complete:Null<String> -> Null<String> -> Void):Void chooseResource(
      true,
      title,
      suggestedName,
      null,
      complete
    );

  public function chooseImport(title:String, complete:Null<String> -> Null<String> -> Void):Void chooseResource(
    false,
    title,
    null,
    null,
    complete
  );

  function chooseResource(
    save:Bool,
    title:String,
    suggestedName:Null<String>,
    current:Null<String>,
    complete:Null<String> -> Null<String> -> Void
  ):Void {
    var options = new FileDialogOptions();
    options.set_title(title);
    if (save) {
      options.set_flags(DialogFlags.ConfirmOverwrite);
      if (suggestedName != null) options.set_suggested_name(suggestedName);
    }
    if (current != null) options.set_initial_path(current);
    var parent = new Handle(host.window.rawValue());
    var id = save ? NativeKit.nk_dialog_save_resource_checked(parent,
      options) : NativeKit.nk_dialog_open_resource_checked(
        parent,
        options
      );
    var future = host.events.requests.trackNative(id, function(event:PlatformEvent):Outcome<Array<Resource>> {
      return switch (event) {
        case Resources(EventKind.DialogResourcesComplete, _, result, accepted, items):
          if (result == Result.ErrorCancelled) Outcome.Cancelled;
          else if (result != Result.Ok) Outcome.Failure(new NativeKitError(result, "SceneFileDialogs.choose", null));
          else if (!accepted) Outcome.Cancelled;
          else Outcome.Success(items);
        default:
          throw "Unexpected file chooser response";
      }
    }, function() NativeKit.nk_dialog_cancel_checked(id));
    var mine = ++ticket;
    stopPending = function() {
      future.cancel();
      future.stopObserving();
    };
    future.onComplete(function(outcome) {
      if (mine != ticket) return;
      stopPending = null;
      switch (outcome) {
        case Success(items):
          if (items.length == 0) {
            complete(null, null);
            return;
          }
          #if wasm
          if (!StringTools.startsWith(items[0].uri, "file://")) {
            browserFile(save, items[0], suggestedName, complete);
            return;
          }
          #end
          var path:String;
          try path = localPath(items[0].uri) catch (failure:Dynamic) {
            complete(null, Std.string(failure));
            return;
          }
          complete(path, null);
        case Cancelled | Stopped:
          complete(null, null);
        case Failure(error):
          complete(null, "The file chooser failed: " + error.message);
      }
    });
  }

  #if wasm
  /**
   * A browser chooser gives a picked file (a blob: URL) or a save target (a retained file handle or a download), not
   * a path. The document becomes /files/<name> in the editor's storage: a picked file is read into it first, and a
   * save target is linked to it, so the writes the editor makes there reach the user's file.
   */
  function browserFile(
    save:Bool,
    item:Resource,
    suggestedName:Null<String>,
    complete:Null<String> -> Null<String> -> Void
  ):Void {
    var path = BrowserFiles.pathFor(item.displayName != null ? item.displayName : suggestedName);
    if (save) {
      BrowserFiles.link(path, item.uri);
      complete(path, null);
      return;
    }
    var resource = new haxeon.platform.resource.Resource(item.uri, null, item.displayName).nativeValue();
    var id = NativeKit.nk_resource_load_async_checked(resource);
    var future = host.events.requests.trackNative(id, function(event:PlatformEvent):Outcome<Bytes> {
      return switch (event) {
        case Raw(kind, _, _, result, _, _, data) if (kind == EventKind.ResourceDataComplete):
          if (result == Result.ErrorCancelled) Outcome.Cancelled;
          else if (result != Result.Ok) Outcome.Failure(new NativeKitError(result, "SceneFileDialogs.read", null));
          else Outcome.Success(data);
        default:
          throw "Unexpected file read response";
      }
    }, function() Resource.cancelLoad(id));
    var mine = ++ticket;
    stopPending = function() {
      future.cancel();
      future.stopObserving();
    };
    future.onComplete(function(outcome) {
      if (mine != ticket) return;
      stopPending = null;
      switch (outcome) {
        case Success(data):
          try {
            if (ProjectSourceLoader.isPrebuilt(path)) path = BrowserFiles.storeArtifact(item.displayName, data);
            else BrowserFiles.store(path, data);
          } catch (failure:Dynamic) {
            complete(null, Std.string(failure));
            return;
          }
          complete(path, null);
        case Cancelled | Stopped:
          complete(null, null);
        case Failure(error):
          complete(null, "Could not read the chosen file: " + error.message);
      }
    });
  }
  #end

  public function dispose():Void {
    var stop = stopPending;
    if (stop == null) return;
    // Cancel the native dialog, then stop observing so a late completion never reaches the caller.
    ticket++;
    stopPending = null;
    stop();
  }

  public static function localPath(uri:String):String {
    if (!StringTools.startsWith(uri, "file://")) throw "Choose a local file for this scene";
    var encoded = uri.substr(7);
    if (StringTools.startsWith(encoded, "localhost/")) encoded = encoded.substr(9);
    if (!StringTools.startsWith(encoded, "/")) throw "Choose a local file for this scene";
    var source = Bytes.ofString(encoded);
    var output = Bytes.alloc(source.length);
    var count = 0;
    var i = 0;
    while (i < source.length) {
      var value = source.get(i++);
      if (value == 37) {
        if (i + 1 >= source.length) throw "Invalid file URI escape";
        var high = hex(source.get(i++));
        var low = hex(source.get(i++));
        if (high < 0 || low < 0) throw "Invalid file URI escape";
        value = high * 16 + low;
      }
      if (value == 0) throw "Invalid file URI";
      output.set(count++, value);
    }
    var path = output.sub(0, count).toString();
    if (Sys.systemName() == "Windows" && path.length > 2 && path.charAt(2) == ":") path = path.substr(1);
    return path;
  }

  static function hex(value:Int):Int {
    if (value >= 48 && value <= 57) return value - 48;
    if (value >= 65 && value <= 70) return value - 55;
    if (value >= 97 && value <= 102) return value - 87;
    return -1;
  }
}
