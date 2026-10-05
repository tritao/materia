package app;

import haxeon.platform.NativeKitEventValue.NativeKitResource;

import haxeon.ui.host.DesktopUiHostContext;
import nativekit.ffi.NativeKit;
import nativekit.ffi.NativeKitTypes;
import haxeon.platform.NativeKitEventValue;
import haxe.io.Bytes;

/**
 * The native file chooser. Documents are files the editor reads and writes by path: a desktop chooser gives a local
 * path, and in the browser a picked file or save target is kept in the editor's own storage (BrowserFiles).
 */
class SceneFileDialogs {
  final host:DesktopUiHostContext;
  var request:Null<haxe.Int64> = null;

  public function new(host:DesktopUiHostContext) this.host = host;

  public function choose(save:Bool, current:Null<String>,
      complete:Null<String>->Null<String>->Void):Void {
    chooseResource(save,save ? "Save scene" : "Open scene",
      save ? "Untitled.materia.json" : null,current,complete);
  }

  public function chooseExport(title:String,suggestedName:String,
      complete:Null<String>->Null<String>->Void):Void
    chooseResource(true,title,suggestedName,null,complete);

  public function chooseImport(title:String,
      complete:Null<String>->Null<String>->Void):Void
    chooseResource(false,title,null,null,complete);

  function chooseResource(save:Bool,title:String,suggestedName:Null<String>,current:Null<String>,
      complete:Null<String>->Null<String>->Void):Void {
    var options = new FileDialogOptions();
    options.set_title(title);
    if (save) {
      options.set_flags(DialogFlags.ConfirmOverwrite);
      if(suggestedName!=null)options.set_suggested_name(suggestedName);
    }
    if (current != null) options.set_initial_path(current);
    var parent = new Handle(host.window.rawValue());
    var id = save ? NativeKit.nk_dialog_save_resource_checked(parent, options)
      : NativeKit.nk_dialog_open_resource_checked(parent, options);
    request = id;
    host.events.requests.track(id, function(event) {
      request = null;
      switch (event) {
        case Resources(_, _, result, accepted, items):
          if (result != Result.Ok) { complete(null, "The file chooser failed: " + Std.string(result)); return; }
          if (!accepted || items.length == 0) { complete(null, null); return; }
          #if wasm
          if (!StringTools.startsWith(items[0].uri, "file://")) {
            browserFile(save, items[0], suggestedName, complete);
            return;
          }
          #end
          var path:String;
          try path = localPath(items[0].uri)
          catch (failure:Dynamic) { complete(null, Std.string(failure)); return; }
          complete(path, null);
        default: complete(null, "Unexpected file chooser response");
      }
    });
  }

  #if wasm
  /**
   * A browser chooser gives a picked file (a blob: URL) or a save target (a retained file handle or a download), not
   * a path. The document becomes /files/<name> in the editor's storage: a picked file is read into it first, and a
   * save target is linked to it, so the writes the editor makes there reach the user's file.
   */
  function browserFile(save:Bool, item:NativeKitResource, suggestedName:Null<String>,
      complete:Null<String>->Null<String>->Void):Void {
    var path = BrowserFiles.pathFor(item.displayName != null ? item.displayName : suggestedName);
    if (save) {
      BrowserFiles.link(path, item.uri);
      complete(path, null);
      return;
    }
    var resource = new Resource();
    resource.set_struct_size(Resource.size());
    resource.set_flags(ResourceFlags.Readable);
    resource.set_uri(item.uri);
    if (item.displayName != null) resource.set_display_name(item.displayName);
    var id = NativeKit.nk_resource_load_async_checked(resource);
    request = id;
    host.events.requests.track(id, function(event) {
      request = null;
      switch (event) {
        case Raw(kind, _, _, result, _, _, data) if (kind == EventKind.ResourceDataComplete):
          if (result != Result.Ok) { complete(null, "Could not read the chosen file: " + Std.string(result)); return; }
          try BrowserFiles.store(path, data)
          catch (failure:Dynamic) { complete(null, Std.string(failure)); return; }
          complete(path, null);
        default: complete(null, "Unexpected file read response");
      }
    });
  }
  #end

  public function dispose():Void {
    var id = request;
    if (id == null) return;
    host.events.requests.cancel(id);
    NativeKit.nk_dialog_cancel(id);
    request = null;
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
