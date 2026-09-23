package app;

import nativekit.ui.host.DesktopUiHostContext;
import nativekit.ffi.NativeKit;
import nativekit.ffi.NativeKitTypes;
import NativeKitEventValue;
import haxe.io.Bytes;

/** Native desktop file chooser; this document format currently uses local files. */
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
          var path:String;
          try path = localPath(items[0].uri)
          catch (failure:Dynamic) { complete(null, Std.string(failure)); return; }
          complete(path, null);
        default: complete(null, "Unexpected file chooser response");
      }
    });
  }

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
