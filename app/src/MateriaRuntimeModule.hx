package app;

import haxe.io.Bytes;

/** Native bridge for compiling a project and calling its isolated entrypoint. */
@:hlNative("haxeon_runtime")
private class MateriaRuntimeNative {
  public static function load(bytes:hl.Bytes, length:Int, identity:hl.Bytes,
      identityLength:Int):hl.Abstract<"realtime_module"> return null;

  public static function call_managed_bytes(module:hl.Abstract<"realtime_module">, index:Int):Bytes return null;

  public static function dispose(module:hl.Abstract<"realtime_module">):Int return -1;
}

class MateriaRuntimeModule {
  static final RETIREMENT_BLOCKED:Int = 8;
  static final pendingDisposals:Array<hl.Abstract<"realtime_module">> = [];

  public static function evaluateBytes(moduleBytes:Bytes, identity:Bytes, functionId:Int):Bytes {
    retryPendingDisposals();
    var module = MateriaRuntimeNative.load(cast moduleBytes.getData(), moduleBytes.length,
      cast identity.getData(), identity.length);
    if (module == null) throw "Haxeon rejected the project runtime module";
    var result:Bytes;
    try {
      result = MateriaRuntimeNative.call_managed_bytes(module, functionId);
      if (result == null) throw "Project entrypoint returned no geometry artifact";
    } catch (error:Dynamic) {
      disposeOrDefer(module);
      throw error;
    }
    hl.Gc.major();
    disposeOrDefer(module);
    retryPendingDisposals();
    return result;
  }

  static function disposeOrDefer(module:hl.Abstract<"realtime_module">):Void {
    var status = MateriaRuntimeNative.dispose(module);
    if (status == 0) return;
    if (status == RETIREMENT_BLOCKED) {
      pendingDisposals.push(module);
      return;
    }
    throw 'Could not dispose the project runtime module (status $status)';
  }

  static function retryPendingDisposals():Void {
    if (pendingDisposals.length == 0) return;
    hl.Gc.major();
    var retained:Array<hl.Abstract<"realtime_module">> = [];
    for (module in pendingDisposals) {
      var status = MateriaRuntimeNative.dispose(module);
      if (status == RETIREMENT_BLOCKED) retained.push(module);
      else if (status != 0) throw 'Could not dispose a project runtime module (status $status)';
    }
    pendingDisposals.resize(0);
    for (module in retained) pendingDisposals.push(module);
  }
}
