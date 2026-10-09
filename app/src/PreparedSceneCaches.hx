package app;

import sys.io.File;

typedef PreparedSceneCacheContext = {
  var store:PreparedSceneStore;
  var buildIdentity:String;
};

/** Host policy only; decoding and preparation accept their cache as a dependency. */
class PreparedSceneCaches {
  static var browser:Null<PreparedSceneCacheContext>;
  public static function configureBrowser(buildIdentity:String):Void {
    if (buildIdentity == null || buildIdentity.length != 64) throw "Missing browser build identity";
    browser = {store: new FilePreparedSceneStore("/cache/materia/prepared-scenes"), buildIdentity: buildIdentity};
  }
  public static function current():Null<PreparedSceneCacheContext> {
    #if wasm
    return browser;
    #else
    try {
      var identity = StringTools.trim(File.getContent(Sys.programPath() + ".build-id"));
      if (identity.length != 64) return null;
      var cache = Sys.getEnv("XDG_CACHE_HOME");
      if (cache == null || cache.length == 0) {
        var home = Sys.getEnv("HOME");
        if (home == null || home.length == 0) return null;
        cache = home + "/.cache";
      }
      return {store: new FilePreparedSceneStore(cache + "/materia/prepared-scenes"), buildIdentity: identity};
    } catch (_:Dynamic) return null;
    #end
  }
}
