package app;

import haxe.Json;
import sys.io.File;

typedef ProjectJob = {
  var id:String;
  var label:String;
  var summary:String;
  var entrypoint:String;
}

/** Project-owned presets select complete generated scenes, never just a runtime mission. */
class ProjectJobs {
  public static function read(path:String):Array<ProjectJob> return decode(Json.parse(File.getContent(path)));

  public static function decode(root:Dynamic):Array<ProjectJob> {
    var raw:Dynamic = Reflect.field(root, "jobs");
    if (raw == null) return [];
    if (!Std.isOfType(raw, Array)) throw "Project jobs must be a list";
    var result:Array<ProjectJob> = [];
    var seen = new Map<String, Bool>();
    var entries:Dynamic = Reflect.field(root, "entrypoints");
    for (value in (cast raw:Array<Dynamic>)) {
      var id = text(value, "id"), label = text(value, "label"), entrypoint = text(value, "entrypoint");
      if (seen.exists(id)) throw 'Duplicate project job "$id"';
      if (entries == null || Reflect.field(entries, entrypoint) == null) throw 'Unknown job entrypoint "$entrypoint"';
      seen.set(id, true);
      result.push({id:id, label:label, entrypoint:entrypoint, summary:text(value, "summary")});
    }
    if (result.length == 0) throw "Project jobs must not be empty";
    var defaultJob = text(root, "defaultJob");
    if (!seen.exists(defaultJob)) throw 'Unknown default project job "$defaultJob"';
    return result;
  }

  public static function selected(root:Dynamic, requested:Null<String>):Null<ProjectJob> {
    var jobs = decode(root);
    if (jobs.length == 0) {
      if (requested != null) throw "This project does not define jobs";
      return null;
    }
    var id = requested == null ? text(root, "defaultJob") : requested;
    for (job in jobs) if (job.id == id) return job;
    throw 'Unknown project job "$id"';
  }

  public static function entrypoint(root:Dynamic, requested:Null<String>):String {
    var job = selected(root, requested);
    return job == null ? text(root, "defaultEntrypoint") : job.entrypoint;
  }

  static function text(value:Dynamic, name:String):String {
    var raw:Dynamic = value == null ? null : Reflect.field(value, name);
    if (!Std.isOfType(raw, String)) throw 'Project job field "$name" must be text';
    var result:String = raw;
    if (StringTools.trim(result).length == 0 || result.length > 500 || SceneCodec.containsNul(result))
      throw 'Invalid project job field "$name"';
    return result;
  }
}
