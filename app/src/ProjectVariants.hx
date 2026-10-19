package app;

import haxe.Json;
import sys.io.File;

typedef ProjectVariant = {
  var id:String;
  var label:String;
  var summary:String;
  var entrypoint:String;
  /** Configuration and study replace a project scene; job is the legacy schema. */
  var kind:String;
}

/** Authored scene variants are independent of runtime missions and their programs. */
class ProjectVariants {
  public static function read(path:String):Array<ProjectVariant> return decode(Json.parse(File.getContent(path)));

  public static function decode(root:Dynamic):Array<ProjectVariant> {
    var raw:Dynamic=Reflect.field(root,"variants");
    if(raw==null)return [for(job in ProjectJobs.decode(root))
      {id:job.id,label:job.label,summary:job.summary,entrypoint:job.entrypoint,kind:"job"}];
    if(Reflect.field(root,"jobs")!=null)throw "A project cannot define both variants and legacy jobs";
    if(!Std.isOfType(raw,Array))throw "Project variants must be a list";
    var result:Array<ProjectVariant> = [],seen:Map<String,Bool> = [];
    var entries:Dynamic=Reflect.field(root,"entrypoints");
    for(value in (cast raw:Array<Dynamic>)) {
      var id=text(value,"id"),entry=text(value,"entrypoint"),kind=text(value,"kind");
      if(seen.exists(id))throw 'Duplicate project variant "$id"';
      // Older demos selected whole scenes. Preserve their IDs as configurations;
      // runtime tasks belong to the scene's missions, not a separate chooser.
      if(kind=="demo")kind="configuration";
      if(["configuration","study"].indexOf(kind)<0)throw 'Unknown project variant kind "$kind"';
      if(entries==null||Reflect.field(entries,entry)==null)throw 'Unknown variant entrypoint "$entry"';
      seen.set(id,true);
      result.push({id:id,label:text(value,"label"),summary:text(value,"summary"),entrypoint:entry,kind:kind});
    }
    if(result.length==0)throw "Project variants must not be empty";
    if(!seen.exists(text(root,"defaultVariant")))throw "Unknown default project variant";
    return result;
  }

  public static function selected(root:Dynamic,requested:Null<String>):Null<ProjectVariant> {
    var variants=decode(root);
    if(variants.length==0) {
      if(requested!=null)throw "This project does not define variants";
      return null;
    }
    var id=requested==null?text(root,Reflect.field(root,"variants")==null?"defaultJob":"defaultVariant"):requested;
    for(variant in variants)if(variant.id==id)return variant;
    throw 'Unknown project variant "$id"';
  }
  public static function entrypoint(root:Dynamic,requested:Null<String>):String {
    var variant=selected(root,requested);
    return variant==null?text(root,"defaultEntrypoint"):variant.entrypoint;
  }
  static function text(value:Dynamic,name:String):String {
    var raw:Dynamic=value==null?null:Reflect.field(value,name);
    if(!Std.isOfType(raw,String))throw 'Project variant field "$name" must be text';
    var result:String=raw;
    if(StringTools.trim(result).length==0||result.length>500||SceneCodec.containsNul(result))throw 'Invalid project variant field "$name"';
    return result;
  }
}
