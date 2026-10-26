package materia.project;

import materia.project.SceneArtifact.SceneArtifactData;
import materia.assembly.AssemblyDefinitionFlattener;

/** Producer-declared source links; IDs are opaque and never inferred from labels. */
typedef SceneSourceMap = {
 var features:Array<SceneSourceFeature>;
 var members:Array<SceneSourceMember>;
}
typedef SceneSourceFeature = {
 var id:String;
 var label:String;
 var occurrence:String;
 var faces:Array<Int>;
 var whole:Bool;
}
typedef SceneSourceMember = {
 var occurrence:String;
 var sources:Array<String>;
 var shared:Bool;
}
typedef SceneNavigation = {
 var occurrence:String;
 var label:String;
 var variant:String;
 var target:String;
}

class SceneSourceMaps {
 static function fields(raw:Dynamic,names:Array<String>):Void {
  if(raw==null || Std.isOfType(raw,Array) || Std.isOfType(raw,String))throw "Invalid scene source record";
  for(name in Reflect.fields(raw))if(names.indexOf(name)<0)throw "Unknown scene source field: "+name;
  for(name in names)if(!Reflect.hasField(raw,name))throw "Missing scene source field: "+name;
 }
 static function array(raw:Dynamic):Array<Dynamic> {
  if(!Std.isOfType(raw,Array))throw "Scene source field must be an array";
  var values:Array<Dynamic>=cast raw;
  if(values.length>100000)throw "Scene source array is too large";
  return values;
 }
 static function string(raw:Dynamic):String {
  if(!Std.isOfType(raw,String))throw "Scene source field must be text";
  var value:String=raw;text(value);return value;
 }
 public static function decode(raw:Dynamic):SceneSourceMap {
  fields(raw,["features","members"]);
  return {features:[for(feature in array(Reflect.field(raw,"features"))) {
   fields(feature,["id","label","occurrence","faces","whole"]);
   var whole=Reflect.field(feature,"whole");if(!Std.isOfType(whole,Bool))throw "Whole source flag must be boolean";
   var faces:Array<Int>=[];
   for(face in array(Reflect.field(feature,"faces"))) {
    if(!Std.isOfType(face,Int))throw "Source face must be an integer";
    faces.push(face);
   }
   {id:string(Reflect.field(feature,"id")),label:string(Reflect.field(feature,"label")),
    occurrence:string(Reflect.field(feature,"occurrence")),faces:faces,whole:(whole:Bool)};
  }],members:[for(member in array(Reflect.field(raw,"members"))) {
   fields(member,["occurrence","sources","shared"]);
   var shared=Reflect.field(member,"shared");if(!Std.isOfType(shared,Bool))throw "Shared source flag must be boolean";
   {occurrence:string(Reflect.field(member,"occurrence")),shared:(shared:Bool),
    sources:[for(source in array(Reflect.field(member,"sources")))string(source)]};
  }]};
 }
 public static function decodeNavigation(raw:Dynamic):Array<SceneNavigation> {
  return [for(link in array(raw)) {
   fields(link,["occurrence","label","variant","target"]);
   {occurrence:string(Reflect.field(link,"occurrence")),label:string(Reflect.field(link,"label")),
    variant:string(Reflect.field(link,"variant")),target:string(Reflect.field(link,"target"))};
  }];
 }

 public static function validate(map:SceneSourceMap,data:SceneArtifactData):Void {
  if (data.assemblyDefinition == null) throw "Source links require assembly occurrences";
  var flat=AssemblyDefinitionFlattener.flattenView(data.assemblyDefinition);
  var occurrences=[for (item in flat.occurrences) item.id => item.definition];
  var parts=[for (part in data.parts) part.id => part];
  var sources=new Map<String,Bool>();
  for (feature in map.features) {
   text(feature.id);text(feature.label);
   if (sources.exists(feature.id)) throw "Duplicate scene source: "+feature.id;
   sources.set(feature.id,true);
   var part=parts.get(occurrences.get(feature.occurrence));
   if (part == null) throw "Unknown scene source occurrence: "+feature.occurrence;
   if(feature.whole && feature.faces.length>0)throw "Whole source surface cannot also name faces";
   var faces=[for (range in part.faceRanges) range.faceIndex => true],seen=new Map<Int,Bool>();
   for (face in feature.faces) {
    if (!faces.exists(face) || seen.exists(face)) throw "Invalid source face: "+feature.id;
    seen.set(face,true);
   }
  }
  var seen=new Map<String,Bool>();
  for (member in map.members) {
   if (!occurrences.exists(member.occurrence) || seen.exists(member.occurrence))
    throw "Invalid source member: "+member.occurrence;
   seen.set(member.occurrence,true);
   if ((member.sources.length == 0) != member.shared) throw "Source member needs source IDs or shared-hardware declaration";
   var ids=new Map<String,Bool>();
   for (id in member.sources) {
    if (!sources.exists(id) || ids.exists(id)) throw "Invalid member source: "+id;
    ids.set(id,true);
   }
  }
 }
 public static function tooling(map:SceneSourceMap,id:String):Array<String>
  return [for (member in map.members) if (member.sources.indexOf(id)>=0) member.occurrence];
 public static function sources(map:SceneSourceMap,id:String):Array<String> {
  for (member in map.members) if (member.occurrence==id) return member.sources.copy();
  return [];
 }
 public static function text(value:String):Void {
  if (value == null || StringTools.trim(value)=="" || value.length>4096)
   throw "Invalid scene source text";
  for(index in 0...value.length)if(value.charCodeAt(index)==0)throw "Invalid scene source text";
 }
}
