package app.editor;

import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyBodies;

/** Resolve old body-based masks once when opening scenes saved before ownership-based trees. */
class LegacyAssemblyVisibility {
  public static function groups(definition:AssemblyDefinition):Map<String,Array<String>> {
    var result:Map<String,Array<String>> = [],bodies=AssemblyBodies.of(definition);
    var ownership=[for(o in definition.occurrences) o.id => o.includePath];
    function descendants(root:String):Array<String> {
      var ids:Array<String> = [];
      for(body in bodies)if(body.id==root)for(id in body.occurrences)ids.push("project:"+id);
      for(body in bodies)if(body.parent==root)for(id in descendants(body.id))ids.push(id);
      return ids;
    }
    function addPaths(container:String,path:Null<String>,ids:Array<String>):Void {
      if(path==null || path=="")return;
      var prefix="";
      for(segment in path.split("/")) {
        prefix+=(prefix==""?"":"/")+segment;
        var key="parts:"+container+":"+prefix;
        if(!result.exists(key))result.set(key,[]);
        for(id in ids)if(result.get(key).indexOf(id)<0)result.get(key).push(id);
      }
    }
    for(body in bodies) {
      var members=descendants(body.id);
      result.set("project:"+body.id,members);
      result.set("parts:"+body.id,[for(id in body.occurrences.slice(1)) "project:"+id]);
      for(id in body.occurrences.slice(1))addPaths("parts:"+body.id,ownership.get(id),["project:"+id]);
      if(body.parent==null)addPaths("scene",ownership.get(body.id),members);
    }
    return result;
  }
}
