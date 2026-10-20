package app.editor;

import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinitionFlattener;
import materia.assembly.AssemblyBodies;

typedef AssemblyTreeGroup = {
  final label:String;
  final path:String;
}

/** Authored occurrence ownership, independent of the solver's rigid bodies and joint roots. */
class AssemblyHierarchy {
  public final definition:AssemblyDefinition;
  public final groups:Map<String,AssemblyTreeGroup> = [];
  public final children:Map<String,Array<String>> = [];
  public final parents:Map<String,String> = [];

  public function key(path:String = ""):String
    return "assembly:" + definition.id + (path == "" ? "" : ":" + path);

  public function new(source:AssemblyDefinition, objectIds:Array<String>) {
    definition = AssemblyDefinitionFlattener.flattenView(source);
    var available = [for (id in objectIds) id => true];
    for (occurrence in definition.occurrences) {
      var object = "project:" + occurrence.id;
      if (!available.exists(object)) continue;
      ensureGroup("");
      var path = occurrence.includePath == null ? "" : occurrence.includePath;
      ensureGroup(path);
      children.get(key(path)).push(object);
      parents.set(object, key(path));
    }
  }

  function ensureGroup(path:String):Void {
    var id = key(path);
    if (groups.exists(id)) return;
    var slash = path.lastIndexOf("/");
    var parentPath = slash < 0 ? "" : path.substr(0, slash);
    if (path != "") ensureGroup(parentPath);
    var label = path == "" ? definition.id : path.substr(slash + 1);
    groups.set(id, {label:AssemblyBodies.displayName(label), path:path});
    children.set(id, []);
    var parent = path == "" ? "scene" : key(parentPath);
    parents.set(id, parent);
    if (!children.exists(parent)) children.set(parent, []);
    children.get(parent).push(id);
  }
}
