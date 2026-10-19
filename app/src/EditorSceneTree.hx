package app;

import haxeon.ui.core.View;
import haxeon.ui.widgets.collections.TreeViewModel;
import haxeon.ui.widgets.collections.TreeView;
import haxeon.ui.widgets.collections.TreeRootMetadata;
import haxeon.ui.icons.IconName;
import app.editor.SceneVisibility;
import app.editor.SceneTreeRow;
import materia.assembly.AssemblyBodies;
import materia.assembly.AssemblyDefinition;

/**
 * The scene hierarchy preserves authored subassembly groups alongside rigid bodies. Fixed joints
 * keep parts in one body; moving joints nest bodies under their carrier. Explicit include paths
 * group sibling bodies and the parts inside each body without changing physical ownership.
 */
@:access(app.EditorScene)
class EditorSceneTree implements TreeViewModel {
  public var canChangeVisibility:Null<Void->Bool>;
  public var onVisibilityChanged:Null<Void->Void>;
  public final scrollController = new haxeon.ui.widgets.scroll.ScrollController();
  var selectedGroup:Null<String>;
  var groupSelectionRevision:Int = -1;
  public function selectionKey():String return selectedGroup != null && groupSelectionRevision == scene.selectionRevision
    ? selectedGroup : scene.treeSelectionKey();
  var syncedSelectionRevision:Int = -1;
  var syncedSelectionKey:String = "";
  var syncedTreeKey:String = "";
  /** Complete root-to-row path; includes every rigid-body group and a CAD feature's owning part. */
  public function selectionPath():Array<String> {
    return objectPath(selectionKey());
  }
  /** Authored tree path for a physical object, independent of expansion and search. */
  public function objectPath(key:String):Array<String> {
    if (key == "scene") return ["scene"];
    var path = [key], cursor = key;
    var marker = cursor.indexOf(":feature:");
    if (marker >= 0) {cursor=cursor.substr(0,marker);path.unshift(cursor);}
    while (parentOf.exists(cursor)) {cursor=parentOf.get(cursor);path.unshift(cursor);}
    path.unshift("scene");
    return path;
  }
  /** Called after TreeView hydrates its retained state. Never notify back or take focus from 3D. */
  public function synchronizeSelection(tree:TreeView):Void {
    var key = selectionKey();
    if (syncedSelectionRevision == scene.selectionRevision && syncedSelectionKey == key && syncedTreeKey == tree.key) return;
    var path = selectionPath();
    for (ancestor in path.slice(0,path.length-1)) tree.setExpanded(ancestor,true);
    tree.select(key,false);
    tree.scrollIntoView(key);
    syncedSelectionRevision=scene.selectionRevision;syncedSelectionKey=key;syncedTreeKey=tree.key;
  }
  public function selectKey(key:String):Void {
    var previous = selectionKey();
    if (isGroup(key)) {
      scene.select("scene");selectedGroup=key;groupSelectionRevision=scene.selectionRevision;
      if (previous != key) {
        // Publish the new tree-only selection as well as its 3D highlight.
        groupSelectionRevision=scene.selectionRevision+1;
        scene.refreshSelectionRevision();
      }
    } else {
      selectedGroup=null;
      if(!scene.selectTreeKey(key) && previous!=selectionKey())scene.refreshSelectionRevision();
    }
  }
  /** One level per completed click; an unrelated selection starts again at the outer group. */
  public function pickedKey(id:String, continueDrill:Bool):String {
    var path=objectPath(id);
    if(path.length<2)return "scene";
    var index=continueDrill ? path.indexOf(selectionKey()) : -1;
    return path[Std.int(Math.min(path.length-1,index<1 ? 1 : index+1))];
  }
  var selectedMembersKey="";
  var selectedMembers:Array<String> =[];
  public function selectedObjects():Array<String> {
    var key=selectionKey();
    // Authored membership is fixed for this tree's lifetime; moving or hiding a member does not change it.
    if(key!=selectedMembersKey) {selectedMembersKey=key;selectedMembers=key=="scene" ? [] : visibilityMembers(key);}
    return selectedMembers;
  }
  /** Resolve the whole subtree, independent of filtering and expansion. CAD features have no separate object visibility. */
  public function visibilityMembers(key:String):Array<String> {
    var marker=key.indexOf(":feature:");
    if(marker>=0)key=key.substr(0,marker);
    if (key=="scene") return [for(item in scene.items()) item.id];
    if(!visibilityGroup(key))return scene.object(key)==null ? [] : [key];
    var objectIds:Map<String,Bool> =[];for(item in scene.items())objectIds.set(item.id,true);
    var visited:Map<String,Bool> =[],result:Array<String> = [];
    function visit(id:String):Void {
      if(visited.exists(id))return;visited.set(id,true);
      if(objectIds.exists(id)) result.push(id);
      var descendants=children.get(id);
      if(descendants!=null)for(child in descendants)visit(child);
    }
    visit(key);return result;
  }
  public function visibilityGroup(key:String):Bool {
    var descendants=children.get(key);
    return key=="scene" || isGroup(key) || (descendants!=null && descendants.length>0);
  }
  public function toggleVisibility(key:String):Bool {
    if (canChangeVisibility != null && !canChangeVisibility()) return false;
    var changed=SceneVisibility.toggle(scene,key,visibilityMembers(key),visibilityGroup(key));
    if(changed && onVisibilityChanged!=null)onVisibilityChanged();
    return changed;
  }
  public function showAll():Bool {
    if (canChangeVisibility != null && !canChangeVisibility()) return false;
    var changed=SceneVisibility.showAll(scene);
    if(changed && onVisibilityChanged!=null)onVisibilityChanged();return changed;
  }
  public function isolateSelected():Bool {
    if (canChangeVisibility != null && !canChangeVisibility()) return false;
    var key = selectionKey(), ancestors = ["scene", key], cursor = key;
    var found = true;
    while (found) {
      found = false;
      for (parent in children.keys()) if (children.get(parent).indexOf(cursor) >= 0) {
        ancestors.push(parent);cursor=parent;found=true;break;
      }
    }
    var changed=SceneVisibility.isolate(scene,visibilityMembers(key),ancestors,!visibilityGroup(key));
    if(changed && onVisibilityChanged!=null)onVisibilityChanged();return changed;
  }
  static inline var PARTS:String = "parts:";
  static inline var PROJECT:String = "project:";
  final scene:EditorScene;
  final generatedLabels:Map<String, String>;
  /** Body roots and Parts nodes to what they contain. */
  final children:Map<String, Array<String>> = [];
  /** Part to the Parts node that holds it. */
  final partsOf:Map<String, String> = [];
  final parentOf:Map<String, String> = [];
  final partsTitles:Map<String, String> = [];
  final authoredMembers:Map<String,Bool> = [];
  /** Body root to the kind of moving joint that carries it. */
  final jointOf:Map<String, String> = [];
  /** Assembly objects that hang below another row instead of standing at the top. */
  final nested:Map<String, Bool> = [];
  final childCache:Map<String, Array<String>> = [];
  var cachedSceneRevision:Int = -1;
  var cachedFilterRevision:Int = -1;
  var cachedSelectionKey:String = "";
  var filter:String = "";
  var filterRevision:Int = 0;
  var revealed:String = "";
  var revealSerial:Int = 0;
  public function setFilter(value:String):Void {
    var next = StringTools.trim(value == null ? "" : value).toLowerCase();
    if (next != filter) { filter = next; filterRevision++; }
  }
  /** Parts rows are selectable groups rather than scene objects. */
  public function isGroup(key:String):Bool return StringTools.startsWith(key, PARTS);
  /** What the row for a scene object says: a readable name for generated parts, or the person's rename. */
  function displayLabel(key:String):String {
    var item = scene.object(key);
    if (item == null) return key;
    var generated = generatedLabels.get(key);
    if (generated != null && generated == item.label && StringTools.startsWith(key, PROJECT)) {
      var id=key.substr(PROJECT.length);
      if (authoredMembers.exists(key)) id=id.substr(id.lastIndexOf("/")+1);
      return AssemblyBodies.displayName(id);
    }
    return item.label;
  }
  function matches(key:String):Bool {
    if (filter == "" || key == selectionKey()) return true;
    var group = children.get(key);
    if (isGroup(key)) {
      var title=partsTitles.get(key);
      if (title!=null && title.toLowerCase().indexOf(filter)>=0) return true;
      if (group != null) for (child in group) if (matches(child)) return true;
      return false;
    }
    var item = scene.object(key);
    if (item != null) {
      if (item.label.toLowerCase().indexOf(filter) >= 0 || displayLabel(key).toLowerCase().indexOf(filter) >= 0 ||
          item.kind.toLowerCase().indexOf(filter) >= 0) return true;
      if (group != null) for (child in group) if (matches(child)) return true;
      for (index in 0...scene.cadFeatureCount(key))
        if (scene.cadFeatureNameAt(key, index).toLowerCase().indexOf(filter) >= 0) return true;
      return false;
    }
    return key == "scene";
  }
  function visibleChildren(parentKey:String):Array<String> {
    if (cachedSceneRevision != scene.revision || cachedFilterRevision != filterRevision || cachedSelectionKey != selectionKey()) {
      childCache.clear();
      cachedSceneRevision = scene.revision;
      cachedFilterRevision = filterRevision;
      cachedSelectionKey = selectionKey();
    }
    var cached = childCache.get(parentKey);
    if (cached != null) return cached;
    var result:Array<String> = [];
    if (parentKey == "scene") {
      var groups=children.get("scene");
      if (groups!=null) for (group in groups) if (matches(group)) result.push(group);
      for (item in scene.items()) if (!nested.exists(item.id) && matches(item.id)) result.push(item.id);
    } else {
      var members = children.get(parentKey);
      if (members != null) for (child in members) if (matches(child)) result.push(child);
      if (!isGroup(parentKey))
        for (index in 0...scene.cadFeatureCount(parentKey)) {
          var key = parentKey + ":feature:" + index;
          if (filter == "" || key == selectionKey() || scene.cadFeatureNameAt(parentKey, index).toLowerCase().indexOf(filter) >= 0)
            result.push(key);
        }
    }
    childCache.set(parentKey, result);
    return result;
  }
  /**
   * `definition` groups the assembly into bodies; `generatedLabels` are the labels the project gave its
   * objects, so a name the person typed is shown as typed.
   */
  public function new(scene:EditorScene, ?definition:AssemblyDefinition, ?generatedLabels:Map<String, String>) {
    this.scene = scene;
    scene.selectionObjects=selectedObjects;
    this.generatedLabels = generatedLabels == null ? new Map() : generatedLabels;
    if (definition == null) return;
    var bodies = AssemblyBodies.of(definition);
    for (body in bodies) {
      var rootKey = PROJECT + body.id;
      if (scene.object(rootKey) == null) continue;
      var members:Array<String> = [];
      for (occurrence in body.occurrences.slice(1)) {
        var partKey = PROJECT + occurrence;
        if (scene.object(partKey) != null) members.push(partKey);
      }
      var list:Array<String> = [];
      if (members.length > 0) {
        var partsKey = PARTS + body.id;
        children.set(partsKey, members);
        partsTitles.set(partsKey, partsTitle(members));
        for (member in members) {
          partsOf.set(member, partsKey);
          nested.set(member, true);
        }
        list.push(partsKey);
      }
      children.set(rootKey, list);
      if (body.jointType != null) jointOf.set(rootKey, Std.string(body.jointType));
    }
    // A body hangs below the body that its moving joint is attached to.
    for (body in bodies) {
      var rootKey = PROJECT + body.id;
      if (body.parent == null || !children.exists(rootKey)) continue;
      var carrier = children.get(PROJECT + body.parent);
      if (carrier == null) continue;
      carrier.push(rootKey);
      nested.set(rootKey, true);
    }
    // Preserve authored subassembly ownership inside each rigid body and among root bodies.
    var ownership:Map<String,String> = [];
    for (occurrence in definition.occurrences)
      if (occurrence.includePath!=null && occurrence.includePath!="")
        ownership.set(PROJECT+occurrence.id,occurrence.includePath);
    var containers=[for (key in children.keys()) if (isGroup(key)) key];
    for (container in containers) children.set(container,assemblyGroups(container,children.get(container),ownership));
    var roots=[for (item in scene.items()) if (!nested.exists(item.id)) item.id];
    var grouped=assemblyGroups("scene",roots,ownership);
    children.set("scene",[for (key in grouped) if (isGroup(key)) key]);
    for (parent in children.keys()) if (parent!="scene") for (child in children.get(parent)) parentOf.set(child,parent);
    var visibilityGroups:Map<String,Array<String>> = ["scene" => visibilityMembers("scene")];
    for(key in children.keys())if(visibilityGroup(key))visibilityGroups.set(key,visibilityMembers(key));
    SceneVisibility.rebind(scene,visibilityGroups);
  }
  /** Group only explicit include paths; slashes in arbitrary object names do not create assemblies. */
  function assemblyGroups(container:String,members:Array<String>,ownership:Map<String,String>):Array<String> {
    var result:Array<String> = [];
    for (member in members) {
      var path=ownership.get(member);
      if (path==null) {result.push(member);continue;}
      var parent=container, prefix="";
      for (segment in path.split("/")) {
        prefix+=(prefix=="" ? "" : "/")+segment;
        var key=PARTS+container+":"+prefix;
        if (!children.exists(key)) {
          children.set(key,[]);
          partsTitles.set(key,AssemblyBodies.displayName(segment));
          if (parent==container) result.push(key); else children.get(parent).push(key);
        }
        parent=key;
      }
      children.get(parent).push(member);
      partsOf.set(member,parent);
      authoredMembers.set(member,true);
      nested.set(member,true);
    }
    return result;
  }
  /** "Tool (6)" when every part shares one nested path such as tool/..., otherwise "Parts (n)". */
  static function partsTitle(members:Array<String>):String {
    var shared:Null<String> = null;
    for (member in members) {
      var id = member.substr(PROJECT.length);
      var slash = id.indexOf("/");
      var first = slash < 0 ? "" : id.substr(0, slash);
      if (first == "" || (shared != null && shared != first)) return "Parts (" + members.length + ")";
      shared = first;
    }
    return AssemblyBodies.displayName(shared == null ? "parts" : shared) + " (" + members.length + ")";
  }
  /** The Parts node that holds the selected part, so it can open to show it. */
  function revealedGroup():String {
    var group = partsOf.get(scene.selectedId);
    return group == null ? "" : group;
  }
  public function rootCount():Int return 1;
  public function rootRange(start:Int, count:Int):Array<TreeRootMetadata>
    return start <= 0 && count > 0 ? [new TreeRootMetadata("scene", true)] : [];
  public function rootKeyAt(index:Int):String return "scene";
  public function childCount(parentKey:String):Int {
    return visibleChildren(parentKey).length;
  }
  public function childKeyAt(parentKey:String, index:Int):String {
    var result = visibleChildren(parentKey);
    if (index < 0 || index >= result.length) throw "Scene tree child index is out of range";
    return result[index];
  }
  public function initiallyExpanded(key:String):Bool {
    if (filter != "") return true;
    if (key == "scene") return true;
    if (isGroup(key)) return revealedGroup() == key;
    var item = scene.object(key);
    return item != null && (scene.isCadPart(item.id) || children.exists(item.id));
  }
  public function estimatedExtent():Float return 28.0;
  public function extentIsUniform():Bool return true;
  public function extentAt(key:String):Float return 28.0;
  function iconLabeledItem(key:String,label:String,icon:IconName,?detail:String):View {
    var members=visibilityMembers(key), shown=0;
    for(id in members)if(scene.isVisible(id))shown++;
    var item=scene.object(key);
    var own=visibilityGroup(key) ? !SceneVisibility.groupHidden(scene,key,members) : item==null || item.visible;
    return new SceneTreeRow(key,label,icon,detail,shown>0 || members.length==0,own,
      shown>0 && shown<members.length,members.length==0 ? null : () -> {toggleVisibility(key);});
  }
  public function buildItem(key:String):View {
    if (isGroup(key)) {
      var title = partsTitles.get(key);
      return iconLabeledItem(key, title == null ? "Parts" : title, IconName.Grid);
    }
    var item=scene.object(key);
    if(item!=null) {
      var joint = jointOf.get(key);
      var shown = displayLabel(key);
      // The generated designation stays one hover away when the row shows a readable name.
      return iconLabeledItem(key, shown + (joint == null ? "" : " · " + joint) +
        (scene.isVisible(key) ? "" : " (hidden)"), IconName.Cube, shown == item.label ? null : item.label);
    }
    var marker=key.indexOf(":feature:");
    if(marker>=0){
      var id=key.substr(0,marker),index=Std.parseInt(key.substr(marker+9));
      return iconLabeledItem(key, index==null||index<0||index>=scene.cadFeatureCount(id)?"Feature":
        "Feature "+(index+1)+" · "+scene.cadFeatureNameAt(id,index), IconName.Grid);
    }
    return iconLabeledItem(key, "Scene", IconName.Hierarchy);
  }
  public function revision():Int {
    var selected = selectionKey();
    if (selected != revealed) {
      revealed = selected;
      revealSerial++;
    }
    return scene.revision + filterRevision * 1000000 + revealSerial;
  }
}
