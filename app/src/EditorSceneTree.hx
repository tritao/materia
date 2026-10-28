package app;

import haxeon.ui.core.View;
import haxeon.ui.widgets.collections.TreeViewModel;
import haxeon.ui.widgets.collections.TreeView;
import haxeon.ui.widgets.collections.TreeRootMetadata;
import haxeon.ui.icons.IconName;
import app.editor.SceneVisibility;
import app.editor.SceneTreeRow;
import app.editor.AssemblyHierarchy;
import app.editor.LegacyAssemblyVisibility;
import materia.assembly.AssemblyDefinition;

/** Authored assembly ownership and CAD features. Physical motion does not reparent part occurrences. */
@:access(app.EditorScene) class EditorSceneTree implements TreeViewModel {
  public var canChangeVisibility:Null<Void -> Bool>;
  public var onVisibilityChanged:Null<Void -> Void>;
  public final scrollController = new haxeon.ui.widgets.scroll.ScrollController();
  var selectedGroup:Null<String>;
  var groupSelectionRevision:Int = -1;
  public function selectionKey():String return selectedGroup != null
    && groupSelectionRevision == scene.selectionRevision ? selectedGroup : scene.treeSelectionKey();
  var syncedSelectionRevision:Int = -1;
  var syncedSelectionKey:String = "";
  var syncedTreeKey:String = "";
  /** Complete root-to-row path; includes every authored assembly and a CAD feature's owning part. */
  public function selectionPath():Array<String> {
    return objectPath(selectionKey());
  }
  /** Authored tree path for a physical object, independent of expansion and search. */
  function sourceFeature(key:String):Null<materia.project.SceneSourceMap.SceneSourceFeature> {
    var map = scene.sourceMap;
    if (!StringTools.startsWith(key, "source:") || map == null) return null;
    for (feature in map.features) if (key == "source:" + feature.id) return feature;
    return null;
  }
  public function sourceFeatures():Array<materia.project.SceneSourceMap.SceneSourceFeature> {
    var map = scene.sourceMap;
    if (map == null) return [];
    var feature = sourceFeature(selectionKey());
    if (feature != null) return [feature];
    var id = StringTools.startsWith(scene.selectedId,
      PROJECT) ? scene.selectedId.substr(PROJECT.length) : scene.selectedId;
    if (scene.selectedCadFaceIndex >= 0) {
      var matches = map.features.filter(feature -> feature.occurrence == id
        && feature.faces.indexOf(scene.selectedCadFaceIndex) >= 0);
      if (matches.length > 1) {
        var rootIds = materia.project.SceneSourceMap.SceneSourceMaps.sources(map, id);
        matches = matches.filter(feature -> rootIds.indexOf(feature.id) < 0);
      }
      if (matches.length > 0) return matches;
    }
    var ids = materia.project.SceneSourceMap.SceneSourceMaps.sources(map, id);
    return map.features.filter(feature -> ids.indexOf(feature.id) >= 0);
  }
  public function objectPath(key:String):Array<String> {
    var source = sourceFeature(key);
    if (source != null) return objectPath(PROJECT + source.occurrence).concat([key]);
    if (key == "scene") return ["scene"];
    var path = [key], cursor = key;
    var marker = cursor.indexOf(":feature:");
    if (marker >= 0) {
      cursor = cursor.substr(0, marker);
      path.unshift(cursor);
    } while (parentOf.exists(cursor)) {
      cursor = parentOf.get(cursor);
      path.unshift(cursor);
    }
    path.unshift("scene");
    return path;
  }
  /** Called after TreeView hydrates its retained state. Never notify back or take focus from 3D. */
  public function synchronizeSelection(tree:TreeView):Void {
    var key = selectionKey();
    if (syncedSelectionRevision == scene.selectionRevision
      && syncedSelectionKey == key && syncedTreeKey == tree.key) return;
    var path = selectionPath();
    for (ancestor in path.slice(0, path.length - 1)) tree.setExpanded(ancestor, true);
    tree.select(key, false);
    tree.scrollIntoView(key);
    syncedSelectionRevision = scene.selectionRevision;
    syncedSelectionKey = key;
    syncedTreeKey = tree.key;
  }
  public function selectKey(key:String):Void {
    var previous = selectionKey();
    var source = sourceFeature(key);
    if (source != null) {
      scene.select(PROJECT + source.occurrence);
      selectedGroup = key;
      scene.refreshSelectionRevision();
      groupSelectionRevision = scene.selectionRevision;
    } else if (isGroup(key)) {
      scene.select("scene");
      selectedGroup = key;
      groupSelectionRevision = scene.selectionRevision;
      if (previous != key) {
        // Publish the new tree-only selection as well as its 3D highlight.
        groupSelectionRevision = scene.selectionRevision + 1;
        scene.refreshSelectionRevision();
      }
    } else {
      selectedGroup = null;
      if (!scene.selectTreeKey(key) && previous != selectionKey()) scene.refreshSelectionRevision();
    }
  }
  var selectedMembersKey = "";
  var selectedMembers:Array<String> = [];
  public function selectedObjects():Array<String> {
    var key = selectionKey();
    var map = scene.sourceMap;
    if (map != null && (sourceFeature(key) != null || scene.selectedCadFaceIndex >= 0)) {
      var result:Array<String> = [];
      for (feature in sourceFeatures()) for (id in materia.project.SceneSourceMap.SceneSourceMaps.tooling(
        map,
        feature.id
      )) if (id != feature.occurrence && result.indexOf(PROJECT + id) < 0) result.push(PROJECT + id);
      return result;
    }
    // Authored membership is fixed for this tree's lifetime; moving or hiding a member does not change it.
    if (key != selectedMembersKey) {
      selectedMembersKey = key;
      selectedMembers = key == "scene" ? [] : visibilityMembers(key);
    }
    return selectedMembers;
  }
  /** Resolve the whole subtree, independent of filtering and expansion. CAD features have no separate object visibility. */
  public function visibilityMembers(key:String):Array<String> {
    var marker = key.indexOf(":feature:");
    if (marker >= 0) key = key.substr(0, marker);
    if (key == "scene") return [for (item in scene.items()) item.id];
    if (!visibilityGroup(key)) return scene.object(key) == null ? [] : [key];
    var objectIds:Map<String, Bool> = [];
    for (item in scene.items()) objectIds.set(item.id, true);
    var visited:Map<String, Bool> = [], result:Array<String> = [];
    function visit(id:String):Void {
      if (visited.exists(id)) return;
      visited.set(id, true);
      if (objectIds.exists(id)) result.push(id);
      var descendants = children.get(id);
      if (descendants != null) for (child in descendants) visit(child);
    }
    visit(key);
    return result;
  }
  public function visibilityGroup(key:String):Bool {
    var descendants = children.get(key);
    return key == "scene" || isGroup(key) || (descendants != null && descendants.length > 0);
  }
  /** Unfiltered hierarchy for secondary visibility controls; excludes CAD feature rows. */
  public function visibilityChildren(key:String):Array<String> {
    if (key == "scene") {
      return rootChildren();
    }
    var result = children.get(key);
    return result == null ? [] : result.copy();
  }
  public function visibilityLabel(key:String):String {
    if (key == "scene") return "All components";
    if (isGroup(key)) {
      var group = groups.get(key);
      return group == null ? key : group.label;
    }
    return displayLabel(key);
  }
  public function visibilityState(key:String): {shown: Bool, mixed: Bool, own: Bool} {
    var members = visibilityMembers(key), count = 0;
    for (id in members) if (scene.isVisible(id)) count++;
    var item = scene.object(key);
    return {
      shown: count > 0,
      mixed: count > 0 && count < members.length,
      own: visibilityGroup(key) ? !SceneVisibility.groupHidden(scene, key, members) : item != null && item.visible
    };
  }
  public function toggleVisibility(key:String):Bool {
    if (canChangeVisibility != null && !canChangeVisibility()) return false;
    var changed = SceneVisibility.toggle(scene, key, visibilityMembers(key), visibilityGroup(key));
    if (changed && onVisibilityChanged != null) onVisibilityChanged();
    return changed;
  }
  public function showAll():Bool {
    if (canChangeVisibility != null && !canChangeVisibility()) return false;
    var changed = SceneVisibility.showAll(scene);
    if (changed && onVisibilityChanged != null) onVisibilityChanged();
    return changed;
  }
  public function isolateSelected():Bool {
    if (canChangeVisibility != null && !canChangeVisibility()) return false;
    var key = selectionKey(), ancestors = ["scene", key], cursor = key;
    var found = true;
    while (found) {
      found = false;
      for (parent in children.keys()) if (children.get(parent).indexOf(cursor) >= 0) {
        ancestors.push(parent);
        cursor = parent;
        found = true;
        break;
      }
    }
    var changed = SceneVisibility.isolate(scene, visibilityMembers(key), ancestors, !visibilityGroup(key));
    if (changed && onVisibilityChanged != null) onVisibilityChanged();
    return changed;
  }
  static inline var PROJECT:String = "project:";
  final scene:EditorScene;
  /** Assemblies contain other assemblies and part occurrences. */
  final children:Map<String, Array<String>> = [];
  /** Occurrence to its authored owning assembly. */
  final partsOf:Map<String, String> = [];
  final parentOf:Map<String, String> = [];
  final groups:Map<String, app.editor.AssemblyHierarchy.AssemblyTreeGroup> = [];
  var hierarchy:Null<AssemblyHierarchy>;
  /** Body root to the kind of moving joint that carries it. */
  final jointOf:Map<String, String> = [];
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
    if (next != filter) {
      filter = next;
      filterRevision++;
    }
  }
  public function isGroup(key:String):Bool return groups.exists(key);
  public function assemblyKey(path:String = ""):String return hierarchy == null ? "scene" : hierarchy.key(path);
  public function groupInfo(key:String):Null<{label:String, path:String, count:Int, shown:Int}> {
    var group = groups.get(key);
    if (group == null) return null;
    var members = visibilityMembers(key), shown = 0;
    for (id in members) if (scene.isVisible(id)) shown++;
    return {label: group.label, path: group.path, count: members.length, shown: shown};
  }
  /** Object names are resolved when project overrides are materialized, never in the tree. */
  function displayLabel(key:String):String {
    var source = sourceFeature(key);
    if (source != null) return source.label;
    var item = scene.object(key);
    return item == null ? key : item.label;
  }
  function matches(key:String):Bool {
    if (filter == "" || key == selectionKey()) return true;
    var source = sourceFeature(key);
    if (source != null) return source.label.toLowerCase().indexOf(filter) >= 0;
    var group = children.get(key);
    if (isGroup(key)) {
      var info = groups.get(key);
      if (info != null && info.label.toLowerCase().indexOf(filter) >= 0) return true;
      if (group != null) for (child in group) if (matches(child)) return true;
      return false;
    }
    var item = scene.object(key);
    if (item != null) {
      if (displayLabel(key).toLowerCase().indexOf(filter) >= 0
        || item.kind.toLowerCase().indexOf(filter) >= 0) return true;
      if (group != null) for (child in group) if (matches(child)) return true;
      var map = scene.sourceMap;
      if (map != null) for (source in map.features) if (PROJECT + source.occurrence == key
        && source.label.toLowerCase().indexOf(filter) >= 0) return true;
      for (index in 0...scene.cadFeatureCount(key)) if (scene.cadFeatureNameAt(key,
        index).toLowerCase().indexOf(filter) >= 0) return true;
      return false;
    }
    return key == "scene";
  }
  function visibleChildren(parentKey:String):Array<String> {
    if (cachedSceneRevision != scene.revision
      || cachedFilterRevision != filterRevision || cachedSelectionKey != selectionKey()) {
      childCache.clear();
      cachedSceneRevision = scene.revision;
      cachedFilterRevision = filterRevision;
      cachedSelectionKey = selectionKey();
    }
    var cached = childCache.get(parentKey);
    if (cached != null) return cached;
    var result:Array<String> = [];
    if (parentKey == "scene") {
      for (key in rootChildren()) if (matches(key)) result.push(key);
    } else {
      var members = children.get(parentKey);
      if (members != null) for (child in members) if (matches(child)) result.push(child);
      var sourceMap = scene.sourceMap;
      if (sourceMap != null) for (feature in sourceMap.features) if (PROJECT + feature.occurrence == parentKey
        && matches("source:" + feature.id)) result.push("source:" + feature.id);
      if (!isGroup(parentKey)) for (index in 0...scene.cadFeatureCount(parentKey)) {
        var key = parentKey + ":feature:" + index;
        if (filter == "" || key == selectionKey() || scene.cadFeatureNameAt(parentKey,
          index).toLowerCase().indexOf(filter) >= 0) result.push(key);
      }
    }
    childCache.set(parentKey, result);
    return result;
  }
  /** Hierarchy membership is independent of object names. */
  public function new(scene:EditorScene, ?definition:AssemblyDefinition) {
    this.scene = scene;
    scene.selectionObjects = selectedObjects;
    scene.sourceSelection = sourceFeatures;
    if (definition == null) return;
    var authored = new AssemblyHierarchy(definition, [for (item in scene.items()) item.id]);
    hierarchy = authored;
    for (key => group in authored.groups) groups.set(key, group);
    for (key => members in authored.children) children.set(key, members);
    for (key => parent in authored.parents) if (parent != "scene") parentOf.set(key, parent);
    for (occurrence in authored.definition.occurrences) {
      var id = PROJECT + occurrence.id;
      if (parentOf.exists(id)) {
        partsOf.set(id, parentOf.get(id));
      }
    }
    for (joint in authored.definition.joints) if (joint.role == "tree" && joint.type != "fixed") jointOf.set(
      PROJECT + joint.child,
      Std.string(joint.type)
    );
    var visibilityGroups:Map<String, Array<String>> = ["scene" => visibilityMembers("scene")];
    for (key in groups.keys()) visibilityGroups.set(key, visibilityMembers(key));
    var legacy = false;
    for (item in scene.items()) for (key in item.hiddenBy) if (StringTools.startsWith(key,
      "parts:") || StringTools.startsWith(
        key,
        PROJECT
      )) legacy = true;
    SceneVisibility.rebind(
      scene,
      visibilityGroups,
      legacy ? LegacyAssemblyVisibility.groups(authored.definition) : null
    );
  }

  /** Preserve scene order, inserting each assembly where its first occurrence appears. */
  function rootChildren():Array<String> {
    var result:Array<String> = [], seen:Map<String, Bool> = [];
    for (item in scene.items()) {
      var key = item.id;
      while (parentOf.exists(key)) key = parentOf.get(key);
      if (!seen.exists(key)) {
        seen.set(key, true);
        result.push(key);
      }
    }
    return result;
  }

  function revealedGroup():String {
    var group = partsOf.get(scene.selectedId);
    return group == null ? "" : group;
  }
  public function rootCount():Int return 1;
  public function rootRange(start:Int,
    count:Int):Array<TreeRootMetadata> return start <= 0 && count > 0 ? [new TreeRootMetadata(
      "scene",
      true
    )] : [];
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
  function iconLabeledItem(key:String, label:String, icon:IconName, ?detail:String):View {
    var members = visibilityMembers(key), shown = 0;
    for (id in members) if (scene.isVisible(id)) shown++;
    var item = scene.object(key);
    var own = visibilityGroup(key) ? !SceneVisibility.groupHidden(scene, key, members) : item == null || item.visible;
    return new SceneTreeRow(key, label, icon, detail, shown > 0 || members.length == 0,
      own, shown > 0 && shown < members.length, members.length == 0 ? null : () -> {toggleVisibility(key);
    });
  }
  public function buildItem(key:String):View {
    var source = sourceFeature(key);
    if (source != null) {
      var map = scene.sourceMap;
      var linked = map == null ? [] : materia.project.SceneSourceMap.SceneSourceMaps.tooling(map, source.id);
      return iconLabeledItem(
        key,
        source.label,
        IconName.Grid,
        "Source feature · " + linked.length + " tooling occurrences"
      );
    }
    var info = groupInfo(key);
    if (info != null) {
      return iconLabeledItem(
        key,
        info.label + " (" + info.count + ")",
        IconName.Hierarchy,
        "Assembly" + (info.path == "" ? "" : " · " + info.path) + " · " + info.count + " part occurrences"
      );
    }
    var item = scene.object(key);
    if (item != null) {
      var joint = jointOf.get(key);
      var shown = displayLabel(key);
      return iconLabeledItem(
        key,
        shown + (joint == null ? "" : " · " + joint) + (scene.isVisible(key) ? "" : " (hidden)"),
        IconName.Cube,
        item.id
      );
    }
    var marker = key.indexOf(":feature:");
    if (marker >= 0) {
      var id = key.substr(0, marker), index = Std.parseInt(key.substr(marker + 9));
      return iconLabeledItem(
        key,
        index == null || index < 0
        || index >= scene.cadFeatureCount(id) ? "Feature" : "Feature " + (index + 1) + " · " + scene.cadFeatureNameAt(
          id,
          index
        ),
        IconName.Grid
      );
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
