package app.editor;

import app.EditorScene;
import haxeon.ui.editing.EditOperation;

typedef VisibilityState = {final id:String; final visible:Bool; final hiddenBy:Array<String>;}

/** Authored object visibility and inherited tree masks are independent. Changes publish atomically. */
@:access(app.EditorScene)
class SceneVisibility {
  public static function masks(value:Null<Array<String>>):Array<String> return value == null ? [] : value.copy();
  public static function shown(visible:Bool, hiddenBy:Null<Array<String>>):Bool return visible && (hiddenBy == null || hiddenBy.length == 0);
  public static function same(a:Null<Array<String>>, b:Null<Array<String>>):Bool {
    var first = masks(a), second = masks(b);
    if (first.length != second.length) return false;
    for (i in 0...first.length) if (second.indexOf(first[i]) < 0) return false;
    return true;
  }
  public static function snapshot(scene:EditorScene):Array<VisibilityState>
    return [for (item in scene.items()) {id:item.id, visible:item.visible, hiddenBy:masks(item.hiddenBy)}];

  public static function groupHidden(scene:EditorScene, key:String, members:Array<String>):Bool {
    for (id in members) { var item = scene.object(id); if (item != null && item.hiddenBy.indexOf(key) >= 0) return true; }
    return false;
  }
  /** Regenerated hierarchy membership is derived. Rebind saved group masks to its current subtree. */
  public static function rebind(scene:EditorScene, groups:Map<String,Array<String>>, ?legacyGroups:Map<String,Array<String>>):Void {
    var active:Map<String,Bool> = [];
    for(item in scene.items())for(key in item.hiddenBy)if(groups.exists(key))active.set(key,true);
    // Old body and Parts keys do not identify authored assemblies. Preserve their
    // effective hiding as local part flags instead of guessing a new owning group.
    var legacyHidden:Map<String,Bool> = [];
    if(legacyGroups!=null)for(item in scene.items())for(key in item.hiddenBy) {
      if(groups.exists(key) || key=="isolate:scene")continue;
      var members=legacyGroups.get(key);
      if(members==null)legacyHidden.set(item.id,true);
      else for(id in members)legacyHidden.set(id,true);
    }
    var states:Array<VisibilityState> = [], changed=false;
    for(item in scene.items()) {
      var masks:Array<String> = [];
      if(item.hiddenBy.indexOf("isolate:scene")>=0)masks.push("isolate:scene");
      for(key in active.keys())if(groups.get(key).indexOf(item.id)>=0)masks.push(key);
      var visible=item.visible&&!legacyHidden.exists(item.id);
      if(visible!=item.visible || !same(item.hiddenBy,masks))changed=true;
      states.push({id:item.id,visible:visible,hiddenBy:masks});
    }
    if(changed)apply(scene,states);
  }
  public static function toggle(scene:EditorScene, key:String, members:Array<String>, group:Bool):Bool {
    if (members.length == 0) return false;
    var after = snapshot(scene);
    var hide = !groupHidden(scene, key, members);
    for (state in after) if (members.indexOf(state.id) >= 0) {
      if (group) {
        state.hiddenBy.remove(key);
        if (hide) state.hiddenBy.push(key);
      }
    }
    if (!group) after = [for (state in after) {id:state.id,
      visible:members.indexOf(state.id) >= 0 ? !state.visible : state.visible, hiddenBy:state.hiddenBy}];
    return edit(scene, group && hide ? "Hide group" : group ? "Show group" : "Toggle object visibility", after);
  }
  public static function showAll(scene:EditorScene):Bool
    return edit(scene, "Show all", [for (item in scene.items()) {id:item.id, visible:true, hiddenBy:[]}]);

  public static function isolate(scene:EditorScene, members:Array<String>, ?revealGroups:Array<String>, forceSelected:Bool = true):Bool {
    if (members.length == 0) return false;
    var after:Array<VisibilityState> = [];
    for (item in scene.items()) {
      var selected = members.indexOf(item.id) >= 0, groups = masks(item.hiddenBy);
      groups.remove("isolate:scene");
      if (selected && revealGroups != null) for (key in revealGroups) groups.remove(key);
      if (!selected) groups.push("isolate:scene");
      after.push({id:item.id, visible:selected && forceSelected ? true : item.visible, hiddenBy:groups});
    }
    return edit(scene, "Isolate selected", after);
  }
  static function edit(scene:EditorScene, label:String, after:Array<VisibilityState>):Bool {
    var before = snapshot(scene), changed = false;
    for (i in 0...before.length) if (before[i].visible != after[i].visible || !same(before[i].hiddenBy, after[i].hiddenBy)) changed = true;
    if (!changed) return false;
    var bytes = 128;
    for (states in [before, after]) for (state in states) {
      bytes += 64 + state.id.length * 2;
      for (key in state.hiddenBy) bytes += 16 + key.length * 2;
    }
    return scene.document.apply(new EditOperation(label, () -> apply(scene, after), () -> apply(scene, before), null, null, null, bytes));
  }
  static function apply(scene:EditorScene, states:Array<VisibilityState>):Void {
    var transaction = scene.scene.beginTransaction();
    var changes:nativekit.scene.ChangeSet;
    try {
      for (state in states) if (scene.object(state.id) != null)
        transaction.setVisibility(scene.runtimeFor(state.id).node, shown(state.visible, state.hiddenBy));
      changes = transaction.commitWithChanges();
    } catch (error:Dynamic) { transaction.dispose(); throw error; }
    for (state in states) {
      var item = scene.object(state.id);
      if (item != null) { item.visible = state.visible; item.hiddenBy = masks(state.hiddenBy); }
    }
    scene.publish(null, false, changes);
  }
}
