package app.editor;

import app.EditorScene;
import app.EditorScene.SceneBridge;
import app.CadGeometryData;
import haxe.Json;
import nativekit.scene.GeometryData;
import nativekit.scene.Geometry;
import nativekit.scene.Material;
import nativekit.scene.Texture;
import nativekit.scene.TextureData;
import nativekit.scene.Image as SceneImage;
import nativekit.scene.Sampler;
import nativekit.scene.SamplerData;
import nativekit.scene.NodeId;
import nativekit.scene.SceneView;
import nativekit.scene.Transform;
import materia.project.ThreadSpecData;

private typedef ThreadTextures = {var color:Texture; var normal:Texture; var colorImage:SceneImage; var normalImage:SceneImage; var references:Int;}
private typedef ThreadNode = {var node:NodeId; var geometry:Geometry; var material:Material; var textureKey:String;}
private typedef ThreadGroup = {var source:GeometryData; var parent:String; var appearance:String; var nodes:Array<ThreadNode>;}

/** Runtime children derived from the base mesh. They share its pose, appearance and highlight state. */
@:access(app.EditorScene)
class CosmeticThreadPresentation {
  final groups:Map<String, ThreadGroup> = [];
  final textures:Map<String, ThreadTextures> = [];
  var sampler:Null<Sampler>;

  public function new() {}

  public function dispose(owner:EditorScene):Void {
    var ids = [for (id in groups.keys()) id];
    for (id in ids) remove(owner, id);
    if (sampler != null) { sampler.dispose(); sampler = null; }
  }

  /** Native owners are rebuilt only when the base geometry or appearance changes. */
  public function ensure(owner:EditorScene):Void {
    var seen:Map<String, Bool> = [];
    var changed = false;
    for (item in owner.items()) {
      var source:Null<GeometryData> = null;
      if (EditorScene.isCadKind(item.kind)) {
        var session = owner.cadSessions.get(item.id);
        if (session != null) source = session.geometry();
      } else if (item.kind == "cad-preview" && item.meshSnapshot != null) source = owner.previewGeometry(item.meshSnapshot);
      if (!Std.isOfType(source, CadGeometryData)) continue;
      var cad:CadGeometryData = cast source;
      if (cad.threads.length == 0) continue;
      seen.set(item.id, true);
      var parent = owner.runtimeFor(item.id).node;
      var parentKey = SceneBridge.nodeKey(parent);
      var appearance = Json.stringify({red: item.red, green: item.green, blue: item.blue, appearance: item.appearance});
      var old = groups.get(item.id);
      if (old != null && old.source == source && old.parent == parentKey && old.appearance == appearance) continue;
      if (old != null) remove(owner, item.id);
      var nodes:Array<ThreadNode> = [];
      try {
        for (thread in cad.threads) {
          var data = CosmeticThreadMesh.build(cad, thread);
          if (data.triangleCount() == 0) continue;
          var key = '${thread.source.spec.profile}|${thread.source.spec.majorDiameter}|${thread.source.spec.pitch}|${thread.source.spec.starts}';
          var pair = acquire(owner, key, thread.source.spec);
          var geometry = owner.scene.createGeometry();
          owner.scene.setGeometryData(geometry, data);
          var material = owner.scene.createMaterial();
          owner.scene.setMaterialData(material, ScenePresentation.materialFor(item.red, item.green, item.blue, item.appearance, item.kind)
            .setBaseColorTexture(pair.color, sampler).setNormalTexture(pair.normal));
          var transaction = owner.scene.beginTransaction();
          var node = transaction.createNode();
          transaction.setName(node, "Thread " + thread.source.faceName);
          transaction.setParent(node, parent); transaction.setVisibility(node, false);
          transaction.setTransform(node, Transform.identity());
          transaction.setGeometry(node, geometry); transaction.setMaterial(node, material);
          transaction.commit();
          owner.bridge.mapNode(node, item.id);
          nodes.push({node: node, geometry: geometry, material: material, textureKey: key});
        }
        groups.set(item.id, {source: cad, parent: parentKey, appearance: appearance, nodes: nodes});
        changed = true;
      } catch (error:Dynamic) {
        groups.set(item.id, {source: cad, parent: parentKey, appearance: appearance, nodes: nodes});
        remove(owner, item.id);
        throw 'Cosmetic threads on ${item.id}: $error';
      }
    }
    var obsolete = [for (id in groups.keys()) if (!seen.exists(id)) id];
    for (id in obsolete) { remove(owner, id); changed = true; }
    if (changed) owner.publish(null, false);
  }

  public function nodesFor(id:String):Array<NodeId> {
    var group = groups.get(id);
    return group == null ? [] : [for (entry in group.nodes) entry.node];
  }

  /** Called after an atomic edit has retired this object's nodes. */
  public function release(owner:EditorScene, id:String):Void remove(owner, id);

  public function show(owner:EditorScene, view:SceneView, visible:Bool):Void {
    for (id in groups.keys()) {
      var item = owner.object(id);
      var group:ThreadGroup = cast groups.get(id);
      for (entry in group.nodes) {
        view.setVisibility(entry.node, visible && item != null && SceneVisibility.shown(item.visible, item.hiddenBy));
      }
    }
  }

  function acquire(owner:EditorScene, key:String, spec:ThreadSpecData):ThreadTextures {
    var pair = textures.get(key);
    if (pair == null) {
      if (sampler == null) {
        sampler = owner.scene.createSampler();
        owner.scene.setSamplerData(sampler, new SamplerData().setWrap(1, 1).setFilters(2, 2));
      }
      var colorImage = owner.scene.createImage(), normalImage = owner.scene.createImage();
      owner.scene.setImageData(colorImage, CosmeticThreadTexture.image(spec, false));
      owner.scene.setImageData(normalImage, CosmeticThreadTexture.image(spec, true));
      var color = owner.scene.createTexture(), normal = owner.scene.createTexture();
      owner.scene.setTextureData(color, new TextureData(colorImage));
      owner.scene.setTextureData(normal, new TextureData(normalImage));
      pair = {color: color, normal: normal, colorImage: colorImage, normalImage: normalImage, references: 0};
      textures.set(key, pair);
    }
    pair.references++;
    return pair;
  }

  function remove(owner:EditorScene, id:String):Void {
    var group = groups.get(id);
    if (group == null) return;
    var snapshot = owner.scene.snapshot();
    var transaction = owner.scene.beginTransaction();
    for (entry in group.nodes) {
      if (snapshot.findNode(entry.node) != null) transaction.destroyNode(entry.node);
      owner.bridge.unmapNode(entry.node);
    }
    snapshot.dispose(); transaction.commit();
    for (entry in group.nodes) {
      entry.geometry.dispose(); entry.material.dispose();
      var pair:ThreadTextures = cast textures.get(entry.textureKey);
      pair.references--;
      if (pair.references == 0) {
        pair.color.dispose(); pair.normal.dispose(); pair.colorImage.dispose(); pair.normalImage.dispose();
        textures.remove(entry.textureKey);
      }
    }
    groups.remove(id);
  }
}
