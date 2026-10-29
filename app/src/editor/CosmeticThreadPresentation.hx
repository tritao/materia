package app.editor;

import app.EditorScene;
import app.EditorScene.SceneBridge;
import app.CadGeometryData;
import cadkit.threads.DetailTexture;
import haxe.Json;
import materia.project.SurfaceDetailRenderData.SurfaceDetailKinds;
import nativekit.scene.MaterialData;
import nativekit.scene.GeometryData;
import nativekit.scene.Geometry;
import nativekit.scene.Material;
import nativekit.scene.Texture;
import nativekit.scene.TextureData;
import nativekit.scene.ImageData;
import nativekit.scene.Image as SceneImage;
import nativekit.scene.Sampler;
import nativekit.scene.SamplerData;
import nativekit.scene.NodeId;
import nativekit.scene.SceneView;
import nativekit.scene.Transform;

private typedef ThreadTextures = {
  var color:Texture;
  var normal:Texture;
  var colorImage:SceneImage;
  var normalImage:SceneImage;
  var references:Int;
}
/** A display's picture: its source, and the image the source rewrites. */
private typedef LiveDisplay = {
  var source:LiveDisplaySource;
  var image:SceneImage;
  var texture:Texture;
}
private typedef ThreadNode = {
  var node:NodeId;
  var geometry:Geometry;
  var material:Material;
  /** The shared texture pair, or null for a display (which owns its picture). */
  var textureKey:Null<String>;
  var faceIndex:Int;
  var live:Null<LiveDisplay>;
}
private typedef ThreadGroup = {
  var source:GeometryData;
  var parent:String;
  var appearance:String;
  var nodes:Array<ThreadNode>;
}

/**
 * Runtime children derived from the base mesh: cosmetic threads, and the knurling and scales of surface details. They share its
 * pose, appearance and highlight state, and are shown or hidden together.
 */
@:access(app.EditorScene) class CosmeticThreadPresentation {
  final groups:Map<String, ThreadGroup> = [];
  final textures:Map<String, ThreadTextures> = [];
  var sampler:Null<Sampler>;

  public function new() {
  }

  public function dispose(owner:EditorScene):Void {
    var ids = [for (id in groups.keys()) id];
    for (id in ids) remove(owner, id);
    if (sampler != null) {
      sampler.dispose();
      sampler = null;
    }
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
      } else if (item.kind == "cad-preview"
        && item.meshSnapshot != null) source = owner.previewGeometry(item.meshSnapshot);
      if (!Std.isOfType(source, CadGeometryData)) continue;
      var cad:CadGeometryData = cast source;
      if (cad.threads.length == 0 && cad.details.length == 0) continue;
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
          var key =
            '${thread.source.spec.profile}|${thread.source.spec.majorDiameter}|${thread.source.spec.pitch}|${thread.source.spec.starts}';
          var spec = thread.source.spec;
          var pair = acquire(
            owner,
            key,
            () -> CosmeticThreadTexture.image(spec, false),
            () -> CosmeticThreadTexture.image(spec, true)
          );
          var geometry = owner.scene.createGeometry();
          owner.scene.setGeometryData(geometry, data);
          var material = owner.scene.createMaterial();
          owner.scene.setMaterialData(
            material,
            ScenePresentation.materialFor(item.red,
            item.green, item.blue, item.appearance, item.kind).setBaseColorTexture(
              pair.color,
              sampler
            ).setNormalTexture(pair.normal)
          );
          var transaction = owner.scene.beginTransaction();
          var node = transaction.createNode();
          transaction.setName(node, "Thread " + thread.source.faceName);
          transaction.setParent(node, parent);
          transaction.setVisibility(node, false);
          transaction.setTransform(node, Transform.identity());
          transaction.setGeometry(node, geometry);
          transaction.setMaterial(node, material);
          transaction.commit();
          owner.bridge.mapNode(node, item.id);
          nodes.push({
            node: node,
            geometry: geometry,
            material: material,
            textureKey: key,
            faceIndex: thread.source.faceIndex,
            live: null
          });
        }
        // Knurling and scales: the same overlay, with texture coordinates made from the face's own axis.
        for (detail in cad.details) {
          if (detail.source.kind == SurfaceDetailKinds.DISPLAY) {
            var display = displayNode(owner, item.id, item.kind, parent, detail);
            if (display != null) nodes.push(display);
            continue;
          }
          var data = CosmeticDetailMesh.build(cad, detail);
          if (data.triangleCount() == 0) continue;
          var key = CosmeticDetailTexture.key(detail);
          var pair = acquire(
            owner,
            key,
            () -> CosmeticDetailTexture.image(detail, false),
            () -> CosmeticDetailTexture.image(detail, true)
          );
          var geometry = owner.scene.createGeometry();
          owner.scene.setGeometryData(geometry, data);
          var material = owner.scene.createMaterial();
          owner.scene.setMaterialData(
            material,
            ScenePresentation.materialFor(item.red,
            item.green, item.blue, item.appearance, item.kind).setBaseColorTexture(
              pair.color,
              sampler
            ).setNormalTexture(pair.normal)
          );
          var transaction = owner.scene.beginTransaction();
          var node = transaction.createNode();
          transaction.setName(node, "Detail " + detail.source.faceName);
          transaction.setParent(node, parent);
          transaction.setVisibility(node, false);
          transaction.setTransform(node, Transform.identity());
          transaction.setGeometry(node, geometry);
          transaction.setMaterial(node, material);
          transaction.commit();
          owner.bridge.mapNode(node, item.id);
          nodes.push({
            node: node,
            geometry: geometry,
            material: material,
            textureKey: key,
            faceIndex: detail.source.faceIndex,
            live: null
          });
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
    for (id in obsolete) {
      remove(owner, id);
      changed = true;
    }
    if (changed) owner.publish(null, false);
  }

  /**
   * A display's quad with a dark glass material that glows with its picture; null when no live source is known for it. The
   * picture is also its base-colour texture (the base colour is black, so it adds nothing), which is what a hover or selection
   * highlight keeps drawing.
   */
  function displayNode(owner:EditorScene,
    id:String, kind:String, parent:NodeId, detail:CadDetailGeometry):Null<ThreadNode> {
    var source = LiveDisplays.create(detail.source.source);
    if (source == null) return null;
    if (sampler == null) {
      sampler = owner.scene.createSampler();
      owner.scene.setSamplerData(sampler, new SamplerData().setWrap(1, 1).setFilters(2, 2));
    }
    var image = owner.scene.createImage();
    owner.scene.setImageData(image, black(source));
    var texture = owner.scene.createTexture();
    owner.scene.setTextureData(texture, new TextureData(image));
    var geometry = owner.scene.createGeometry();
    owner.scene.setGeometryData(geometry, LiveDisplayMesh.build(detail));
    var material = owner.scene.createMaterial();
    owner.scene.setMaterialData(
      material,
      new MaterialData().setBaseColor(0.0, 0.0, 0.0).setRoughness(0.25).setMetallic(0.0).setEmissive(2.0,
      2.0, 2.0).setEmissiveTexture(texture).setBaseColorTexture(
        texture,
        sampler
      )
    );
    var transaction = owner.scene.beginTransaction();
    var node = transaction.createNode();
    transaction.setName(node, "Display " + detail.source.faceName);
    transaction.setParent(node, parent);
    transaction.setVisibility(node, false);
    transaction.setTransform(node, Transform.identity());
    transaction.setGeometry(node, geometry);
    transaction.setMaterial(node, material);
    transaction.commit();
    owner.bridge.mapNode(node, id);
    return {
      node: node,
      geometry: geometry,
      material: material,
      textureKey: null,
      faceIndex: detail.source.faceIndex,
      live: {source: source, image: image, texture: texture}
    };
  }

  static function picture(source:LiveDisplaySource, rgba:haxe.io.Bytes):ImageData {
    var chain = DetailTexture.mipChain(rgba, source.width(), source.height(), false);
    return new ImageData(chain.width, chain.height).setMipCount(chain.mips).setPixels(chain.pixels);
  }

  static function black(source:LiveDisplaySource):ImageData {
    var empty = haxe.io.Bytes.alloc(source.width() * source.height() * 4);
    for (i in 0...source.width() * source.height()) empty.set(i * 4 + 3, 255);
    return picture(source, empty);
  }

  public function hasLiveDisplays():Bool {
    for (group in groups) for (entry in group.nodes) if (entry.live != null) return true;
    return false;
  }

  /** Lets every display's source draw its next picture; true when any picture changed. */
  public function advance(owner:EditorScene, nowMs:Int, controls:LiveControls):Bool {
    var changed = false;
    for (group in groups) for (entry in group.nodes) {
      var live = entry.live;
      if (live == null) continue;
      var rgba = live.source.advance(nowMs, controls);
      if (rgba == null) continue;
      owner.scene.setImageData(live.image, picture(live.source, rgba));
      changed = true;
    }
    return changed;
  }

  /** The overlay that textures face `faceIndex` of object `id` (a thread or a surface detail), or null when that face has none. */
  public function faceNode(id:String, faceIndex:Int):Null<NodeId> {
    var group = groups.get(id);
    if (group == null) return null;
    for (entry in group.nodes) if (entry.faceIndex == faceIndex) return entry.node;
    return null;
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

  function acquire(owner:EditorScene, key:String, color:() -> ImageData, normal:() -> ImageData):ThreadTextures {
    var pair = textures.get(key);
    if (pair == null) {
      if (sampler == null) {
        sampler = owner.scene.createSampler();
        owner.scene.setSamplerData(sampler, new SamplerData().setWrap(1, 1).setFilters(2, 2));
      }
      var colorImage = owner.scene.createImage(), normalImage = owner.scene.createImage();
      owner.scene.setImageData(colorImage, color());
      owner.scene.setImageData(normalImage, normal());
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
    snapshot.dispose();
    transaction.commit();
    for (entry in group.nodes) {
      entry.geometry.dispose();
      entry.material.dispose();
      var live = entry.live;
      if (live != null) {
        live.texture.dispose();
        live.image.dispose();
        continue;
      }
      var pair:ThreadTextures = cast textures.get(entry.textureKey);
      pair.references--;
      if (pair.references == 0) {
        pair.color.dispose();
        pair.normal.dispose();
        pair.colorImage.dispose();
        pair.normalImage.dispose();
        textures.remove(entry.textureKey);
      }
    }
    groups.remove(id);
  }
}
