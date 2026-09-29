package app.editor;

import animkit.AnimationAsset;
import humankit.HumanCharacter;
import humankit.HumanoidRig;
import nativekit.scene.NodeId;
import nativekit.scene.Scene;

/** Idle skinned character attached beneath an authored worker's transform. */
class WorkerVisual {
  public final assetPath:String;
  public final character:HumanCharacter;
  final asset:AnimationAsset;

  public function new(scene:Scene, parent:NodeId, path:String) {
    assetPath = path;
    asset = AnimationAsset.load(WorkerAssetPath.resolve(path));
    try {
      character = new HumanCharacter(scene, asset, HumanoidRig.detect(asset), parent, "Worker character");
      character.player.playNamed("Idle", 0.0);
      character.advance(0.0);
    } catch (error:Dynamic) {
      var cleanupError:Dynamic = null;
      if (character != null) {
        try destroyNodes(scene) catch (failure:Dynamic) cleanupError = failure;
        try character.dispose() catch (failure:Dynamic) if (cleanupError == null) cleanupError = failure;
      }
      asset.dispose();
      if (cleanupError != null) throw 'failed preview construction cleanup: $cleanupError';
      throw error;
    }
  }

  public function nodes():Array<NodeId> return [character.root].concat(character.model.primitiveNodes);

  public function dispose(scene:Scene, destroyNode:Bool):Void {
    var failure:Dynamic = null;
    if (destroyNode) try destroyNodes(scene) catch (error:Dynamic) failure = error;
    try character.dispose() catch (error:Dynamic) if (failure == null) failure = error;
    asset.dispose();
    if (failure != null) throw 'failed preview disposal: $failure';
  }

  function destroyNodes(scene:Scene):Void {
    var snapshot = scene.snapshot();
    var rootExists = snapshot.findNode(character.root) != null;
    var children = [for (node in character.model.primitiveNodes)
      if (snapshot.findNode(node) != null) node];
    snapshot.dispose();
    if (!rootExists && children.length == 0) return;
    var transaction = scene.beginTransaction();
    for (node in children) transaction.destroyNode(node);
    if (rootExists) transaction.destroyNode(character.root);
    transaction.commit();
  }
}
