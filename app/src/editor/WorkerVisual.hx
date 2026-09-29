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
    character = new HumanCharacter(scene, asset, HumanoidRig.detect(asset), parent, "Worker character");
    character.player.playNamed("Idle", 0.0);
    character.advance(0.0);
  }

  public function nodes():Array<NodeId> return [character.root].concat(character.model.primitiveNodes);

  public function dispose(scene:Scene, destroyNode:Bool):Void {
    if (destroyNode) {
      var transaction = scene.beginTransaction();
      transaction.destroyNode(character.root);
      transaction.commit();
    }
    character.dispose();
    asset.dispose();
  }
}
