package nativekit.sim;

import NativeKitScene;

/** The single scene transaction committed by one simulation step. */
class SceneChanges {
    final value:nkscene_change_set;
    var disposed:Bool = false;

    @:allow(StepResult)
    private function new(value:nkscene_change_set) {
        this.value = value;
    }

    public function revision():haxe.Int64 {
        ensureLive();
        var result = NativeKitScene.nkscene_change_set_get_revision(value);
        if (result.status != NativeKitSceneConstants.NKS_OK)
            throw 'sceneChanges.revision failed with NativeKit scene status ${result.status}';
        return result.out_revision;
    }

    public function nativeHandle():nkscene_change_set {
        ensureLive();
        return value;
    }

    public function dispose():Void {
        if (disposed)
            return;
        NativeKitScene.nkscene_change_set_destroy(value);
        disposed = true;
    }

    public function isDisposed():Bool
        return disposed;

    function ensureLive():Void {
        if (disposed)
            throw "Simulation scene changes have been disposed";
    }
}
