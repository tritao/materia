package app.editor;

import app.Main.ReferenceEditorApp;
import haxeon.ui.core.View;

/** Process selection and presentation are independent; neither edits the authored assembly. */
@:access(app.Main.ReferenceEditorApp) class HobberScenePanel {
  public static function build(app:ReferenceEditorApp, availableWidth:Float = 0):Null<View> {
    var motion = app.session.machineMotion, config = motion == null ? null : motion.hobbing;
    if (config == null || config.scenario == null) return null;
    var player = app.simulation.hobbingPlayer();
    return HobberScenarioSelector.build(app.appearance.theme.tokens, app.session.hobbingScenario ?? config.scenario,
      !app.simulation.isRunning() && !app.documents.blocked(), id -> {
        if (id == app.session.hobbingScenario) return;
        app.simulation.selectHobbingScenario(app.session, id);
        app.invalidateView();
      }, availableWidth > 0 ? availableWidth : app.viewportWidth, player == null ? null : player.scenarioLabel());
  }
}
