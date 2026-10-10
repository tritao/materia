package app.editor;

import app.MissionController;
import app.MissionStepPresentation;
import materia.project.SceneArtifact.SceneArtifactMission;
import materia.project.SceneArtifact.SceneArtifactMissionStep;
import haxeon.ui.Insets;
import haxeon.ui.LayoutAxis;
import haxeon.ui.LayoutStyle;
import haxeon.ui.core.View;
import haxeon.ui.theme.ThemeTokens;
import haxeon.ui.widgets.KeyedView;
import haxeon.ui.widgets.collections.ListView;
import haxeon.ui.widgets.collections.ListViewModel;
import haxeon.ui.widgets.controls.Button;
import haxeon.ui.widgets.layout.Column;
import haxeon.ui.widgets.scroll.ScrollController;
import haxeon.ui.widgets.text.Text;

/** Read-only mission monitor. Execution stays on the shared simulation transport. */
class MissionPanel {
  public static inline final ROW_HEIGHT = 32.0;
  final scroll = new ScrollController();
  final model = new MissionListModel();
  var source:Null<SceneArtifactMission>;
  var generation = -1;
  var selected = -1;
  var followed = -1;
  var lastOffset = 0.0;
  var following = true;

  public function new() {}

  public function build(controller:MissionController, tokens:ThemeTokens):View {
    var state = controller.snapshot();
    var mission = state.mission;
    var style = fill();
    style.padding = new Insets(12.0, 12.0, 12.0, 12.0);
    style.childGap = 8.0;
    style.background = tokens.surface;
    var items = [new KeyedView("heading", new Text("MISSION", null, tokens.textSecondary))];
    items.push(new KeyedView("job", MissionJobView.build(controller, tokens, "mission-panel-job")));
    if (mission == null || mission.steps.length == 0) {
      items.push(new KeyedView("empty", new Text("This project has no mission.")));
      return new Column("mission-panel", items, style);
    }
    if (mission != source || generation != state.generation) {
      source = mission; generation = state.generation; selected = -1; followed = -1; following = true;
      scroll.jumpTo(0, 0); lastOffset = scroll.offsetY;
    }
    var progress = state.progress;
    var active = progress == null || progress.phase == "startup" || progress.phase == "complete" ? -1 : progress.stepIndex;
    if (Math.abs(scroll.offsetY - lastOffset) > 0.5) following = false;
    if (following && active >= 0 && active != followed) {
      scroll.jumpTo(0, Math.max(0, (active - 2) * ROW_HEIGHT));
      followed = active;
    }
    lastOffset = scroll.offsetY;
    if (!following) items.push(new KeyedView("follow", new Button("Follow current step", null, function() {
      following = true; followed = -1; selected = -1;
    }, "mission-follow")));
    model.show(mission, progress, tokens);
    var listStyle = fill(); listStyle.clipVertical = true;
    items.push(new KeyedView("steps", new ListView("mission-steps", model, listStyle, scroll, 300.0,
      selected, function(index) {
        selected = index;
        controller.inspectStep(index);
      })));
    var detailIndex = selected >= 0 ? selected : active;
    if (detailIndex >= 0) {
      var step = mission.steps[detailIndex];
      items.push(new KeyedView("detail", new Text((selected >= 0 ? "Selected" : "Current") + " step " +
        (detailIndex + 1) + " · " + MissionStepPresentation.label(step))));
      if (step.at != null) items.push(new KeyedView("connector", new Text("Connector: " + step.at.connector)));
      var pose = step.pose;
      if (pose != null) items.push(new KeyedView("pose", new Text('Target: ${pose.x}, ${pose.y} m · heading ${Math.round(pose.yaw * 180 / Math.PI)}°')));
      if (step.joints != null) items.push(new KeyedView("joints", new Text([for (joint in step.joints)
        joint.joint + ": " + joint.position].join("\n"))));
    }
    return new Column("mission-panel", items, style);
  }

  static function fill():LayoutStyle {
    var style = new LayoutStyle(); style.width = LayoutAxis.grow(); style.height = LayoutAxis.grow();
    return style;
  }
}

private class MissionListModel implements ListViewModel {
  var mission:SceneArtifactMission;
  var progress:Null<app.MissionPlayer.MissionProgress>;
  var tokens:ThemeTokens;
  var version = 0;
  var stateKey = "";
  public function new() {}
  public function show(mission:SceneArtifactMission, progress:Null<app.MissionPlayer.MissionProgress>, tokens:ThemeTokens):Void {
    var nextKey = progress == null ? "ready" : progress.phase + ":" + progress.stepIndex + ":" + progress.completedInLoop;
    if (this.mission != mission || this.tokens != tokens || stateKey != nextKey) version++;
    stateKey = nextKey;
    this.mission = mission; this.progress = progress; this.tokens = tokens;
  }
  public function count():Int return mission.steps.length;
  public function keyAt(index:Int):String return 'step-$index';
  public function estimatedExtent():Float return MissionPanel.ROW_HEIGHT;
  public function extentIsUniform():Bool return true;
  public function extentAt(index:Int):Float return MissionPanel.ROW_HEIGHT;
  public function extentRevisionAt(index:Int):Int return 0;
  public function totalExtent():Null<Float> return count() * MissionPanel.ROW_HEIGHT;
  public function revision():Int return version;
  public function buildItem(index:Int):View {
    var state = progress == null ? "" : index < progress.completedInLoop ? "Done" :
      index == progress.stepIndex && progress.phase == "failed" ? "Failed" :
      index == progress.stepIndex && progress.phase == "executing" ? "Current" : "";
    var style = new LayoutStyle(); style.width = LayoutAxis.grow(); style.height = LayoutAxis.fixed(MissionPanel.ROW_HEIGHT);
    style.padding = new Insets(6.0, 6.0, 6.0, 6.0);
    if (state == "Current" || state == "Failed") style.background = tokens.selectionField;
    return new Text((index + 1) + "  " + MissionStepPresentation.label(mission.steps[index]) +
      (state == "" ? "" : " · " + state), style, state == "Failed" ? tokens.danger : tokens.text);
  }
}
