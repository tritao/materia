package app.editor;

import Observable;

import app.MissionController;
import app.MissionStepPresentation;
import materia.project.SceneArtifact.SceneArtifactMission;
import haxeon.ui.Insets;
import haxeon.ui.LayoutAxis;
import haxeon.ui.LayoutStyle;
import haxeon.ui.core.View;
import haxeon.ui.core.BuildContext;
import haxeon.ui.core.RenderNode;
import haxeon.ui.theme.ThemeTokens;
import haxeon.ui.widgets.KeyedView;
import haxeon.ui.widgets.collections.ListView;
import haxeon.ui.widgets.collections.ListViewModel;
import haxeon.ui.widgets.controls.Button;
import haxeon.ui.widgets.layout.Column;
import haxeon.ui.widgets.scroll.ScrollController;
import haxeon.ui.widgets.text.Text;

/** Read-only mission monitor. Execution stays on the shared simulation transport. */
class MissionPanel implements Observable {
  public static inline final ROW_HEIGHT = 32.0;
  final scroll = new ScrollController();
  final model = new MissionListModel();
  var source:Null<SceneArtifactMission>;
  var generation = -1;
  var selected = -1;
  var followed = -1;
  var lastOffset = 0.0;
  var following = true;

  public function new() {
  }

  /** Run before retained dependencies are checked; building only describes the current presentation. */
  public function update(controller:MissionController):Void {
    var mission = controller.currentMission(), nextGeneration = controller.generation();
    if (mission != source || generation != nextGeneration) {
      source = mission;
      generation = nextGeneration;
      selected = -1;
      followed = -1;
      following = true;
      scroll.jumpTo(0, 0);
      lastOffset = scroll.offsetY;
    }
    var active = controller.activeStep();
    if (following && Math.abs(scroll.offsetY - lastOffset) > 0.5) {
      following = false;
      controller.requestRefresh();
    }
    if (following && active >= 0 && active != followed) {
      scroll.jumpTo(0, Math.max(0, (active - 2) * ROW_HEIGHT));
      followed = active;
    }
    lastOffset = scroll.offsetY;
  }

  public function build(controller:MissionController, tokens:ThemeTokens, ?showProgram:Void -> Void):View {
    return new MissionPanelView(this, controller, tokens, showProgram);
  }

  function describe(context:BuildContext, controller:MissionController, tokens:ThemeTokens, showProgram:Null<Void->Void>):View {
    var state = controller.snapshot();
    var mission = state.mission;
    var style = fill();
    style.padding = new Insets(12.0, 12.0, 12.0, 12.0);
    style.childGap = 8.0;
    style.background = tokens.surface;
    var items = [new KeyedView("heading", new Text("MISSION", null, tokens.textSecondary))];
    items.push(new KeyedView("status", MissionStatusView.build(controller, tokens, "mission-panel-status")));
    // A cell with several robot controllers runs one program on each; the panel follows one at a time.
    if (state.programs.length > 1) for (id in state.programs) items.push(new KeyedView(
      "program-" + id,
      new Button(
        (id == state.selectedProgram ? "● " : "○ ") + id,
        null,
        function() controller.selectProgram(id),
        "mission-program-" + id
      )
    ));
    if (state.machining != null) {
      items.push(new KeyedView("machining-label", new Text(state.machining.label)));
      items.push(new KeyedView("machining-status", new Text(state.machiningStatus)));
      if (showProgram != null) items.push(new KeyedView(
        "program",
        new Button("Open CNC program", null, showProgram, "mission-cnc-program")
      ));
      items.push(new KeyedView(
        "machining-task",
        new Text("Run the CNC program to completion. Use the CNC panel for feed hold, resume and program details.")
      ));
    }
    if (mission == null || mission.steps.length == 0) {
      if (!state.hasWork) items.push(new KeyedView("empty", new Text("This project has no mission.")));
      return new Column("mission-panel", items, style);
    }
    var progress = state.progress;
    var active = controller.activeStep();
    if (!following) items.push(new KeyedView("follow", new Button("Follow current step", null, function() {
      following = true;
      followed = -1;
      selected = -1;
      scroll.jumpTo(scroll.offsetX, scroll.offsetY);
      lastOffset = scroll.offsetY;
      controller.requestRefresh();
    }, "mission-follow")));
    model.show(mission, progress, tokens);
    var listStyle = fill();
    listStyle.clipVertical = true;
    var list = new ListView("mission-steps", model, listStyle, scroll, 300.0, selected, function(index) {
      selected = index;
      controller.inspectStep(index);
    });
    list.selectionProvider = () -> selected;
    list.onScroll = _ -> {
      if (following) {
        following = false;
        controller.requestRefresh();
      }
    };
    items.push(new KeyedView("steps", list));
    var detailIndex = selected >= 0 ? selected : active;
    if (detailIndex >= 0 && detailIndex < mission.steps.length) {
      var step = mission.steps[detailIndex];
      items.push(new KeyedView(
        "detail",
        new Text((selected
        >= 0 ? "Selected" : "Current") + " step " + (detailIndex + 1) + " · " + MissionStepPresentation.label(step))
      ));
      if (step.at != null) items.push(new KeyedView("connector", new Text("Connector: " + step.at.connector)));
      var pose = step.pose;
      if (pose != null) items.push(new KeyedView(
        "pose",
        new Text('Target: ${pose.x}, ${pose.y} m · heading ${Math.round(pose.yaw * 180 / Math.PI)}°')
      ));
      if (step.joints != null) items.push(new KeyedView(
        "joints",
        new Text([for (joint in step.joints) joint.joint + ": " + joint.position].join("\n"))
      ));
    }
    return new Column("mission-panel", items, style);
  }

  static function fill():LayoutStyle {
    var style = new LayoutStyle();
    style.width = LayoutAxis.grow();
    style.height = LayoutAxis.grow();
    return style;
  }
}

private class MissionListModel implements ListViewModel {
  var mission:SceneArtifactMission;
  var progress:Null<app.MissionPlayer.MissionProgress>;
  var tokens:ThemeTokens;
  var stateKey = "";
  public function new() {
  }
  public function show(
    mission:SceneArtifactMission,
    progress:Null<app.MissionPlayer.MissionProgress>,
    tokens:ThemeTokens
  ):Void {
    var nextKey = progress
      == null ? "ready" : progress.phase + ":" + progress.stepIndex + ":" + progress.completedInLoop;
    stateKey = nextKey;
    this.mission = mission;
    this.progress = progress;
    this.tokens = tokens;
  }
  public function count():Int return mission.steps.length;
  public function keyAt(index:Int):String return 'step-$index';
  public function estimatedExtent():Float return MissionPanel.ROW_HEIGHT;
  public function extentIsUniform():Bool return true;
  public function extentAt(index:Int):Float return MissionPanel.ROW_HEIGHT;
  public function extentRevisionAt(index:Int):Int return 0;
  public function totalExtent():Null<Float> return count() * MissionPanel.ROW_HEIGHT;
  public function buildItem(index:Int):View {
    var state = progress == null ? "" : index < progress.completedInLoop ? "Done" : index == progress.stepIndex
      && progress.phase == "failed" ? "Failed" : index == progress.stepIndex
      && progress.phase == "executing" ? "Current" : "";
    var style = new LayoutStyle();
    style.width = LayoutAxis.grow();
    style.height = LayoutAxis.fixed(MissionPanel.ROW_HEIGHT);
    style.padding = new Insets(6.0, 6.0, 6.0, 6.0);
    if (state == "Current" || state == "Failed") style.background = tokens.selectionField;
    return new Text(
      (index + 1) + "  " + MissionStepPresentation.label(mission.steps[index]) + (state == "" ? "" : " · " + state),
      style,
      state == "Failed" ? tokens.danger : tokens.text
    );
  }
}

@:access(app.editor.MissionPanel) private class MissionPanelView implements View {
  final panel:MissionPanel;
  final controller:MissionController;
  final tokens:ThemeTokens;
  final showProgram:Null<Void->Void>;
  public function new(panel:MissionPanel, controller:MissionController, tokens:ThemeTokens, showProgram:Null<Void->Void>) {
    this.panel = panel;
    this.controller = controller;
    this.tokens = tokens;
    this.showProgram = showProgram;
  }
  public function build(context:BuildContext):RenderNode return panel.describe(context, controller, tokens, showProgram).build(context);
}
