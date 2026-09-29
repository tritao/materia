package app.editor;

import app.ApplicationPresentationSnapshot;
import app.ApplicationSimulation;
import Color;
import Insets;
import LayoutAxis;
import LayoutStyle;
import nativekit.ui.core.View;
import nativekit.ui.plotting.PlotModel;
import nativekit.ui.plotting.PlotPoint;
import nativekit.ui.plotting.PlotSeries;
import nativekit.ui.widgets.KeyedView;
import nativekit.ui.widgets.layout.Column;
import nativekit.ui.widgets.plotting.PlotView;
import nativekit.ui.widgets.text.Text;
import nativekit.ui.core.TextStyleOverride;

/** Builds the telemetry dock panel around its retained plot model. */
class TelemetryPanel {
  final model:PlotModel;
  final demo:Bool;

  public function new(demo:Bool = false) {
    this.demo = demo;
    model = new PlotModel();
    var frameTime = new PlotSeries("frame-time", "Frame time", Color.rgba(0.28, 0.75, 0.98, 1.0), 2.0);
    var gpuTime = new PlotSeries("gpu-time", "GPU submission", Color.rgba(0.78, 0.45, 0.98, 1.0), 2.0);
    if (demo) for (index in 0...64) {
      frameTime.add(new PlotPoint(index, 10.0 + Math.sin(index * 0.24) * 2.2));
      gpuTime.add(new PlotPoint(index, 4.0 + Math.cos(index * 0.19) * 1.2));
    }
    model.addSeries(frameTime);
    model.addSeries(gpuTime);
  }

  /** Colours come from the caller on every build so a theme switch reaches this panel. */
  public function build(frame:Null<ApplicationPresentationSnapshot>, surface:Color, textSecondary:Color,
      ?simulation:ApplicationSimulation):View {
    var style = fillStyle();
    style.padding = new Insets(12.0, 12.0, 12.0, 12.0);
    style.background = surface;
    var plotStyle = fillStyle();
    plotStyle.height = LayoutAxis.grow();
    var plot = new PlotView("frame-telemetry", model, plotStyle, "Frame telemetry");
    var physicsStatus = frame == null ? "Physics snapshot unavailable" :
      "Physics step " + frame.revision + " · time " + Std.string(frame.simulationTime) + " s";
    var rows:Array<KeyedView> = [
      new KeyedView("heading", new Text("TELEMETRY", null, textSecondary, TextStyleOverride.text(11.0, 0.8))),
      new KeyedView("plot", plot),
      new KeyedView("caption", new Text(demo ? "Demo frame time · GPU submission" :
        "Telemetry is available when a runtime is active")),
      new KeyedView("physics-revision", new Text(physicsStatus))
    ];
    if (simulation != null) for (id in simulation.humanWorkerIds()) {
      var worker = simulation.humanWorker(id);
      var signals = simulation.humanSignals(id);
      var step = worker == null ? null : worker.currentStep();
      var failure = worker == null ? null : worker.currentJobFailure();
      var warnings = simulation.humanWarnings(id);
      rows.push(new KeyedView("worker-" + id,
        new Text(id + " · step " + (step == null ? "done" : Std.string(step + 1)) +
          (failure == null ? "" : " · " + failure) +
          (warnings.length == 0 ? "" : " · " + warnings.join("; ")))));
      rows.push(new KeyedView("worker-zones-" + id,
        new Text("Zones: " + (signals == null ? "—" : signals.zones.join(", ")))));
      if (signals != null) for (robotId in simulation.simulatedRobotIds()) {
        var distance = signals.separation.get(robotId);
        if (distance != null) rows.push(new KeyedView("worker-separation-" + id + "-" + robotId,
          new Text(robotId + " separation: " + Std.string(distance) + " m")));
      }
    }
    return new Column("telemetry-panel", rows, style);
  }

  static function fillStyle():LayoutStyle {
    var style = new LayoutStyle();
    style.width = LayoutAxis.grow();
    style.height = LayoutAxis.grow();
    return style;
  }
}
