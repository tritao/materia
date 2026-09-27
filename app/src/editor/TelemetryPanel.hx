package app.editor;

import app.ApplicationPresentationSnapshot;
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
  final surface:Color;
  final textSecondary:Color;

  public function new(surface:Color, textSecondary:Color) {
    this.surface = surface;
    this.textSecondary = textSecondary;
    model = new PlotModel();
    var frameTime = new PlotSeries("frame-time", "Frame time", Color.rgba(0.28, 0.75, 0.98, 1.0), 2.0);
    var gpuTime = new PlotSeries("gpu-time", "GPU submission", Color.rgba(0.78, 0.45, 0.98, 1.0), 2.0);
    for (index in 0...64) {
      frameTime.add(new PlotPoint(index, 10.0 + Math.sin(index * 0.24) * 2.2));
      gpuTime.add(new PlotPoint(index, 4.0 + Math.cos(index * 0.19) * 1.2));
    }
    model.addSeries(frameTime);
    model.addSeries(gpuTime);
  }

  public function build(frame:Null<ApplicationPresentationSnapshot>):View {
    var style = fillStyle();
    style.padding = new Insets(12.0, 12.0, 12.0, 12.0);
    style.background = surface;
    var plotStyle = fillStyle();
    plotStyle.height = LayoutAxis.grow();
    var plot = new PlotView("frame-telemetry", model, plotStyle, "Frame telemetry");
    var physicsStatus = frame == null ? "Physics snapshot unavailable" :
      "Physics step " + frame.revision + " · time " + Std.string(frame.simulationTime) + " s";
    return new Column("telemetry-panel", [
      new KeyedView("heading", new Text("TELEMETRY", null, textSecondary, TextStyleOverride.text(11.0, 0.8))),
      new KeyedView("plot", plot),
      new KeyedView("caption", new Text("Frame time · GPU submission · layout cost")),
      new KeyedView("physics-revision", new Text(physicsStatus))
    ], style);
  }

  static function fillStyle():LayoutStyle {
    var style = new LayoutStyle();
    style.width = LayoutAxis.grow();
    style.height = LayoutAxis.grow();
    return style;
  }
}
