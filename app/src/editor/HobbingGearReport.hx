package app.editor;

import gearkit.GearTrace;
import haxeon.ui.*;
import haxeon.ui.core.*;
import haxeon.ui.plotting.*;
import haxeon.ui.theme.ThemeTokens;
import haxeon.ui.widgets.KeyedView;
import haxeon.ui.widgets.controls.Button;
import haxeon.ui.widgets.layout.Column;
import haxeon.ui.widgets.layout.Row;
import haxeon.ui.widgets.plotting.PlotView;
import haxeon.ui.widgets.scroll.ScrollView;
import haxeon.ui.widgets.text.Text;

/** Scrollable report with progressive disclosure; opening details never reruns the cut. */
class HobbingGearReport implements View {
  final data:HobbingReportData;
  public function new(data:HobbingReportData) this.data = data;

  public function build(context:BuildContext):RenderNode return context.withScope(new Key("hobbing-report"), function() {
    var tokens = context.theme.tokens;
    var wide = context.state(context.id("wide"), false);
    var groups = data.warnings(), warningCount = 0;
    for (group in groups) warningCount += group.length;
    var rows:Array<KeyedView> = [];
    rows.push(text("heading", "HOBBING · GEAR REPORT", tokens.textSecondary));
    rows.push(text("status", data.status, data.failure == null ? tokens.textPrimary : tokens.danger));
    if (data.failure != null) rows.push(text("failure", data.failure, tokens.danger));
    var summary:Array<KeyedView> = [
      text("cycle", 'Cycle  ${n(data.seconds)} s', tokens.textPrimary),
      text("sync", 'Sync error  ${n(data.syncArcsec)} arcsec', tokens.textPrimary),
      text("contacts", 'Fixture contacts  ${data.contacts}', data.contacts > 0 ? tokens.danger : tokens.textPrimary),
      text("load", 'Estimated cutting load  ${Math.round(data.cutting)} N · ${Math.round(data.power)} W', tokens.textSecondary)
    ];
    rows.push(new KeyedView("summary", new Column("cycle-summary", summary, card(tokens))));
    rows.push(new KeyedView("quality", new Column("quality-status", [
      text("qualification", data.quality(), tokens.textPrimary),
      text("reason", data.measurement == null ?
        (data.inspectionFailure != null ? data.inspectionFailure : data.complete ? "No inspection traces available." : "Profile and lead inspection runs when the cycle finishes.") :
        '$warningCount inspection warnings · qualification depends on sampling, coverage and assessment scope.', tokens.textSecondary)
    ], card(tokens))));
    var measured = data.measurement;
    var inspection = context.resourceState(context.id("inspection"),
      function() return {measurement:measured,revision:1}, _ -> {});
    if (inspection.value.measurement != measured) {inspection.value.measurement = measured;inspection.value.revision++;}
    if (measured != null) {
      var plots = [new KeyedView("profile", plot("profile", "Profile", "Roll [mm]", measured.profiles, tokens, inspection.value.revision)),
        new KeyedView("lead", plot("lead", "Lead", "Axial position [mm]", measured.helices, tokens, inspection.value.revision))];
      var plotsStyle = flow(); plotsStyle.childGap = 12;
      rows.push(new KeyedView("plots", wide.value ? new Row("plots-wide", plots, plotsStyle) : new Column("plots-stacked", plots, plotsStyle)));
      rows.push(text("warning-count", warningCount == 0 ? "No inspection warnings" : '$warningCount inspection warnings', tokens.textSecondary));
      for (name in ["Sampling", "Coverage", "Assessment"]) {
        var warnings:Array<String> = groups.get(name);
        if (warnings.length > 0) disclosure(context, rows, name, '$name (${warnings.length})', [
          for (i in 0...warnings.length) text('warning-$i', "• " + warnings[i], tokens.textPrimary)
        ], tokens);
      }
    }
    disclosure(context, rows, "loads", "Load assumptions · estimated", [
      text("forces", '${Math.round(data.cutting)} N cutting · ${Math.round(data.radial)} N radial · ${Math.round(data.power)} W', tokens.textPrimary),
      text("torques", 'B torque ${n(data.hobTorque)} Nm · C torque bound ${n(data.tableTorque)} Nm', tokens.textPrimary),
      text("basis", data.materialBasis, tokens.textSecondary),
      text("limits", "Kienzle window estimates; effective edge width 1 mm assumed. Overlap summed; instantaneous peaks unresolved. Requires cutting-test calibration at the stated hardness.", tokens.textSecondary)
    ], tokens);
    disclosure(context, rows, "run-details", "Run details", [
      text("windows", '${data.windows} gash windows · ${data.poses} poses', tokens.textPrimary),
      text("memory", 'Peak ${data.buffered} buffered samples · ${data.stockKiB} KiB retained stock and force data', tokens.textSecondary)
    ], tokens);
    if (measured != null) samples(context, rows, tokens);
    var content = flow(); content.padding = new Insets(12,12,28,12); content.childGap = 10; content.background = tokens.surface;
    var viewport = flow(); viewport.height = LayoutAxis.grow(); viewport.background = tokens.surface;
    var root = new ScrollView("gear-report-scroll", new Column("gear-report-content", rows, content), viewport).build(context);
    root.onResolved(bounds -> {
      var next = bounds.width >= 760;
      if (next != wide.value) { wide.update(next); context.requestLayoutFeedback(); }
    });
    return root;
  });

  function disclosure(context:BuildContext, rows:Array<KeyedView>, id:String, label:String, children:Array<KeyedView>, tokens:ThemeTokens):Void {
    var expanded = context.state(context.id("expanded-" + id), false);
    var toggle = new ReportDisclosure(label, expanded.value, flow(), () -> expanded.update(!expanded.value), "toggle-" + id);
    rows.push(new KeyedView("toggle-" + id, toggle));
    if (expanded.value) rows.push(new KeyedView("details-" + id, new Column("details-" + id, children, card(tokens))));
  }

  function samples(context:BuildContext, rows:Array<KeyedView>, tokens:ThemeTokens):Void {
    var expanded = context.state(context.id("samples-expanded"), false);
    var page = context.state(context.id("samples-page"), 0);
    var copied = context.state(context.id("samples-copied"), false);
    var source = context.state(context.id("sample-source"), data.measurement);
    if (source.value != data.measurement) {source.update(data.measurement);copied.update(false);page.update(0);}
    rows.push(new KeyedView("toggle-samples", new ReportDisclosure("Raw samples", expanded.value, flow(),
      () -> expanded.update(!expanded.value), "toggle-samples")));
    if (!expanded.value) return;
    var children:Array<KeyedView> = [];
    children.push(new KeyedView("copy", new Button(copied.value ? "Copied CSV" : "Copy all samples as CSV", flow(),
      () -> { context.clipboard.writeText(data.csv()); copied.update(true); }, "copy-samples")));
    children.push(text("scope", "Preview: first tooth, both flanks. CSV includes all inspected teeth. Positions in mm; deviations in µm.", tokens.textSecondary));
    var measured = data.measurement;
    if (measured == null) return;
    var records:Array<{kind:String, trace:GearTrace, index:Int}> = [];
    for (kind in ["Profile", "Lead"]) for (trace in (kind == "Profile" ? measured.profiles : measured.helices).slice(0,2))
      for (i in 0...trace.x.length) records.push({kind:kind,trace:trace,index:i});
    var pages = Std.int(Math.max(1,Math.ceil(records.length/24))), current = Std.int(Math.min(page.value,pages-1));
    children.push(text("page-label", 'Page ${current+1} / $pages · ${records.length} preview samples', tokens.textSecondary));
    children.push(new KeyedView("table-header", sampleRow("sample-header",["x [mm]","Raw [µm]","Filtered [µm]"],tokens.textSecondary)));
    var lastTrace = "";
    for (r in records.slice(current*24,(current+1)*24)) {
      var t = r.trace, i = r.index, traceLabel = '${r.kind} · tooth ${t.tooth} · flank ${t.flank}';
      if (traceLabel != lastTrace) {children.push(text('trace-${r.kind}-${t.tooth}-${t.flank}',traceLabel,flankColor(t.flank,tokens)));lastTrace = traceLabel;}
      children.push(new KeyedView('sample-${r.kind}-${t.tooth}-${t.flank}-$i',
        sampleRow('sample-$i',[n(t.x[i]),n(t.raw[i]),n(t.filtered[i])],tokens.textPrimary)));
    }
    var previous = new Button("Previous", null, () -> page.update(current-1), "samples-previous"); previous.enabled = current > 0;
    var next = new Button("Next", null, () -> page.update(current+1), "samples-next"); next.enabled = current < pages-1;
    children.push(new KeyedView("pagination", new Row("sample-pages", [new KeyedView("previous",previous),new KeyedView("next",next)], flow())));
    rows.push(new KeyedView("samples", new Column("sample-table", children, card(tokens))));
  }

  static function sampleRow(id:String, cells:Array<String>, color:Color):View {
    return new Row(id,[for (i in 0...cells.length) text('cell-$i',cells[i],color)],flow());
  }

  static function plot(id:String, title:String, xLabel:String, traces:Array<GearTrace>, tokens:ThemeTokens, revision:Int):View {
    var children = [text("title", title, tokens.textPrimary)];
    var selected = traces.slice(0,2);
    if (selected.length == 0) {
      children.push(text("empty", "No traces available for this inspection.", tokens.textSecondary));
      return new Column(id + "-card", children, card(tokens));
    }
    var model = new PlotModel();
    for (t in selected) {
      var series = new PlotSeries('tooth-${t.tooth}-flank-${t.flank}', 'Tooth ${t.tooth}, flank ${t.flank}', flankColor(t.flank,tokens), 2);
      series.setPoints([for (i in 0...t.x.length) new PlotPoint(t.x[i], t.filtered[i])]); model.addSeries(series);
    }
    var range = model.range();
    if (range == null) throw "Gear trace has no plotting range";
    var pad = Math.max(0.5,(range.maximumY-range.minimumY)*0.08);
    model.setRange(null,null,range.minimumY-pad,range.maximumY+pad);
    children.push(text("y-range", 'Deviation [µm] · ${n(range.minimumY-pad)} … ${n(range.maximumY+pad)}', tokens.textSecondary));
    var plotStyle = flow(); plotStyle.height = LayoutAxis.fixed(160);
    var chart = new ReportPlot(id + "-chart", model, plotStyle, title + " filtered deviations", revision);
    chart.setAppearance(tokens.surfaceSunken,tokens.border,tokens.textSecondary,4,10);
    children.push(new KeyedView("chart",chart));
    children.push(text("x-range", '$xLabel · ${n(range.minimumX)} … ${n(range.maximumX)}', tokens.textSecondary));
    for (t in selected) children.push(text('legend-${t.tooth}-${t.flank}',
      'Tooth ${t.tooth} · flank ${t.flank} · total ${n(t.total)} µm',flankColor(t.flank,tokens)));
    return new Column(id + "-card",children,card(tokens));
  }

  static function flankColor(flank:Int, tokens:ThemeTokens):Color return flank == 0 ? tokens.accent : tokens.surface.red > 0.5 ? Color.rgba(0.05,0.42,0.27,1) : tokens.success;
  static function n(value:Float):String return HobbingReportData.number(value);
  static function text(key:String, value:String, color:Color):KeyedView {
    var style = flow();
    return new KeyedView(key,new Text(value,style,color,TextStyleOverride.text(12)));
  }
  static function flow():LayoutStyle {var style = new LayoutStyle();style.width = LayoutAxis.grow();return style;}
  static function card(tokens:ThemeTokens):LayoutStyle {
    var style = flow();style.childGap = 6;style.padding = new Insets(10,10,10,10);style.background = tokens.surfaceRaised;
    style.radiusTopLeft = style.radiusTopRight = style.radiusBottomLeft = style.radiusBottomRight = 6;
    return style;
  }
}

/** Keep small diagnostic charts on the direct custom-paint path, like the machine dial. */
private class ReportPlot implements View {
  final plot:PlotView;
  final inspectionRevision:Int;
  public function new(key:String,model:PlotModel,style:LayoutStyle,label:String,inspectionRevision:Int) {
    plot = new PlotView(key,model,style,label);this.inspectionRevision = inspectionRevision;
  }
  public function setAppearance(background:Color,grid:Color,axis:Color,divisions:Int,padding:Float):Void
    plot.setAppearance(background,grid,axis,divisions,padding);
  public function build(context:BuildContext):RenderNode {
    var node = plot.build(context);
    node.cachePolicy = CachePolicy.None;
    // A replacement PlotModel can have the same local revision as the old model.
    node.onPaint((_,_) -> {}, "inspection:"+inspectionRevision);
    return node;
  }
}

/** Native accessibility announces disclosure state along with its action. */
private class ReportDisclosure implements View {
  final button:Button;
  final expanded:Bool;
  public function new(label:String,expanded:Bool,style:LayoutStyle,onClick:Void->Void,key:String) {
    button = new Button(label,style,onClick,key);
    button.leadingIcon = expanded ? haxeon.ui.icons.IconName.ChevronDown : haxeon.ui.icons.IconName.ChevronRight;
    this.expanded = expanded;
    button.accessibilityLabel = (expanded ? "Collapse " : "Expand ")+label;
  }
  public function build(context:BuildContext):RenderNode {
    var node = button.build(context);
    var semantics = node.semantics;
    if (expanded && semantics != null) semantics.states |= haxeon.ui.semantics.AccessibilityState.Expanded;
    return node;
  }
}
