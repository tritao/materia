package tests;

import app.editor.HobbingGearReport;
import app.editor.HobbingReportData;
import gearkit.*;
import haxeon.ui.*;
import haxeon.ui.core.*;
import haxeon.ui.host.*;

@:access(gearkit.GearMeasurement)
@:access(haxeon.ui.core.RenderNode)
class HobbingReportTests {
  static var checks = 0;
  static function check(ok:Bool, message:String):Void {checks++;if (!ok) throw message;}

  /** Synthetic traces exercise the UI without running or claiming a qualified cut. */
  public static function fixture():HobbingReportData {
    var data = new HobbingReportData(), spec = new GearSpec(1,40,10,20*Math.PI/180);
    var measured = new GearMeasurement(spec,new GearInspection(spec,spec.pitchRadius()));
    for (tooth in 0...2) for (flank in 0...2) {
      var x = [for (i in 0...40) i/4.0], y = [for (i in 0...40) 5+flank*2+Math.sin(i/6.0)+tooth];
      measured.profiles.push(new GearTrace(tooth,flank,x,y,0,0,9.75,9.75));
      measured.helices.push(new GearTrace(tooth,flank,x,[for(i in 0...40) 12+flank*9-i/2.0+Math.cos(i/3.0)],0,0,9.75,9.75));
    }
    measured.warnings.push("Profile source spacing does not resolve filter cutoff");
    measured.warnings.push("Face endpoints not sampled; end excess material unchecked");
    measured.warnings.push("Coarse app diagnostic: profile inspected from pitch to tip only; full active profile unchecked");
    measured.extrapolation.push("Outside reference diameter scope");
    data.measurement = measured;data.complete = true;data.status = "Cycle complete";
    data.seconds = 6.003;data.syncArcsec = 0.004;data.windows = 982;data.poses = 421515;
    data.buffered = 619;data.stockKiB = 946;data.cutting = 573;data.radial = 209;data.power = 2865;
    data.hobTorque = 9.167;data.tableTorque = 12.194;data.materialBasis = "Assumed kc1.1=2400 N/mm², mc=0.25";
    return data;
  }

  static function find(root:RenderNode,label:String):Null<RenderNode> {
    var result:Null<RenderNode> = null;
    root.walk(node -> {if (node.semantics != null && node.semantics.label == label) result = node;});return result;
  }
  static function words(root:RenderNode):String {
    var out = new StringBuf();root.walk(node -> {if (node.layout.text != null) {out.add(node.layout.text);out.add("\n");}});return out.toString();
  }
  static function charts(root:RenderNode):Array<RenderNode> {
    var out:Array<RenderNode> = [];root.walk(node -> {var semantics = node.semantics;if (semantics != null && StringTools.endsWith(semantics.label,"filtered deviations")) out.push(node);});return out;
  }
  static function activate(ui:UiContext, root:RenderNode, label:String):Void {
    var node = find(root,label);if (node == null) throw "Missing control " + label;
    ui.focusWidget(node.id);ui.key(UiEventKind.KeyDown,UiKey.Enter);ui.key(UiEventKind.KeyUp,UiKey.Enter);
  }

  public static function main():Int {
    if (Sys.args().indexOf("--preview") >= 0) {
      var options = new DesktopUiHostOptions();options.title = "Hobbing report preview";
      options.width = Sys.args().indexOf("--wide") >= 0 ? 1050 : 507;options.height = 900;
      options.captureDirectory = Sys.args().indexOf("--wide") >= 0 ? "/tmp/hobbing-report-wide" : "/tmp/hobbing-report-narrow";
      options.frameLimit = 4;
      if (Sys.args().indexOf("--interactive") >= 0) {options.frameLimit = 0;options.captureSeconds = 6;}
      return DesktopUiHost.run(options,host -> new ReportPreview(host.fonts,fixture()));
    }
    var data = fixture();
    check(data.quality().indexOf("ISO class not qualified") >= 0,"incomplete measurement never claims ISO class");
    check(data.quality().indexOf("incomplete or worse than 11") >= 0,"ambiguous grade retains both possible meanings");
    var warnings = data.warnings();
    var sampling = warnings.get("Sampling"), coverage = warnings.get("Coverage"), assessment = warnings.get("Assessment");
    check(sampling != null && sampling.length == 1,"filter sampling warning classified");
    check(coverage != null && coverage.length == 2,"face endpoints and partial profile classified as coverage");
    check(assessment != null && assessment.length == 1,"scope warning retained");
    var csv = data.csv().split("\n");
    check(csv.length == 322,"CSV includes all 320 samples plus header and terminal newline");
    check(csv[0] == "trace,tooth,flank,position_mm,raw_um,filtered_um","CSV labels raw and filtered units");
    check(data.csv().indexOf("lead,1,1,") >= 0,"CSV includes teeth beyond plot preview");
    var fonts = FontCollection.create();fonts.add("haxeon/packages/ui/vendor/skribidi/example/data/IBMPlexSans-Regular.ttf");
    var ui = new UiContext(null,fonts), frame = new LayoutFrame(507,900);
    try {
      var root = ui.submit(new HobbingGearReport(data),frame), text = words(root), plots = charts(root);
      check(text.indexOf("Estimated cutting load  573 N") >= 0,"estimated load visible without opening details");
      check(text.indexOf("source spacing") < 0,"warning bodies collapsed by default");
      check(text.indexOf("effective edge width") < 0,"load assumptions collapsed by default");
      check(text.indexOf("raw ") < 0,"raw table collapsed by default");
      check(plots.length == 2,"profile and lead charts exist");
      check(plots[0].globalBounds().width > 400 && plots[0].globalBounds().height == 160,"chart has useful nonzero size");
      check(plots[1].globalBounds().y > plots[0].globalBounds().y+160,"narrow charts stack");
      for (plot in plots) {
        check(plot.cachePolicy == CachePolicy.None,"charts use direct paint path");
        var canvas = new Canvas();check(plot.paintContent(canvas),"chart records paint commands");
        var display = DisplayList.create();canvas.update(display);check(display.info().commandCount >= 5,"chart paints background, grid and traces: " + display.info().commandCount);display.dispose();canvas.reset();
      }
      activate(ui,root,"Expand Sampling (1)");root = ui.submit(new HobbingGearReport(data),frame);
      check(words(root).indexOf("Profile source spacing") >= 0,"keyboard expands diagnostic group");
      activate(ui,root,"Collapse Sampling (1)");root = ui.submit(new HobbingGearReport(data),frame);
      check(words(root).indexOf("Profile source spacing") < 0,"keyboard collapses diagnostic group");
      activate(ui,root,"Expand Raw samples");root = ui.submit(new HobbingGearReport(data),frame);
      check(words(root).indexOf("Page 1 / 7") >= 0,"raw preview is paginated");
      activate(ui,root,"Next");root = ui.submit(new HobbingGearReport(data),frame);
      check(words(root).indexOf("Page 2 / 7") >= 0,"next page changes samples");
      frame = new LayoutFrame(1050,900);root = ui.submit(new HobbingGearReport(data),frame);root = ui.submit(new HobbingGearReport(data),frame);plots = charts(root);
      check(Math.abs(plots[0].globalBounds().y-plots[1].globalBounds().y)<1,"wide charts are side by side");
      check(plots[1].globalBounds().x > plots[0].globalBounds().x+plots[0].globalBounds().width,"wide charts have gap");
      var oldPaintKey = plots[0].retainedPaintKey();
      var replacement = fixture();root = ui.submit(new HobbingGearReport(replacement),frame);
      check(charts(root)[0].retainedPaintKey() != oldPaintKey,"replacement inspection invalidates plot paint even with matching local revisions");
      root = ui.submit(new HobbingGearReport(replacement),frame);var stableKey = charts(root)[0].retainedPaintKey();
      root = ui.submit(new HobbingGearReport(replacement),frame);
      check(charts(root)[0].retainedPaintKey() == stableKey,"unchanged inspection retains chart display lists");
      frame = new LayoutFrame(320,500);root = ui.submit(new HobbingGearReport(data),frame);root = ui.submit(new HobbingGearReport(data),frame);plots = charts(root);
      check(plots[1].globalBounds().y > plots[0].globalBounds().y,"resizing back stacks charts");
      root.walk(node -> {if (node.hasPaintHandler()) check(node.globalBounds().x+node.globalBounds().width <= 320,"chart fits narrow pane");});
      var headingY = find(root,"Collapse Raw samples");if (headingY == null) headingY = find(root,"Expand Raw samples");if (headingY == null) throw "Missing raw samples toggle";
      var beforeScroll = headingY.globalBounds().y;
      ui.scroll(150,300,0,500);root = ui.submit(new HobbingGearReport(data),frame);
      var afterScroll = find(root,"Collapse Raw samples");
      if (afterScroll == null) afterScroll = find(root,"Expand Raw samples");
      check(afterScroll != null && afterScroll.globalBounds().y < beforeScroll,"short panes scroll to details");
      var empty = new HobbingReportData();root = ui.submit(new HobbingGearReport(empty),frame);
      check(charts(root).length == 0 && words(root).indexOf("Inspection pending") >= 0,"running report has honest pending state");
      empty.complete = true;empty.inspectionFailure = "Missing tooth flank";root = ui.submit(new HobbingGearReport(empty),frame);
      check(words(root).indexOf("Missing tooth flank") >= 0,"inspection failure stays visible");
      var measured = data.measurement;if (measured == null) throw "Missing test measurement";
      measured.overallGrade = 7;measured.isoClass = 7;
      check(data.quality() == "Candidate grade 7 · ISO class 7","qualified result preserved");
    } catch (error:Dynamic) {ui.dispose();fonts.dispose();throw error;}
    ui.dispose();fonts.dispose();Sys.println('Hobbing report: $checks checks passed');return 0;
  }
}

private class ReportPreview implements DesktopUiApplication {
  final ui:UiContext;
  final data:HobbingReportData;
  public function new(fonts:FontCollection,data:HobbingReportData) {ui = new UiContext(null,fonts,Sys.args().indexOf("--dark") >= 0 ? haxeon.ui.theme.Theme.dark() : haxeon.ui.theme.Theme.light());this.data = data;}
  public function context():UiContext return ui;
  public function submit(frame:LayoutFrame):RenderNode return ui.submit(new HobbingGearReport(data),frame);
  public function diagnosticState():Dynamic return {preview:"synthetic hobbing inspection"};
  public function dispose():Void ui.dispose();
}
