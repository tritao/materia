import app.Main.ReferenceEditorApp;
import app.editor.InspectorPanel;
import haxeon.platform.Runtime;
import haxeon.ui.FontCollection;
import haxeon.ui.LayoutFrame;
import haxeon.ui.TextLayout;
import haxeon.ui.core.RenderNode;
import haxeon.ui.core.RetainedView;
import haxeon.ui.core.UiContext;
import haxeon.ui.semantics.AccessibilityRole;

@:access(app.Main.ReferenceEditorApp)
class InspectorResizeTests {
  static function check(value:Bool, message:String):Void if (!value) throw message;
  public static function main():Int {
    var runtime = Runtime.start(), fonts = FontCollection.create();
    fonts.addSystemFallbacks();
    check(Sys.args().length == 1, "Pass a hobber artifact path to exercise the Xbridge inspector");
    var path = Sys.args()[0];
    var app = new ReferenceEditorApp(fonts, "/tmp/materia-inspector-resize-" + Sys.getPid() + ".json",
      null, null, null, null, path);
    app.treeModel.selectKey("project:machine/Xbridge");
    check(app.scene.selectedId == "project:machine/Xbridge", "Xbridge can be selected");
    var ui = new UiContext(null, fonts);
    ui.buildContext.environment.reducedMotion = true;
    var pane = new RetainedView("inspector", context -> InspectorPanel.build(app, context));
    var frame = new LayoutFrame(900, 900);
    var submit = () -> ui.submitCached(() -> pane, frame, "inspector");
    for (width in [900.0, 300.0, 200.0, 600.0, 240.0, 900.0]) {
      frame.width = width;
      var root = submit();
      var lines:Array<RenderNode> = [];
      root.walk(node -> {
        if (node.layout.text != null && node.layout.text.indexOf(" · fixed · ") >= 0) lines.push(node);
      });
      check(lines.length >= 10, "fixture exercises a long joint list");
      var previousBottom = -1e20;
      for (line in lines) {
        var bounds = line.resolved;
        if (bounds == null || bounds.width <= 0) continue;
        var text = TextLayout.createStyled(fonts, line.layout.text, bounds.width,
          line.layout.textStyle, line.layout.paragraphStyle);
        var height = text.measure().height;
        text.dispose();
        check(bounds.y + 1 >= previousBottom, "joint text does not paint into the following row");
        previousBottom = bounds.y + height;
        check(bounds.height + 1 >= height,
          'width=$width wrapped row allocated ${bounds.height}px for ${height}px of text: ${line.layout.text}');
      }
      var inspector = app.sceneInspector;
      if (inspector == null) throw "Missing property inspector";
      check(!inspector.scrollable, "the parent owns the inspector's single scroll area");
      var controller = inspector.controller;
      if (width <= 300) {
        check(controller.contentHeight > controller.viewportHeight, "wrapped metadata contributes to scroll extent");
        ui.scroll(width / 2, 100, 0, 400);
        submit();
        check(controller.offsetY > 0, "the joint list scrolls with the property controls");
        controller.jumpTo(0, controller.maxScrollY);
        root = submit();
        var controls = 0;
        root.walk(node -> {
          if (node.focusable && node.resolved != null
            && (node.semantics == null || node.semantics.role != AccessibilityRole.ScrollArea)) {
            var bounds = node.resolved.clippedViewportBounds();
            if (bounds.width > 0 && bounds.height > 0) controls++;
          }
        });
        check(controls > 0, "property controls remain reachable below the joint list");
        controller.jumpTo(0, 0);
        submit();
      }
      Sys.println('Inspector resize width=$width joint_rows=${lines.length} passed');
    }
    ui.dispose(); app.dispose(); fonts.dispose(); runtime.dispose();
    return 0;
  }
}
