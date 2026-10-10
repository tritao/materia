package app.editor;

import app.editor.ExampleBrowser.ExampleFamily;
import haxeon.ui.Color;
import haxeon.ui.LayoutAxis;
import haxeon.ui.LayoutStyle;
import haxeon.ui.PathBuilder;
import haxeon.ui.LineCap;
import haxeon.ui.LineJoin;
import haxeon.ui.widgets.CanvasView;
import haxeon.ui.core.View;

/** Lightweight category illustrations; no model compilation or image loading on the Start page. */
class StartExamplePreview {
  public static function build(family:ExampleFamily, ink:Color, width:Float = 250.0):View {
    var style = new LayoutStyle();
    style.width = LayoutAxis.fixed(width);
    style.height = LayoutAxis.fixed(88.0);
    return new CanvasView("start-preview:" + family.id, function(canvas, geometry) {
      var path = new PathBuilder();
      switch family.category {
        case "Welding", "Robotics & handling":
          if (family.id == "gantry-welder" || family.id == "gantry-picker") {
            path.moveTo(48, 72).lineTo(48, 18).lineTo(200, 18).lineTo(200, 72);
            path.moveTo(142, 18).lineTo(142, 50).lineTo(157, 60);
          } else {
            path.moveTo(55, 72).lineTo(82, 72).lineTo(82, 49).lineTo(113, 23).lineTo(161, 39).lineTo(174, 59);
          }
          if (family.category == "Welding") {
            path.moveTo(142, 72).lineTo(205, 72).moveTo(174, 72).lineTo(174, 63);
            path.moveTo(184, 56).lineTo(191, 50).moveTo(187, 63).lineTo(198, 63).moveTo(180, 49).lineTo(183, 40);
          }
          if (family.id == "track-welder") path.moveTo(37, 80).lineTo(209, 80);
          if (family.id == "mobile-welding" || family.id == "mobile-base") {
            path.moveTo(49, 78).lineTo(101, 78).moveTo(55, 83).lineTo(62, 83).moveTo(87, 83).lineTo(94, 83);
          }
        case "Machining":
          path.moveTo(59, 74).lineTo(59, 18).lineTo(190, 18).lineTo(190, 74).close();
          path.moveTo(124, 18).lineTo(124, 49).moveTo(115, 49).lineTo(133, 49);
          path.moveTo(80, 64).lineTo(169, 64).moveTo(93, 64).lineTo(93, 56).lineTo(155, 56).lineTo(155, 64);
        case "CAD & design":
          path.moveTo(83, 34).lineTo(131, 15).lineTo(178, 34).lineTo(178, 64).lineTo(131, 81).lineTo(83, 64).close();
          path.moveTo(83, 34).lineTo(131, 52).lineTo(178, 34).moveTo(131, 52).lineTo(131, 81);
        default:
          path.moveTo(118, 19).lineTo(130, 19).lineTo(130, 31).lineTo(118, 31).close();
          path.moveTo(124, 32).lineTo(124, 55).lineTo(104, 77).moveTo(124, 55).lineTo(147, 77);
          path.moveTo(101, 47).lineTo(124, 38).lineTo(151, 45);
      }
      canvas.strokeTransient(path.build(), ink, 3.0, LineCap.Round, LineJoin.Round);
    }, style, null, false);
  }
}
