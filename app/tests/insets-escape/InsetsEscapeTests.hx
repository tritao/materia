import haxeon.ui.Insets;
import haxeon.ui.LayoutStyle;
class InsetsEscapeTests {
 static var escaped:Array<Dynamic> = [];
 static function retain():Void {
  for (i in 0...1000) {
   var style = new LayoutStyle();
   style.padding = new Insets(i + 1, i + 2, i + 3, i + 4);
   escaped.push(style.padding);
  }
 }
 static function main():Void {
  retain();

  for (round in 0...8) {
   for (i in 0...100000) { var trash = [i, i + 1, i + 2]; if (trash.length != 3) throw 'bad'; }
   hl.Gc.major();
   for (i in 0...escaped.length) {
    var value:Insets = cast escaped[i];
    if (value.left != i + 1 || value.bottom != i + 4) throw 'escaped padding corrupted at ' + i + ': ' + value.left + ', ' + value.bottom;
   }
  }
  Sys.println('PASS: retained Insets survives owner release and GC');
 }
}
