package stockkit.internal;

import StockKitNative;
import toolpathkit.tool.CutterProfile;
import toolpathkit.tool.Tool;

/** Not part of the API: used by `Stock`. */
/** A tool registered with the core: its flutes cut, its shank and holder are checked for contact. */
class NativeTool {
  public final tool:Tool;
  final owner:Ownedsk_tool_handle;
  var disposed = false;

  public function new(tool:Tool) {
    this.tool = tool;
    var segments:Array<sk_profile_segment> = [];
    for (segment in tool.profile().segments) {
      var native = new sk_profile_segment();
      native.set_struct_size(sk_profile_segment.size());
      native.set_zone(switch CutterProfile.zoneOf(segment) {
        case Cutting:
          StockKitNativeConstants.SK_ZONE_CUTTING;
        case Shank:
          StockKitNativeConstants.SK_ZONE_SHANK;
        case Holder:
          StockKitNativeConstants.SK_ZONE_HOLDER;
      }
      );
      switch segment {
        case Line(r0, z0, r1, z1, _):
          native.set_kind(StockKitNativeConstants.SK_SEGMENT_LINE);
          native.set_r0(r0);
          native.set_z0(z0);
          native.set_r1(r1);
          native.set_z1(z1);
        case Arc(cr, cz, r0, z0, r1, z1, _):
          native.set_kind(StockKitNativeConstants.SK_SEGMENT_ARC);
          native.set_r0(r0);
          native.set_z0(z0);
          native.set_r1(r1);
          native.set_z1(z1);
          native.set_center_r(cr);
          native.set_center_z(cz);
      }
      segments.push(native);
    }
    var created = StockKitNative.sk_tool_create(segments);
    if (created.status != StockKitNativeConstants
      .SK_OK) throw 'tool ${tool.number} profile rejected by StockKit core (error ${created.status})';
    owner = created.out_tool;
  }

  public function borrow():sk_tool_handle return owner.borrow();

  public function dispose():Void {
    if (disposed) return;
    disposed = true;
    owner.close();
  }
}
