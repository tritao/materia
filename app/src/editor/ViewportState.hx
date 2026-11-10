package app.editor;

/**
 * A perspective viewport's frame as data: what is open, the camera, the viewport
 * size and inspection lens, and the presentation cursor. Copying it from the
 * editor and replaying it headlessly reproduces the same view, so a view found
 * by hand can become a regression check.
 */
typedef ViewportState = {
  /** Always `materia.viewport-state`; other values are rejected. */
  var format:String;
  /** Project reference as the editor opened it, when a project is open. */
  @:optional var project:String;
  /** Selected project configuration, study or demo. */
  @:optional var variant:String;
  /** Scene document path, when a plain document (not a project) is open. */
  @:optional var document:String;
  /** Orbit target in world metres. */
  var target:Array<Float>;
  var yaw:Float;
  var pitch:Float;
  var distance:Float;
  /** Viewport size in layout pixels. */
  var width:Float;
  var height:Float;
  /** Inspection lens zoom; 1 unless the viewport is a read-only inspection camera. */
  var inspectionZoom:Float;
  @:optional var presentation:ViewportPresentationState;
}

/** The presentation cursor: view, explode amount and procedure position. */
typedef ViewportPresentationState = {
  var view:String;
  var explode:Float;
  @:optional var procedure:String;
  var step:Int;
  var progress:Float;
}

class ViewportStates {
  public static inline var FORMAT = "materia.viewport-state";

  /** Readable JSON with named fields, so a copied state can be read and tweaked by hand. */
  public static function encode(state:ViewportState):String {
    var out:Dynamic = {
      format: state.format,
      target: state.target,
      yaw: state.yaw,
      pitch: state.pitch,
      distance: state.distance,
      width: state.width,
      height: state.height,
      inspectionZoom: state.inspectionZoom
    };
    if (state.project != null) out.project = state.project;
    if (state.variant != null) out.variant = state.variant;
    if (state.document != null) out.document = state.document;
    var p = state.presentation;
    if (p != null) {
      var cursor:Dynamic = {view: p.view, explode: p.explode, step: p.step, progress: p.progress};
      if (p.procedure != null) cursor.procedure = p.procedure;
      out.presentation = cursor;
    }
    return haxe.Json.stringify(out, null, "  ");
  }

  public static function decode(source:String):ViewportState {
    var data:Dynamic = haxe.Json.parse(source);
    if (data == null || Reflect.field(data, "format") != FORMAT) throw 'Viewport state format must be $FORMAT';
    var target:Array<Dynamic> = Reflect.field(data, "target");
    if (target == null || target.length != 3) throw "Viewport state target needs three coordinates";
    var state:ViewportState = {
      format: FORMAT,
      target: [for (value in target) number(value, "target")],
      yaw: number(Reflect.field(data, "yaw"), "yaw"),
      pitch: number(Reflect.field(data, "pitch"), "pitch"),
      distance: number(Reflect.field(data, "distance"), "distance"),
      width: number(Reflect.field(data, "width"), "width"),
      height: number(Reflect.field(data, "height"), "height"),
      inspectionZoom: number(Reflect.field(data, "inspectionZoom"), "inspectionZoom")
    };
    if (!(state.distance > 0.0) || !(state.width >= 1.0) || !(state.height >= 1.0))
      throw "Viewport state needs a positive distance and size";
    var project = string(data, "project"), variant = string(data, "variant"), document = string(data, "document");
    if (project != null) state.project = project;
    if (variant != null) state.variant = variant;
    if (document != null) state.document = document;
    var cursor:Dynamic = Reflect.field(data, "presentation");
    if (cursor != null) {
      var view = string(cursor, "view");
      if (view == null) throw "Viewport state presentation needs a view";
      var recorded:ViewportPresentationState = {
        view: view,
        explode: number(Reflect.field(cursor, "explode"), "presentation.explode"),
        step: Std.int(number(Reflect.field(cursor, "step"), "presentation.step")),
        progress: number(Reflect.field(cursor, "progress"), "presentation.progress")
      };
      var procedure = string(cursor, "procedure");
      if (procedure != null) recorded.procedure = procedure;
      state.presentation = recorded;
    }
    return state;
  }

  static function number(value:Dynamic, name:String):Float {
    if (!Std.isOfType(value, Float) && !Std.isOfType(value, Int)) throw 'Viewport state $name must be a number';
    var result:Float = value;
    if (!Math.isFinite(result)) throw 'Viewport state $name must be finite';
    return result;
  }

  static function string(data:Dynamic, name:String):Null<String> {
    var value:Dynamic = Reflect.field(data, name);
    if (value == null) return null;
    if (!Std.isOfType(value, String)) throw 'Viewport state $name must be a string';
    return value;
  }
}
