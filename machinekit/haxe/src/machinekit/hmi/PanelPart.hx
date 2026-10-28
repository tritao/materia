package machinekit.hmi;

import cadkit.modeling.Footprint;
import cadkit.modeling.FootprintGeometry;
/**
 * What a part on the panel says about itself to the layout. The part's frame has its origin at the
 * centre of the panel's front face, +z toward the operator: it stands `aboveFace()` out and reaches
 * `behindFace()` back from that face. Every number here is derived from the part's recipe, so the layout
 * and the part's solid cannot disagree.
 */
interface PanelPart {
  /** The opening the face plate needs for it. */
  function panelCutout():Footprint;
  /** The face area it occupies (knob, cap, flange), kept clear of other parts. */
  function panelKeepOut():Footprint;
  /** Extra round holes (studs), offsets from its centre. */
  function panelHoles():Array<{x:Float, y:Float, diameter:Float}>;
  function aboveFace():Float;
  function behindFace():Float;
  /** What it occupies behind the face, seen from the front. */
  function rearBody():Footprint;
  function massGrams():Float;
  /** The thickest panel it can be fixed to (its studs or thread allow); no limit unless the part says. */
  function maxPanelThickness():Float;
  /** "vendor-drawing", "listing", "measured" or "assumed", from the part's value basis. */
  function dataSource():String;
}
