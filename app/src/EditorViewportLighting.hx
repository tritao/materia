package app;

import haxeon.editor.ViewportLook;

/** CAD inspection lighting; presentation changes do not edit the parts' authored materials. */
class EditorViewportLighting {
  public static function intensities(preset:Int):Array<Float> return preset==ViewportLook.Studio
    ? [0.95,0.16,0.30] : ViewportLook.intensities(preset);
  public static function sky(preset:Int):Array<Float> return preset==ViewportLook.Studio
    ? [0.28,0.30,0.33] : ViewportLook.sky(preset);
  public static function ground(preset:Int):Array<Float> return preset==ViewportLook.Studio
    ? [0.07,0.08,0.10] : ViewportLook.ground(preset);
  public static function directions():Array<Array<Float>> return [
    [0.60,-0.60,-0.53], // Key from above-left; avoids a frontal wash of the recess walls.
    [-0.65,-0.15,-0.74],
    [-0.45,0.75,0.48]
  ];
  public static function detail(preset:Int):Array<Float> return preset==ViewportLook.Soft
    ? [0.025,0.50,0.08,0.48] : [0.025,0.75,0.08,0.40];
}
