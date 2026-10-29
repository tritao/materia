package app.editor;

import haxe.io.Bytes;

/** What fills a display in the scene while a simulation runs: a picture that changes. */
interface LiveDisplaySource {
  public function width():Int;
  public function height():Int;

  /**
   * RGBA8, `width() * height() * 4` bytes, when the picture changed since the last call, else null. `nowMs` is the editor
   * clock; `controls` are the model's joints, which the source reads as the device's switches and wheels.
   */
  public function advance(nowMs:Int, controls:LiveControls):Null<Bytes>;
}
