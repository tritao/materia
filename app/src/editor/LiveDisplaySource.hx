package app.editor;

import haxe.io.Bytes;

/** What fills a display in the scene while a simulation runs: a picture that changes. */
interface LiveDisplaySource {
  public function width():Int;
  public function height():Int;

  /** RGBA8, `width() * height() * 4` bytes, when the picture changed since the last call, else null. `nowMs` is the editor clock. */
  public function advance(nowMs:Int):Null<Bytes>;
}
