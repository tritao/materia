package tests;

import haxeon.ui.core.BuildContext;
import haxeon.ui.core.RenderNode;
import haxeon.ui.core.View;

/** A view made during the build, for editor helpers that measure with the BuildContext. */
class ContextView implements View {
  final make:BuildContext -> View;

  public function new(make:BuildContext -> View) this.make = make;

  public function build(context:BuildContext):RenderNode return make(context).build(context);
}
