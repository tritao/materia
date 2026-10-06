package app;

import haxeon.ui.docking.DockNode;
import haxeon.ui.icons.IconName;

/** Task-focused presets for the editor's dock layout and command surface. */
class EditorMode {
  public static final Design = new EditorMode("design", "Design", IconName.Cube);
  public static final Simulate = new EditorMode("simulate", "Simulate", IconName.Radar);

  public static function all():Array<EditorMode> return [Design, Simulate];

  public final id:String;
  public final label:String;
  public final icon:IconName;

  function new(id:String, label:String, icon:IconName) {
    this.id = id;
    this.label = label;
    this.icon = icon;
  }

  public function layout():DockNode
    return this == Simulate ? EditorWorkspaceLayout.simulateLayout() : EditorWorkspaceLayout.defaultLayout();
}
