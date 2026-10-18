package app.editor;

import haxeon.ui.core.*;
import haxeon.ui.widgets.collections.TreeView;

/** Space belongs to the focused tree, not to the application's global shortcuts. */
class HierarchyTreeView implements View {
  final tree:TreeView;
  final toggle:Void->Void;
  public function new(tree:TreeView,toggle:Void->Void){this.tree=tree;this.toggle=toggle;}
  public function build(context:BuildContext):RenderNode {
    var node=tree.build(context);
    node.on(UiEventKind.KeyDown, function(event) handleKey(event,toggle));
    node.on(UiEventKind.KeyRepeat, function(event) {
      if(event.key==UiKey.Space && event.modifiers==0){event.preventDefault();event.stopPropagation();}
    });
    return node;
  }
  public static function handleKey(event:UiEvent,toggle:Void->Void):Void {
    if(event.key!=UiKey.Space || event.modifiers!=0 || event.kind!=UiEventKind.KeyDown) return;
    event.preventDefault();event.stopPropagation();toggle();
  }
}
