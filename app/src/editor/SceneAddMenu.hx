package app.editor;

import haxeon.ui.core.CommandContext;
import haxeon.ui.core.CommandRegistry;
import haxeon.ui.widgets.overlays.MenuItem;

/** Creation and import choices; selected-object operations belong to the inspector/context menu. */
class SceneAddMenu {
  public static function items(commands:CommandRegistry, context:CommandContext, onSelected:Void->Void):Array<MenuItem> {
    var result:Array<MenuItem> = [MenuItem.section("geometry-heading", "Geometry")];
    function addKinds(sections:Array<String>, separator:Bool):Void {
      var first = true;
      for (provider in ObjectKindRegistry.all()) {
        if (sections.indexOf(provider.addSection()) < 0 || provider.addCommand() == "") continue;
        var commandId = provider.addCommand();
        var command = commands.get(commandId);
        if (command == null) continue;
        var label = provider.id() == "cad-part" ? "Empty part" : provider.addLabel();
        if (provider.addSection() == "Import") label = "Import " + label + "…";
        result.push(new MenuItem(commandId, label, function() {
          commands.executeContext(commandId, context);
          onSelected();
        }, command.isEnabled(context), null, separator && first));
        first = false;
      }
    }
    addKinds(["Primitive", "CAD"], false);
    addKinds(["Machining", "People"], true);
    addKinds(["Import"], true);
    return result;
  }
}
