package machinekit.routing;

import machinekit.component.ComponentRegistry;

/** Assumed router hardware recipes; supplier verification belongs to procurement. */
class RoutingComponents {
  public static function register(registry:ComponentRegistry):Void {
    registry.register(AtcSpindle.recipeType());
    registry.register(ToolHolder.recipeType());
    registry.register(Collet.recipeType());
    registry.register(ToolFork.recipeType());
    registry.register(ToolRack.recipeType());
    registry.register(ToolLengthSetter.recipeType());
    registry.register(DustShoe.recipeType());
    registry.register(VacuumTable.recipeType());
    registry.register(Spoilboard.recipeType());
    registry.register(VacuumBlower.recipeType());
    registry.register(VacuumZoneValve.recipeType());
  }
}
