package machinekit.pendant;

import machinekit.component.ComponentRegistry;

/** The pendant's component recipes. */
class PendantComponents {
  public static function register(registry:ComponentRegistry):Void {
    registry.register(PendantHandWheel.recipeType());
    registry.register(PendantEmergencyStop.recipeType());
    registry.register(PendantRotarySwitch.recipeType());
    registry.register(PendantPushButton.recipeType());
    registry.register(PendantOledModule.recipeType());
  }
}
