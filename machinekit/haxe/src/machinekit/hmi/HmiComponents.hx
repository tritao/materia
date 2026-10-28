package machinekit.hmi;

import machinekit.component.ComponentRegistry;

/** The pendant's component recipes. */
class HmiComponents {
  public static function register(registry:ComponentRegistry):Void {
    registry.register(HandWheel.recipeType());
    registry.register(EmergencyStop.recipeType());
    registry.register(RotarySwitch.recipeType());
    registry.register(PushButton.recipeType());
    registry.register(OledModule.recipeType());
  }
}
