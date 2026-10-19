package machinekit.hobbing;

import machinekit.component.ComponentRegistry;

class HobbingComponents {
  public static function register(registry:ComponentRegistry):Void {
    registry.register(FloatingRadialBearing.recipeType());
    LegacyFoundationRecipes.register(registry);
    registry.register(HobberBracket.recipeType());
    registry.register(HobberSaddle.recipeType());
    registry.register(HobArbor.recipeType());registry.register(OutboardSupport.recipeType());
    registry.register(Hob.recipeType());registry.register(ExpandingMandrel.recipeType());
    registry.register(RotaryClamp.recipeType());registry.register(TablePlaten.recipeType());
    registry.register(TableHousing.recipeType());registry.register(SpindleHousing.recipeType());registry.register(HobSpindlePart.recipeType());
    registry.register(SwivelHousing.recipeType());

  }
}
