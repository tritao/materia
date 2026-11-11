package hobber;

import machinekit.component.ComponentRegistry;
import machinekit.component.ComponentKind;
import machinekit.component.MachineKitComponents;

/** Application recipes keep their stable catalogue IDs while living with the machine. */
class HobberComponents {
  public static function register(registry:ComponentRegistry):Void {
    for(recipe in [HobberMadePart.recipeType(),HobberXCarriage.carriageType(),HobberDriveBar.barType(),HobberScaleSeat.seatType(),HobberZScaleArm.armType(),HobberCoolantPort.portType(),HobberChipTray.trayType(),HobberCarriageRib.recipeType(),HobberChipCover.recipeType(),
      HobberYPart.recipeType(),HobberZBellowsPart.recipeType()])
      if(!registry.has(recipe.id))registry.register(recipe);
    registry.classify(HobberMadePart.recipeType().id,"Structure",ComponentKind.MachinePart);
    registry.classify(HobberXCarriage.carriageType().id,"Structure",ComponentKind.MachinePart);
    registry.classify(HobberDriveBar.barType().id,"Structure",ComponentKind.MachinePart);
    registry.classify(HobberScaleSeat.seatType().id,"Structure",ComponentKind.MachinePart);
    registry.classify(HobberZScaleArm.armType().id,"Structure",ComponentKind.MachinePart);
    registry.classify(HobberCoolantPort.portType().id,"Structure",ComponentKind.MachinePart);
    registry.classify(HobberChipTray.trayType().id,"Structure",ComponentKind.MachinePart);
    registry.classify(HobberChipCover.recipeType().id,"Covers",ComponentKind.MachinePart);
    registry.classify(HobberZBellowsPart.recipeType().id,"Covers and wipers",ComponentKind.Supplier);
  }
  public static function registry():ComponentRegistry {
    var registry=MachineKitComponents.standard();
    register(registry);
    return registry;
  }
  public static function install():Void register(MachineKitComponents.defaultRegistry());
}
