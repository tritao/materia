package machinekit.robotics;

import machinekit.assembly.MachineAssembly;

/** Selected purchased harness for the two PGC-140-50 concept.
    Expert provides an external-power entry; DH wiring compatibility and installation
    remain unqualified. Selection is not a current rating or a collision certificate. */
class DhDualGripperServices {
    public static function adapter():FairinoDualToolAdapter return new FairinoDualToolAdapter("Expert");

    /** Inventory-only until a real installation pose/routing is defined.
        Unknown mass intentionally avoids treating short illustrative CAD leads as stock mass. */
    public static function addPurchases(assembly:MachineAssembly):Void {
        assembly.addBomItem(adapter().bom);
    }
}
