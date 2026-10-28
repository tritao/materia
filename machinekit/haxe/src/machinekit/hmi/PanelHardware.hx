package machinekit.hmi;

import machinekit.component.MachineComponent;

/** A bought fastener or insert placed in a panel: its member id, and where its origin sits in panel coordinates. */
typedef PanelHardware = {id:String, component:MachineComponent, x:Float, y:Float, z:Float};
