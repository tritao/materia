package machinekit.component;

/**
 * How MachineKit's built-in recipes are filed for catalogues and part pickers: a category within each package
 * (the id's second segment) and a kind. Categories are listed here in display order. A recipe added to
 * `MachineKitComponents` without a row here fails RecipeContractTests.
 */
class ComponentClassification {
	public static function apply(registry:ComponentRegistry):Void {
		// assembly
		file(registry, "machinekit.assembly.carriage", "Axis modules", ComponentKind.MachinePart);
		file(registry, "machinekit.assembly.sfu20-axis-part", "Axis modules", ComponentKind.MachinePart);
		file(registry, "machinekit.assembly.twin-rail-screw-part", "Axis modules", ComponentKind.MachinePart);
		file(registry, "machinekit.assembly.hzmotion-drawing-component", "Axis modules", ComponentKind.Reference);
		file(registry, "machinekit.assembly.hzmotion-supplier-component", "Axis modules", ComponentKind.Supplier);
		file(registry, "machinekit.assembly.nema-mount-plate", "Mounts and couplings", ComponentKind.Generic);
		file(registry, "machinekit.assembly.rbk-motor-mount", "Mounts and couplings", ComponentKind.MachinePart);
		file(registry, "machinekit.assembly.screw-axis-coupling", "Mounts and couplings", ComponentKind.Generic);
		// control
		file(registry, "machinekit.control.cnc-controller", "Controllers and cabinets", ComponentKind.Generic);
		file(registry, "machinekit.control.control-cabinet", "Controllers and cabinets", ComponentKind.Generic);
		file(registry, "machinekit.control.robot-controller", "Controllers and cabinets", ComponentKind.Generic);
		file(registry, "machinekit.control.emergency-stop-button", "Safety", ComponentKind.Generic);
		file(registry, "machinekit.control.safety-relay", "Safety", ComponentKind.Generic);
		file(registry, "machinekit.control.digital-sensor", "Sensors and inputs", ComponentKind.Generic);
		file(registry, "machinekit.control.thermistor-input", "Sensors and inputs", ComponentKind.Generic);
		// enclosure
		file(registry, "machinekit.enclosure.panel", "Panels", ComponentKind.Generic);
		file(registry, "machinekit.enclosure.cut-panel", "Panels", ComponentKind.Generic);
		file(registry, "machinekit.enclosure.tray-toe-clamp", "Fixings", ComponentKind.Generic);
		// hobbing
		file(registry, "machinekit.hobbing.hob", "Cutting tools", ComponentKind.Generic);
		file(registry, "machinekit.hobbing.hob-arbor", "Cutting tools", ComponentKind.Reference);
		file(registry, "machinekit.hobbing.expanding-mandrel", "Workholding", ComponentKind.Reference);
		file(registry, "machinekit.hobbing.mandrel-sleeve", "Workholding", ComponentKind.MachinePart);
		file(registry, "machinekit.hobbing.rotary-clamp", "Workholding", ComponentKind.Reference);
		file(registry, "machinekit.hobbing.spindle-housing", "Spindle and supports", ComponentKind.Reference);
		file(registry, "machinekit.hobbing.spindle-part", "Spindle and supports", ComponentKind.MachinePart);
		file(registry, "machinekit.hobbing.outboard-support", "Spindle and supports", ComponentKind.Reference);
		file(registry, "machinekit.hobbing.floating-radial-bearing", "Spindle and supports", ComponentKind.MachinePart);
		file(registry, "machinekit.hobbing.table-housing", "Rotary table", ComponentKind.Reference);
		file(registry, "machinekit.hobbing.table-platen", "Rotary table", ComponentKind.Reference);
		file(registry, "machinekit.hobbing.hobber-bracket", "Structure", ComponentKind.MachinePart);
		file(registry, "machinekit.hobbing.saddle", "Structure", ComponentKind.MachinePart);
		file(registry, "machinekit.hobbing.swivel-housing", "Structure", ComponentKind.Reference);
		file(registry, "machinekit.hobbing.flexspline-cup-blank", "Workpieces", ComponentKind.MachinePart);
		// milling
		file(registry, "machinekit.milling.casting", "Structure", ComponentKind.MachinePart);
		file(registry, "machinekit.milling.cross-saddle", "Structure", ComponentKind.MachinePart);
		file(registry, "machinekit.milling.cross-table", "Structure", ComponentKind.MachinePart);
		file(registry, "machinekit.milling.z-head", "Structure", ComponentKind.MachinePart);
		file(registry, "machinekit.milling.z-carriage", "Structure", ComponentKind.MachinePart);
		file(registry, "machinekit.milling.panel", "Structure", ComponentKind.MachinePart);
		file(registry, "machinekit.milling.support-tray", "Structure", ComponentKind.MachinePart);
		file(registry, "machinekit.milling.rke92-reference-part", "Structure", ComponentKind.Reference);
		file(registry, "machinekit.milling.zpp-supplier-member", "Structure", ComponentKind.Supplier);
		file(registry, "machinekit.milling.spindle-cartridge", "Spindle", ComponentKind.MachinePart);
		file(registry, "machinekit.milling.spindle-motor", "Spindle", ComponentKind.MachinePart);
		file(registry, "machinekit.milling.spindle-motor-mount", "Spindle", ComponentKind.MachinePart);
		file(registry, "machinekit.milling.spindle-motor-support", "Spindle", ComponentKind.MachinePart);
		file(registry, "machinekit.milling.bt30-cartridge-study", "Spindle", ComponentKind.Reference);
		file(registry, "machinekit.milling.bt30-spindle-motor-study", "Spindle", ComponentKind.Reference);
		file(registry, "machinekit.milling.spindle-drive-study-part", "Spindle", ComponentKind.Reference);
		file(registry, "machinekit.milling.spindle-drive-study-guard", "Spindle", ComponentKind.Reference);
		file(registry, "machinekit.milling.power-drawbar-study-part", "Spindle", ComponentKind.Reference);
		file(registry, "machinekit.milling.ball-nut-bracket", "Drives and mounts", ComponentKind.MachinePart);
		file(registry, "machinekit.milling.belt-motor-plate", "Drives and mounts", ComponentKind.MachinePart);
		file(registry, "machinekit.milling.hzmotion-y-drive-seat", "Drives and mounts", ComponentKind.MachinePart);
		file(registry, "machinekit.milling.screw-pedestal", "Drives and mounts", ComponentKind.MachinePart);
		file(registry, "machinekit.milling.servo-mount", "Drives and mounts", ComponentKind.MachinePart);
		file(registry, "machinekit.milling.y-nut-bridge", "Drives and mounts", ComponentKind.MachinePart);
		file(registry, "machinekit.milling.y-nut-mount", "Drives and mounts", ComponentKind.MachinePart);
		file(registry, "machinekit.milling.z-bearing-plate", "Drives and mounts", ComponentKind.MachinePart);
		file(registry, "machinekit.milling.z-drive-adapter", "Drives and mounts", ComponentKind.MachinePart);
		file(registry, "machinekit.milling.z-motor-stand", "Drives and mounts", ComponentKind.MachinePart);
		file(registry, "machinekit.milling.drive-service-hatch", "Guards and covers", ComponentKind.MachinePart);
		file(registry, "machinekit.milling.motor-cover", "Guards and covers", ComponentKind.MachinePart);
		file(registry, "machinekit.milling.spindle-belt-guard", "Guards and covers", ComponentKind.MachinePart);
		file(registry, "machinekit.milling.z-coupling-guard", "Guards and covers", ComponentKind.MachinePart);
		file(registry, "machinekit.milling.er20-holder", "Tooling and workholding", ComponentKind.Generic);
		file(registry, "machinekit.milling.flat-end-mill", "Tooling and workholding", ComponentKind.Generic);
		file(registry, "machinekit.milling.fixture-adapter", "Tooling and workholding", ComponentKind.MachinePart);
		// motion
		file(registry, "machinekit.motion.linear-rail", "Linear guides", ComponentKind.Generic);
		file(registry, "machinekit.motion.linear-rail-block", "Linear guides", ComponentKind.Generic);
		file(registry, "machinekit.motion.hg-rail", "Linear guides", ComponentKind.Generic);
		file(registry, "machinekit.motion.linear-bearing", "Linear guides", ComponentKind.Generic);
		file(registry, "machinekit.motion.supplier-hgr25-rail", "Linear guides", ComponentKind.Supplier);
		file(registry, "machinekit.motion.supplier-hgh-block", "Linear guides", ComponentKind.Supplier);
		file(registry, "machinekit.motion.hg-block-reference", "Linear guides", ComponentKind.Reference);
		file(registry, "machinekit.motion.hgh15ca-reference", "Linear guides", ComponentKind.Reference);
		file(registry, "machinekit.motion.ball-screw", "Screws and nuts", ComponentKind.Generic);
		file(registry, "machinekit.motion.ball-nut", "Screws and nuts", ComponentKind.Generic);
		file(registry, "machinekit.motion.lead-screw", "Screws and nuts", ComponentKind.Generic);
		file(registry, "machinekit.motion.lead-screw-nut", "Screws and nuts", ComponentKind.Generic);
		file(registry, "machinekit.motion.sfu-nut", "Screws and nuts", ComponentKind.Generic);
		file(registry, "machinekit.motion.screw-support-unit", "Screws and nuts", ComponentKind.Generic);
		file(registry, "machinekit.motion.ballscrew-retainer", "Screws and nuts", ComponentKind.Generic);
		file(registry, "machinekit.motion.dsg-nut-housing", "Screws and nuts", ComponentKind.Supplier);
		file(registry, "machinekit.motion.vallder-sfu1605-screw", "Screws and nuts", ComponentKind.Supplier);
		file(registry, "machinekit.motion.vallder-sfu1605-nut", "Screws and nuts", ComponentKind.Supplier);
		file(registry, "machinekit.motion.vallder-bf12-support", "Screws and nuts", ComponentKind.Supplier);
		file(registry, "machinekit.motion.vallder-screw-support", "Screws and nuts", ComponentKind.Supplier);
		file(registry, "machinekit.motion.sfu-drawing-screw", "Screws and nuts", ComponentKind.Reference);
		file(registry, "machinekit.motion.hzmotion-bk12-screw-end-study", "Screws and nuts", ComponentKind.Reference);
		file(registry, "machinekit.motion.flange-bearing-housing", "Bearings", ComponentKind.Generic);
		file(registry, "machinekit.motion.pillow-block", "Bearings", ComponentKind.Generic);
		file(registry, "machinekit.motion.angular-contact-bearing", "Bearings", ComponentKind.Reference);
		file(registry, "machinekit.motion.crossed-roller-bearing", "Bearings", ComponentKind.Reference);
		file(registry, "machinekit.motion.nema-stepper", "Motors and gearboxes", ComponentKind.Generic);
		file(registry, "machinekit.motion.generic-nema-stepper", "Motors and gearboxes", ComponentKind.Generic);
		file(registry, "machinekit.motion.servo-motor", "Motors and gearboxes", ComponentKind.Generic);
		file(registry, "machinekit.motion.axis-servo-motor", "Motors and gearboxes", ComponentKind.Generic);
		file(registry, "machinekit.motion.brake-servo", "Motors and gearboxes", ComponentKind.Generic);
		file(registry, "machinekit.motion.gearbox", "Motors and gearboxes", ComponentKind.Generic);
		file(registry, "machinekit.motion.planetary-reducer", "Motors and gearboxes", ComponentKind.Generic);
		file(registry, "machinekit.motion.leadshine-servo", "Motors and gearboxes", ComponentKind.Supplier);
		file(registry, "machinekit.motion.purchased-rotary-part", "Motors and gearboxes", ComponentKind.Supplier);
		file(registry, "machinekit.motion.torque-motor", "Motors and gearboxes", ComponentKind.Reference);
		file(registry, "machinekit.motion.motor-driver", "Drives and power", ComponentKind.Generic);
		file(registry, "machinekit.motion.power-supply", "Drives and power", ComponentKind.Generic);
		file(registry, "machinekit.motion.limit-switch", "Sensors", ComponentKind.Generic);
		file(registry, "machinekit.motion.proximity-switch", "Sensors", ComponentKind.Generic);
		file(registry, "machinekit.motion.shaft-encoder", "Sensors", ComponentKind.Generic);
		file(registry, "machinekit.motion.linear-scale", "Sensors", ComponentKind.Generic);
		file(registry, "machinekit.motion.rotary-encoder-ring", "Sensors", ComponentKind.Reference);
		file(registry, "machinekit.motion.shaft-coupling", "Couplings, shafts and brakes", ComponentKind.Generic);
		file(registry, "machinekit.motion.stepped-shaft", "Couplings, shafts and brakes", ComponentKind.Generic);
		file(registry, "machinekit.motion.shaft-brake-part", "Couplings, shafts and brakes", ComponentKind.Generic);
		file(registry, "machinekit.motion.caster-wheel", "Wheels", ComponentKind.Generic);
		file(registry, "machinekit.motion.drive-wheel", "Wheels", ComponentKind.Generic);
		file(registry, "machinekit.motion.cable-chain", "Cable management", ComponentKind.Generic);
		// pneumatic
		file(registry, "machinekit.pneumatic.air-supply", "Air supply and valves", ComponentKind.Generic);
		file(registry, "machinekit.pneumatic.solenoid-valve", "Air supply and valves", ComponentKind.Generic);
		file(registry, "machinekit.pneumatic.manifold", "Air supply and valves", ComponentKind.Generic);
		file(registry, "machinekit.pneumatic.vacuum-control-valve", "Air supply and valves", ComponentKind.Generic);
		file(registry, "machinekit.pneumatic.cylinder", "Cylinders", ComponentKind.Generic);
		file(registry, "machinekit.pneumatic.cylinder-rod", "Cylinders", ComponentKind.Generic);
		file(registry, "machinekit.pneumatic.suction-cup", "Vacuum", ComponentKind.Generic);
		file(registry, "machinekit.pneumatic.vacuum-generator", "Vacuum", ComponentKind.Generic);
		file(registry, "machinekit.pneumatic.vacuum-pressure-sensor", "Vacuum", ComponentKind.Generic);
		file(registry, "machinekit.pneumatic.schmalz-suction-cup", "Vacuum", ComponentKind.Supplier);
		file(registry, "machinekit.pneumatic.schmalz-vacuum-generator", "Vacuum", ComponentKind.Supplier);
		file(registry, "machinekit.pneumatic.routed-hose", "Hoses and fittings", ComponentKind.Generic);
		file(registry, "machinekit.pneumatic.schmalz-vacuum-hose", "Hoses and fittings", ComponentKind.Supplier);
		file(registry, "machinekit.pneumatic.schmalz-push-in-fitting", "Hoses and fittings", ComponentKind.Supplier);
		// power
		file(registry, "machinekit.power.vfd", "Spindle drives", ComponentKind.Generic);
		file(registry, "machinekit.power.brake-resistor", "Spindle drives", ComponentKind.Generic);
		file(registry, "machinekit.power.three-phase-rectifier", "Spindle drives", ComponentKind.Generic);
		file(registry, "machinekit.power.contactor", "Switching and protection", ComponentKind.Generic);
		file(registry, "machinekit.power.circuit-breaker", "Switching and protection", ComponentKind.Generic);
		// pressbrake
		file(registry, "machinekit.pressbrake.hydraulic-cylinder", "Hydraulics", ComponentKind.Generic);
		file(registry, "machinekit.pressbrake.hydraulic-cylinder-rod", "Hydraulics", ComponentKind.Generic);
		file(registry, "machinekit.pressbrake.hydraulic-manifold", "Hydraulics", ComponentKind.Generic);
		file(registry, "machinekit.pressbrake.hydraulic-supply", "Hydraulics", ComponentKind.Generic);
		file(registry, "machinekit.pressbrake.hydraulic-valve", "Hydraulics", ComponentKind.Generic);
		file(registry, "machinekit.pressbrake.tool-clamp", "Tooling", ComponentKind.Generic);
		file(registry, "machinekit.pressbrake.tool-segment", "Tooling", ComponentKind.Generic);
		file(registry, "machinekit.pressbrake.gauge-finger", "Backgauge", ComponentKind.Generic);
		// printing
		file(registry, "machinekit.printing.extruder", "Hotend and extrusion", ComponentKind.Generic);
		file(registry, "machinekit.printing.hotend", "Hotend and extrusion", ComponentKind.Generic);
		file(registry, "machinekit.printing.nozzle", "Hotend and extrusion", ComponentKind.Generic);
		file(registry, "machinekit.printing.heater-cartridge", "Hotend and extrusion", ComponentKind.Generic);
		file(registry, "machinekit.printing.thermistor", "Hotend and extrusion", ComponentKind.Generic);
		file(registry, "machinekit.printing.heated-bed", "Bed and cooling", ComponentKind.Generic);
		file(registry, "machinekit.printing.fan", "Bed and cooling", ComponentKind.Generic);
		// robotics
		file(registry, "machinekit.robotics.arm-joint", "Arm modules", ComponentKind.Generic);
		file(registry, "machinekit.robotics.geared-arm-joint", "Arm modules", ComponentKind.Generic);
		file(registry, "machinekit.robotics.cobot-joint", "Arm modules", ComponentKind.Generic);
		file(registry, "machinekit.robotics.arm-link", "Arm modules", ComponentKind.Generic);
		file(registry, "machinekit.robotics.arm-wrist-fork", "Arm modules", ComponentKind.Generic);
		file(registry, "machinekit.robotics.pedestal", "Arm modules", ComponentKind.Generic);
		file(registry, "machinekit.robotics.robot-flange", "Flanges and adapters", ComponentKind.Generic);
		file(registry, "machinekit.robotics.end-effector-plate", "Flanges and adapters", ComponentKind.Generic);
		file(registry, "machinekit.robotics.frame-bar", "Flanges and adapters", ComponentKind.Generic);
		file(registry, "machinekit.robotics.fairino-dual-tool-adapter", "Flanges and adapters", ComponentKind.Supplier);
		file(registry, "machinekit.robotics.parallel-gripper-body", "Grippers", ComponentKind.Generic);
		file(registry, "machinekit.robotics.parallel-gripper-jaw", "Grippers", ComponentKind.Generic);
		file(registry, "machinekit.robotics.parallel-gripper-finger", "Grippers", ComponentKind.Generic);
		file(registry, "machinekit.robotics.offset-gripper-finger", "Grippers", ComponentKind.Generic);
		file(registry, "machinekit.robotics.tool-changer-master", "Tool changers", ComponentKind.Generic);
		file(registry, "machinekit.robotics.tool-changer-tool", "Tool changers", ComponentKind.Generic);
		file(registry, "machinekit.robotics.tool-service-module", "Tool changers", ComponentKind.Generic);
		file(registry, "machinekit.robotics.schmalz-sxt-master", "Tool changers", ComponentKind.Supplier);
		file(registry, "machinekit.robotics.schmalz-sxt-tool", "Tool changers", ComponentKind.Supplier);
		// routing
		file(registry, "machinekit.routing.atc-spindle", "Spindle and tool change", ComponentKind.Generic);
		file(registry, "machinekit.routing.tool-holder", "Spindle and tool change", ComponentKind.Generic);
		file(registry, "machinekit.routing.collet", "Spindle and tool change", ComponentKind.Generic);
		file(registry, "machinekit.routing.tool-rack", "Spindle and tool change", ComponentKind.Generic);
		file(registry, "machinekit.routing.tool-fork", "Spindle and tool change", ComponentKind.Generic);
		file(registry, "machinekit.routing.tool-length-setter", "Spindle and tool change", ComponentKind.Generic);
		file(registry, "machinekit.routing.vacuum-table", "Vacuum workholding", ComponentKind.Generic);
		file(registry, "machinekit.routing.vacuum-zone-valve", "Vacuum workholding", ComponentKind.Generic);
		file(registry, "machinekit.routing.vacuum-blower", "Vacuum workholding", ComponentKind.Generic);
		file(registry, "machinekit.routing.spoilboard", "Vacuum workholding", ComponentKind.Generic);
		file(registry, "machinekit.routing.dust-shoe", "Dust collection", ComponentKind.Generic);
		// standard
		file(registry, "machinekit.standard.socket-head-cap-screw", "Fasteners", ComponentKind.Generic);
		file(registry, "machinekit.standard.hex-bolt", "Fasteners", ComponentKind.Generic);
		file(registry, "machinekit.standard.hex-nut", "Fasteners", ComponentKind.Generic);
		file(registry, "machinekit.standard.fine-pitch-nut", "Fasteners", ComponentKind.Generic);
		file(registry, "machinekit.standard.flat-washer", "Fasteners", ComponentKind.Generic);
		file(registry, "machinekit.standard.socket-set-screw", "Fasteners", ComponentKind.Generic);
		file(registry, "machinekit.standard.deep-groove-bearing", "Bearings and bushings", ComponentKind.Generic);
		file(registry, "machinekit.standard.bushing", "Bearings and bushings", ComponentKind.Generic);
		file(registry, "machinekit.standard.shaft-collar", "Shaft retention", ComponentKind.Generic);
		file(registry, "machinekit.standard.retaining-ring", "Shaft retention", ComponentKind.Generic);
		file(registry, "machinekit.standard.parallel-key", "Shaft retention", ComponentKind.Generic);
		// structural
		file(registry, "machinekit.structural.tslot-member", "Profiles and tubes", ComponentKind.Generic);
		file(registry, "machinekit.structural.rect-tube-member", "Profiles and tubes", ComponentKind.Generic);
		file(registry, "machinekit.structural.drawing-profile-member", "Profiles and tubes", ComponentKind.Reference);
		file(registry, "machinekit.structural.block-bracket", "Brackets and fixings", ComponentKind.Generic);
		file(registry, "machinekit.structural.slide-in-slot-nut", "Brackets and fixings", ComponentKind.Generic);
		file(registry, "machinekit.structural.leveling-foot", "Brackets and fixings", ComponentKind.Generic);
		file(registry, "machinekit.structural.mineral-casting", "Castings", ComponentKind.Generic);
		file(registry, "machinekit.structural.cast-in-steel-insert", "Castings", ComponentKind.Generic);
		// tooling
		file(registry, "machinekit.tooling.panel", "Casting tooling", ComponentKind.Generic);
		file(registry, "machinekit.tooling.block", "Casting tooling", ComponentKind.Generic);
		file(registry, "machinekit.tooling.volume", "Casting tooling", ComponentKind.Generic);
		file(registry, "machinekit.tooling.plain-pin", "Casting tooling", ComponentKind.Generic);
		file(registry, "machinekit.tooling.source-casting", "Casting tooling", ComponentKind.Generic);
		// transmission
		file(registry, "machinekit.transmission.timing-belt", "Belts and pulleys", ComponentKind.Generic);
		file(registry, "machinekit.transmission.timing-pulley", "Belts and pulleys", ComponentKind.Generic);
		file(registry, "machinekit.transmission.custom-timing-pulley", "Belts and pulleys", ComponentKind.Generic);
		file(registry, "machinekit.transmission.spur-gear", "Gears and racks", ComponentKind.Generic);
		file(registry, "machinekit.transmission.involute-rack", "Gears and racks", ComponentKind.Generic);
		file(registry, "machinekit.transmission.helical-pinion", "Gears and racks", ComponentKind.Generic);
		file(registry, "machinekit.transmission.helical-rack", "Gears and racks", ComponentKind.Generic);
		file(registry, "machinekit.transmission.preloaded-pinion-mount", "Gears and racks", ComponentKind.Generic);
		file(registry, "machinekit.transmission.sprocket", "Chains and sprockets", ComponentKind.Generic);
		file(registry, "machinekit.transmission.generic-sprocket", "Chains and sprockets", ComponentKind.Generic);
	}

	static function file(registry:ComponentRegistry, id:String, category:String, kind:ComponentKind):Void
		registry.classify(id, category, kind);
}
