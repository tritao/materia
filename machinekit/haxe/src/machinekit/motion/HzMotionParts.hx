package machinekit.motion;

import machinekit.assembly.HzMotionCatalog;
import machinekit.component.MachineComponent;

/** HZMotion's catalogue across the product kinds it spans: motor mounts, screw supports and adapter plates. */
class HzMotionParts {
	/** The component for HZMotion's `model`: drawn from its pinned STEP when `supplier` and reviewed, else nominally. */
	public static function of(model:String, supplier:Bool = false):MachineComponent {
		var reviewed = supplier && HzMotionCatalog.cadModels().indexOf(model) >= 0;
		return switch HzMotionCatalog.get(model).kind {
			case "mount": MotorMount.of(model, null, reviewed ? "hzmotion" : MotorMount.NOMINAL);
			case "support": ScrewSupportUnit.model(model, reviewed ? "hzmotion" : ScrewSupportUnit.NOMINAL);
			case "adapter": new MotorAdapterPlate(model);
			case kind: throw 'Unknown HZMotion product kind $kind';
		};
	}
}
