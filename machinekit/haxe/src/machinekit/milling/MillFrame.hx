package machinekit.milling;

import machinekit.assembly.MachineAssembly;
import machinekit.structural.MineralFoundation;

/** Machine-facing frame and functional datums. Casting recipes and foundation
 * construction belong to the machine example that supplies this assembly. */
class MillFrame extends MachineAssembly {
	public final layout:MillFrameLayout;
	public final base:MineralFoundation;
	public final column:MineralFoundation;
	public final zAxis:Null<MillZAxisLayout>;
	public final xyAxis:Null<MillXYStageLayout>;

	function new(layout:MillFrameLayout,base:MineralFoundation,column:MineralFoundation,source:MachineAssembly,
		?zAxis:MillZAxisLayout,?xyAxis:MillXYStageLayout) {
		super();
		if (layout==null || base==null || column==null || source==null)
			throw "Mill frame needs authored foundations and their assembly";
		if (zAxis!=null && zAxis.frame!=layout) throw "Frame and Z datums must share one layout";
		if (xyAxis!=null && xyAxis.frame!=layout) throw "Frame and XY datums must share one layout";
		this.layout=layout;
		this.base=base;
		this.column=column;
		this.zAxis=zAxis;
		this.xyAxis=xyAxis;
		source.copyInto(this);
	}
}
