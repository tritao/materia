package machinekit.gantry;

import machinekit.motion.CableChain;
import machinekit.component.MachineComponent;
import machinekit.component.ComponentDetail;
import materia.assembly.AssemblyFrames as F;

/** Initial routed chain body. The machine owns the axis coordinate and asks
 * the source chain for posed geometry when updating its preview. */
class GantryCableChain extends MachineComponent {
    public final chain:CableChain;
    public function new(chain:CableChain) {
        super(chain.designation+"-ROUTED",chain.description,chain.materialSpec(),true);
        this.chain=chain;
        addConnector("fixed",Mount,F.identity());addConnector("moving",Mount,chain.movingFrame(0));
    }
    override public function hasGeometry():Bool return true;
    override public function geometry(detail:ComponentDetail=Preview):cadkit.modeling.Part return chain.geometryAt(0,detail);
}
