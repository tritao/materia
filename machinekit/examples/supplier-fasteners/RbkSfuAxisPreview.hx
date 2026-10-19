import machinekit.assembly.MachineAssembly;
import machinekit.assembly.AssemblyPreview;
import machinekit.assembly.RbkMotorMount;
import materia.project.SceneArtifact;

/** Openable Materia artifact retaining the worked axis's components and joints. */
class RbkSfuAxisPreview {
    public static function main():Void {
        var root=Sys.getEnv("MATERIA_ROOT");if(root==null) root=Sys.getCwd();
        var directory=root+"/build/suppliers/vallder/";
        RbkMotorMount.setSupplierDirectory(directory);
        var assembly=MachineAssembly.decode(sys.io.File.getContent(directory+"rbk-sfu1605-axis.assembly.json"),RbkSfuAxis.registry());
        var scene=AssemblyPreview.scene(assembly,"vallder-rbk-sfu1605-axis");
        var definition=scene.assemblyDefinition;
        if(definition==null) throw "Axis preview lost its assembly definition";
        // Guide closures constrain the carriage but do not command its travel.
        // Keep the manual design coordinate independent; screwTurn follows its lead coupling.
        for(joint in definition.joints) if(joint.id=="travel") joint.driven=true;
        var state=new cadkit.modeling.AssemblyState(definition);
        if(state.dependentJoints().indexOf("travel")>=0) throw "Travel must remain an input";
        for(position in [0.0,100,200]) {
            state.setJoint("travel",position);
            state.forwardKinematics();
            for(residual in state.closureResiduals())
                if(residual.position>1e-6 || residual.axis>1e-9 || residual.rotation>1e-9)
                    throw "Axis closure changed during travel";
            if(Math.abs(state.joint("screwTurn")+position*2*Math.PI/5)>1e-8) throw "Lost screw lead coupling";
        }
        scene.assemblyState=new cadkit.modeling.AssemblyState(definition).record();
        var output=directory+"rbk-sfu1605-axis.mtrg";
        sys.io.File.saveBytes(output,SceneArtifact.encode(scene));
        Sys.println("Exported Materia assembly: "+output);
    }
}
