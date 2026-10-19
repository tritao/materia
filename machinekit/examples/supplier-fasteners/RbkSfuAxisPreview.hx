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
        // Look from the component side (+Y), rather than through the base plate.
        scene.project={dynamicParts:[],motions:[],initialView:{yaw:Math.PI-0.9652516631899266,pitch:0.5639426413606289}};
        var output=directory+"rbk-sfu1605-axis.mtrg";
        var bytes=SceneArtifact.encode(scene);
        var restored=SceneArtifact.decode(bytes);
        var project=restored.project;
        if(project==null || project.initialView==null) throw "Axis preview lost its preferred view";
        var view=project.initialView;
        if(Math.abs(view.yaw-(Math.PI-0.9652516631899266))>1e-10 || Math.abs(view.pitch-0.5639426413606289)>1e-10)
            throw "Axis preview changed its preferred view";
        sys.io.File.saveBytes(output,bytes);
        Sys.println("Exported Materia assembly: "+output);
    }
}
