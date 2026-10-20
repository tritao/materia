class FlexsplineHobberPreview {
  /** Scene defaults apply to direct artifact/project opens as well as saved view wrappers. */
  public static function mechanicalView(data:materia.project.SceneArtifact.SceneArtifactData):materia.project.SceneArtifact.SceneArtifactData {
    if(data.assemblyDefinition!=null)for(occurrence in data.assemblyDefinition.occurrences)
      if(occurrence.id=="electricalPanel"||occurrence.id=="powerSupply"||StringTools.endsWith(occurrence.id,"-driver"))occurrence.visible=false;
    return data;
  }
  /** Nominal full-depth reference setup, at the middle of the toothed face. */
  public static function inspectionState(machine:FlexsplineHobber):cadkit.modeling.AssemblyState {
    var model=new cadkit.modeling.AssemblyModel("mm");machine.addTo(model,"");
    var state=model.initialState("flexspline-hobber-detailed"),job=FlexsplineHobberCam.job(machine),pass=job.passes[job.passes.length-1];
    state.setJoint("X",pass.centreDistance);state.setJoint("Y",pass.shift);state.setJoint("Z",machine.spec.faceWidth/2);
    state.setJoint(FlexsplineHobber.A,-machine.hob.design.leadAngle);state.setJoint(FlexsplineHobber.B,0);state.setJoint(FlexsplineHobber.C,pass.tablePhase);
    state.forwardKinematics();return state;
  }
}
