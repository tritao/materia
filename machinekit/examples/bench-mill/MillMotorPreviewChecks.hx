import cadkit.parametric.Document;
import cadkit.parametric.DocumentCodec;
import cadkit.parametric.TypedProperty;
import cadkit.parametric.InstanceElement;
import machinekit.document.MachineKitDocuments;
import machinekit.document.MachineKitRecipes;
import machinekit.milling.MillSpindleDrivePart;
import materia.project.SceneArtifact;
import cadkit.modeling.AssemblyState;

/** Exercise the actual saved-document preview callback, rather than only a recipe factory. */
class MillMotorPreviewChecks {
	public static function main():Void {
		MachineKitRecipes.register();
		var c=MillMotorStudy.context(),document=new Document();
		var selected=MillMotorStudy.selection(c.frame,"ELM2M-2000LB130F-H",80,25);
		var definition=MachineKitDocuments.define(document,MillSpindleDrivePart.recipeType(),MillMotorStudy.selection(c.frame).values());
		// The editor marks definition defaults as user edits; unedited old source
		// defaults are intentionally replaced by fresh source during reconciliation.
		definition.setUserEditedTypedDefault("motor","ELM2M-2000LB130F-H");
		definition.setUserEditedTypedDefault("motorTeeth",80);
		definition.setUserEditedTypedDefault("spindleTeeth",25);
		definition.setUserEditedTypedDefault("faceZ",selected.face());
		var instance=document.createInstance("spindle/motorCarrier",definition);
		instance.restoreProperty("machinekit.occurrence",TypedProperty.text("machinekit.occurrence","spindle/motorCarrier"));
		var saved=DocumentCodec.encode(document);MachineKitRecipes.forget(document);document.close();
		var fresh=new Document(),freshDefinition=MachineKitDocuments.define(fresh,MillSpindleDrivePart.recipeType(),MillMotorStudy.selection(c.frame).values());
		var freshInstance=fresh.createInstance("spindle/motorCarrier",freshDefinition),messages:Array<String> = [];
		freshInstance.restoreProperty("machinekit.occurrence",TypedProperty.text("machinekit.occurrence","spindle/motorCarrier"));
		MachineKitRecipes.reconcileDocument(fresh,saved,messages);
		var rebuilt=MachineKitRecipes.component(freshInstance);
		if(rebuilt.values().token("motor")!="ELM2M-2000LB130F-H")throw "Motor recipe reconciliation failed: "+haxe.Json.stringify({values:rebuilt.values().token("motor"),messages:messages});
		MachineKitRecipes.forget(fresh);fresh.close();
		Sys.println("Drive recipe reconciliation passed before rendering");
		if(Sys.getEnv("MILL_MOTOR_RECIPE_ONLY")=="1")return;
		var bytes=MillMotorStudy.preview(saved),scene=SceneArtifact.decode(bytes);
		var found=false;for(part in scene.parts) if(part.name=="ELM2M-2000LB130F-H") found=true;
		if(!found)throw "Saved motor selection did not rebuild the visible model: "+haxe.Json.stringify([for(p in scene.parts)if(p.id.indexOf("motor")>=0 || p.name.indexOf("ELM")>=0){id:p.id,name:p.name}]);
		if(scene.recipeDocument==null || scene.recipeDiagnostics==null || scene.recipeDiagnostics.length==0)
			throw "Editable preview lost its recipe or engineering findings";
		var state=new AssemblyState(scene.assemblyDefinition);
		state.setJoint("spindle/rotor-turn",3.2);state.forwardKinematics();
		if(Math.abs(state.joint("spindle/motor-turn")-1)>1e-8)throw "Preview lost edited pulley ratio";
		var reopened=DocumentCodec.decode(scene.recipeDocument,false,false),retained=false;
		for(element in reopened.allElements()) if(element.kind=="instance") {
			var editable=cast(element,InstanceElement),part=MachineKitRecipes.component(editable);
			if(part.values().token("motor")=="ELM2M-2000LB130F-H")retained=true;
		}
		MachineKitRecipes.forget(reopened);reopened.close();
		if(!retained)throw "Returned document lost selected motor";
		var output=Sys.getEnv("MILL_MOTOR_PREVIEW_OUTPUT");
		if(output!=null)sys.io.File.saveBytes(output,bytes);
		Sys.println('Editable mill preview: ${scene.parts.length} render parts; motor, ratio and recipe round-trip passed');
	}
}
