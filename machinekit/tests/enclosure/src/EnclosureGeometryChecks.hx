import machinekit.enclosure.EnclosurePanel;
import machinekit.enclosure.EnclosureFrame;
import machinekit.component.ComponentValues;
import machinekit.component.Bom;
import cadkit.modeling.Part;
import cadkit.modeling.Location;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import materia.assembly.AssemblyFrames;
import materia.project.SceneArtifact;

class EnclosureGeometryChecks {
  static function posed(assembly:machinekit.assembly.MachineAssembly, state:cadkit.modeling.AssemblyState, id:String):Part {
    var pose = state.worldPose(id);
    var x = AssemblyFrames.transformVector(pose,1,0,0), z = AssemblyFrames.transformVector(pose,0,0,1);
    for (entry in assembly.components()) if (entry.id == id) {
      var local = entry.component.geometry();
      var result = local.placed(new Location(new Plane(new Vector(pose.x,pose.y,pose.z),new Vector(x.x,x.y,x.z),new Vector(z.x,z.y,z.z))));
      local.close(); return result;
    }
    throw 'Missing $id';
  }
  public static function main():Void {
    var panel = new EnclosurePanel(1000,800,"stainless-faced-acm");
    var solid = panel.geometry();
    if (Math.abs(solid.volume()-1000*800*4.7)>0.01) throw "Panel geometry lost layer thickness";
    solid.close();
    if (Math.abs(panel.massProperties().mass-0.8*11.39)>1e-9) throw "Panel mass used only core density";
    var rebuilt = EnclosurePanel.recipeType().create(panel.values());
    if (rebuilt.bom.valuesKey != panel.bom.valuesKey || Math.abs(rebuilt.massProperties().mass-panel.massProperties().mass)>1e-9)
      throw "Panel recipe did not reproduce construction";
    var bom = new Bom(); bom.addComponent(panel,2);
    var lines = bom.lines();
    var metadata:machinekit.component.BomItem.EnclosurePanelBom = lines[0].enclosurePanel;
    if (lines[0].quantity != 2 || metadata == null || metadata.construction.layers.length != 3)
      throw "Aggregate BOM lost layer metadata or quantity";
    var frame = new EnclosureFrame(1000,600,800);
    if (frame.validate().length != 0 || frame.components().length != 12) throw "Frame members/mates invalid";
    var assembly = EnclosureKitPreview.assembly();
    if (assembly.validate().length != 0) throw "Enclosure assembly invalid";
    var restored = machinekit.assembly.MachineAssembly.decode(assembly.encode());
    if (restored.components().length != assembly.components().length) throw "Enclosure recipe round trip lost parts";
    var saved:machinekit.assembly.MachineAssemblyDescription.SavedBomItem = {
      partNumber:panel.bom.partNumber, description:panel.bom.description, quantity:1, material:panel.bom.material,
      enclosurePanel:metadata
    };
    var savedAgain:machinekit.assembly.MachineAssemblyDescription.SavedBomItem = haxeon.wire.JsonWire.decode(haxeon.wire.JsonWire.encode(saved));
    var savedMetadata:machinekit.component.BomItem.EnclosurePanelBom = savedAgain.enclosurePanel;
    if (savedMetadata == null || savedMetadata.construction.layers.length != 3) throw "Saved BOM lost construction metadata";
    var model = new cadkit.modeling.AssemblyModel("mm");
    assembly.addTo(model, "");
    var state = new cadkit.modeling.AssemblyState(model.definition("enclosure-kit"));
    var start = state.worldPose("door/leaf").x;
    for (step in 0...11) {
      state.setJoint("door/slide",510*step/10);
      var leaf = posed(assembly,state,"door/leaf");
      for (entry in assembly.components()) if (entry.id != "door/leaf") {
        var fixed = posed(assembly,state,entry.id), common = leaf.intersect(fixed), volume = common.volume();
        common.close(); fixed.close();
        if (volume>0.001) throw 'Door intersects ${entry.id} at $step';
      }
      leaf.close();
    }
    if (Math.abs(state.worldPose("door/leaf").x-start-510)>1e-6) throw "Door stroke or mount moved incorrectly";
    var scene = SceneArtifact.decode(EnclosureKitPreview.preview());
    if (scene.assemblyDefinition == null || scene.parts.length == 0) throw "Scene lost enclosure geometry";
    EnclosureKitPreview.main();
    Sys.println("Enclosure panel, recipe, BOM, frame, door sweep and scene checks passed");
  }
}
