package machinekit.assembly;

import cadkit.modeling.AssemblyModel;
import machinekit.component.Bom;
import machinekit.component.MachineComponent;
import machinekit.motion.PillowBlockHousing;
import machinekit.standard.DeepGrooveBearing;
import machinekit.standard.SocketHeadCapScrew;
import materia.project.AssemblyRecord.AssemblyFrame;

/** Flange bearing housing with its bearing pressed in and four mounting screws, composed from
 * standalone `MachineComponent`s rather than being one itself: `addTo` places the housing at
 * `pose` and mates the bearing and screws onto it, so the whole block moves together. The
 * bearing's own `front`/`axis`/`back` connectors (named `'<id>-bearing'`) stay reachable for
 * mating a shaft through it.
 */
class PillowBlock {
	public final housing:PillowBlockHousing;
	public final bearing:DeepGrooveBearing;
	public final screw:SocketHeadCapScrew;

	public function new(bearing:DeepGrooveBearing) {
		this.bearing = bearing;
		housing = new PillowBlockHousing(bearing);
		screw = housing.mountScrewPart(10);
	}

	public function addTo(model:AssemblyModel, id:String, ?pose:AssemblyFrame):Void {
		housing.addTo(model, '$id-housing', pose);
		bearing.addTo(model, '$id-bearing');
		model.mate('$id-bearing-seat', "fixed", '$id-housing', "bore", '$id-bearing', "front");
		for (i in 1...5) {
			var screwId = '$id-screw$i';
			screw.addTo(model, screwId);
			model.mate('$screwId-seat', "fixed", '$id-housing', 'bolt$i', screwId, "head");
		}
	}

	public function bom():Bom {
		var result = new Bom();
		result.addComponent(housing);
		result.addComponent(bearing);
		result.addComponent(screw, 4);
		return result;
	}

	/** Instance id to component, for geometry generation by a preview or exporter. */
	public function components():Array<{id:String, component:MachineComponent}> {
		var result:Array<{id:String, component:MachineComponent}> = [
			{id: "housing", component: housing}, {id: "bearing", component: bearing}];
		for (i in 1...5) result.push({id: 'screw$i', component: screw});
		return result;
	}
}
