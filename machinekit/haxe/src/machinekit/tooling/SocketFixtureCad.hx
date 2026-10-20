package machinekit.tooling;

import machinekit.assembly.MachineAssembly;
import machinekit.component.MachineComponent;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.tooling.casting.InsertFixturePlanner.InsertPlatePlan;
import materia.tooling.casting.MoldPlan.MoldBox;
import materia.tooling.panel.PanelAssemblyPlan.PanelAssemblyRecord;
import materia.tooling.panel.PanelJoints;
import materia.casting.BoxAlgebra;

/** Coupon review walls around the common source-insert fixture; no socket specification is duplicated here. */
class SocketFixtureCad {
	public static function panels(plan:InsertPlatePlan):PanelAssemblyRecord
		return plan.assembly;

	/** This review enclosure deliberately accepts one exact rectangular casting envelope. */
	public static function cavity(plan:InsertPlatePlan):MoldBox {
		var boxes = BoxAlgebra.material([for (prism in plan.document.input.envelope) {
			id:prism.id,profile:prism.profile,height:prism.height,pose:F.compose(plan.document.setup.sourceToMold,prism.pose)
		}],[]);
		if (boxes.length != 1) throw "Socket coupon walls require one rectangular casting envelope";
		return boxes[0];
	}

	static function add(assembly:MachineAssembly,root:String,id:String,part:MachineComponent,pose:AssemblyFrame):Void {
		var local = F.compose(F.inverse(assembly.solvedPoses().get(root)),pose);
		assembly.addComponent(id,part,pose);
		assembly.addMemberConnector(root,id+"-seat",local);
		assembly.addMemberConnector(id,"fixture-seat",F.identity());
		assembly.addMate(id+"-position","fixed",root,id+"-seat",id,"fixture-seat");
	}

	public static function check(plan:InsertPlatePlan):InsertFixtureCad.InsertFixtureReview {
		return InsertFixtureCad.checkAssembly(plan,assembly(plan,true),[
			{id:"south-wall",direction:new cadkit.modeling.Vector(0,-1,0)},
			{id:"north-wall",direction:new cadkit.modeling.Vector(0,1,0)},
			{id:"west-wall",direction:new cadkit.modeling.Vector(-1,0,0)},
			{id:"east-wall",direction:new cadkit.modeling.Vector(1,0,0)}
		]);
	}

	public static function assembly(plan:InsertPlatePlan,cured:Bool = false,exploded:Bool = false):MachineAssembly {
		var assembly = InsertFixtureCad.assembly(plan,cured), root = plan.assembly.members[0].panel.id;
		var c = cavity(plan), t = plan.document.recipe.panel.measuredThickness, stock = plan.document.recipe.panel;
		var walls = [
			{id:"south-wall",width:c.width+2*t,height:c.height,pose:F.fromRotationMatrix(c.x-t,c.y,c.z,[1,0,0,0,0,-1,0,1,0])},
			{id:"north-wall",width:c.width+2*t,height:c.height,pose:F.fromRotationMatrix(c.x-t,c.y+c.depth+t,c.z,[1,0,0,0,0,-1,0,1,0])},
			{id:"west-wall",width:c.depth,height:c.height,pose:F.fromRotationMatrix(c.x-t,c.y,c.z,[0,0,1,1,0,0,0,1,0])},
			{id:"east-wall",width:c.depth,height:c.height,pose:F.fromRotationMatrix(c.x+c.width,c.y,c.z,[0,0,1,1,0,0,0,1,0])}
		];
		for (wall in walls)
			add(assembly,root,wall.id,ToolingPanel.fromPlan(PanelJoints.blank(wall.id,wall.width,wall.height,stock,
				plan.document.recipe.routerDiameter,plan.document.recipe.minimumLigament)),wall.pose);
		if (!exploded) return assembly;
		var out = new MachineAssembly(), poses = assembly.solvedPoses();
		for (row in assembly.components()) {
			var pose = F.compose(F.identity(),poses.get(row.id));
			if (row.id == plan.assembly.members[0].panel.id) pose.z -= 75;
			else if (row.id == plan.assembly.members[1].panel.id) pose.z -= 35;
			else if (row.id == "cured-casting") {pose.x += c.width+150; pose.z += 80;}
			else if (row.id == "south-wall") pose.y -= 100;
			else if (row.id == "north-wall") pose.y += 100;
			else if (row.id == "west-wall") pose.x -= 100;
			else if (row.id == "east-wall") pose.x += 100;
			else if (plan.document.input.inserts.filter(insert -> row.id == materia.tooling.ToolingIds.derive("insert-",insert.id)).length == 1) pose.z += 60;
			else pose.z -= 70;
			out.addComponent(row.id,row.component,pose);
		}
		return out;
	}
}
