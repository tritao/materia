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
import materia.assembly.AssemblyDefinition;
import materia.presentation.AssemblyPresentation;
import materia.presentation.Presentations;

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

	/** Views refer to the single engineering assembly, in its length unit. */
	public static function presentation(plan:InsertPlatePlan,definition:AssemblyDefinition):AssemblyPresentation {
		var result=Presentations.assembled(definition,-1.1,0.35,"socket-fixture");
		var plates=[for(member in plan.assembly.members){occurrence:member.panel.id,subtree:false}];
		var inserts=plan.document.input.inserts;
		if(inserts.length<2)throw "Socket fixture section needs two socket axes";
		var first=F.compose(plan.document.setup.sourceToMold,inserts[0].pose);
		var second=F.compose(plan.document.setup.sourceToMold,inserts[1].pose);
		var axis=F.transformVector(first,0,0,1),dx=second.x-first.x,dy=second.y-first.y,dz=second.z-first.z;
		var normal={x:dy*axis.z-dz*axis.y,y:dz*axis.x-dx*axis.z,z:dx*axis.y-dy*axis.x};
		var length=Math.sqrt(normal.x*normal.x+normal.y*normal.y+normal.z*normal.z);
		if(length<1e-9)throw "Socket axes must be distinct for the section view";
		normal={x:normal.x/length,y:normal.y/length,z:normal.z/length};
		var sockets:Array<PartSelector> = [for(insert in inserts){occurrence:materia.tooling.ToolingIds.derive("insert-",insert.id),subtree:false}];
		// Oblique elevation separates the hatched side-wall ends from the uncut north wall.
		result.views.push({id:"see-inside",label:"See inside",camera:Frame(sockets,-1.25,0.65,2.0),
			visibility:[for(select in plates){select:select,mode:Transparent}].concat(
                definition.occurrences.filter(part->part.id=="cured-casting").map(part->({select:{occurrence:part.id,subtree:false},mode:Transparent}))),
			sections:[{occurrence:null,point:{x:first.x,y:first.y,z:first.z},normal:normal}],explode:0,source:Generated("socket-fixture")});
		var mounting:Array<PartSelector> = [];
		for(site in plan.sites.slice(0,1)) {
			mounting.push({occurrence:site.id,subtree:false});
			mounting.push({occurrence:materia.tooling.ToolingIds.derive("",site.id,"-washer"),subtree:false});
		}
		result.views.push({id:"socket-mounting",label:"Socket mounting",camera:Frame(mounting,0.6,-0.65,2),
			visibility:[{select:plates[0],mode:Hidden}],sections:[],explode:0,source:Generated("socket-fixture")});
		var offsets=new Map<String,materia.assembly.AssemblyDefinition.AssemblyVector>(),c=cavity(plan);
		var flat=materia.assembly.AssemblyDefinitionFlattener.flattenView(definition);
		for(row in flat.occurrences)offsets.set(row.id,
			if(row.id==plan.assembly.members[0].panel.id){x:0.0,y:0.0,z:-75.0}
			else if(row.id==plan.assembly.members[1].panel.id){x:0.0,y:0.0,z:-35.0}
			else if(row.id=="cured-casting"){x:c.width+150,y:0.0,z:80.0}
			else if(row.id=="south-wall"){x:0.0,y:-100.0,z:0.0}
			else if(row.id=="north-wall"){x:0.0,y:100.0,z:0.0}
			else if(row.id=="west-wall"){x:-100.0,y:0.0,z:0.0}
			else if(row.id=="east-wall"){x:100.0,y:0.0,z:0.0}
			else if(inserts.filter(insert->row.id==materia.tooling.ToolingIds.derive("insert-",insert.id)).length==1){x:0.0,y:0.0,z:60.0}
			else {x:0.0,y:0.0,z:-70.0});
		result.explode=ToolingExplode.presentation(definition,"socket-fixture",offsets).explode;
		materia.presentation.PresentationCodec.validate(definition,result);
		return result;
	}

	public static function assembly(plan:InsertPlatePlan,cured:Bool = false):MachineAssembly {
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
		return assembly;
	}
}
