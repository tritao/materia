package materia.tooling.casting;

import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.tooling.ToolingEnums.ContactFace;
import materia.tooling.ToolingEnums.MachiningSide;
import materia.tooling.panel.PanelJoints;
import materia.tooling.panel.PanelJoints.PanelJointKind;
import materia.tooling.panel.PanelJoints.PanelClosure;
import materia.tooling.panel.PanelContours;
import materia.tooling.panel.PanelCodec;
import materia.tooling.panel.PanelAssemblyPlan.PanelAssemblyRecord;
import materia.tooling.casting.MoldPlan.MoldBox;
import materia.tooling.casting.MoldPlan.PenetrationPlan;
import materia.tooling.ToolingRecipe;

enum abstract RaisedWalls(String) to String {
	var NorthSouth = "north-south";
	var EastWest = "east-west";
	var NoRaised = "none";
}

typedef PanelBoxKit = {
	var panels:PanelAssemblyRecord;
	var penetrations:Array<PenetrationPlan>;
}

/** Shared removable five-panel shell; no feature classification or machine dimensions. */
class MoldPanels {
	/** Outward-facing removable former shell. Unlike the mold walls, its cap
	 * and four sides stay within the authored exterior. The withdrawal end is
	 * open. Two cap rebates and four corner rebates locate the sides; closure
	 * uses pocket screws with heads and access inside the dry cavity.
	 * Board/grip qualification remains a physical shop input.
	 */
	public static function formerBox(id:String,width:Float,depth:Float,height:Float,r:ToolingRecipe):PanelAssemblyRecord {
		var t=r.panel.measuredThickness,c=r.jointClearance;
		if (width<=2*(t+c)+r.minimumLigament || depth<=2*(t+c)+r.minimumLigament || height<=2*t)
			throw "Former panel box has insufficient internal room: "+id;
		var result:PanelAssemblyRecord={id:id,members:[],joints:[]};
		function add(name:String,w:Float,h:Float,pose:AssemblyFrame):Void {
			var panel=PanelJoints.blank(materia.tooling.ToolingIds.derive("",id,"-"+name),w,h,
				r.panel,r.routerDiameter,r.minimumLigament);
			panel.contactFace=ContactFace.None;
			result.members.push({panel:panel,pose:pose,withdrawal:{x:0,y:0,z:-1}});
		}
		add("cap",width,depth,F.translation(0,0,height-t));
		add("west",depth,height-t/2,F.fromRotationMatrix(0,0,0,[0,0,1,1,0,0,0,1,0]));
		add("east",depth,height-t/2,F.fromRotationMatrix(width,depth,0,[0,0,-1,-1,0,0,0,1,0]));
		add("south",height-t,width-t,F.fromRotationMatrix(width-t/2,t,0,[0,-1,0,0,0,-1,1,0,0]));
		add("north",height-t,width-t,F.fromRotationMatrix(t/2,depth-t,0,[0,1,0,0,0,1,1,0,0]));
		function seat(fixed:Int,moving:Int,name:String,x:Float,length:Float,memberLength:Float,side:MachiningSide):Void {
			var a=result.members[fixed],b=result.members[moving];
			var operation=PanelJoints.operation(name,Rebate,PanelContours.rectangle(x,0,t+c,length),
				t/2,r.routerDiameter,side,false);
			a.panel.operations.push(operation);
			result.joints.push({id:materia.tooling.ToolingIds.derive("",id,"-joint-"+name),
				kind:PanelJointKind.Rebate,fixedId:a.panel.id,movingId:b.panel.id,
				fixedFrame:F.compose(F.inverse(a.pose),b.pose),movingFrame:F.identity(),clearance:c,
				closure:PanelClosure.InternalPocketScrews,liningAllowance:r.jointLiningAllowance,
				machiningAllowance:r.jointMachiningAllowance,fits:[],pins:[],
				seat:{operationId:name,acrossX:true,openingWidth:t+c,openingLength:length,
					memberWidth:t,memberLength:memberLength,thicknessFraction:1,widthClearance:c,
					lengthClearance:length-memberLength,reliefDiameter:0}});
		}
		seat(0,1,"cap-west",0,depth,depth,MachiningSide.NegativeZ);
		seat(0,2,"cap-east",width-t-c,depth,depth,MachiningSide.NegativeZ);
		seat(1,3,"west-south",0,height-t/2,height-t,MachiningSide.PositiveZ);
		seat(1,4,"west-north",depth-t-c,height-t/2,height-t,MachiningSide.PositiveZ);
		seat(2,4,"east-north",0,height-t/2,height-t,MachiningSide.PositiveZ);
		seat(2,3,"east-south",depth-t-c,height-t/2,height-t,MachiningSide.PositiveZ);
		for (member in result.members) member.panel=PanelCodec.copy(member.panel);
		materia.tooling.panel.PanelAssemblyPlan.validate(result);
		return result;
	}

	public static function box(b:MoldBox, r:ToolingRecipe, raised:RaisedWalls = RaisedWalls.NorthSouth):PanelBoxKit {
		var t = r.panel.measuredThickness, c = r.jointClearance, seat = t / 2;
		if (seat < r.minimumLigament)
			throw "Bed sockets leave insufficient stock";
		var margin = 2 * t + 2 * r.minimumLigament + r.routerDiameter,
			ox = b.x - margin,
			oy = b.y - margin;
		var bed = PanelJoints.blank("bed", b.width + 2 * margin, b.depth + 2 * margin, r.panel, r.routerDiameter, r.minimumLigament);
		bed.contactFace = ContactFace.PositiveZ;
		bed.labelSide = MachiningSide.NegativeZ;
		bed.contactOpenings = [];
		var assembly:PanelAssemblyRecord = {
			id: "box-panels",
			members: [
				{
					panel: bed,
					pose: F.translation(ox, oy, b.z - t),
					withdrawal: {
						x: 0,
						y: 0,
						z: -1
					}
				}
			],
			joints: []
		};
		var penetrations:Array<PenetrationPlan> = [];
		function wall(id:String, width:Float, pose:AssemblyFrame, sx:Float, sy:Float, sw:Float, sh:Float):Void {
			var panel = PanelJoints.blank(id, width,
				b.height + (raised == RaisedWalls.NorthSouth
					&& (id == "south" || id == "north")
					|| raised == RaisedWalls.EastWest
					&& (id == "east" || id == "west") ? 2 * seat : seat),
				r.panel, r.routerDiameter, r.minimumLigament);
			panel.contactFace = ContactFace.NegativeZ;
			panel.labelSide = MachiningSide.PositiveZ;
			panel.contactOpenings = ["bottom-land"];
			panel.operations.push(PanelJoints.operation("bottom-land", Rebate, PanelContours.rectangle(0, 0, width, seat), t / 2, r.routerDiameter,
				MachiningSide.NegativeZ, false));
			penetrations.push({
				panelId: id,
				operationId: "bottom-land",
				kind: OutsideCasting,
				insertId: null,
				coreId: null,
				sealMargin: 0,
				sealMaterial: r.hardware.sealMaterial,
				sealingFace: null,
				plug: null
			});
			var op = PanelJoints.operation(materia.tooling.ToolingIds.derive("", id, "-socket"), Pocket,
				PanelContours.relievedRectangle(sx - ox + sw / 2, sy - oy + sh / 2, sw, sh, r.routerDiameter / 2), seat, r.routerDiameter,
				MachiningSide.PositiveZ, false, r.routerDiameter);
			bed.operations.push(op);
			bed.contactOpenings.push(op.id);
			penetrations.push({
				panelId: bed.id,
				operationId: op.id,
				kind: OutsideCasting,
				insertId: null,
				coreId: null,
				sealMargin: 0,
				sealMaterial: r.hardware.sealMaterial,
				sealingFace: null,
				plug: null
			});
			assembly.members.push({
				panel: PanelCodec.copy(panel),
				pose: pose,
				withdrawal: {x: 0, y: 0, z: 1}
			});
			assembly.joints.push({
				id: materia.tooling.ToolingIds.derive("", id, "-bed"),
				kind: PanelJointKind.TabSlot,
				fixedId: bed.id,
				movingId: id,
				fixedFrame: F.compose(F.inverse(assembly.members[0].pose), pose),
				movingFrame: F.identity(),
				clearance: c,
				liningAllowance: r.jointLiningAllowance,
				machiningAllowance: r.jointMachiningAllowance,
				fits: [],
				pins: [],
				seat: {
					operationId: op.id,
					acrossX: sw < sh,
					openingWidth: Math.min(sw, sh),
					openingLength: Math.max(sw, sh),
					memberWidth: t / 2,
					memberLength: width,
					thicknessFraction: 0.5,
					widthClearance: c,
					lengthClearance: 2 * c,
					reliefDiameter: r.routerDiameter
				},
				closure: PanelClosure.ExternalClamps
			});
		}
		wall("south", b.width
			+ 2 * t, F.fromRotationMatrix(b.x - t, b.y, b.z - seat, [1, 0, 0, 0, 0, -1, 0, 1, 0]), b.x
			- t
			- c, b.y
			- t
			- c,
			b.width
			+ 2 * t
			+ 2 * c, t / 2
			+ c);
		wall("north", b.width
			+ 2 * t, F.fromRotationMatrix(b.x + b.width + t, b.y + b.depth, b.z - seat, [-1, 0, 0, 0, 0, 1, 0, 1, 0]), b.x
			- t
			- c,
			b.y
			+ b.depth
			+ t / 2, b.width
			+ 2 * t
			+ 2 * c, t / 2
			+ c);
		// Side sockets remain separate from long sockets; cutter reliefs stay outside contact lands.
		wall("east", b.depth, F.fromRotationMatrix(b.x + b.width, b.y, b.z - seat, [0, 0, 1, 1, 0, 0, 0, 1, 0]), b.x
			+ b.width
			+ t / 2, b.y
			- c, t / 2
			+ c,
			b.depth
			+ 2 * c);
		wall("west", b.depth, F.fromRotationMatrix(b.x, b.y + b.depth, b.z - seat, [0, 0, -1, -1, 0, 0, 0, 1, 0]), b.x
			- t
			- c, b.y
			- c, t / 2
			+ c,
			b.depth
			+ 2 * c);
		assembly.members[0].panel = PanelCodec.copy(bed);
		return {panels: assembly, penetrations: penetrations};
	}
}
