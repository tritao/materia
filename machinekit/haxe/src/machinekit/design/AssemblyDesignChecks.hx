package machinekit.design;

import cadkit.modeling.Part;
import cadkit.modeling.AssemblyModel;
import cadkit.modeling.AssemblyState;
import machinekit.assembly.MachineAssembly;
import machinekit.assembly.Diagnostics;
import machinekit.assembly.PosedParts;
import machinekit.component.MachineComponent;
import machinekit.standard.ScrewFastener;
import machinekit.design.DesignRequirement;
import machinekit.design.DesignInterfaces;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** No machine names, dimensions or member IDs: evaluates component facets and
 * portable assembly requirements. Assembly phases explicitly name installed
 * parts; service requirements see the entire containing machine by default.
 */
class AssemblyDesignChecks {
	public static function check(assembly:MachineAssembly):Diagnostics {
		var out=new Diagnostics(), cache=new PosedParts();
		try run(assembly,out,cache) catch(e:Dynamic) {cache.close();throw e;}
		cache.close();return out;
	}
	static function run(a:MachineAssembly,out:Diagnostics,cache:PosedParts):Void {
		var members:Map<String,MachineComponent> = [];
		for(m in a.components()) {
			members.set(m.id,m.component);
			var interfaces=DesignInterfaces.of(m.component);
			if(interfaces==null) continue;
			interfaces.check(m.component);
			var host=cache.local(m.component);
			for(e in interfaces.material) material(out,m.id+"/"+e.id,e,host);
			for(h in interfaces.holes) hole(out,m.id,h,host);
		}
		var model=new AssemblyModel("mm");a.addTo(model,"");
		var definition=model.definition("assembly-design"), initial=new AssemblyState(definition);
		initial.forwardKinematics();
		var requirements=a.designRequirements();
		for (member in a.components()) {
			var interfaces=DesignInterfaces.of(member.component);
			if (interfaces==null) continue;
			for (source in interfaces.precisionRequirements()) for (definitionId in source.relationshipIds) {
				var found=false;
				for (requirement in requirements) switch requirement {
					case InterfaceAlignment(_,relationship):
						if (relationship.id==definitionId) for (reference in relationship.members) switch reference {
							case ComponentInterface(part,face): if (part==member.id && face==source.id) found=true;
							case MachineAxis(_):
						}
					default:
				}
				if (!found) out.error("design.interface.relationship-reference",member.id+"/"+source.id,
					"Source interface references an unknown or unrelated relationship: "+definitionId);
			}
		}
		for(r in requirements) {
			var id=DesignRequirements.id(r);
			try switch(r) {
				case InterfaceAlignment(_,relationship): interfaceAlignment(out,id,relationship,members,definition);
				case FoundationConnection(_,joint): FoundationJointDesignChecks.check(out,id,joint,members,initial);
				case Coaxial(_,first,second,tolerance):
					if (!Math.isFinite(tolerance) || tolerance<=0) throw "Invalid nominal coaxial tolerance";
					nominalCoaxial(out,id,axisPose(first,members,initial),axisPose(second,members,initial),tolerance);
				case FastenerJoint(_,screw,target,through): fastener(out,id,screw,target,through,members,initial,cache);
				case AccessPath(_,anchor,envelopes,pose,selection):
					var state=new AssemblyState(definition);
					for(j in pose) state.setJoint(j.joint,j.value);
					state.forwardKinematics();
					if(envelopes.length==0)throw "Access requirement has no envelopes";
					var origin=featurePose(anchor,members,state);
					var ids=switch(selection) {case WholeAssembly:[for(m in a.components())m.id];case InstalledMembers(parts):if(parts.length==0)throw "Assembly phase has no installed members";parts;};
					var obstacles:Array<{id:String,part:Part}> = [];
					try {
						for(partId in ids) {
							var component=members.get(partId);
							if(component==null) throw 'Unknown installed member "$partId"';
							try obstacles.push({id:partId,part:cache.posed(component,state.worldPose(partId))})
							catch(_:machinekit.component.NoGeometry) {} // Logical endpoints have no physical obstacle.
						}
						if(obstacles.length==0)throw "Access requirement has no physical obstacles to assess";
						for(e in envelopes) {
							var shape=DesignGeometry.envelope({id:e.id,frame:F.compose(origin,e.frame),volume:e.volume});
							try DesignChecks.access(out,id+"/"+e.id,shape,obstacles) catch(error:Dynamic) {shape.close();throw error;}
							shape.close();
						}
					} catch(error:Dynamic) {for(p in obstacles)p.part.close();throw error;}
					for(p in obstacles)p.part.close();
			} catch(error:Dynamic) out.error("design.invalid-requirement",id,Std.string(error));
		}
	}

	static function interfaceAlignment(out:Diagnostics,id:String,authored:materia.casting.InterfaceRelationship,
			members:Map<String,MachineComponent>,definition:materia.assembly.AssemblyDefinition):Void {
		var relationship=materia.casting.InterfaceRelationship.InterfaceRelationships.snapshot([authored])[0];
		for (member in relationship.members) switch member {
			case ComponentInterface(partId,interfaceId):
				InterfaceSourceSupport.precision(members,partId,interfaceId,relationship.id);
			case MachineAxis(axisId): InterfaceSourceSupport.axis(definition,axisId);
		}
		var margin=materia.casting.InterfaceRelationship.InterfaceRelationships.uncertaintyMargin(relationship);
		if (margin==null || margin<0 || relationship.metrology.instrumentReference==null)
			out.warning("design.interface.metrology",id,"Relationship metrology is absent or its uncertainty exceeds the design tolerance.");
		out.warning("design.interface.qualification",id,
			"Relationship references resolve; geometry, replication jig and measured alignment remain unqualified.");
	}
	static function nominalCoaxial(out:Diagnostics,id:String,a:AssemblyFrame,b:AssemblyFrame,tolerance:Float):Void {
		var u=F.transformVector(a,0,0,1),v=F.transformVector(b,0,0,1);
		var dx=b.x-a.x,dy=b.y-a.y,dz=b.z-a.z;
		var x=dy*u.z-dz*u.y,y=dz*u.x-dx*u.z,z=dx*u.y-dy*u.x;
		var offset=Math.sqrt(x*x+y*y+z*z);
		var ax=u.y*v.z-u.z*v.y,ay=u.z*v.x-u.x*v.z,az=u.x*v.y-u.y*v.x;
		var angular=Math.sqrt(ax*ax+ay*ay+az*az);
		// Numerical coincidence of nominal axes, not a physical metrology claim.
		if (offset>tolerance || angular>1e-9)
			out.error("design.fastener-axis",id,"Nominal axes differ: offset="+offset+" mm; sin(angle)="+angular);
	}

	static function axisPose(ref:DesignReference,members:Map<String,MachineComponent>,state:AssemblyState):AssemblyFrame {
		var member=members.get(ref.member);
		if (member==null) throw "Unknown coaxial member: "+ref.member;
		var interfaces=DesignInterfaces.of(member);
		if (interfaces!=null) for (hole in interfaces.holes)
			if (hole.id==ref.feature) return F.compose(state.worldPose(ref.member),hole.frame);
		for (connector in member.connectors()) if (connector.name==ref.feature) {
			// Component connector +Y is the axis; design-hole +Z enters steel.
			var axis=F.fromRotationMatrix(0,0,0,[1,0,0,0,0,1,0,-1,0]);
			return F.compose(state.worldPose(ref.member),F.compose(connector.frame,axis));
		}
		throw "Coaxial reference needs a source hole or connector: "+ref.member+"/"+ref.feature;
	}

	static function facet(ref:DesignReference,members:Map<String,MachineComponent>):DesignInterfaces {
		var member=members.get(ref.member);if(member==null)throw 'Unknown design member "${ref.member}"';
		var result=DesignInterfaces.of(member);if(result==null)throw 'No design interfaces on "${ref.member}"';return result;
	}
	static function featurePose(ref:DesignReference,members:Map<String,MachineComponent>,state:AssemblyState):AssemblyFrame
		return F.compose(state.worldPose(ref.member),facet(ref,members).frame(ref.feature));
	static function material(out:Diagnostics,id:String,e:DesignEnvelope,host:Part):Void {
		var shape=DesignGeometry.envelope(e);
		try DesignChecks.material(out,id,shape,host) catch(error:Dynamic) {shape.close();throw error;}
		shape.close();
	}
	static function clear(out:Diagnostics,id:String,e:DesignEnvelope,host:Part):Void {
		var shape=DesignGeometry.envelope(e);
		try DesignChecks.access(out,id,shape,[{id:"host",part:host}]) catch(error:Dynamic) {shape.close();throw error;}
		shape.close();
	}
	static function hole(out:Diagnostics,member:String,h:HoleInterface,host:Part):Void {
		var id=member+"/"+h.id;
		clear(out,id+" drilling",{id:id,frame:h.frame,volume:CylinderVolume(h.radius,0,h.depth)},host);
		var start=0.0;
		if(h.counterbore!=null) {
			var c=h.counterbore;
			clear(out,id+" counterbore",{id:id,frame:h.frame,volume:CylinderVolume(c.radius,0,c.depth)},host);
			material(out,id+" counterbore minimum wall "+h.minWall,{id:id,frame:h.frame,volume:RingVolume(c.radius+h.minWall,c.radius,0,c.depth)},host);
			start=c.depth;
		}
		material(out,id+" minimum wall "+h.minWall,{id:id,frame:h.frame,volume:RingVolume(h.radius+h.minWall,h.radius,start,h.relief == null ? h.depth : h.relief.start)},host);
		if (h.relief != null) {
			var r = h.relief, radius = r.diameter / 2;
			clear(out,id+" thread relief",{id:id,frame:h.frame,volume:CylinderVolume(radius,r.start,h.depth)},host);
			material(out,id+" relief minimum wall "+h.minWall,{id:id,frame:h.frame,volume:RingVolume(radius+h.minWall,radius,r.start,h.depth)},host);
		}
		if(h.thread!=null) {
			var t=h.thread;
			if(Math.abs(t.tapDrill/2-h.radius)>1e-6) out.error("design.tap-size",id,"Tap drill does not match thread designation");
			if (h.through != true) {
				DesignChecks.minimum(out,id+" tap runout",t.cylindricalDepth-t.usableDepth,0.5);
				DesignChecks.minimum(out,id+" drill back wall",t.stock-h.depth,t.minBack);
				material(out,id+" actual back wall "+t.minBack,{id:id,frame:h.frame,volume:CylinderVolume(h.relief == null ? h.radius : h.relief.diameter/2,h.depth,h.depth+t.minBack)},host);
			}
		}
		if(h.approach!=null) clear(out,id+" drill/tap approach",{id:id,frame:h.frame,volume:CylinderVolume(h.approach.radius,-h.approach.length,0)},host);
	}
	static function fastener(out:Diagnostics,id:String,screwId:String,target:DesignReference,through:Array<DesignReference>,
		members:Map<String,MachineComponent>,state:AssemblyState,cache:PosedParts):Void {
		var component=members.get(screwId);
		if(!Std.isOfType(component,ScrewFastener)) throw 'Unsupported screw "$screwId"';
		var screw:ScrewFastener=cast component, h=facet(target,members).hole(target.feature), t=h.thread;
		if(t==null)throw "Fastener target is not a tapped hole";
		if(!screw.threadSpec().matches(t.spec) || screw.threadSpec().internal) out.error("design.fastener-size",id,"Screw and tapped hole thread specifications do not match");
		var head=F.compose(state.worldPose(screwId),screw.bearingFrame()), tap=featurePose(target,members,state);
		DesignChecks.coaxial(out,id,head,tap);
		var shaft=F.transformVector(head,0,0,1), into=F.transformVector(tap,0,0,1);
		if(shaft.x*into.x+shaft.y*into.y+shaft.z*into.z<0.999999)out.error("design.fastener-direction",id,"Screw points away from tapped hole");
		var grip=(tap.x-head.x)*shaft.x+(tap.y-head.y)*shaft.y+(tap.z-head.z)*shaft.z;
		if(grip<=0)throw "Fastener has no positive grip stack";
		var threadStart=h.counterbore==null ? 0.0 : h.counterbore.depth;
		var threadedGrip=grip+threadStart,usable=t.usableDepth-threadStart;
		if (h.through == true)
			DesignChecks.minimum(out,id+" thread engagement",Math.min(screw.shankLength()-threadedGrip,usable),t.minEngagement);
		else
			DesignChecks.blindFastener(out,id,screw.shankLength(),threadedGrip,usable,
				t.cylindricalDepth-threadStart,t.point,t.stock-threadStart,t.minEngagement,t.minTipGap,t.minBack,
				h.relief == null ? null : h.relief.start-threadStart);
		if(screw.shankLength()-screw.threadedLength()>threadedGrip+1e-6)
			out.error("design.unthreaded-shank",id,"Unthreaded shank enters tapped hole");
		var intervals:Array<{start:Float,end:Float}> = [];
		for(ref in through) {
			var clearance=facet(ref,members).hole(ref.feature), face=featurePose(ref,members,state);
			DesignChecks.coaxial(out,id+"/"+ref.member,head,face);
			var axis=F.transformVector(face,0,0,1), sign=axis.x*shaft.x+axis.y*shaft.y+axis.z*shaft.z;
			var faceDistance=(face.x-head.x)*shaft.x+(face.y-head.y)*shaft.y+(face.z-head.z)*shaft.z;
			var end=faceDistance+sign*clearance.depth;
			intervals.push({start:Math.min(faceDistance,end)+(clearance.counterbore==null?0:clearance.counterbore.depth),end:Math.max(faceDistance,end)});
			DesignChecks.minimum(out,id+" diametral clearance",2*clearance.radius-screw.threadSpec().majorDiameter,clearance.minClearance);
			if(clearance.counterbore!=null) {
				var c=clearance.counterbore;
				DesignChecks.minimum(out,id+" head clearance",2*c.radius-2*screw.headClearanceRadius(),0.5);
				var headLocal=F.compose(F.inverse(face),head);
				if(Math.abs(headLocal.z-c.depth)>1e-6)out.error("design.head-seat",id,"Screw bearing face is not at counterbore seat");
				var e:DesignEnvelope={id:id,frame:clearance.frame,volume:RingVolume(screw.headBearingRadius(),clearance.radius,c.depth,c.depth+c.bearingDepth)};
				material(out,id+" head bearing",e,cache.local(members.get(ref.member)));
			}
		}
		intervals.sort((a,b)->a.start<b.start?-1:a.start>b.start?1:0);
		var covered=0.0;
		for(interval in intervals) {
			if(Math.abs(interval.start-covered)>0.01 || interval.end>grip+0.01) out.error("design.grip-stack",id,"Declared clamped parts have a gap, overlap or extend past the tapped face");
			covered=interval.end;
		}
		if(Math.abs(covered-grip)>0.01)out.error("design.grip-stack",id,"Clamped interfaces do not cover the grip stack");
	}
}
