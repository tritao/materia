package machinekit.structural;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentValue;
import machinekit.component.ComponentParameter;
import machinekit.component.ComponentRecipeSupport;
import machinekit.structural.FrameAssembly.StructuralProfile;

/** Cut structural stock along local +Z. Hollow sections retain their actual material and mass. */
class ProfileMember extends MachineComponent {
	public final profile:StructuralProfile;
	public final length:Float;
	static var tslotRecipe:Null<ComponentType>;
	static var drawingRecipe:Null<ComponentType>;

	public function new(profile:StructuralProfile, length:Float, material:String = "steel") {
		if (profile == null || !(length > 0) || !Math.isFinite(length))
			throw "Structural member needs a profile and a finite positive length";
		super('${profile.profileDesignation()}-L${Dimension.format(length)}', profile.profileDescription(), material);
		this.profile = profile;
		this.length = length;
		addConnector("start", Mount, Solids.axial(0, 0, 0));
		addConnector("end", Mount, Solids.axial(0, 0, length));
	}

	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part return profile.geometry(length);

	/** Shared modular T-slot members, optionally carrying a complete measured section variant. */
	public static function tslotRecipeType():ComponentType {
		if(tslotRecipe==null) tslotRecipe=new ProfileMemberRecipeType("machinekit.structural.tslot-member",[
			ComponentRecipeSupport.choice("system",["TSLOT-30-SLOT8","TSLOT-40-SLOT8"],"TSLOT-40-SLOT8"),
			ComponentRecipeSupport.length("width",40),ComponentRecipeSupport.length("height",40),
			ComponentRecipeSupport.length("length",100),ComponentRecipeSupport.optionalText("section")
		],v -> {
			var section=v.optionalToken("section");
			return new ProfileMember(TSlotExtrusion.forSystem(v.token("system"),v.number("width"),v.number("height"),
				section==null ? null : DrawingExtrusionProfile.fromText(section)),v.number("length"));
		});
		return tslotRecipe;
	}

	/** Section-defined members such as slotted plates; exact lines/arcs/holes travel with the assembly. */
	public static function drawingRecipeType():ComponentType {
		if(drawingRecipe==null) drawingRecipe=new ProfileMemberRecipeType("machinekit.structural.drawing-profile-member",[
			ComponentRecipeSupport.text("section",DrawingExtrusionProfile.defaultSectionText()),
			ComponentRecipeSupport.length("length",100)
		],v -> new ProfileMember(DrawingExtrusionProfile.fromText(v.token("section")),v.number("length")));
		return drawingRecipe;
	}

	override public function componentType():Null<ComponentType> {
		if(Std.isOfType(profile,DrawingExtrusionProfile)) return drawingRecipeType();
		if(Std.isOfType(profile,TSlotExtrusion)) {
			var slot:TSlotExtrusion=cast profile;
			if(portableSlot(slot)) return tslotRecipeType();
		}
		return null;
	}

	override public function values():ComponentValues {
		var values=new ComponentValues().setNumber("length",length).setToken("material",materialSpec());
		if(Std.isOfType(profile,DrawingExtrusionProfile)) {
			var drawing:DrawingExtrusionProfile=cast profile;
			return values.setToken("section",drawing.sectionText());
		}
		if(Std.isOfType(profile,TSlotExtrusion)) {
			var slot:TSlotExtrusion=cast profile;
			if(portableSlot(slot)) {
				values.setToken("system",slot.systemId).setNumber("width",slot.size).setNumber("height",slot.height);
				var drawing=slot.drawingSection();
				return drawing==null ? values.setUnset("section") : values.setToken("section",drawing.sectionText());
			}
		}
		throw "Structural profile has no portable recipe";
	}

	/** A hand-authored spec cannot claim a registered system while changing its recipe geometry. */
	static function portableSlot(slot:TSlotExtrusion):Bool {
		if(slot.systemId==null || (slot.sectionVariantId!=null && slot.drawingSection()==null)) return false;
		var expected:TSlotExtrusion;
		try expected=TSlotExtrusion.forSystem(slot.systemId,slot.size,slot.height) catch(_:Dynamic) return false;
		var designation=expected.designation+(slot.sectionVariantId==null ? "" : "-SECTION-"+slot.sectionVariantId);
		if(slot.designation!=designation || slot.family!=expected.family) return false;
		if(slot.slotWidth!=expected.slotWidth || slot.slotDepth!=expected.slotDepth || slot.throatDepth!=expected.throatDepth ||
			slot.headDepth!=expected.headDepth || slot.tWidth!=expected.tWidth || slot.boreDiameter!=expected.boreDiameter) return false;
		for(face in 0...4) {
			if(slot.sectionVariantId==null && slot.modeledSlotDepth(face)!=expected.modeledSlotDepth(face)) return false;
			var a=slot.slotPositions(face),b=expected.slotPositions(face);
			if(a.length!=b.length) return false;
			for(i in 0...a.length) if(a[i]!=b[i]) return false;
		}
		var a=slot.boreCentres(),b=expected.boreCentres();
		if(a.length!=b.length) return false;
		for(i in 0...a.length) if(a[i].x!=b[i].x || a[i].y!=b[i].y) return false;
		return true;
	}
}

/** Saved values retain section geometry; identity keys use a compact digest of that payload. */
private class ProfileMemberRecipeType extends ComponentType {
	public function new(id:String,inputs:Array<ComponentParameter>,build:ComponentValues->MachineComponent) {
		super(id,inputs,build,true);
	}
	function compact(values:ComponentValues):ComponentValues {
		var result=resolve(values);
		switch result.get("section") {
			case Token(text):
				result.setToken("section",DrawingExtrusionProfile.fromText(text).profileDesignation());
			default:
		}
		return result;
	}
	override public function key(values:ComponentValues):String return super.key(compact(values));
	override public function bomKey(values:ComponentValues):String return super.bomKey(compact(values));
	override public function partNumber(component:MachineComponent):String
		return component.designation+(component.materialSpec()=="steel" ? "" : "-M"+component.materialId);
}
