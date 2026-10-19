import machinekit.assembly.MachineAssembly;
import machinekit.component.SourcingPlan;
import machinekit.standard.SocketHeadCapScrew;
import machinekit.standard.PropertyClass;
import materia.assembly.AssemblyFrames;
import cadkit.Shape;
import cadkit.modeling.Part;
import machinekit.component.SupplierRecipeLibrary;
import machinekit.component.SupplierCadComponent;
import machinekit.component.ComponentRegistry;
import machinekit.component.ComponentValues;
import machinekit.motion.LinearRail;
import machinekit.structural.DrawingExtrusionProfile;
import machinekit.structural.ProfileMember;
import machinekit.structural.TSlotExtrusion;
import machinekit.structural.FrameAssembly.StructuralProfile;

/** Generates generic ISO 4762 geometry from supplier-derived size/class/length mappings.
 * Supplier dimensions remain evidence: discrepancies do not rewrite generic geometry.
 */
class SupplierFasteners {
    static function main() {
        SourcingChecks.run();
        TSlotFamilyChecks.run();
        ProfileRecipeChecks.run();
        var root = Sys.getEnv("MATERIA_ROOT");
        if (root == null) root = Sys.getCwd();
        var output = root + "/build/suppliers/vallder";
        var input:Dynamic = haxe.Json.parse(sys.io.File.getContent(output + "/fasteners.json"));
        var mappings:Array<Dynamic> = input.mappings;
        var registry = machinekit.component.MachineKitComponents.standard();
        var library = new SupplierRecipeLibrary(sys.io.File.getContent(output + "/recipes.json"),registry);
        var assembly = new MachineAssembly(), plan = new SourcingPlan();
        var geometry:Array<Dynamic> = [];
        var exportStep = Sys.args().indexOf("--export-step") >= 0;
        var directory = output + "/step";
        if (exportStep && !sys.FileSystem.exists(directory)) sys.FileSystem.createDirectory(directory);
        for (i in 0...mappings.length) {
            var mapping = mappings[i], values = mapping.values;
            if (mapping.type_id != "machinekit.standard.socket-head-cap-screw" ||
                (values.propertyClass != "A2-70" && values.propertyClass != "8.8"))
                throw "Unsupported fastener mapping";
            var screw:SocketHeadCapScrew = cast library.create(mapping.supplier_id,mapping.sku);
            var part = screw.geometry();
            var filename = screw.designation + ".step";
            var digest:Null<String> = null;
            try {
                if (!part.valid() || part.volume() <= 0) throw "Invalid generated fastener";
                if (exportStep) {
                part.exportStep(directory + "/" + filename);
                var roundtrip = new Part(Shape.importStep(directory + "/" + filename));
                try {
                    if (!roundtrip.valid() || Math.abs(roundtrip.volume()-part.volume()) > 0.0001)
                        throw "STEP roundtrip changed fastener volume";
                } catch (error:Dynamic) { roundtrip.close(); throw error; }
                roundtrip.close();
                var bytes = haxe.crypto.Sha256.make(sys.io.File.getBytes(directory + "/" + filename));
                var hex = new StringBuf();
                for (j in 0...bytes.length) hex.add(StringTools.hex(bytes.get(j),2).toLowerCase());
                digest = hex.toString();
                }
            } catch (error:Dynamic) { part.close(); throw error; }
            part.close();
            // Two occurrences demonstrate supplier-independent BOM aggregation.
            assembly.addComponent('screw-$i-a', screw, AssemblyFrames.translation(i*25,0,0));
            assembly.addComponent('screw-$i-b', screw, AssemblyFrames.translation(i*25,25,0));
            var price:Dynamic = mapping.price.raw;
            var minor:Null<Int> = price.price == null || price.price == "" ? null : Std.parseInt(price.price);
            if (minor != null && Std.string(minor) != price.price) throw "Invalid price minor units";
            var offerId = mapping.supplier_id + ":" + mapping.supplier_product_id;
            plan.add({id:offerId, partNumber:screw.bom.partNumber, supplierId:mapping.supplier_id,
                supplierSku:mapping.sku, productUrl:mapping.url, observedAt:mapping.source.fetched_at,
                currency:price.currency_code, currencyMinorUnit:price.currency_minor_unit,
                unitPriceMinor:minor, taxBasis:mapping.price.tax_basis,
                inStock:mapping.availability.is_in_stock == true, compatibilityVerified:false});
            plan.select(screw.bom.partNumber, offerId);
            geometry.push({partNumber:screw.bom.partNumber, typeId:mapping.type_id, values:values,
                geometrySource:"generator", step:exportStep ? "step/" + filename : null, sha256:digest, geometryStatus:"generic_nominal",
                sourceUrl:mapping.source_url, supplierEvidence:mapping.evidence,
                dimensionalComparison:{headDiameter:{model:screw.spec.headDiameter, supplier:mapping.evidence.head_diameter_mm},
                    headHeight:{model:screw.spec.headHeight, supplier:mapping.evidence.head_height_mm}},
                compatibilityVerified:false});
        }
        var bom = assembly.billOfMaterials();
        for (line in bom.lines()) if (line.quantity != 2) throw "Expected two occurrences per engineering part";
        var sourced = plan.lines(bom);
        for (line in sourced) if (line.status != "review_required") throw "Unreviewed offer qualified unexpectedly";
        save(output + "/geometry.json", geometry);
        save(output + "/engineering-bom.json", bom.lines());
        save(output + "/sourcing-bom.json", sourced);
        sys.io.File.saveContent(output + "/fasteners.assembly.json", assembly.encode());
        // Cut stock is configured at runtime: a supplier SKU does not invent a cut length.
        var missingLengthRejected = false;
        try library.create("vallder","1058001") catch (_:Dynamic) missingLengthRejected = true;
        if (!missingLengthRejected) throw "Rail binding silently chose a cut length";
        var overrideRejected = false;
        try library.create("vallder",mappings[0].sku,new ComponentValues().setToken("size","M20"))
            catch (_:Dynamic) overrideRejected = true;
        if (!overrideRejected) throw "Supplier-bound screw size was overridden";
        var rail = library.create("vallder","1058001",new ComponentValues().setNumber("length",300));
        var railShape = rail.geometry();
        if (!railShape.valid()) throw "Invalid generator rail";
        railShape.close();
        save(output + "/rail-recipe-example.json",{bom:rail.bom,geometrySource:"generator",lengthMm:300,
            sourcingStatus:"quote_required",note:"Price per metre and cutting charges require separate treatment"});
        var profileInput:Dynamic=haxe.Json.parse(sys.io.File.getContent(output + "/profiles.json"));
        var profiles:Array<Dynamic> = profileInput.profiles, profileResults:Array<Dynamic> = [];
        var profileOffersInput:Dynamic=haxe.Json.parse(sys.io.File.getContent(output + "/profile-families.json"));
        var profileOffers:Array<Dynamic> = profileOffersInput.supplier_offers;
        var profileAssembly=new MachineAssembly(),profilePlan=new SourcingPlan();
        for(section in profiles) {
            var dimensions:Array<Float> = section.dimensions_mm;
            var drawing=new DrawingExtrusionProfile(section);
            var changedSource:Dynamic=haxe.Json.parse(haxe.Json.stringify(section));
            changedSource.supplier_id="another-supplier";changedSource.sku="different-sku";
            changedSource.source_url="https://example.test/another-source";
            var other=new DrawingExtrusionProfile(changedSource);
            if(drawing.profileDesignation()!=other.profileDesignation()) throw "Supplier changed section geometry identity";
            if(section.system_id!=null) {
                var mismatchedBoundsRejected=false;
                try TSlotExtrusion.forSystem("TSLOT-40-SLOT8",120,120,drawing)
                    catch(_:Dynamic) mismatchedBoundsRejected=true;
                if(!mismatchedBoundsRejected) throw "Section variant attached to a different sized family";
            }
            var profile:StructuralProfile=section.system_id==null ? drawing :
                TSlotExtrusion.forSystem(section.system_id,dimensions[0],dimensions[1],drawing);
            var member=new ProfileMember(profile,100,"unspecified"),shape=member.geometry();
            profileAssembly.addComponent("profile-"+section.sku,member);
            for(offer in profileOffers) if(offer.sku==section.sku && offer.source_section_sha256==section.source_sha256) {
                var raw:Dynamic=offer.price.raw;
                profilePlan.add({id:offer.supplier_id+":"+offer.sku,partNumber:member.bom.partNumber,
                    supplierId:offer.supplier_id,supplierSku:offer.sku,productUrl:offer.url,
                    observedAt:offer.source.fetched_at,currency:raw.currency_code,currencyMinorUnit:raw.currency_minor_unit,
                    unitPriceMinor:null,taxBasis:offer.price.tax_basis,inStock:offer.availability.is_in_stock==true,
                    compatibilityVerified:false});
            }
            var bounds=shape.shape.bounds(),low=bounds.get_min(),high=bounds.get_max();
            if(Math.abs(high.get_x()-low.get_x()-dimensions[0])>0.02 ||
               Math.abs(high.get_y()-low.get_y()-dimensions[1])>0.02)
                throw "Drawing section native bounds differ from supplier dimensions";
            var volume=shape.volume();shape.close();
            var longer=profile.geometry(250);
            if(Math.abs(longer.volume()/volume-2.5)>0.00001) throw "Profile length parameter changed section";
            longer.close();
            profileResults.push({productId:section.product_id,sku:section.sku,bom:member.bom,
                familyId:section.family_id,systemId:section.system_id,sectionVariantId:drawing.profileDesignation(),
                geometrySource:section.system_id==null ? "drawing_extrusion" : "tslot_section_variant",
                status:"valid_parametric_geometry",sourceSha256:section.source_sha256,
                testedLengthsMm:[100,250],areaMm2:volume/100,compatibilityVerified:false});
        }
        save(output + "/profile-validation.json",profileResults);
        var savedProfiles=profileAssembly.encode(),reloadedProfiles=MachineAssembly.decode(savedProfiles);
        if(haxe.Json.stringify(profileAssembly.billOfMaterials().lines())!=haxe.Json.stringify(reloadedProfiles.billOfMaterials().lines()))
            throw "Measured profile BOM changed on reload";
        var originalProfiles=profileAssembly.components(),restoredProfiles=reloadedProfiles.components();
        for(i in 0...originalProfiles.length) {
            var original=originalProfiles[i].component,restored=restoredProfiles[i].component;
            if(original.geometryKey()!=restored.geometryKey()) throw "Measured section geometry identity changed on reload";
            var a=original.geometry(),b=restored.geometry();
            if(!b.valid() || Math.abs(a.volume()-b.volume())>0.001) throw "Measured section geometry changed on reload";
            a.close();b.close();
        }
        sys.io.File.saveContent(output + "/profiles.assembly.json",savedProfiles);
        var profileLines=profilePlan.lines(profileAssembly.billOfMaterials());
        for(line in profileLines) if(line.selected!=null || line.offers.length!=1 || line.estimatedLinePriceMinor!=null)
            throw "Profile sourcing must remain optional and require a cut-length quote";
        save(output + "/profile-sourcing-bom.json",profileLines);
        var importedLibrary:Dynamic = haxe.Json.parse(sys.io.File.getContent(output + "/cad/library.json"));
        var assets:Array<Dynamic> = importedLibrary.assets;
        for (asset in assets) if (asset.usable_geometry == true && asset.identity_status == "table_linked") {
            var wrongHashRejected = false;
            try new SupplierCadComponent(output + "/" + asset.file,"invalid","test","Test")
                catch (_:Dynamic) wrongHashRejected = true;
            if (!wrongHashRejected) throw "Supplier CAD hash mismatch accepted";
            var reference = new SupplierCadComponent(output + "/" + asset.file,asset.sha256,
                "CAD-" + asset.sha256,"Supplier reference; mounting interfaces unqualified");
            var shape = reference.geometry();
            reference.close();
            if (!shape.valid() || reference.connectors().length != 0) throw "Invalid CAD reference ownership or inferred mounts";
            shape.close();
            var useAfterCloseRejected = false;
            try reference.geometry() catch (_:Dynamic) useAfterCloseRejected = true;
            if (!useAfterCloseRejected) throw "Supplier import did not close";
            save(output + "/imported-reference-example.json",{bom:reference.bom,assetId:asset.asset_id,
                geometrySource:"supplier_cad",mountingFrameStatus:"unverified"});
            break;
        }
        Sys.println('Built ${geometry.length} generator configurations, ${mappings.length*2} occurrences and optional sourcing BOM; STEP export: $exportStep');
    }

    static function save(path:String, value:Dynamic):Void sys.io.File.saveContent(path,haxe.Json.stringify(value,null,"  ") + "\n");
}
