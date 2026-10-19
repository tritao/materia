package machinekit.component;

/** Supplier bindings instantiate existing recipes; this class contains no geometry code.
 * Bound size/material inputs cannot be overridden. Required cut-length inputs remain explicit.
 */
class SupplierRecipeLibrary {
    final registry:ComponentRegistry;
    final bindings:Map<String, Dynamic> = [];

    public function new(text:String, registry:ComponentRegistry) {
        this.registry = registry;
        var data:Dynamic = haxe.Json.parse(text);
        if (data.schema_version != 1) throw "Unsupported supplier recipe library";
        var recipes:Array<Dynamic> = data.recipes;
        for (entry in recipes) {
            var key = entry.supplier_id + ":" + entry.sku;
            if (bindings.exists(key)) throw 'Duplicate supplier recipe "$key"';
            if (!registry.has(entry.type_id)) throw 'Unregistered supplier recipe "${entry.type_id}"';
            bindings.set(key,entry);
        }
    }

    public function create(supplierId:String, sku:String, ?requested:ComponentValues):MachineComponent {
        var entry = bindings.get(supplierId + ":" + sku);
        if (entry == null) throw "No generator binding for supplier SKU";
        var type = registry.byId(entry.type_id), values = new ComponentValues();
        var required:Array<String> = entry.required_inputs;
        if (requested != null) for (name in requested.names())
            if (required.indexOf(name) < 0) throw 'Cannot override supplier-bound parameter "$name"';
        for (name in Reflect.fields(entry.values)) {
            var found = false;
            for (parameter in type.parameters()) if (parameter.name == name) {
                found = true;
                var raw:Dynamic = Reflect.field(entry.values,name);
                switch parameter.type {
                    case Scalar, Length, Angle:
                        if (!Std.isOfType(raw,Float) && !Std.isOfType(raw,Int)) throw 'Invalid numeric binding "$name"';
                        values.setNumber(name,raw);
                    case Choice(_), CatalogDesignation(_), Text:
                        if (!Std.isOfType(raw,String)) throw 'Invalid token binding "$name"';
                        values.setToken(name,raw);
                    case Bool:
                        if (!Std.isOfType(raw,Bool)) throw 'Invalid boolean binding "$name"';
                        values.setBoolean(name,raw);
                    case Count:
                        if (!Std.isOfType(raw,Int)) throw 'Invalid count binding "$name"';
                        values.setInteger(name,raw);
                    case Optional(_): throw "Optional supplier-bound inputs require an explicit codec";
                }
            }
            if (!found) throw 'Unknown recipe parameter "$name"';
        }
        for (name in required) {
            if (requested == null || requested.get(name) == null) throw 'Supplier recipe requires "$name"';
            if (values.get(name) != null) throw "Required input conflicts with bound recipe value";
            values.set(name,requested.get(name));
        }
        return type.create(values);
    }
}
