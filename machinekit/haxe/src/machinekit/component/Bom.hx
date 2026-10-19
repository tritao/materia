package machinekit.component;

/** Aggregates bill-of-materials lines by part number. */
class Bom {
	final byPartNumber:Map<String, BomItem> = [];
	final order:Array<String> = [];
	final includedOwners:Array<String> = [];

	public function new() {}

	public function add(item:BomItem, quantity:Int = 1):Void {
		if (item.partNumber == null || item.partNumber.length == 0) throw "BOM item needs a part number";
		if (quantity <= 0) throw 'BOM quantity for "${item.partNumber}" must be positive';
		var existing = byPartNumber.get(item.partNumber);
		if (existing == null) {
			byPartNumber.set(item.partNumber, {partNumber: item.partNumber, description: item.description,
				quantity: item.quantity * quantity, material: item.material,
				familyId: item.familyId, manufacturingVariant: item.manufacturingVariant, manufacturingProcess: item.manufacturingProcess,
					typeId: item.typeId, valuesKey: item.valuesKey});
			order.push(item.partNumber);
		} else {
			if (existing.description != item.description || existing.material != item.material ||
				existing.typeId != item.typeId || existing.valuesKey != item.valuesKey ||
				existing.familyId != item.familyId || existing.manufacturingVariant != item.manufacturingVariant || existing.manufacturingProcess != item.manufacturingProcess)
				throw 'BOM part number "${item.partNumber}" has conflicting metadata or values';
			existing.quantity += item.quantity * quantity;
		}
	}

	public function addComponent(component:MachineComponent, quantity:Int = 1):Void {
		if(quantity<=0)throw "BOM component quantity must be positive";
		var included=IncludedInPurchase.of(component);
		if(included==null)add(component.bom, quantity);else includedOwners.push(included.ownerPartNumber);
	}

	/** Lines in first-added order. */
	public function lines():Array<BomItem> {
		for(owner in includedOwners)if(!byPartNumber.exists(owner))throw 'Included purchase owner missing from BOM: $owner';
		var result:Array<BomItem> = [];
		for (partNumber in order) {
			var item = byPartNumber.get(partNumber);
			if (item != null)
				result.push({partNumber: item.partNumber, description: item.description, quantity: item.quantity,
					material: item.material, familyId: item.familyId, manufacturingVariant: item.manufacturingVariant, manufacturingProcess: item.manufacturingProcess,
					typeId: item.typeId, valuesKey: item.valuesKey});
		}
		return result;
	}

	public function quantity(partNumber:String):Int {
		var item = byPartNumber.get(partNumber);
		return item == null ? 0 : item.quantity;
	}
}
