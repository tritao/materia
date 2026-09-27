package cadkit.parametric;

import cadkit.parametric.TypedProperty;

class DefinitionCreateChange implements DocumentChange {
	final document:Document;
	final definition:Definition;
	final index:Int;

	public function new(document, definition, index) {
		this.document = document;
		this.definition = definition;
		this.index = index;
	}

	public function undo():Void
		document.restoreDefinitionRemoval(definition);

	public function redo():Void
		document.restoreDefinitionInsertion(definition, index);
}

class DefinitionDefaultChange implements DocumentChange {
	final document:Document;
	final definition:Definition;
	final name:String;
	final before:Dynamic;
	final beforeRevision:Int;
	final after:Dynamic;
	final afterRevision:Int;

	public function new(doc:Document, d:Definition, n:String, b:Dynamic, br:Int, a:Dynamic, ar:Int) {
		document = doc;
		definition = d;
		name = n;
		before = b;
		beforeRevision = br;
		after = a;
		afterRevision = ar;
	}

	public function undo():Void
		document.restoreDefinitionDefault(definition, name, before, beforeRevision);

	public function redo():Void
		document.restoreDefinitionDefault(definition, name, after, afterRevision);
}

class InstanceOverrideChange implements DocumentChange {
	final document:Document;
	final instance:InstanceElement;
	final name:String;
	final before:Dynamic;
	final after:Dynamic;

	public function new(d:Document, i:InstanceElement, n:String, b:Dynamic, a:Dynamic) {
		document = d;
		instance = i;
		name = n;
		before = b;
		after = a;
	}

	public function undo():Void
		document.restoreInstanceOverride(instance, name, before);

	public function redo():Void
		document.restoreInstanceOverride(instance, name, after);
}

class InstanceDefinitionChange implements DocumentChange {
	final document:Document;
	final instance:InstanceElement;
	final before:DefinitionId;
	final after:DefinitionId;

	public function new(document:Document, instance:InstanceElement, before:DefinitionId, after:DefinitionId) {
		this.document = document;
		this.instance = instance;
		this.before = before;
		this.after = after;
	}

	public function undo():Void
		document.restoreInstanceDefinition(instance, before);

	public function redo():Void
		document.restoreInstanceDefinition(instance, after);
}

class DefinitionPropertyChange implements DocumentChange {
	final document:Document;
	final definition:Definition;
	final name:String;
	final before:Null<TypedProperty>;
	final after:Null<TypedProperty>;

	public function new(document:Document, definition:Definition, name:String, before:Null<TypedProperty>, after:Null<TypedProperty>) {
		this.document = document;
		this.definition = definition;
		this.name = name;
		this.before = before;
		this.after = after;
	}

	public function undo():Void
		document.restoreDefinitionProperty(definition, name, before);

	public function redo():Void
		document.restoreDefinitionProperty(definition, name, after);
}
