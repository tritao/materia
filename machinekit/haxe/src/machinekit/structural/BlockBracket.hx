package machinekit.structural;

import cadkit.modeling.Align;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.CollisionHullFacet;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** One block of a `BlockBracket`: its corners in the bracket's own frame, millimetres. */
typedef BracketBlock = {min:Array<Float>, max:Array<Float>};

/**
 * A small bracket made of rectangular blocks square to its own frame, welded or machined into one part: what holds a
 * switch or a sensor off the member it is bolted to. Its blocks are given in its own frame.
 */
class BlockBracket extends MachineComponent {
  final label:String;
  static final recipe = new machinekit.cache.Once<machinekit.component.ComponentType>();
  public final blocks:Array<BracketBlock>;

  public function new(name:String, blocks:Array<BracketBlock>, material:String = "aluminium 6061") {
    if (blocks == null || blocks.length == 0) throw 'Bracket "$name" needs a block';
    for (block in blocks) {
      if (block.min == null || block.max == null
        || block.min.length != 3 || block.max.length != 3) throw "Bracket needs three dimensional corners";
      for (i in 0...3) if (!Math.isFinite(block.min[i] + block.max[i])
        || !(block.max[i] - block.min[i] > 0)) throw 'Bracket "$name" has an empty or nonfinite block';
    }
    super(numberFor(name, blocks), name, material, true);
    addFacet(new machinekit.component.EngineeringMetadata(new machinekit.component.ComponentValues()
      .setToken("ratingsBasis", "Stated").setToken("errorsBasis", "Assumed").setToken("source", "")));
    this.label = name;
    this.blocks = [for (block in blocks) {
      min:block.min.copy(), max:block.max.copy()
    }
    ];
    this.blocks.sort((a, b) -> Reflect.compare(blockKey(a), blockKey(b)));
    // Preserve the space inside an L-shaped support in simulation and planning.
    addFacet(CollisionHullFacet.fromBoxes([for (block in this.blocks)[
      block.min[0],
      block.min[1],
      block.min[2],
      block.max[0],
      block.max[1],
      block.max[2]
    ]]));
  }

  /** Identity uses exact block inputs; block declaration order does not change the part. */
  static function blockKey(block:BracketBlock):String return [for (axis in 0...3)
    machinekit.component.ComponentNumberKey.encode(block.min[axis]) + ":"
    + machinekit.component.ComponentNumberKey.encode(block.max[axis])].join(",");

  static function numberFor(name:String, blocks:Array<BracketBlock>):String {
    var keys = [for (block in blocks) blockKey(block)];
    keys.sort(Reflect.compare);
    return machinekit.component.PartNumbers.geometry("BRACKET", name, keys.join(";"));
  }

  static function readBlocks(text:String):Array<BracketBlock> {
    var raw:Array<Dynamic> = haxe.Json.parse(text);
    return [for (item in raw) {
      var low:Array<Dynamic> = item.min, high:Array<Dynamic> = item.max;
      {
        min:[for (v in low) Std.parseFloat(Std.string(v))], max:[for (v in high) Std.parseFloat(Std.string(v))]
      };
    }
    ];
  }

  public static function recipeType():machinekit.component.ComponentType {
    return recipe.get(() -> new machinekit.component.ComponentType(
      "machinekit.structural.block-bracket",
      [
        machinekit.component.ComponentRecipeSupport.text("label", "bracket"),
        machinekit.component.ComponentRecipeSupport.text("blocks", '[{"min":[0,0,0],"max":[10,10,10]}]')
      ],
      v -> new BlockBracket(v.token("label"), readBlocks(v.token("blocks"))),
      true
    ).numbered(v -> numberFor(v.token("label"), readBlocks(v.token("blocks"))))
      .procured(_ -> Make("CNC machining")));
  }
  override public function componentType():Null<machinekit.component.ComponentType> return recipeType();
  override public function values():machinekit
    .component.ComponentValues return new machinekit.component.ComponentValues().setToken(
      "label",
      label
    ).setToken(
      "blocks",
      haxe.Json.stringify(blocks)
    ).setToken(
      "material",
      materialSpec()
    );

  override public function hasGeometry():Bool return true;

  override public function geometry(detail:ComponentDetail = Preview):Part {
    var parts = [for (block in blocks) Part.box(
      block.max[0] - block.min[0],
      block.max[1] - block.min[1],
      block.max[2] - block.min[2],
      Min,
      Min,
      Min
    ).translated(new Vector(
      block.min[0],
      block.min[1],
      block.min[2]
    ))];
    return parts.length == 1 ? parts[0] : Solids.union(parts);
  }
}
