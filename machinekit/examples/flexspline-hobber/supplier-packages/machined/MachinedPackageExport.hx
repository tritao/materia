import cadkit.MeshFeatureEdges;
import cadkit.modeling.Location;
import cadkit.modeling.Part;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import machinekit.assembly.PosedParts;
import machinekit.component.MachineComponent;
import machinekit.component.ManufacturingRoute;
import machinekit.component.ManufacturingRoute.RouteHole;
import machinekit.hobbing.HobberDrives;
import machinekit.hobbing.*;
import machinekit.motion.BallNut;
import machinekit.motion.NutBlock;

/** Writes the machined and ground-stock parts of FlexsplineHobber for a machine shop or a ground-stock vendor: one STEP per part in
    its own frame (longest side along X, shortest along Z, minimum corner at the origin, holes cut), and machined-package.json with
    the route, stock, mass, holes (probed in the solid: present, through or blind), a tessellation for the drawings and the STEP
    re-import volume. The installed routed components and their joint machining are the source
    FlexsplineHobber installs. machined_package.py makes the drawings, the tables and the checks.
    Usage: hl main.hl <output dir>. */
class MachinedPackageExport {


  static function r(v:Float, digits:Int = 4):Float {
    var f = Math.pow(10, digits);
    return Math.round(v * f) / f;
  }

  static function cylinderAlong(position:Array<Float>, direction:Array<Float>, radius:Float, from:Float, to:Float):Part {
    var cylinder = Part.cylinderSpan(radius, from, to);
    var d = new Vector(direction[0], direction[1], direction[2]);
    var up = Math.abs(direction[2]) < 0.9 ? new Vector(0, 0, 1) : new Vector(1, 0, 0);
    // A plane needs x perpendicular to the normal.
    var x = up.subtract(d.scale(up.dot(d))).normalized();
    var placed = cylinder.placed(new Location(new Plane(new Vector(position[0], position[1], position[2]), x, d)));
    cylinder.close();
    return placed;
  }

  /** The probe of a slot: the cylinders at its two ends and the box between them (axis-aligned: the part frame is a permutation of the work frame). */
  static function stadiumAlong(hole:RouteHole, radius:Float, from:Float, to:Float):Part {
    var a = hole.slotAxis, d = hole.direction, half = (hole.slotLength ?? 0.0) / 2, p = hole.position;
    var w = [d[1] * a[2] - d[2] * a[1], d[2] * a[0] - d[0] * a[2], d[0] * a[1] - d[1] * a[0]];
    var lo = [1e9, 1e9, 1e9], hi = [-1e9, -1e9, -1e9];
    for (sa in [-1.0, 1.0]) for (sw in [-1.0, 1.0]) for (sd in [from, to]) for (i in 0...3) {
      var v = p[i] + sd * d[i] + sa * half * a[i] + sw * radius * w[i];
      lo[i] = Math.min(lo[i], v);
      hi[i] = Math.max(hi[i], v);
    }
    var raw = Part.box(hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]), middle = raw.translated(new Vector((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, lo[2]));
    raw.close();
    var parts = [middle];
    for (s in [-1.0, 1.0]) parts.push(cylinderAlong([p[0] + s * half * a[0], p[1] + s * half * a[1], p[2] + s * half * a[2]], d, radius, from, to));
    var joined = machinekit.component.Solids.union(parts);
    for (q in parts) q.close();
    return joined;
  }

  /** Volume of the solid inside a probe cylinder (a stadium for a slot) along a hole's axis between `from` and `to` (mm from the mouth). */
  static function probe(body:Part, hole:RouteHole, radius:Float, from:Float, to:Float):{inside:Float, whole:Float} {
    var slot = hole.slotLength != null && hole.slotLength > 0;
    var tool = slot ? stadiumAlong(hole, radius, from, to) : cylinderAlong(hole.position, hole.direction, radius, from, to);
    var common = body.intersect(tool);
    var inside = common.volume(), whole = (Math.PI * radius * radius + (slot ? 2 * radius * (hole.slotLength ?? 0.0) : 0)) * (to - from);
    common.close();
    tool.close();
    return {inside: inside, whole: whole};
  }

  static function holeJson(body:Part, hole:RouteHole):Dynamic {
    var depth = hole.depth, r0 = hole.diameter / 2, k = 1.02, slotExtra = hole.slotLength != null && hole.slotLength > 0 ? 2 * r0 * (hole.slotLength ?? 0.0) : 0.0;
    // Empty bore at 0.98 r; the material in a 1.02 r probe is the wall ring, so its volume gives the length of wall the hole runs
    // through: L = V / (pi r^2 (k^2 - 1)). A hole that is missing gives 25 times that, a wider one gives none.
    var inner = probe(body, hole, r0 * 0.98, 0.05, depth - 0.05), outer = probe(body, hole, r0 * k, 0, depth);
    var emptyBore = inner.inside < 0.005 * inner.whole;
    // A slot's ring area is that of the offset stadium: pi r^2 (k^2 - 1) + 2 r L (k - 1).
    var wall = outer.inside / (Math.PI * r0 * r0 * (k * k - 1) + (slotExtra > 0 ? slotExtra * (k - 1) : 0.0));
    var behind = probe(body, hole, r0 * 0.9, depth + 0.1, depth + 0.6);
    var backing = behind.inside / behind.whole;
    var through = backing < 0.5;
    var present = emptyBore && wall > 0.5 && wall <= depth + 0.3 && (through || Math.abs(wall - depth) < 0.3);
    return {
      id: hole.id,
      kind: hole.kind,
      thread: hole.thread,
      diameter: r(hole.diameter),
      nominal: r(hole.nominal),
      fit: hole.fit,
      depth: r(hole.depth),
      position: [for (v in hole.position) r(v)],
      direction: [for (v in hole.direction) r(v, 6)],
      slotLength: hole.slotLength == null ? 0 : r(hole.slotLength),
      slotAxis: hole.slotAxis == null ? null : [for (v in hole.slotAxis) r(v, 6)],
      present: present,
      emptyBore: emptyBore,
      wallLength: r(wall, 3),
      backing: r(backing),
      through: through,
      cutDepth: r(Math.min(wall, depth), 3)
    };
  }

  static function stockJson(route:ManufacturingRoute):Dynamic {
    var s = route.stock;
    if (s == null) return null;
    return {
      id: s.id, kind: s.kind, material: s.material, thickness: s.thickness, thicknessTolerance: s.thicknessTolerance,
      flatness: s.flatness, parallelism: s.parallelism, roughnessRa: s.roughnessRa, source: s.source, assumed: s.assumed
    };
  }

  static function routeJson(route:ManufacturingRoute):Dynamic return {
    kind: Std.string(route.kind),
    process: route.process(),
    supplier: route.supplier(),
    setups: route.setups,
    toleranceClass: route.toleranceClass,
    stock: stockJson(route),
    sheet: route.sheet,
    bar: route.bar,
    faces: [for (f in route.machinedFaces()) {name: f.name, tolerance: f.tolerance, roughnessRa: f.roughnessRa}]
  };

  static function floats(bytes:haxe.io.Bytes, count:Int, digits:Int):Array<Float> return [for (i in 0...count) r(bytes.getDouble(i * 8), digits)];

  /** One part: STEP, JSON entry. `holes` are in the authored frame; both are moved into the part's drawing frame. */
  static function exportPart(dir:String, component:MachineComponent, holes:Array<RouteHole>, quantity:Int, title:String):Dynamic {
    var route = ManufacturingRoute.of(component);
    if (route == null) throw 'No manufacturing route on ${component.designation}';
    var authored = component.geometry();
    var box = PosedParts.boxOf(authored);
    var lo = [box.minX, box.minY, box.minZ], hi = [box.maxX, box.maxY, box.maxZ];
    var dims = [for (i in 0...3) hi[i] - lo[i]];
    // Old axis o0 (longest) becomes X, o1 Y, o2 Z; an odd permutation flips Z so the move stays a rotation.
    var order = [0, 1, 2];
    order.sort((a, b) -> dims[a] > dims[b] + 1e-9 ? -1 : dims[b] > dims[a] + 1e-9 ? 1 : a - b);
    var images:Array<Array<Float>> = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
    images[order[0]][0] = 1;
    images[order[1]][1] = 1;
    var inversions = 0;
    for (i in 0...3) for (j in i + 1...3) if (order[i] > order[j]) inversions++;
    images[order[2]][2] = inversions % 2 == 0 ? 1 : -1;
    function image(p:Array<Float>):Array<Float> return [for (k in 0...3) p[0] * images[0][k] + p[1] * images[1][k] + p[2] * images[2][k]];
    var rotated = authored.placed(new Location(new Plane(new Vector(0, 0, 0), new Vector(images[0][0], images[0][1], images[0][2]),
      new Vector(images[2][0], images[2][1], images[2][2]))));
    authored.close();
    var rb = PosedParts.boxOf(rotated), shift = [-rb.minX, -rb.minY, -rb.minZ];
    var part = rotated.translated(new Vector(shift[0], shift[1], shift[2]));
    rotated.close();
    function local(p:Array<Float>):Array<Float> {
      var q = image(p);
      return [q[0] + shift[0], q[1] + shift[1], q[2] + shift[2]];
    }
    function turn(v:Array<Float>):Array<Float> return image(v);
    var size = PosedParts.boxOf(part);
    var moved:Array<RouteHole> = [for (h in holes) {
      id: h.id, kind: h.kind, thread: h.thread, diameter: h.diameter, nominal: h.nominal, fit: h.fit, depth: h.depth,
      position: local(h.position), direction: turn(h.direction), slotLength: h.slotLength, slotAxis: h.slotAxis == null ? null : turn(h.slotAxis)
    }];
    var id = component.designation, volume = part.volume();
    var physical = materia.project.MaterialLibrary.require(component.materialId).physical;
    var mass = volume * 1e-9 * physical.density;
    var stepPath = '$dir/$id.step';
    part.shape.exportStep(stepPath);
    var back = cadkit.Shape.importStep(stepPath);
    var stepVolume = back.volume();
    back.close();
    var holeData = [for (h in moved) holeJson(part, h)];
    var mesh = part.shape.tessellate(0.05, 0.3);
    var vertexCount = mesh.vertexCount, indexCount = mesh.indexCount;
    var vertices = [for (i in 0...vertexCount * 3) r(mesh.vertices.getDouble(i * 8), 3)];
    var triangles = [for (i in 0...indexCount) mesh.indices.getInt32(i * 4)];
    var flags = MeshFeatureEdges.ofMesh(mesh), edges:Array<Float> = [];
    for (i in 0...flags.length) if (flags[i]) for (k in 0...6) edges.push(r(mesh.edgeSegments.getDouble(i * 48 + k * 8), 3));
    part.close();
    return {
      id: id,
      title: title,
      partNumber: component.bom.partNumber,
      description: component.description,
      step: '$id.step',
      quantity: quantity,
      material: component.materialSpec(),
      density: physical.density,
      volume: volume,
      stepVolume: stepVolume,
      mass: mass,
      size: [r(size.maxX - size.minX), r(size.maxY - size.minY), r(size.maxZ - size.minZ)],
      authoredSize: dims,
      frame: {images: images, shift: shift, note: "own frame = image(authored) + shift; images[i] is where authored axis i goes"},
      route: routeJson(route),
      holes: holeData,
      mesh: {vertices: vertices, triangles: triangles, edges: edges}
    };
  }

  static function main() {
    var args = Sys.args();
    if (args.length != 1) throw "usage: hl main.hl <output dir>";
    var dir = args[0];
    sys.FileSystem.createDirectory(dir);
    sys.FileSystem.createDirectory(dir + "/step");
    var parts:Array<Dynamic> = [];
    // The parts of the built machine: the holes its bolted joints derive (BOM_PLAN F1) are cut into them.
    var machine = new FlexsplineHobber();
    HobberAssemblyTooling.writeCsv(dir+"/assembly-tooling.csv",machine);
    // The installed assembly is the inventory. A new routed member cannot be
    // forgotten by a second hand-maintained design list. BOM revision checking
    // also prevents two different finished parts sharing one drawing number.
    machine.billOfMaterials();
    var grouped=new Map<String,{component:MachineComponent,quantity:Int,instance:String}>(),order:Array<String>=[];
    for(entry in machine.components())if(ManufacturingRoute.of(entry.component)!=null) {
      var key=entry.component.bom.partNumber,existing=grouped.get(key);
      if(existing==null){grouped.set(key,{component:entry.component,quantity:1,instance:entry.id});order.push(key);}else existing.quantity++;
    }
    for(key in order) {
      var item=grouped.get(key);if(item==null)throw "Missing routed part: "+key;
      var part=item.component,holes=part.derivedRouteHoles();
      if(Std.isOfType(part,HobberMadePart))holes=cast(part,HobberMadePart).holes();
      else if(Std.isOfType(part,NutBlock))holes=cast(part,NutBlock).holes();
      else if(Std.isOfType(part,machinekit.hobbing.HobberChipCover))holes=holes.concat(cast(part,machinekit.hobbing.HobberChipCover).toolHoles());
      else if(Std.isOfType(part,machinekit.motion.ScrewLinearModulePart)) {
        var modulePart=cast(part,machinekit.motion.ScrewLinearModulePart),s=modulePart.spec;
        if(modulePart.role=="nutCarrier") {
          var diameter=new BallNut().screwDiameter+2*(machinekit.motion.ScrewLinearModuleSpec.GAP+0.2);
          holes.push({id:"screw-clearance",kind:"bore",thread:null,diameter:diameter,nominal:diameter,fit:null,depth:s.carrierThickness,
            position:[s.centre-s.carrierThickness/2,0.0,s.axisHeight()],direction:[1.0,0,0]});
        }
      }
      if(part==machine.spindle.arbor)holes.push({id:"pull-stud-thread",kind:"tapped",thread:"M12",diameter:12.0,nominal:12.0,fit:null,depth:20.0,position:[0.0,0,-48.4],direction:[0.0,0,1]});
      if(item.instance=="outboardGland")for(x in [-16.0,16.0])holes.push({id:x<0?"spanner1":"spanner2",kind:"bore",thread:null,diameter:2.0,nominal:2.0,fit:null,depth:2.0,position:[x,0,27],direction:[0.0,0,-1]});
      parts.push(exportPart(dir+"/step",part,holes,item.quantity,part.description));
      Sys.println("exported "+part.designation+" x"+item.quantity);
    }
    var data = {
      machine: "FlexsplineHobber",
      generated: "machined-package export",
      barWidths: machinekit.structural.GroundStock.barWidths(),
      limits: {maxSetups: ManufacturingRoute.MAX_SETUPS, maxExtent: ManufacturingRoute.MAX_EXTENT},
      parts: parts
    };
    sys.io.File.saveContent('$dir/machined-package.json', haxe.Json.stringify(data));
    Sys.println('wrote $dir/machined-package.json with ${parts.length} parts');
  }
}
