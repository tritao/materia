# Optional NativeKit mesh bridge

This example is application code, not a CadKit dependency. With a live
NativeKit GPU `Renderer`, the handoff is deliberately just bulk byte buffers:

```haxe
var shape = Shape.box(100.0, 50.0, 10.0)
    .cut(Shape.cylinder(5.0, 20.0));
var mesh = shape.tessellate();
var gpu = MeshUpload.upload(renderer, mesh);
```

`gpu.vertices`, `gpu.normals`, and `gpu.indices` can be bound to separate
NativeKit vertex/index buffer slots. Configure the pipeline with a 24-byte
vertex stride for the `cad_vec3` streams and `UInt32` indices. The example
does not add NativeKit to CadKit's build or dependency graph.
