# Universal Animation Library tools

`extract_clips.py` reads clips from the library's Godot pack and writes one glTF per clip on the free pack's rig, with provenance;
`build_library.py` assembles the runtime library from them; `godot_resource.py` is a reader for Godot 4 binary resources and
`ual_retarget.py` the retargeting. See `animkit/assets/quaternius-ual/PROVENANCE.md` for what they do and how to rebuild.
They need Python 3 with numpy, and the `zstd` command for the compressed library.
