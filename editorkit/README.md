# EditorKit

EditorKit owns editable UTF-8 text and its document coordinates. UIKit owns the
widget, native text layout, viewport, focus, and input events.

`TextDocument` stores text in roughly 2 KiB UTF-8 segments. It prefers a nearby
newline, but also splits long paragraphs at code-point boundaries. Each
segment has its own `TextOffsetMap`; paragraph ranges that cross segment
boundaries are joined when queried. An edit rebuilds the affected segments and
the segment prefix index while leaving other segments' offset arrays in place.

The document exposes code-point range replacement and slicing, coordinate
conversion, and paragraph lookup. `text` is assembled lazily for clients that
need a complete string. UIKit still materializes the complete value for its
native paragraph layout and the compatibility `TextField.value` and
`onChange(String)` APIs. Long-paragraph shaping remains linear in paragraph
size; bounding document index segments alone does not bound that layout work.

Run the headless Unicode coordinate and edit check with
`./haxeon/scripts/haxeon run --project editorkit/tests/haxeon.json` from the
Materia root. It checks Unicode edits in both many-paragraph documents and a
long paragraph that crosses multiple storage segments.
