# EditorKit

EditorKit owns editable UTF-8 text and its document coordinates. UIKit owns the
widget, native text layout, viewport, focus, and input events.

`TextDocument` stores text in segments that end at paragraph boundaries. Each
segment has its own `TextOffsetMap` for code-point, UTF-8, UTF-16, grapheme, and
paragraph coordinates. An edit rebuilds the affected segments and updates the
small segment prefix index. It leaves the offset arrays for other segments in
place. The current target is roughly 2 KiB per segment; a single paragraph
can exceed that size because splitting inside a paragraph would require more
careful grapheme-boundary handling.

The document exposes code-point range replacement and slicing, coordinate
conversion, and paragraph lookup. `text` is assembled lazily for clients that
need a complete string. UIKit's `TextField.value` and `onChange(String)` still
request that string on every edit, so those API calls retain a whole-document
copy. Initial parsing and layout also remain linear in document size.
