# Text editor performance (2026-09-23)

## Method

The baseline used UIKit from `e67f1714`, immediately before the retained-layout
change. The current build used the same HashLink runtime and native UIKit library.
The synthetic document contained distinct, numbered 81-byte ASCII lines. Both versions used
IBM Plex Sans at an 800-pixel layout width. Measurements ran headlessly on Linux;
they include native layout and UI frame submission, but not GPU presentation.

The core workload loaded each document, inserted and deleted one character near
the middle five times, then moved the caret to
20 positions. The UI workload alternated Control+End and Control+Home in an
800×600 multiline `TextField`, then inserted and deleted a character at the
start eight times and inserted one newline. Each reported result is the median
of three runs.

## Results

| Lines | Baseline core edit | Current core edit | Current UI caret jump | Current UI insert + frame | Current UI delete + frame | Current UI newline + frame |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 1,000 | 29 ms | 4.3 ms | 0.22 ms | 5.9 ms | 6.0 ms | 7.5 ms |
| 5,000 | 149 ms | 13 ms | 0.69 ms | 19.5 ms | 20.1 ms | 21.2 ms |
| 10,000 | 309 ms | 24 ms | 1.27 ms | 36.5 ms | 38.5 ms | 38.7 ms |

The previous full-document text node failed native UI submission at 800 lines in
this workload. The first paragraph-backed version exhausted native layout
resources at 5,000 lines. Grouping 64 paragraphs per retained layout removed
that limit for the measured 10,000-line case. The current layout paints only
chunks intersecting the visible range.

Initial load remains linear: about 0.13 s, 0.63 s, and 1.26 s for 1,000,
5,000, and 10,000 lines. Constructing the whole-document Unicode offset map
alone took about 0.47 s at 10,000 lines. In a 10,000-line middle edit, document
byte assembly took about 7 ms and updating the offset map about 13 ms; the
rest of the roughly 24 ms core edit included layout and state work.

Repeated full UI caret jumps initially took about 279 ms at 10,000 lines
(input and frame combined). Passing the cached code-point length into text
semantics reduced that to about 1.27 ms. The text node now spans document height
so its local clip includes scrolled chunks; its parent clips to the viewport.

With ordinal 64-paragraph chunks, one newline near the start of distinct text
reshaped most of the document and took about 295 ms at 10,000 lines. The
incremental paragraph edit path keeps unaffected chunks stable and reduced
that operation to about 39 ms through the full UI workload.

## Next bottleneck

Typing at the start of 10,000 lines still exceeds a 16 ms frame budget. The
whole-document offset arrays and text bytes are shifted or copied for each
edit. A segmented document and offset index is the next performance change to
evaluate. These measurements do not establish a need for tile rendering.

The workload is synthetic ASCII text with one font. It does not measure GPU
rasterization, long wrapped lines, syntax highlighting, or IME composition.
