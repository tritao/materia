package nativekit.ui.widgets;

import nativekit.ffi.NativeKitTypes.TextEditAction;
import FontCollection;
import Color;
import LayoutMeasureConstraints;
import LayoutMeasureResult;
import LayoutMeasuredContent;
import LayoutRenderableContent;
import NativeKitEventValue.NativeKitTextEdit;
import ParagraphStyle;
import Rect;
import TextStyle;
import nativekit.editorkit.TextDocument;
import nativekit.editorkit.TextOffsetMap;

/** Persistent editable text, selection and IME composition state for one widget ID. */
class TextEditorState {
	static inline var caretBlinkHalfPeriod:Float = 0.5;

	public var text(default, null):String;
	public var selectionStart(default, null):Int;
	public var selectionEnd(default, null):Int;
	public var compositionStart(default, null):Int;
	public var compositionEnd(default, null):Int;
	public var selectionAnchor(default, null):Int;
	public var selectionFocus(default, null):Int;
	public var selectionAnchorLayoutOffset(default, null):Int;
	public var selectionFocusLayoutOffset(default, null):Int;
	public var selectionAnchorAffinity(default, null):Int;
	public var selectionFocusAffinity(default, null):Int;
	/** Vertical scroll offset in the editor's content coordinate space. */
	public var scrollOffsetY(default, null):Float;
	public var focused:Bool;
	public var draggingSelection:Bool;
	public final layout:TextEditorLayout;
	public final renderContent:LayoutRenderableContent;
	public final textStyle:TextStyle;
	public final paragraphStyle:ParagraphStyle;
	/** Cached conversions between document code points, UTF-8 bytes and UTF-16 units. */
	final offsets:TextDocument;
	final renderMeasurement:LayoutMeasuredContent;
	var renderColor:Color;
	var lastLayoutWidth:Float;
	var lastLayoutText:String;
	var lastPointerClickTime:Float;
	var lastPointerClickX:Float;
	var lastPointerClickY:Float;
	var lastPointerWordStart:Int;
	var lastPointerWordEnd:Int;
	var lastPointerClickCount:Int;
	var lastPointerClickArmed:Bool;
	var pointerClickPending:Bool;
	var viewportHeight:Float;
	var desiredVerticalX:Float;
	var hasDesiredVerticalX:Bool;
	var caretBlinkResetTime:Float;
	var disposed:Bool;

	public function new(fonts:FontCollection, text:String, ?textStyle:TextStyle,
			?paragraphStyle:ParagraphStyle) {
		if (fonts == null || fonts.isDisposed())
			throw "Text editor requires a live font collection";
		this.text = text == null ? "" : text;
		offsets = new TextDocument(this.text);
		this.textStyle = copyTextStyle(textStyle == null ? new TextStyle() : textStyle);
		this.paragraphStyle = copyParagraphStyle(paragraphStyle == null ? new ParagraphStyle() : paragraphStyle);
		var end = offsets.codepointCount;
		selectionStart = end;
		selectionEnd = end;
		selectionAnchor = end;
		selectionFocus = end;
		selectionAnchorLayoutOffset = end;
		selectionFocusLayoutOffset = end;
		selectionAnchorAffinity = 0;
		selectionFocusAffinity = 0;
		scrollOffsetY = 0.0;
		compositionStart = -1;
		compositionEnd = -1;
		focused = false;
		draggingSelection = false;
		layout = new TextEditorLayout(fonts, layoutText(), 1.0, this.textStyle,
			this.paragraphStyle, offsets);
		renderColor = Color.rgba(1.0, 1.0, 1.0, 1.0);
		renderMeasurement = new LayoutMeasuredContent(function(constraints:LayoutMeasureConstraints) {
			if (Math.isFinite(constraints.maxWidth) && constraints.maxWidth > 0.0)
				updateLayout(constraints.maxWidth);
			return layout.measureForConstraints(constraints);
		});
		renderContent = new LayoutRenderableContent(renderMeasurement, function(canvas, geometry) {
			var visible = geometry.visibleLocalBounds();
			canvas.translate(0.0, -scrollOffsetY);
			layout.paint(canvas, renderColor, scrollOffsetY + visible.y,
				scrollOffsetY + visible.y + visible.height);
		});
		lastLayoutWidth = 1.0;
		lastLayoutText = layoutText();
		lastPointerClickTime = -1.0;
		lastPointerClickX = 0.0;
		lastPointerClickY = 0.0;
		lastPointerWordStart = -1;
		lastPointerWordEnd = -1;
		lastPointerClickCount = 0;
		lastPointerClickArmed = false;
		pointerClickPending = false;
		viewportHeight = 0.0;
		desiredVerticalX = 0.0;
		hasDesiredVerticalX = false;
		caretBlinkResetTime = 0.0;
		disposed = false;
	}

	public function syncExternal(value:String):Bool {
		ensureLive();
		var next = value == null ? "" : value;
		if (next == text)
			return false;
		cancelPointerClick();
		text = next;
		offsets.replace(0, offsets.codepointCount, next);
		var caret = offsets.codepointCount;
		selectionStart = caret;
		selectionEnd = caret;
		selectionAnchor = caret;
		selectionFocus = caret;
		selectionAnchorLayoutOffset = caret;
		selectionFocusLayoutOffset = caret;
		selectionAnchorAffinity = 0;
		selectionFocusAffinity = 0;
		resetVerticalNavigation();
		clearComposition();
		layout.setText(layoutText(), offsets);
		renderMeasurement.invalidate();
		lastLayoutText = layoutText();
		return true;
	}

	public function updateLayout(width:Float):Void {
		ensureLive();
		var nextWidth = Math.max(1.0, width);
		var value = layoutText();
		if (nextWidth != lastLayoutWidth || value != lastLayoutText) {
			layout.update(value, nextWidth, textStyle, paragraphStyle, offsets);
			renderMeasurement.invalidate();
			lastLayoutWidth = nextWidth;
			lastLayoutText = value;
		}
		clampScrollOffset();
	}

	/** Updates inherited typography without replacing the retained editor state. */
	public function updateStyle(nextTextStyle:TextStyle, nextParagraphStyle:ParagraphStyle):Bool {
		ensureLive();
		if (nextTextStyle == null || nextParagraphStyle == null)
			throw "Text editor styles cannot be null";
		var changed = textStyle.font != nextTextStyle.font ||
			textStyle.fontSize != nextTextStyle.fontSize ||
			textStyle.letterSpacing != nextTextStyle.letterSpacing ||
			paragraphStyle.wrap != nextParagraphStyle.wrap ||
			paragraphStyle.alignment != nextParagraphStyle.alignment ||
			paragraphStyle.lineHeight != nextParagraphStyle.lineHeight ||
			paragraphStyle.direction != nextParagraphStyle.direction;
		if (!changed)
			return false;
		textStyle.font = nextTextStyle.font;
		textStyle.fontSize = nextTextStyle.fontSize;
		textStyle.letterSpacing = nextTextStyle.letterSpacing;
		paragraphStyle.wrap = nextParagraphStyle.wrap;
		paragraphStyle.alignment = nextParagraphStyle.alignment;
		paragraphStyle.lineHeight = nextParagraphStyle.lineHeight;
		paragraphStyle.direction = nextParagraphStyle.direction;
		layout.update(layoutText(), Math.max(1.0, lastLayoutWidth), textStyle, paragraphStyle, offsets);
		renderMeasurement.invalidate();
		lastLayoutText = layoutText();
		clampScrollOffset();
		return true;
	}

	/** Inserts committed text over the current selection. */
	public function insert(value:String):Bool {
		if (value == null || value.length == 0)
			return false;
		return replace(selectionStart, selectionEnd, value);
	}

	/** Replaces a code-point range and places a collapsed caret after the insertion. */
	public function replace(start:Int, end:Int, value:String):Bool {
		ensureLive();
		var count = offsets.codepointCount;
		var first = clamp(start, 0, count);
		var last = clamp(end, 0, count);
		if (last < first) {
			var swap = first;
			first = last;
			last = swap;
		}
		var caret = first + TextOffsetMap.countCodepoints(value == null ? "" : value);
		return applyTransaction(new EditTransaction(first, last, value, caret, caret));
	}

	/** Applies one replacement and its resulting selection and composition together. */
	public function applyTransaction(transaction:EditTransaction):Bool {
		ensureLive();
		if (transaction == null) return false;
		var count = offsets.codepointCount;
		var first = clamp(transaction.replacementStart, 0, count);
		var last = clamp(transaction.replacementEnd, 0, count);
		if (last < first) {
			var swap = first;
			first = last;
			last = swap;
		}
		var replacement = transaction.replacementText == null ? "" : transaction.replacementText;
		var textChanged = offsets.sliceCodepoints(first, last) != replacement;
		var previousStart = selectionStart;
		var previousEnd = selectionEnd;
		var previousCompositionStart = compositionStart;
		var previousCompositionEnd = compositionEnd;
		if (textChanged) {
			var oldDocumentLength = offsets.codepointCount;
			cancelPointerClick();
			offsets.replace(first, last, replacement);
			text = offsets.text;
			var newEnd = first + TextOffsetMap.countCodepoints(replacement);
			layout.setTextAfterEdit(layoutText(), offsets, first, last, first, newEnd,
				oldDocumentLength);
			renderMeasurement.invalidate();
			lastLayoutText = layoutText();
		}
		setSelection(transaction.selectionStart, transaction.selectionEnd);
		if (transaction.hasComposition && transaction.compositionStart >= 0 &&
			transaction.compositionEnd >= transaction.compositionStart)
			setComposition(transaction.compositionStart, transaction.compositionEnd);
		else
			clearComposition();
		return textChanged || previousStart != selectionStart || previousEnd != selectionEnd ||
			previousCompositionStart != compositionStart || previousCompositionEnd != compositionEnd;
	}

	public function documentOffsets():TextDocument {
		ensureLive();
		return offsets;
	}

	public function setRenderColor(color:Color):Void {
		ensureLive();
		if (color == null)
			return;
		if (renderColor.red == color.red && renderColor.green == color.green &&
			renderColor.blue == color.blue && renderColor.alpha == color.alpha)
			return;
		renderColor = color;
		renderMeasurement.invalidate();
	}

	public function documentLength():Int {
		ensureLive();
		return offsets.codepointCount;
	}

	/** Returns surrounding text while keeping selection and composition in the window. */
	public function surroundingText(maxBefore:Int, maxAfter:Int):TextInputWindow {
		ensureLive();
		var first = selectionStart;
		var last = selectionEnd;
		if (compositionStart >= 0 && compositionEnd >= compositionStart) {
			first = Std.int(Math.min(first, compositionStart));
			last = Std.int(Math.max(last, compositionEnd));
		}
		var start = Std.int(Math.max(0, first - Std.int(Math.max(0, maxBefore))));
		var end = Std.int(Math.min(offsets.codepointCount, last + Std.int(Math.max(0, maxAfter))));
		return new TextInputWindow(offsets.sliceCodepoints(start, end), start, end);
	}

	/** Applies one transactional NativeKit composition/edit update. */
	public function applyTextEdit(edit:NativeKitTextEdit):Bool {
		ensureLive();
		if (edit == null)
			return false;
		var previousStart = selectionStart;
		var previousEnd = selectionEnd;
		var previousCompositionStart = compositionStart;
		var previousCompositionEnd = compositionEnd;
		switch (edit.action) {
			case TextEditAction.Compose | TextEditAction.Commit | TextEditAction.Delete:
				return applyTransaction(new EditTransaction(edit.replaceStart, edit.replaceEnd,
					edit.action == TextEditAction.Delete ? "" : edit.text,
					edit.selectionStart, edit.selectionEnd, edit.action == TextEditAction.Compose,
					edit.compositionStart, edit.compositionEnd));
			case TextEditAction.SetSelection:
				setSelection(edit.selectionStart, edit.selectionEnd);
				setComposition(edit.compositionStart, edit.compositionEnd);
			case TextEditAction.SetComposition:
				setComposition(edit.compositionStart, edit.compositionEnd);
			case TextEditAction.FinishComposition:
				clearComposition();
				setSelection(edit.selectionStart, edit.selectionEnd);
			case _:
				return false;
		}
		return previousStart != selectionStart || previousEnd != selectionEnd ||
			previousCompositionStart != compositionStart || previousCompositionEnd != compositionEnd;
	}

	public function setSelection(start:Int, end:Int):Bool {
		ensureLive();
		var count = offsets.codepointCount;
		var first = clamp(start, 0, count);
		var last = clamp(end, 0, count);
		if (first > last) {
			var swap = first;
			first = last;
			last = swap;
		}
		var changed = first != selectionStart || last != selectionEnd ||
			selectionAnchor != first || selectionFocus != last ||
			selectionAnchorLayoutOffset != first || selectionFocusLayoutOffset != last ||
			selectionAnchorAffinity != 0 || selectionFocusAffinity != 0;
		selectionStart = first;
		selectionEnd = last;
		selectionAnchor = first;
		selectionFocus = last;
		selectionAnchorLayoutOffset = first;
		selectionFocusLayoutOffset = last;
		selectionAnchorAffinity = 0;
		selectionFocusAffinity = 0;
		resetVerticalNavigation();
		return changed;
	}

	public function selectAll():Bool {
		var first = selectionStart != 0 || selectionEnd != offsets.codepointCount ||
			selectionAnchor != 0 || selectionFocus != offsets.codepointCount ||
			selectionAnchorLayoutOffset != 0 ||
			selectionFocusLayoutOffset != offsets.codepointCount ||
			selectionAnchorAffinity != 0 || selectionFocusAffinity != 0;
		selectionAnchor = 0;
		selectionFocus = offsets.codepointCount;
		selectionAnchorLayoutOffset = 0;
		selectionFocusLayoutOffset = selectionFocus;
		selectionStart = 0;
		selectionEnd = selectionFocus;
		selectionAnchorAffinity = 0;
		selectionFocusAffinity = 0;
		resetVerticalNavigation();
		return first;
	}

	/** Places a caret and optionally extends the existing anchored selection. */
	public function placeCaret(offset:Int, extend:Bool, affinity:Int = 0):Bool {
		ensureLive();
		resetVerticalNavigation();
		var next = clamp(layout.alignGrapheme(clamp(offset, 0, offsets.codepointCount)),
			0, offsets.codepointCount);
		var previousFocusLayoutOffset = selectionFocusLayoutOffset;
		var previousAnchorLayoutOffset = selectionAnchorLayoutOffset;
		if (!extend) {
			selectionAnchor = next;
			selectionAnchorLayoutOffset = next;
			selectionAnchorAffinity = affinity;
		} else if (selectionStart == selectionEnd) {
			selectionAnchor = selectionFocus;
			selectionAnchorLayoutOffset = previousFocusLayoutOffset;
			selectionAnchorAffinity = selectionFocusAffinity;
		}
		selectionFocus = next;
		selectionFocusLayoutOffset = next;
		var changed = selectionAnchorLayoutOffset != previousAnchorLayoutOffset ||
			selectionFocusLayoutOffset != previousFocusLayoutOffset ||
			selectionAnchorAffinity != affinity || selectionFocusAffinity != affinity;
		selectionFocusAffinity = affinity;
		var first = selectionAnchor < next ? selectionAnchor : next;
		var last = selectionAnchor > next ? selectionAnchor : next;
		changed = changed || first != selectionStart || last != selectionEnd;
		selectionStart = first;
		selectionEnd = last;
		return changed;
	}

	/** Moves by grapheme; a non-extended move collapses an existing selection first. */
	public function moveCaret(direction:Int, extend:Bool):Bool {
		ensureLive();
		if (direction == 0)
			return false;
		resetVerticalNavigation();
		var next:Int;
		if (!extend && selectionStart != selectionEnd)
			next = direction < 0 ? selectionStart : selectionEnd;
		else
			next = direction < 0 ? layout.previousGrapheme(selectionFocus) : layout.nextGrapheme(selectionFocus);
		cancelPointerClick();
		return moveFocusTo(next, extend);
	}

	/** Moves by Skribidi word boundaries; direction is -1 or +1. */
	public function moveCaretByWord(direction:Int, extend:Bool, macStyle:Bool = false):Bool {
		ensureLive();
		if (direction == 0)
			return false;
		resetVerticalNavigation();
		var next = !extend && selectionStart != selectionEnd
			? (direction < 0 ? selectionStart : selectionEnd)
			: layout.moveWord(selectionFocus, direction, macStyle);
		cancelPointerClick();
		return moveFocusTo(next, extend);
	}

	/** Moves by paragraph for Control+Up/Down or macOS Option+Up/Down. */
	public function moveCaretByParagraph(direction:Int, extend:Bool,
			macStyle:Bool = false):Bool {
		ensureLive();
		if (direction == 0)
			return false;
		resetVerticalNavigation();
		var next = !extend && selectionStart != selectionEnd
			? (direction < 0 ? selectionStart : selectionEnd)
			: layout.moveParagraph(selectionFocus, direction, macStyle);
		cancelPointerClick();
		return moveFocusTo(next, extend);
	}

	/** Moves to the nearest visual line while preserving the requested x column. */
	public function moveCaretVertically(direction:Int, extend:Bool):Bool {
		ensureLive();
		if (direction == 0)
			return false;
		if (!extend && selectionStart != selectionEnd) {
			resetVerticalNavigation();
			return moveFocusTo(direction < 0 ? selectionStart : selectionEnd, false);
		}

		var current = focusPosition();
		var currentCaret = layout.caret(current);
		if (!hasDesiredVerticalX) {
			desiredVerticalX = currentCaret.x;
			hasDesiredVerticalX = true;
		}
		var lineStep = paragraphStyle.lineHeight == null ?
			absolute(currentCaret.descender - currentCaret.ascender) : paragraphStyle.lineHeight;
		lineStep = Math.max(1.0, lineStep);

		var candidate:TextPosition = null;
		var candidateCaret:TextCaret = null;
		// Hit testing at the next baseline handles wrapped visual lines and keeps
		// the shaping engine's bidi affinity. A small widening probe is useful at
		// line boundaries where hit testing deliberately favors the current line.
		for (probe in 0...5) {
			var distance = lineStep * (1.0 + probe * 0.25);
			var hit = layout.hitTest(desiredVerticalX, currentCaret.y + direction * distance);
			var hitCaret = layout.caret(hit);
			if ((direction < 0 && hitCaret.y < currentCaret.y - 0.01) ||
				(direction > 0 && hitCaret.y > currentCaret.y + 0.01)) {
				candidate = hit;
				candidateCaret = hitCaret;
				break;
			}
		}
		if (candidate == null || candidateCaret == null)
			return false;

		var currentRange = layout.lineRangeAt(current.offset);
		var candidateRange = layout.lineRangeAt(candidate.offset);
		if (currentRange.start == candidateRange.start && currentRange.end == candidateRange.end)
			return false;
		cancelPointerClick();
		var changed = placeCaretAt(candidate, extend);
		// placeCaretAt resets navigation state as a normal horizontal/pointer
		// move would. Restore the column for a continued up/down sequence.
		hasDesiredVerticalX = true;
		return changed;
	}

	/** Moves to the start or end of the current visual line. */
	public function moveCaretToLineBoundary(endOfLine:Bool, extend:Bool):Bool {
		ensureLive();
		var range = layout.lineRangeAt(selectionFocusLayoutOffset);
		var target = endOfLine ? trimLineBreak(range.end, range.start) : range.start;
		resetVerticalNavigation();
		return placeCaret(target, extend);
	}

	/** Adjusts internal scrolling so the active caret or selection remains in the viewport. */
	public function ensureCaretVisible(height:Float):Bool {
		ensureLive();
		if (!Math.isFinite(height) || height <= 0.0)
			return false;
		if (viewportHeight != height)
			renderMeasurement.invalidate();
		viewportHeight = height;
		var metrics = layout.measure();
		var contentHeight = Math.max(height, metrics.height);
		var caret = layout.caret(focusPosition());
		var top = caret.y + Math.min(caret.ascender, caret.descender);
		var bottom = caret.y + Math.max(caret.ascender, caret.descender);
		if (selectionStart != selectionEnd) {
			var selectionTop = top;
			var selectionBottom = bottom;
			for (rect in layout.selectionRects(anchorPosition(), focusPosition())) {
				selectionTop = Math.min(selectionTop, rect.y);
				selectionBottom = Math.max(selectionBottom, rect.y + rect.height);
			}
			// A selection can be taller than the viewport. In that case both
			// endpoints cannot be shown simultaneously, so keep the active focus
			// visible instead of jumping away from it.
			if (selectionBottom - selectionTop <= height) {
				top = selectionTop;
				bottom = selectionBottom;
			}
		}
		var next = scrollOffsetY;
		if (top < next)
			next = top;
		else if (bottom > next + height)
			next = bottom - height;
		var maximum = Math.max(0.0, contentHeight - height);
		next = clampFloat(next, 0.0, maximum);
		if (next == scrollOffsetY)
			return false;
		scrollOffsetY = next;
		renderMeasurement.invalidate();
		return true;
	}

	function moveFocusTo(next:Int, extend:Bool):Bool {
		var previousFocusLayoutOffset = selectionFocusLayoutOffset;
		var wasCollapsed = selectionStart == selectionEnd;
		if (!extend) {
			selectionAnchor = next;
			selectionAnchorLayoutOffset = next;
		} else if (wasCollapsed) {
			selectionAnchor = selectionFocus;
			selectionAnchorLayoutOffset = previousFocusLayoutOffset;
		}
		var first = extend ? (selectionAnchor < next ? selectionAnchor : next) : next;
		var last = extend ? (selectionAnchor > next ? selectionAnchor : next) : next;
		var changed = first != selectionStart || last != selectionEnd ||
			selectionFocusLayoutOffset != next || selectionFocusAffinity != 0;
		selectionFocus = next;
		selectionFocusLayoutOffset = next;
		selectionFocusAffinity = 0;
		if (!extend) {
			selectionAnchorAffinity = 0;
		}
		selectionStart = first;
		selectionEnd = last;
		return changed;
	}

	public function deleteBackward():Bool {
		if (selectionStart != selectionEnd)
			return replace(selectionStart, selectionEnd, "");
		if (selectionStart == 0)
			return false;
		var previous = layout.previousGrapheme(selectionStart);
		return replace(previous, selectionStart, "");
	}

	public function deleteForward():Bool {
		var length = offsets.codepointCount;
		if (selectionStart != selectionEnd)
			return replace(selectionStart, selectionEnd, "");
		if (selectionEnd >= length)
			return false;
		var next = layout.nextGrapheme(selectionEnd);
		return replace(selectionEnd, next, "");
	}

	public function hitTest(x:Float, y:Float):TextPosition {
		ensureLive();
		var hit = layout.hitTest(x, y);
		return new TextPosition(layout.alignGrapheme(hit.offset), hit.affinity);
	}

	/** Places the caret from a shaping-engine hit while keeping its visual affinity. */
	public function placeCaretAt(position:TextPosition, extend:Bool):Bool {
		ensureLive();
		resetVerticalNavigation();
		if (position == null)
			return false;
		var previousFocusLayoutOffset = selectionFocusLayoutOffset;
		var previousFocusAffinity = selectionFocusAffinity;
		var wasCollapsed = selectionStart == selectionEnd;
		var next = layout.offsetFromPosition(position);
		var changed = placeCaret(next, extend, position.affinity);
		selectionFocusLayoutOffset = position.offset;
		if (!extend)
			selectionAnchorLayoutOffset = position.offset;
		else if (wasCollapsed) {
			selectionAnchorLayoutOffset = previousFocusLayoutOffset;
			selectionAnchorAffinity = previousFocusAffinity;
		}
		return changed || previousFocusLayoutOffset != selectionFocusLayoutOffset;
	}

	public function focusPosition():TextPosition
		return new TextPosition(selectionFocusLayoutOffset, selectionFocusAffinity);

	public function anchorPosition():TextPosition
		return new TextPosition(selectionAnchorLayoutOffset, selectionAnchorAffinity);

	/** Restarts the caret blink after keyboard, pointer, or text-edit activity. */
	public function resetCaretBlink(timeSeconds:Float):Void {
		if (finite(timeSeconds))
			caretBlinkResetTime = Math.max(0.0, timeSeconds);
	}

	/** Returns whether the focused caret should be painted at the given UI time. */
	public function isCaretVisible(timeSeconds:Float):Bool {
		if (!focused || !finite(timeSeconds))
			return false;
		var elapsed = Math.max(0.0, timeSeconds - caretBlinkResetTime);
		return Std.int(elapsed / caretBlinkHalfPeriod) % 2 == 0;
	}

	/** Returns the shaped rectangles used to paint the active IME preedit underline. */
	public function compositionRects():Array<Rect> {
		if (compositionStart < 0 || compositionEnd <= compositionStart)
			return [];
		return layout.selectionRects(new TextPosition(compositionStart, 0),
			new TextPosition(compositionEnd, 0));
	}

	/** Selects the word under a pointer position using the shaped text engine's boundaries. */
	public function selectWordAt(position:TextPosition):Bool {
		ensureLive();
		if (position == null || offsets.codepointCount == 0)
			return false;
		var range = layout.wordRange(position);
		return setSelection(range[0], range[1]);
	}

	/** Selects the visual line under a pointer position. */
	public function selectLineAt(position:TextPosition):Bool {
		ensureLive();
		if (position == null || offsets.codepointCount == 0)
			return false;
		var range = layout.lineRangeAt(position.offset);
		return setSelection(range.start, range.end);
	}

	/** Returns 1 for a single click, 2 for a double click, and 3 for a triple click. */
	public function registerPointerClick(position:TextPosition, timestamp:Float,
			x:Float, y:Float):Int {
		ensureLive();
		if (position == null || !finite(timestamp) || !finite(x) || !finite(y)) {
			cancelPointerClick();
			return 1;
		}
		var range = layout.wordRange(position);
		var dx = x - lastPointerClickX;
		var dy = y - lastPointerClickY;
		var sameSequence = lastPointerClickTime >= 0.0 && timestamp >= lastPointerClickTime &&
			timestamp - lastPointerClickTime <= 0.5 && range[0] == lastPointerWordStart &&
			range[1] == lastPointerWordEnd && dx * dx + dy * dy <= 36.0 && lastPointerClickArmed;
		lastPointerClickCount = sameSequence
			? (lastPointerClickCount >= 3 ? 1 : lastPointerClickCount + 1)
			: 1;
		lastPointerClickTime = timestamp;
		lastPointerClickX = x;
		lastPointerClickY = y;
		lastPointerWordStart = range[0];
		lastPointerWordEnd = range[1];
		lastPointerClickArmed = false;
		pointerClickPending = true;
		return lastPointerClickCount;
	}

	/** Arms a click for a possible double click only after a nearby pointer release. */
	public function completePointerClick(x:Float, y:Float):Void {
		if (!pointerClickPending || !finite(x) || !finite(y)) {
			cancelPointerClick();
			return;
		}
		var dx = x - lastPointerClickX;
		var dy = y - lastPointerClickY;
		if (dx * dx + dy * dy > 36.0) {
			cancelPointerClick();
			return;
		}
		pointerClickPending = false;
		lastPointerClickArmed = true;
	}

	/** Cancels click recognition once movement exceeds the platform-independent slop. */
	public function cancelPointerClickIfMoved(x:Float, y:Float):Void {
		if (!pointerClickPending)
			return;
		var dx = x - lastPointerClickX;
		var dy = y - lastPointerClickY;
		if (dx * dx + dy * dy > 36.0)
			cancelPointerClick();
	}

	public function cancelPointerClick():Void {
		lastPointerClickTime = -1.0;
		lastPointerClickX = 0.0;
		lastPointerClickY = 0.0;
		lastPointerWordStart = -1;
		lastPointerWordEnd = -1;
		lastPointerClickCount = 0;
		lastPointerClickArmed = false;
		pointerClickPending = false;
	}

	public function dispose():Void {
		if (disposed)
			return;
		renderContent.dispose();
		layout.dispose();
		disposed = true;
	}

	public function isDisposed():Bool
		return disposed;

	function setComposition(start:Int, end:Int):Bool {
		var count = offsets.codepointCount;
		if (start < 0 || end < start || start > count || end > count) {
			return clearComposition();
		}
		if (start == compositionStart && end == compositionEnd)
			return false;
		compositionStart = start;
		compositionEnd = end;
		return true;
	}

	function clearComposition():Bool {
		if (compositionStart == -1 && compositionEnd == -1)
			return false;
		compositionStart = -1;
		compositionEnd = -1;
		return true;
	}

	function trimLineBreak(offset:Int, lineStart:Int):Int {
		var result = offset;
		while (result > lineStart) {
			var value = Utf8Text.slice(text, result - 1, result);
			if (value != "\n" && value != "\r")
				break;
			result--;
		}
		return result;
	}

	function resetVerticalNavigation():Void
		hasDesiredVerticalX = false;

	function clampScrollOffset():Void {
		if (viewportHeight <= 0.0)
			return;
		var maximum = Math.max(0.0, layout.measure().height - viewportHeight);
		scrollOffsetY = clampFloat(scrollOffsetY, 0.0, maximum);
	}

	function ensureLive():Void {
		if (disposed)
			throw "Text editor state has been disposed";
	}

	static inline function finite(value:Float):Bool
		return value == value && value - value == 0.0;

	/** Native text services interpret Haxeon's null representation of empty strings as empty UTF-8. */
	public function layoutText():String
		return text == null ? "" : text;

	static function copyTextStyle(value:TextStyle):TextStyle
		return new TextStyle(value.fontSize, value.font, value.letterSpacing);

	static function copyParagraphStyle(value:ParagraphStyle):ParagraphStyle
		return new ParagraphStyle(value.wrap, value.alignment, value.lineHeight, value.direction);

	static inline function clamp(value:Int, minimum:Int, maximum:Int):Int
		return value < minimum ? minimum : value > maximum ? maximum : value;

	static inline function absolute(value:Float):Float
		return value < 0.0 ? -value : value;

	static inline function clampFloat(value:Float, minimum:Float, maximum:Float):Float
		return value < minimum ? minimum : value > maximum ? maximum : value;
}
