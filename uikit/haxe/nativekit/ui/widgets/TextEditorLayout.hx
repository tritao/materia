package nativekit.ui.widgets;

import FontCollection;
import Canvas;
import Color;
import LayoutMeasureConstraints;
import LayoutMeasureResult;
import ParagraphStyle;
import Rect;
import TextLayout;
import TextStyle;
import nativekit.editorkit.TextDocument;

/** Retained layouts for bounded groups of paragraphs in one editor document. */
class TextEditorLayout {
	static inline var paragraphsPerLayout:Int = 64;
	public var text(get, never):String;
	public var width(default, null):Float;
	public final textStyle:TextStyle;
	public final paragraphStyle:ParagraphStyle;
	public var paragraphCount(get, never):Int;

	final fonts:FontCollection;
	var paragraphs:Array<TextEditorParagraphRecord>;
	var rangeGeometryCache:Array<TextEditorRangeGeometryCache>;
	var offsets:TextDocument;
	var paragraphLineCount:Int;
	var contentWidth:Float;
	var contentHeight:Float;
	var firstBaseline:Float;
	var hasBaseline:Bool;
	var disposed:Bool;

	public function new(fonts:FontCollection, value:String, width:Float, textStyle:TextStyle,
			paragraphStyle:ParagraphStyle, ?offsetMap:TextDocument) {
		if (fonts == null || fonts.isDisposed())
			throw "Editor layout requires a live font collection";
		if (width <= 0.0 || textStyle == null || paragraphStyle == null)
			throw "Editor layout arguments are invalid";
		this.fonts = fonts;
		this.textStyle = copyTextStyle(textStyle);
		this.paragraphStyle = copyParagraphStyle(paragraphStyle);
		paragraphs = [];
		rangeGeometryCache = [];
		offsets = null;
		paragraphLineCount = 0;
		contentWidth = 0.0;
		contentHeight = 0.0;
		firstBaseline = 0.0;
		hasBaseline = false;
		disposed = false;
		if (offsetMap == null)
			update(value, width, this.textStyle, this.paragraphStyle);
		else
			updateDocument(offsetMap, width, this.textStyle, this.paragraphStyle);
	}

	function get_paragraphCount():Int
		return paragraphLineCount;

	function get_text():String
		return offsets == null ? "" : offsets.text;

	static function chunkCount(offsetMap:TextDocument):Int
		return Std.int((offsetMap.paragraphCount() + paragraphsPerLayout - 1) / paragraphsPerLayout);

	static function chunkRangeAt(offsetMap:TextDocument, index:Int):TextRange {
		var firstParagraph = index * paragraphsPerLayout;
		var lastParagraph = Std.int(Math.min(offsetMap.paragraphCount() - 1,
			firstParagraph + paragraphsPerLayout - 1));
		return new TextRange(offsetMap.paragraphRangeAtIndex(firstParagraph).start,
			offsetMap.paragraphRangeAtIndex(lastParagraph).end);
	}

	/** Updates only paragraph resources whose text or shaping inputs changed. */
	public function update(value:String, nextWidth:Float, nextTextStyle:TextStyle,
			nextParagraphStyle:ParagraphStyle, ?offsetMap:TextDocument):Void {
		ensureLive();
		if (nextWidth <= 0.0 || nextTextStyle == null || nextParagraphStyle == null)
			throw "Editor layout update arguments are invalid";
		var actualText = value == null ? "" : value;
		var nextOffsets = offsetMap == null ? new TextDocument(actualText) : offsetMap;
		if (nextOffsets.text != actualText)
			nextOffsets = new TextDocument(actualText);
		updateDocument(nextOffsets, nextWidth, nextTextStyle, nextParagraphStyle);
	}

	/** Updates retained paragraph layouts directly from the segmented document. */
	public function updateDocument(nextOffsets:TextDocument, nextWidth:Float,
			nextTextStyle:TextStyle, nextParagraphStyle:ParagraphStyle):Void {
		ensureLive();
		if (nextOffsets == null || nextWidth <= 0.0 || nextTextStyle == null || nextParagraphStyle == null)
			throw "Editor layout update arguments are invalid";
		var styleChanged = textStyle.font != nextTextStyle.font ||
			textStyle.fontSize != nextTextStyle.fontSize ||
			textStyle.letterSpacing != nextTextStyle.letterSpacing ||
			paragraphStyle.wrap != nextParagraphStyle.wrap ||
			paragraphStyle.alignment != nextParagraphStyle.alignment ||
			paragraphStyle.lineHeight != nextParagraphStyle.lineHeight ||
			paragraphStyle.direction != nextParagraphStyle.direction;
		textStyle.font = nextTextStyle.font;
		textStyle.fontSize = nextTextStyle.fontSize;
		textStyle.letterSpacing = nextTextStyle.letterSpacing;
		paragraphStyle.wrap = nextParagraphStyle.wrap;
		paragraphStyle.alignment = nextParagraphStyle.alignment;
		paragraphStyle.lineHeight = nextParagraphStyle.lineHeight;
		paragraphStyle.direction = nextParagraphStyle.direction;

		var previous = paragraphs;
		var reusable = new Map<String, Array<TextEditorParagraphRecord>>();
		for (record in previous) {
			var records = reusable.get(record.text);
			if (records == null) {
				records = [];
				reusable.set(record.text, records);
			}
			records.push(record);
		}
		var used:Array<TextEditorParagraphRecord> = [];
		var next:Array<TextEditorParagraphRecord> = [];
		var nextCount = chunkCount(nextOffsets);
		for (index in 0...nextCount) {
			var range = chunkRangeAt(nextOffsets, index);
			var paragraphText = nextOffsets.sliceCodepoints(range.start, range.end);
			var previousRecord = index < previous.length ? previous[index] : null;
			var record:TextEditorParagraphRecord = null;
			if (previousRecord != null && !containsRecord(used, previousRecord) &&
				previousRecord.text == paragraphText &&
				previousRecord.layout.width == nextWidth && !styleChanged) {
				record = previousRecord;
			} else if (!styleChanged) {
				var matching = reusable.get(paragraphText);
				if (matching != null)
					while (matching.length > 0 && record == null) {
						var candidate:TextEditorParagraphRecord = matching.pop();
						if (candidate != null && candidate.layout.width == nextWidth &&
							!containsRecord(used, candidate))
							record = candidate;
					}
			}
			if (record == null && previousRecord != null && !containsRecord(used, previousRecord))
				record = previousRecord;
			if (record != null) {
				var textChanged = record.text != paragraphText;
				if (textChanged || record.layout.width != nextWidth || styleChanged) {
					record.layout.update(paragraphText, nextWidth, textStyle, paragraphStyle);
					record.text = paragraphText;
				}
			} else {
				record = new TextEditorParagraphRecord(paragraphText,
					TextLayout.create(fonts, paragraphText, nextWidth, textStyle, paragraphStyle));
			}
			used.push(record);
			record.start = range.start;
			record.end = range.end;
			record.y = 0.0;
			record.height = 0.0;
			next.push(record);
		}
		for (record in previous)
			if (!containsRecord(used, record))
				record.layout.dispose();

		width = nextWidth;
		offsets = nextOffsets;
		paragraphLineCount = nextOffsets.paragraphCount();
		clearRangeGeometryCache();
		paragraphs = next;
		recomputeMetrics();
	}

	public function setText(value:String, ?offsetMap:TextDocument):Void
		update(value, width, textStyle, paragraphStyle, offsetMap);

	/**
	 * Updates paragraph records after one document replacement. Unaffected
	 * records are carried by paragraph index, so a keystroke does not rebuild a
	 * document-wide text-to-record lookup table or reslice every paragraph.
	 */
	public function setTextAfterEdit(nextOffsets:TextDocument,
			oldStart:Int, oldEnd:Int, newStart:Int, newEnd:Int,
			oldDocumentLength:Int):Void {
		ensureLive();
		if (nextOffsets == null)
			throw "Editor layout update requires a document";
		if (paragraphs.length == 0) {
			updateDocument(nextOffsets, width, textStyle, paragraphStyle);
			return;
		}
		if (nextOffsets.paragraphCount() != paragraphLineCount) {
			setTextAfterParagraphEdit(nextOffsets, oldStart, oldEnd,
				oldDocumentLength);
			return;
		}

		var previous = paragraphs;
		var oldFirst = paragraphIndexAtOffsetIn(previous, oldStart, oldDocumentLength);
		var oldLast = paragraphIndexAtOffsetIn(previous, oldEnd, oldDocumentLength);
		oldFirst = oldFirst > 0 ? oldFirst - 1 : oldFirst;
		oldLast = oldLast + 1 < previous.length ? oldLast + 1 : oldLast;

		var nextCount = chunkCount(nextOffsets);
		var newFirst = Std.int(nextOffsets.paragraphIndexAtOffset(newStart) / paragraphsPerLayout);
		var newLast = Std.int(nextOffsets.paragraphIndexAtOffset(newEnd) / paragraphsPerLayout);
		newFirst = newFirst > 0 ? newFirst - 1 : newFirst;
		newLast = newLast + 1 < nextCount ? newLast + 1 : newLast;

		var oldDirtyCount = oldLast - oldFirst + 1;
		var newDirtyCount = newLast - newFirst + 1;
		var paragraphDelta = newDirtyCount - oldDirtyCount;
		var reusable = new Map<String, Array<TextEditorParagraphRecord>>();
		for (index in oldFirst...(oldLast + 1)) {
			var oldRecord = previous[index];
			var records = reusable.get(oldRecord.text);
			if (records == null) {
				records = [];
				reusable.set(oldRecord.text, records);
			}
			records.push(oldRecord);
		}
		var usedDirty:Array<TextEditorParagraphRecord> = [];
		var next:Array<TextEditorParagraphRecord> = [];
		for (index in 0...nextCount) {
			var range = chunkRangeAt(nextOffsets, index);
			var record:TextEditorParagraphRecord = null;
			var paragraphText:Null<String> = null;
			if (index < newFirst) {
				// The edit is after this prefix, so its record text is unchanged.
				record = previous[index];
			} else if (index > newLast) {
				// Paragraph indexes after the dirty window shift by the local delta.
				var oldIndex = index - paragraphDelta;
				if (oldIndex >= 0 && oldIndex < previous.length)
					record = previous[oldIndex];
			}

			if (record == null) {
				paragraphText = nextOffsets.sliceCodepoints(range.start, range.end);
				var matching = reusable.get(paragraphText);
				if (matching != null)
					while (matching.length > 0 && record == null) {
						var candidate = matching.pop();
						if (candidate != null && !containsRecord(usedDirty, candidate))
							record = candidate;
					}
				if (record == null)
					record = new TextEditorParagraphRecord(paragraphText,
						TextLayout.create(fonts, paragraphText, width, textStyle, paragraphStyle));
				else if (record.text != paragraphText) {
					record.layout.update(paragraphText, width, textStyle, paragraphStyle);
					record.text = paragraphText;
				}
				usedDirty.push(record);
			}
			record.start = range.start;
			record.end = range.end;
			record.y = 0.0;
			next.push(record);
		}
		for (index in oldFirst...(oldLast + 1)) {
			var oldRecord = previous[index];
			if (!containsRecord(usedDirty, oldRecord))
				oldRecord.layout.dispose();
		}

		offsets = nextOffsets;
		paragraphLineCount = nextOffsets.paragraphCount();
		clearRangeGeometryCache();
		paragraphs = next;
		recomputeMetrics();
	}

	/** Keeps chunks outside a newline edit and repartitions only its neighborhood. */
	function setTextAfterParagraphEdit(nextOffsets:TextDocument,
			oldStart:Int, oldEnd:Int, oldDocumentLength:Int):Void {
		var previous = paragraphs;
		var first = paragraphIndexAtOffsetIn(previous, oldStart, oldDocumentLength);
		var last = paragraphIndexAtOffsetIn(previous, oldEnd, oldDocumentLength);
		first = first > 0 ? first - 1 : first;
		last = last + 1 < previous.length ? last + 1 : last;
		var delta = nextOffsets.codepointCount - oldDocumentLength;
		var firstOffset = previous[first].start;
		var lastOffset = clamp(previous[last].end + delta, firstOffset,
			nextOffsets.codepointCount);
		var firstParagraph = nextOffsets.paragraphIndexAtOffset(firstOffset);
		var lastParagraph = nextOffsets.paragraphIndexAtOffset(lastOffset);
		var result:Array<TextEditorParagraphRecord> = [];
		for (index in 0...first)
			result.push(previous[index]);
		var used:Array<TextEditorParagraphRecord> = [];
		var paragraph = firstParagraph;
		while (paragraph <= lastParagraph) {
			var remaining = lastParagraph - paragraph + 1;
			var chunksRemaining = Std.int((remaining + paragraphsPerLayout - 1) /
				paragraphsPerLayout);
			var chunkSize = Std.int((remaining + chunksRemaining - 1) / chunksRemaining);
			var paragraphEnd = paragraph + chunkSize - 1;
			var chunkStart = paragraph == firstParagraph ? firstOffset :
				nextOffsets.paragraphRangeAtIndex(paragraph).start;
			var chunkEnd = paragraphEnd == lastParagraph ? lastOffset :
				nextOffsets.paragraphRangeAtIndex(paragraphEnd).end;
			var chunkText = nextOffsets.sliceCodepoints(chunkStart, chunkEnd);
			var record:TextEditorParagraphRecord = null;
			for (index in first...(last + 1)) {
				var candidate = previous[index];
				if (!containsRecord(used, candidate) && candidate.text == chunkText) {
					record = candidate;
					break;
				}
			}
			if (record == null)
				for (index in first...(last + 1)) {
					var candidate = previous[index];
					if (!containsRecord(used, candidate)) {
						record = candidate;
						break;
					}
				}
			if (record == null)
				record = new TextEditorParagraphRecord(chunkText,
					TextLayout.create(fonts, chunkText, width, textStyle, paragraphStyle));
			else if (record.text != chunkText) {
				record.layout.update(chunkText, width, textStyle, paragraphStyle);
				record.text = chunkText;
			}
			used.push(record);
			record.start = chunkStart;
			record.end = chunkEnd;
			result.push(record);
			paragraph = paragraphEnd + 1;
		}
		for (index in first...(last + 1)) {
			var record = previous[index];
			if (!containsRecord(used, record))
				record.layout.dispose();
		}
		for (index in last + 1...previous.length) {
			var record = previous[index];
			record.start += delta;
			record.end += delta;
			result.push(record);
		}
		offsets = nextOffsets;
		paragraphLineCount = nextOffsets.paragraphCount();
		clearRangeGeometryCache();
		paragraphs = result;
		recomputeMetrics();
	}

	public function measure():TextMetrics
		return new TextMetrics(0.0, 0.0, contentWidth, contentHeight);

	/** Measures this retained content against the constraints of a Custom node. */
	public function measureForConstraints(constraints:LayoutMeasureConstraints):LayoutMeasureResult {
		ensureLive();
		if (constraints == null || constraints.maxWidth < constraints.minWidth ||
			constraints.maxHeight < constraints.minHeight)
			throw "Editor layout constraints are invalid";
		var measured = measure();
		var measuredWidth = Math.max(measured.width, constraints.minWidth);
		if (Math.isFinite(constraints.maxWidth))
			measuredWidth = Math.min(measuredWidth, constraints.maxWidth);
		var measuredHeight = Math.max(measured.height, constraints.minHeight);
		if (Math.isFinite(constraints.maxHeight))
			measuredHeight = Math.min(measuredHeight, constraints.maxHeight);
		var baseline = firstBaseline;
		var measuredHasBaseline = hasBaseline && Math.isFinite(baseline) && baseline >= 0.0 &&
			baseline <= measuredHeight;
		if (!measuredHasBaseline && paragraphs.length > 0 && paragraphs[0].height > 0.0) {
			var firstCaret = paragraphs[0].layout.caret(new TextPosition(0, 0));
			baseline = firstCaret.y;
			measuredHasBaseline = Math.isFinite(baseline) && baseline >= 0.0 && baseline <= measuredHeight;
		}
		return new LayoutMeasureResult(measuredWidth, measuredHeight, baseline, measuredHasBaseline);
	}

	/** Paints retained paragraphs intersecting the visible document range. */
	public function paint(canvas:Canvas, color:Color, minY:Float = 0.0,
			maxY:Float = 1.0e30):Void {
		ensureLive();
		if (canvas == null || color == null)
			throw "Editor layout paint arguments are invalid";
		var low = 0;
		var high = paragraphs.length;
		while (low < high) {
			var middle = (low + high) >> 1;
			var candidate = paragraphs[middle];
			if (candidate.y + candidate.height <= minY)
				low = middle + 1;
			else
				high = middle;
		}
		for (index in low...paragraphs.length) {
			var record = paragraphs[index];
			if (record.y >= maxY)
				break;
			if (record.text.length > 0) {
				var previous = record.renderColor;
				if (previous == null || previous.red != color.red ||
					previous.green != color.green || previous.blue != color.blue ||
					previous.alpha != color.alpha) {
					record.layout.setColor(color);
					record.renderColor = color;
				}
				canvas.drawText(record.layout, 0.0, record.y);
			}
		}
	}

	public function hitTest(x:Float, y:Float):TextPosition {
		ensureLive();
		if (paragraphs.length == 0)
			return new TextPosition(0, 0);
		var record = paragraphAtY(y);
		var hit = record.layout.hitTest(x, y - record.y);
		return new TextPosition(clamp(hit.offset + record.start, record.start, record.end), hit.affinity);
	}

	public function offsetFromPosition(position:TextPosition):Int {
		ensureLive();
		if (position == null)
			throw "Text position cannot be null";
		var record = paragraphAtOffset(position.offset);
		return clamp(record.layout.offsetFromPosition(
			new TextPosition(clamp(position.offset - record.start, 0, record.end - record.start),
				position.affinity)) + record.start, record.start, record.end);
	}

	public function caret(position:TextPosition):TextCaret {
		ensureLive();
		if (position == null)
			throw "Text position cannot be null";
		var record = paragraphAtOffset(position.offset);
		var local = clamp(position.offset - record.start, 0, record.end - record.start);
		var value = record.layout.caret(new TextPosition(local, position.affinity));
		return new TextCaret(value.x, value.y + record.y, value.ascender, value.descender,
			value.slope, value.direction);
	}

	public function selectionRects(start:TextPosition, end:TextPosition):Array<Rect> {
		ensureLive();
		if (start == null || end == null)
			throw "Text selection endpoints cannot be null";
		var first = clamp(start.offset, 0, offsets.codepointCount);
		var last = clamp(end.offset, 0, offsets.codepointCount);
		if (last < first) {
			var swap = first;
			first = last;
			last = swap;
		}
		if (first == last)
			return [];
		var result:Array<Rect> = [];
		for (record in paragraphs) {
			var localStart:Int = first > record.start ? first : record.start;
			var localEnd:Int = last < record.end ? last : record.end;
			if (localEnd <= localStart)
				continue;
			for (rect in record.layout.selectionRects(
				new TextPosition(localStart - record.start, start.affinity),
				new TextPosition(localEnd - record.start, end.affinity)))
				result.push(new Rect(rect.x, rect.y + record.y, rect.width, rect.height));
		}
		return result;
	}

	/** Returns shaped grapheme rectangles paired with absolute document ranges. */
	public function selectionRangeRects(start:TextPosition, end:TextPosition,
			minY:Float = -1.0e30, maxY:Float = 1.0e30):Array<TextRangeRect> {
		ensureLive();
		if (start == null || end == null)
			throw "Text selection endpoints cannot be null";
		if (!Math.isFinite(minY) || !Math.isFinite(maxY) || maxY < minY)
			throw "Text selection geometry bounds are invalid";
		var forward = start.offset <= end.offset;
		var firstPosition = forward ? start : end;
		var lastPosition = forward ? end : start;
		var first = clamp(firstPosition.offset, 0, offsets.codepointCount);
		var last = clamp(lastPosition.offset, 0, offsets.codepointCount);
		var firstAffinity = firstPosition.affinity;
		var lastAffinity = lastPosition.affinity;
		if (first == last)
			return [];
		for (entry in rangeGeometryCache)
			if (entry.start == first && entry.end == last &&
				entry.startAffinity == firstAffinity && entry.endAffinity == lastAffinity &&
				entry.minY == minY && entry.maxY == maxY)
				return entry.rectangles.copy();
		var result:Array<TextRangeRect> = [];
		var low = 0;
		var high = paragraphs.length;
		while (low < high) {
			var middle = (low + high) >> 1;
			if (paragraphs[middle].y + paragraphs[middle].height <= minY)
				low = middle + 1;
			else
				high = middle;
		}
		for (index in low...paragraphs.length) {
			var record = paragraphs[index];
			if (record.y >= maxY)
				break;
			var localStart = first > record.start ? first : record.start;
			var localEnd = last < record.end ? last : record.end;
			if (localEnd <= localStart)
				continue;
			if (minY > record.y || maxY < record.y + record.height) {
				var visibleTop = Math.max(0.0, minY - record.y);
				var visibleBottom = Math.min(record.height, maxY - record.y);
				if (visibleBottom <= visibleTop)
					continue;
				var sampleTop = Math.min(visibleBottom, visibleTop + 1.0);
				var sampleBottom = Math.max(visibleTop, visibleBottom - 1.0);
				var topLeft = record.layout.hitTest(0.0, sampleTop).offset;
				var topRight = record.layout.hitTest(record.layout.width, sampleTop).offset;
				var bottomLeft = record.layout.hitTest(0.0, sampleBottom).offset;
				var bottomRight = record.layout.hitTest(record.layout.width, sampleBottom).offset;
				var visibleStart = clamp(Std.int(Math.min(Math.min(topLeft, topRight),
					Math.min(bottomLeft, bottomRight))), 0, record.end - record.start);
				var visibleEnd = clamp(Std.int(Math.max(Math.max(topLeft, topRight),
					Math.max(bottomLeft, bottomRight))), 0, record.end - record.start);
				for (_ in 0...2) {
					if (visibleStart > 0)
						visibleStart = record.layout.previousGrapheme(visibleStart);
					if (visibleEnd < record.end - record.start)
						visibleEnd = record.layout.nextGrapheme(visibleEnd);
				}
				var clippedStart = record.start + visibleStart;
				var clippedEnd = record.start + visibleEnd;
				if (localStart < clippedStart)
					localStart = clippedStart;
				if (localEnd > clippedEnd)
					localEnd = clippedEnd;
			}
			if (localEnd <= localStart)
				continue;
			for (rect in record.layout.selectionRangeRects(
				new TextPosition(localStart - record.start,
					localStart == first ? firstAffinity : 0),
				new TextPosition(localEnd - record.start,
					localEnd == last ? lastAffinity : 0)))
				result.push(new TextRangeRect(rect.start + record.start, rect.end + record.start,
					rect.x, rect.y + record.y, rect.width, rect.height, rect.visualLeftIsStart));
		}
		rangeGeometryCache.push(new TextEditorRangeGeometryCache(first, last, firstAffinity,
			lastAffinity, minY, maxY, result.copy()));
		if (rangeGeometryCache.length > 2)
			rangeGeometryCache.shift();
		return result;
	}

	public function nextGrapheme(offset:Int):Int {
		ensureLive();
		var recordIndex = paragraphIndexAtOffset(offset);
		var record = paragraphs[recordIndex];
		var local = clamp(offset - record.start, 0, record.end - record.start);
		var next = record.layout.nextGrapheme(local);
		if (next == local && local >= record.end - record.start && recordIndex + 1 < paragraphs.length)
			return paragraphs[recordIndex + 1].start;
		return clamp(record.start + next, record.start, record.end);
	}

	public function previousGrapheme(offset:Int):Int {
		ensureLive();
		var recordIndex = paragraphIndexAtOffset(offset);
		var record = paragraphs[recordIndex];
		var local = clamp(offset - record.start, 0, record.end - record.start);
		var previous = record.layout.previousGrapheme(local);
		if (previous == local && local == 0 && recordIndex > 0)
			return paragraphs[recordIndex - 1].end;
		return clamp(record.start + previous, record.start, record.end);
	}

	public function alignGrapheme(offset:Int):Int {
		ensureLive();
		var record = paragraphAtOffset(offset);
		var local = clamp(offset - record.start, 0, record.end - record.start);
		return clamp(record.start + record.layout.alignGrapheme(local), record.start, record.end);
	}

	public function wordRange(position:TextPosition):Array<Int> {
		ensureLive();
		var record = paragraphAtOffset(position.offset);
		var local = clamp(position.offset - record.start, 0, record.end - record.start);
		var range = record.layout.wordRange(new TextPosition(local, position.affinity));
		return [record.start + range[0], record.start + range[1]];
	}

	public function wordRangeAt(offset:Int):TextRange {
		ensureLive();
		var record = paragraphAtOffset(offset);
		var local = clamp(offset - record.start, 0, record.end - record.start);
		var range = record.layout.wordRangeAt(local);
		return new TextRange(record.start + range.start, record.start + range.end);
	}

	public function lineRangeAt(offset:Int):TextRange {
		ensureLive();
		var record = paragraphAtOffset(offset);
		var local = clamp(offset - record.start, 0, record.end - record.start);
		var range = record.layout.lineRangeAt(local);
		var end = record.start + range.end;
		if (end == record.end && end < offsets.codepointCount)
			end++;
		return new TextRange(record.start + range.start, end);
	}

	public function moveWord(offset:Int, direction:Int, macStyle:Bool = false):Int {
		ensureLive();
		if (direction != -1 && direction != 1)
			throw "Text word movement arguments are invalid";
		var recordIndex = paragraphIndexAtOffset(offset);
		var record = paragraphs[recordIndex];
		var local = clamp(offset - record.start, 0, record.end - record.start);
		var moved = record.layout.moveWord(local, direction, macStyle);
		if (moved == local && local >= record.end - record.start && direction > 0 &&
			recordIndex + 1 < paragraphs.length)
			return paragraphs[recordIndex + 1].start;
		if (moved == local && local == 0 && direction < 0 && recordIndex > 0)
			return paragraphs[recordIndex - 1].end;
		return clamp(record.start + moved, record.start, record.end);
	}

	public function moveParagraph(offset:Int, direction:Int, macStyle:Bool = false):Int {
		ensureLive();
		if (direction != -1 && direction != 1)
			throw "Text paragraph movement arguments are invalid";
		var recordIndex = paragraphIndexAtOffset(offset);
		var record = paragraphs[recordIndex];
		var local = clamp(offset - record.start, 0, record.end - record.start);
		var moved = record.layout.moveParagraph(local, direction, macStyle);
		if (moved != local)
			return clamp(record.start + moved, record.start, record.end);
		if (direction < 0 && local == 0 && recordIndex > 0) {
			var previousParagraph = offsets.paragraphIndexAtOffset(record.start) - 1;
			return offsets.paragraphRangeAtIndex(previousParagraph).start;
		}
		if (direction > 0 && local == record.end - record.start &&
			recordIndex + 1 < paragraphs.length)
			return paragraphs[recordIndex + 1].start;
		return clamp(record.start + moved, record.start, record.end);
	}

	public function dispose():Void {
		if (disposed)
			return;
		for (record in paragraphs)
			record.layout.dispose();
		clearRangeGeometryCache();
		paragraphs = [];
		disposed = true;
	}

	function recomputeMetrics():Void {
		contentWidth = 0.0;
		contentHeight = 0.0;
		firstBaseline = 0.0;
		hasBaseline = false;
		for (record in paragraphs) {
			record.y = contentHeight;
			var metrics = record.layout.measure();
			var lineHeight = paragraphStyle.lineHeight == null ? 0.0 : paragraphStyle.lineHeight;
			if (lineHeight <= 0.0) {
				var caret = record.layout.caret(new TextPosition(0, 0));
				lineHeight = Math.abs(caret.descender - caret.ascender);
			}
			record.height = Math.max(metrics.height, Math.max(1.0, lineHeight));
			contentWidth = Math.max(contentWidth, metrics.width);
			if (!hasBaseline) {
				firstBaseline = record.y + record.layout.caret(new TextPosition(0, 0)).y;
				hasBaseline = Math.isFinite(firstBaseline) && firstBaseline >= 0.0;
			}
			contentHeight += record.height;
		}
	}

	function paragraphAtY(y:Float):TextEditorParagraphRecord {
		if (y <= 0.0)
			return paragraphs[0];
		for (record in paragraphs)
			if (y < record.y + record.height)
				return record;
		return paragraphs[paragraphs.length - 1];
	}

	function paragraphAtOffset(offset:Int):TextEditorParagraphRecord
		return paragraphs[paragraphIndexAtOffset(offset)];

	function paragraphIndexAtOffset(offset:Int):Int {
		return paragraphIndexAtOffsetIn(paragraphs, offset, offsets.codepointCount);
	}

	static function paragraphIndexAtOffsetIn(records:Array<TextEditorParagraphRecord>,
			offset:Int, documentLength:Int):Int {
		var value = clamp(offset, 0, documentLength);
		var low = 0;
		var high = records.length - 1;
		var result = high;
		while (low <= high) {
			var middle = (low + high) >> 1;
			var record = records[middle];
			if (value < record.start)
				high = middle - 1;
			else if (value > record.end)
				low = middle + 1;
			else
				return middle;
		}
		return clamp(low, 0, records.length - 1);
	}

	function ensureLive():Void {
		if (disposed)
			throw "Editor layout has been disposed";
	}

	function clearRangeGeometryCache():Void
		rangeGeometryCache = [];

	static function containsRecord(records:Array<TextEditorParagraphRecord>,
			value:TextEditorParagraphRecord):Bool {
		for (record in records)
			if (record == value)
				return true;
		return false;
	}

	static function copyTextStyle(style:TextStyle):TextStyle
		return new TextStyle(style.fontSize, style.font, style.letterSpacing);

	static function copyParagraphStyle(style:ParagraphStyle):ParagraphStyle
		return new ParagraphStyle(style.wrap, style.alignment, style.lineHeight, style.direction);

	static inline function clamp(value:Int, low:Int, high:Int):Int
		return value < low ? low : (value > high ? high : value);
}

class TextEditorParagraphRecord {
	public var start:Int;
	public var end:Int;
	public var text:String;
	public var y:Float;
	public var height:Float;
	public var renderColor:Null<Color>;
	public final layout:TextLayout;

	public function new(text:String, layout:TextLayout) {
		this.text = text;
		this.layout = layout;
		start = 0;
		end = 0;
		y = 0.0;
		height = 0.0;
		renderColor = null;
	}
}

/** Cached selection geometry for one active editor range and viewport. */
class TextEditorRangeGeometryCache {
	public final start:Int;
	public final end:Int;
	public final startAffinity:Int;
	public final endAffinity:Int;
	public final minY:Float;
	public final maxY:Float;
	public final rectangles:Array<TextRangeRect>;

	public function new(start:Int, end:Int, startAffinity:Int, endAffinity:Int,
			minY:Float, maxY:Float, rectangles:Array<TextRangeRect>) {
		this.start = start;
		this.end = end;
		this.startAffinity = startAffinity;
		this.endAffinity = endAffinity;
		this.minY = minY;
		this.maxY = maxY;
		this.rectangles = rectangles;
	}
}
