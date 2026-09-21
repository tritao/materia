package robotkit.protocol;

import haxe.io.Bytes;
import haxe.io.BytesInput;
import haxe.io.BytesOutput;
import haxeon.wire.MessagePackError;

typedef RobotFrameHeader = {
  final messageType:Int;
  final flags:Int;
  final payloadLength:Int;
  final attachmentCount:Int;
  final sessionId:haxe.Int64;
  final sequence:haxe.Int64;
  final timestampNs:haxe.Int64;
}

/** RobotKit's session envelope around one Haxeon Wire payload. */
class RobotFrame {
  public static inline final VERSION:Int = 1;
  public static inline final HEADER_BYTES:Int = 44;
  public static inline final MAX_PAYLOAD:Int = 16 * 1024 * 1024;
  public static inline final MAX_ATTACHMENTS:Int = 64;

  public final messageType:Int;
  public final flags:Int;
  public final payload:Bytes;
  public final attachments:Array<Bytes>;
  public final sessionId:haxe.Int64;
  public final sequence:haxe.Int64;
  public final timestampNs:haxe.Int64;

  public function new(messageType:Int, payload:Bytes, ?flags:Int = 0,
      ?attachments:Array<Bytes> = null, ?sessionId:haxe.Int64 = null,
      ?sequence:haxe.Int64 = null, ?timestampNs:haxe.Int64 = null) {
    if (messageType <= 0 || messageType > 0xffff)
      throw new MessagePackError("RobotFrame message type must fit in u16");
    // Haxe Int is a signed 32-bit bit pattern. Negative values represent
    // valid u32 flags with the high bit set, so preserve all 32 bits here.
    if (payload == null || payload.length > MAX_PAYLOAD)
      throw new MessagePackError("RobotFrame payload exceeds the configured limit");
    this.messageType = messageType;
    this.flags = flags;
    this.payload = payload;
    this.attachments = attachments == null ? [] : attachments.copy();
    if (this.attachments.length > MAX_ATTACHMENTS)
      throw new MessagePackError("RobotFrame has too many attachments");
    for (attachment in this.attachments)
      if (attachment == null || attachment.length > MAX_PAYLOAD)
        throw new MessagePackError("RobotFrame attachment exceeds the configured limit");
    this.sessionId = sessionId == null ? haxe.Int64.ofInt(0) : sessionId;
    this.sequence = sequence == null ? haxe.Int64.ofInt(0) : sequence;
    this.timestampNs = timestampNs == null ? haxe.Int64.ofInt(0) : timestampNs;
  }

  public function encode():Bytes {
    var output = new BytesOutput();
    output.bigEndian = true;
    output.writeByte(0x52);
    output.writeByte(0x4b);
    output.writeByte(0x46);
    output.writeByte(0x31);
    writeU16(output, VERSION);
    writeU16(output, messageType);
    writeU32(output, flags);
    writeU32(output, payload.length);
    writeU32(output, attachments.length);
    writeU64(output, sessionId);
    writeU64(output, sequence);
    writeU64(output, timestampNs);
    output.write(payload);
    for (attachment in attachments) {
      writeU32(output, attachment.length);
      output.write(attachment);
    }
    return output.getBytes();
  }

  public static function decode(bytes:Bytes):RobotFrame {
    if (bytes == null || bytes.length < HEADER_BYTES)
      throw new MessagePackError("RobotFrame is truncated");
    var input = new BytesInput(bytes);
    input.bigEndian = true;
    if (input.readByte() != 0x52 || input.readByte() != 0x4b ||
        input.readByte() != 0x46 || input.readByte() != 0x31)
      throw new MessagePackError("Invalid RobotFrame magic");
    if (readU16(input) != VERSION)
      throw new MessagePackError("Unsupported RobotFrame version");
    var messageType = readU16(input);
    var flags = readU32(input);
    var payloadLength = readLength(input, "payload");
    var attachmentCount = readU32(input);
    if (attachmentCount < 0 || attachmentCount > MAX_ATTACHMENTS)
      throw new MessagePackError("RobotFrame attachment count is invalid");
    var sessionId = readU64(input);
    var sequence = readU64(input);
    var timestampNs = readU64(input);
    var payload = input.read(payloadLength);
    var attachments:Array<Bytes> = [];
    for (_ in 0...attachmentCount)
      attachments.push(input.read(readLength(input, "attachment")));
    if (input.position != bytes.length)
      throw new MessagePackError("RobotFrame has trailing bytes");
    return new RobotFrame(messageType, payload, flags, attachments, sessionId,
      sequence, timestampNs);
  }

  public static function requiredLengthAt(length:Int, readByte:Int->Int):Int {
    if (length < HEADER_BYTES)
      return 0;
    if (readByte(0) != 0x52 || readByte(1) != 0x4b || readByte(2) != 0x46 ||
        readByte(3) != 0x31)
      throw new MessagePackError("Invalid RobotFrame magic");
    var payloadLength = readU32At(readByte, 12);
    if (payloadLength < 0 || payloadLength > MAX_PAYLOAD)
      throw new MessagePackError("RobotFrame payload exceeds the configured limit");
    var attachmentCount = readU32At(readByte, 16);
    if (attachmentCount < 0 || attachmentCount > MAX_ATTACHMENTS)
      throw new MessagePackError("RobotFrame attachment count is invalid");
    var position = HEADER_BYTES + payloadLength;
    for (_ in 0...attachmentCount) {
      if (length < position + 4)
        return 0;
      var attachmentLength = readU32At(readByte, position);
      if (attachmentLength < 0 || attachmentLength > MAX_PAYLOAD)
        throw new MessagePackError("RobotFrame attachment exceeds the configured limit");
      position += 4 + attachmentLength;
      if (position < 0 || position > HEADER_BYTES + MAX_PAYLOAD * (MAX_ATTACHMENTS + 1))
        throw new MessagePackError("RobotFrame is too large");
      if (length < position)
        return 0;
    }
    return position;
  }

  static function readLength(input:BytesInput, name:String):Int {
    var length = readU32(input);
    if (length < 0 || length > MAX_PAYLOAD)
      throw new MessagePackError('RobotFrame $name exceeds the configured limit');
    return length;
  }

  static function readU16(input:BytesInput):Int
    return (input.readByte() << 8) | input.readByte();

  static function readU32(input:BytesInput):Int {
    var high = input.readByte();
    var value = (high << 24) | (input.readByte() << 16) |
      (input.readByte() << 8) | input.readByte();
    return high > 0x7f ? -1 : value;
  }

  static function readU64(input:BytesInput):haxe.Int64
    return haxe.Int64.make(input.readInt32(), input.readInt32());

  static function readU32At(readByte:Int->Int, offset:Int):Int {
    var high = readByte(offset);
    var value = (high << 24) | (readByte(offset + 1) << 16) |
      (readByte(offset + 2) << 8) | readByte(offset + 3);
    return high > 0x7f ? -1 : value;
  }

  static function writeU16(output:BytesOutput, value:Int):Void {
    output.writeByte(value >>> 8);
    output.writeByte(value);
  }

  static function writeU32(output:BytesOutput, value:Int):Void {
    output.writeByte(value >>> 24);
    output.writeByte(value >>> 16);
    output.writeByte(value >>> 8);
    output.writeByte(value);
  }

  static function writeU64(output:BytesOutput, value:haxe.Int64):Void {
    output.writeInt32(haxe.Int64.toInt(haxe.Int64.ushr(value, 32)));
    output.writeInt32(haxe.Int64.toInt(value));
  }
}

/** Accumulates arbitrary TCP chunks and emits complete RobotFrames. */
class RobotFrameStream {
  var chunks:Array<Bytes> = [];
  var head:Int = 0;
  var headOffset:Int = 0;
  var buffered:Int = 0;

  public function new() {}

  public function push(bytes:Bytes):Array<RobotFrame> {
    if (bytes == null || bytes.length == 0)
      return [];
    chunks.push(bytes);
    buffered += bytes.length;
    var frames:Array<RobotFrame> = [];
    while (buffered >= RobotFrame.HEADER_BYTES) {
      var length = RobotFrame.requiredLengthAt(buffered,
        function(offset:Int) return byteAt(offset));
      if (length == 0)
        break;
      frames.push(RobotFrame.decode(take(length)));
    }
    return frames;
  }

  public function pendingBytes():Int
    return buffered;

  function byteAt(offset:Int):Int {
    var remaining = offset;
    for (index in head...chunks.length) {
      var chunk = chunks[index], start = index == head ? headOffset : 0,
        available = chunk.length - start;
      if (remaining < available)
        return chunk.get(start + remaining);
      remaining -= available;
    }
    throw "RobotFrame stream byte offset is outside the buffered data";
  }

  function take(length:Int):Bytes {
    var output = new BytesOutput();
    var written = 0;
    while (written < length) {
      var chunk = chunks[head], available = chunk.length - headOffset,
        amount = length - written < available ? length - written : available;
      output.write(Bytes.view(chunk, headOffset, amount));
      written += amount;
      headOffset += amount;
      buffered -= amount;
      if (headOffset == chunk.length) {
        head++;
        headOffset = 0;
      }
    }
    if (head == chunks.length) {
      chunks = [];
      head = 0;
    } else if (head > 64 && head * 2 > chunks.length) {
      chunks = chunks.slice(head);
      head = 0;
    }
    return output.getBytes();
  }
}
