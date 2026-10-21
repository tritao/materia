import haxe.io.Bytes;

/** A minimal PNG writer for RGBA8 images: no filtering, stored (uncompressed) deflate blocks. */
class Png {
	static var crcTable:Null<Array<Int>>;

	public static function rgba(width:Int, height:Int, pixels:Bytes):Bytes {
		// Raw scanlines, each prefixed by filter type 0.
		var raw = Bytes.alloc(height * (width * 4 + 1));
		for (y in 0...height) {
			raw.set(y * (width * 4 + 1), 0);
			raw.blit(y * (width * 4 + 1) + 1, pixels, y * width * 4, width * 4);
		}
		var blocks = Math.ceil(raw.length / 65535);
		var zlib = Bytes.alloc(2 + raw.length + blocks * 5 + 4);
		zlib.set(0, 0x78);
		zlib.set(1, 0x01);
		var offset = 2;
		for (block in 0...blocks) {
			var start = block * 65535, size = Std.int(Math.min(65535, raw.length - start));
			zlib.set(offset, block == blocks - 1 ? 1 : 0);
			zlib.set(offset + 1, size & 0xff);
			zlib.set(offset + 2, size >> 8);
			zlib.set(offset + 3, ~size & 0xff);
			zlib.set(offset + 4, (~size >> 8) & 0xff);
			zlib.blit(offset + 5, raw, start, size);
			offset += 5 + size;
		}
		var a = 1, b = 0;
		for (index in 0...raw.length) {
			a = (a + raw.get(index)) % 65521;
			b = (b + a) % 65521;
		}
		bigEndian(zlib, offset, (b << 16) | a);
		var header = Bytes.alloc(13);
		bigEndian(header, 0, width);
		bigEndian(header, 4, height);
		header.set(8, 8); // bit depth
		header.set(9, 6); // RGBA
		var chunks = [chunk("IHDR", header), chunk("IDAT", zlib), chunk("IEND", Bytes.alloc(0))];
		var length = 8;
		for (piece in chunks)
			length += piece.length;
		var out = Bytes.alloc(length);
		var signature = [0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A];
		for (index in 0...signature.length)
			out.set(index, signature[index]);
		var position = 8;
		for (piece in chunks) {
			out.blit(position, piece, 0, piece.length);
			position += piece.length;
		}
		return out;
	}

	static function chunk(type:String, data:Bytes):Bytes {
		var out = Bytes.alloc(12 + data.length);
		bigEndian(out, 0, data.length);
		for (index in 0...4)
			out.set(4 + index, type.charCodeAt(index));
		out.blit(8, data, 0, data.length);
		bigEndian(out, 8 + data.length, crc(out, 4, 4 + data.length));
		return out;
	}

	static function crc(bytes:Bytes, start:Int, length:Int):Int {
		if (crcTable == null) {
			var table:Array<Int> = [];
			for (n in 0...256) {
				var c = n;
				for (_ in 0...8)
					c = (c & 1) != 0 ? 0xEDB88320 ^ (c >>> 1) : c >>> 1;
				table.push(c);
			}
			crcTable = table;
		}
		var table:Array<Int> = cast crcTable;
		var c = 0xFFFFFFFF;
		for (index in start...start + length)
			c = table[(c ^ bytes.get(index)) & 0xff] ^ (c >>> 8);
		return c ^ 0xFFFFFFFF;
	}

	static function bigEndian(bytes:Bytes, offset:Int, value:Int):Void {
		bytes.set(offset, (value >>> 24) & 0xff);
		bytes.set(offset + 1, (value >>> 16) & 0xff);
		bytes.set(offset + 2, (value >>> 8) & 0xff);
		bytes.set(offset + 3, value & 0xff);
	}
}
