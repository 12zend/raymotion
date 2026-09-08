'use strict';
// Copy each payload byte once, regardless of transport fragmentation.
class FrameDecoder {
  constructor(onFrame) {
    this.onFrame = onFrame;
    this.line = '';
    this.header = null;
    this.payload = null;
    this.offset = 0;
  }
  push(chunk) {
    let cursor = 0;
    while (cursor < chunk.length) {
      if (!this.header) {
        const end = chunk.indexOf(10, cursor);
        const stop = end < 0 ? chunk.length : end;
        if (this.line.length + stop - cursor > 128) throw new Error('Invalid preview header');
        this.line += chunk.subarray(cursor, stop).toString();
        if (end < 0) return;
        cursor = end + 1;
        const match = /^RAY1 (\d+) (\d+) (\d+) (\d+)$/.exec(this.line);
        if (!match) throw new Error('Invalid preview protocol');
        const [w, h, frame, size] = match.slice(1).map(Number);
        if (w < 1 || h < 1 || w > 1920 || h > 1080 || size !== w * h * 3 || !Number.isSafeInteger(frame))
          throw new Error('Invalid preview dimensions');
        this.header = {w, h, frame, size};
        this.line = '';
        this.payload = Buffer.allocUnsafe(size);
        this.offset = 0;
      }
      const count = Math.min(chunk.length - cursor, this.header.size - this.offset);
      chunk.copy(this.payload, this.offset, cursor, cursor + count);
      cursor += count;
      this.offset += count;
      if (this.offset < this.header.size) return;
      const frame = {...this.header, rgb: this.payload.toString('base64')};
      this.header = null;
      this.payload = null;
      this.onFrame(frame);
    }
  }
}
module.exports = {FrameDecoder};
