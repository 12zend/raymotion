'use strict';
// Incremental binary decoder, independent of VS Code and transport chunk sizes.
class FrameDecoder {
  constructor(onFrame) { this.onFrame = onFrame; this.buffer = Buffer.alloc(0); this.header = null; }
  push(chunk) {
    this.buffer = Buffer.concat([this.buffer, chunk]);
    for (;;) {
      if (!this.header) {
        const end = this.buffer.indexOf(10);
        if (end < 0) {
          if (this.buffer.length > 128) throw new Error('Invalid preview header');
          return;
        }
        const match = /^RAY1 (\d+) (\d+) (\d+) (\d+)$/.exec(this.buffer.subarray(0, end).toString());
        if (!match) throw new Error('Invalid preview protocol');
        const [w, h, frame, size] = match.slice(1).map(Number);
        if (w < 1 || h < 1 || w > 1920 || h > 1080 || size !== w * h * 3 || !Number.isSafeInteger(frame))
          throw new Error('Invalid preview dimensions');
        this.header = {w, h, frame, size};
        this.buffer = this.buffer.subarray(end + 1);
      }
      if (this.buffer.length < this.header.size) return;
      const header = this.header;
      const rgb = this.buffer.subarray(0, header.size);
      this.buffer = this.buffer.subarray(header.size);
      this.header = null;
      this.onFrame({...header, rgb: rgb.toString('base64')});
    }
  }
}
module.exports = {FrameDecoder};
