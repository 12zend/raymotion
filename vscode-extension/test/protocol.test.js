const {test} = require('node:test');
const assert = require('node:assert/strict');
const {FrameDecoder} = require('../protocol');
test('frames survive bytewise fragmentation and coalescing', () => {
  const frames = [];
  const decoder = new FrameDecoder(f => frames.push(f));
  const packet = Buffer.concat([Buffer.from('RAY1 1 1 0 3\n'), Buffer.from([0, 10, 255])]);
  for (const byte of packet) decoder.push(Buffer.from([byte]));
  decoder.push(Buffer.concat([packet, packet]));
  assert.equal(frames.length, 3);
  assert.deepEqual(Buffer.from(frames[0].rgb, 'base64'), Buffer.from([0, 10, 255]));
});
test('rejects malformed and oversized frames', () => {
  for (const header of ['RAY1 9999 1 0 29997\n', 'RAY1 1 1 0 8\n', 'broken\n', 'x'.repeat(129)])
    assert.throws(() => new FrameDecoder(() => {}).push(Buffer.from(header)));
});
