// node vscode-extension/test/benchmark-protocol.js [path/to/baseline/protocol.js]
const {performance} = require('node:perf_hooks');
const path = require('node:path');
const {FrameDecoder} = require(process.argv[2] ? path.resolve(process.argv[2]) : '../protocol');
for (const [w,h] of [[640,360],[1920,1080]]) {
  const packet = Buffer.concat([Buffer.from(`RAY1 ${w} ${h} 0 ${w*h*3}\n`), Buffer.alloc(w*h*3,127)]);
  let frames=0;
  const decoder=new FrameDecoder(() => frames++);
  const run=() => { for(let j=0;j<packet.length;j+=65536) decoder.push(packet.subarray(j,j+65536)); };
  for(let i=0;i<10;i++) run();
  const start=performance.now();
  for(let i=0;i<100;i++) run();
  console.log(`${w}x${h}: ${((performance.now()-start)/100).toFixed(3)} ms/frame (${frames} frames incl. warmup)`);
}
