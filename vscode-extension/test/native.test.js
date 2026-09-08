const {test} = require('node:test');
const assert = require('node:assert/strict');
const {spawn} = require('node:child_process');
const fs = require('node:fs/promises');
const path = require('node:path');
const os = require('node:os');
const {FrameDecoder} = require('../protocol');
test('direct Metal frames use backpressure and preserve scene time', {skip: process.platform !== 'darwin', timeout: 60000}, async () => {
  const root = path.resolve(__dirname, '../..');
  const temp = await fs.mkdtemp(path.join(os.tmpdir(), 'raymotion-test-'));
  let child;
  try {
    // Reuse the already-built engine while testing the installed-prefix layout.
    await fs.symlink(path.join(root, 'include'), path.join(temp, 'include'));
    await fs.symlink(path.join(root, 'src'), path.join(temp, 'src'));
    await fs.mkdir(path.join(temp, 'lib'));
    await fs.symlink(path.join(root, 'build/libraymotion_engine.a'), path.join(temp, 'lib/libraymotion_engine.a'));
    await new Promise((resolve, reject) => {
      const build = spawn('python3', [path.join(root, 'vscode-extension/compile.py'), temp,
        path.join(temp, 'scene.ray'), path.join(temp, 'out'), 'c++']);
      let error = '';
      build.stderr.on('data', b => error += b);
      build.on('error', reject);
      build.on('close', c => c === 0 ? resolve() : reject(new Error(error)));
      build.stdin.end('std::cout << "user stdout";\nobject.render();\nif (u_timer < 0.03) throw std::runtime_error("timer");\n');
    });
    child = spawn(path.join(temp, 'out/preview'), ['', '16', '16', '1', '30', 'mp4', '0'],
      {stdio:['pipe','pipe','pipe','pipe']});
    let count = 0, error = '';
    const result = new Promise((resolve, reject) => {
      const decoder = new FrameDecoder(frame => {
        try { assert.equal(frame.frame, count++); assert.equal(frame.size, 768); }
        catch(e) { reject(e); }
        if (count === 1) setTimeout(() => {
          try { assert.equal(count, 1); child.stdin.write('\n'); } catch(e) { reject(e); }
        }, 150);
        if (count === 2) child.stdin.end();
      });
      child.stdio[3].on('data', b => { try { decoder.push(b); } catch(e) { reject(e); } });
      child.stderr.on('data', b => error += b);
      child.on('error', reject);
      child.on('close', code => code === 0 ? resolve() : reject(new Error(error)));
    });
    await result; assert.equal(count, 2);
    const files = await fs.readdir(path.join(temp, 'out'));
    assert.deepEqual(files.sort(), ['preview', 'preview.cpp']);
  } finally { child?.kill('SIGKILL'); await fs.rm(temp, {recursive:true, force:true}); }
});
