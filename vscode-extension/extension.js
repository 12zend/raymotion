'use strict';
const vscode = require('vscode');
const {spawn} = require('node:child_process');
const fs = require('node:fs/promises');
const path = require('node:path');
const os = require('node:os');
const crypto = require('node:crypto');
const {FrameDecoder} = require('./protocol');

function kill(child) {
  if (!child?.pid) return;
  try { process.kill(-child.pid, 'SIGKILL'); } catch {}
}
function activate(context) {
  const output = vscode.window.createOutputChannel('Raymotion');
  context.subscriptions.push(output, vscode.commands.registerCommand('raymotion.openPreview', async () => {
    const doc = vscode.window.activeTextEditor?.document;
    if (!doc || doc.uri.scheme !== 'file' || !doc.fileName.endsWith('.ray')) {
      vscode.window.showErrorMessage('.rayファイルを開いてからプレビューを実行してください。'); return;
    }
    const panel = vscode.window.createWebviewPanel('raymotion.preview', 'Raymotion Preview',
      vscode.ViewColumn.Beside, {enableScripts: true, retainContextWhenHidden: true,
        localResourceRoots: [vscode.Uri.joinPath(context.extensionUri, 'media')]});
    let state = 'stopped', child, directory, generation = 0, timer, pending, disposed = false;
    const post = message => { if (!disposed) panel.webview.postMessage(message); };
    const status = (next, detail = '') => { state = next; post({type: 'state', state, detail}); };
    const stop = () => {
      generation++; clearTimeout(timer); pending = null; kill(child); child = undefined;
      const old = directory; directory = undefined;
      if (old) fs.rm(old, {recursive: true, force: true}).catch(() => {});
      status('stopped', '停止しました。次の再生で再コンパイルします。');
    };
    const fail = error => { stop(); status('error', String(error.message || error)); output.show(true); };
    const config = () => vscode.workspace.getConfiguration('raymotion', doc.uri);
    let fps = 30;
    function present() {
      if (state !== 'playing' || !pending) return;
      post({type: 'frame', ...pending}); pending = null;
      // Next credit is issued only after the canvas acknowledges presentation.
    }
    async function start() {
      if (!vscode.workspace.isTrusted) { status('error', 'プレビューにはワークスペースの信頼が必要です。'); return; }
      if (process.platform !== 'darwin') { status('error', 'プレビューはmacOSとMetalが必要です。'); return; }
      const token = ++generation;
      status('compiling', 'シーンをコンパイル中…');
      try {
        const cfg = config();
        const root = cfg.get('runtimePath') || path.resolve(context.extensionPath, '..');
        const options = ['width', 'height', 'sample', 'framerate'].map((key, i) => {
          const defaults = [640, 360, 4, 30], limits = [1920, 1080, 256, 120];
          const value = cfg.get('preview.' + key, defaults[i]);
          if (!Number.isInteger(value) || value < 1 || value > limits[i]) throw new Error('不正なプレビュー設定: ' + key);
          return value;
        });
        fps = options[3];
        const temp = await fs.mkdtemp(path.join(os.tmpdir(), 'raymotion-preview-'));
        if (token !== generation) { await fs.rm(temp, {recursive:true, force:true}); return; }
        directory = temp;
        await new Promise((resolve, reject) => {
          const build = spawn(cfg.get('pythonPath', 'python3'),
            [path.join(context.extensionPath, 'compile.py'), root, doc.fileName, temp, cfg.get('compilerPath', 'c++')],
            {cwd:path.dirname(doc.fileName), detached:true, stdio:['pipe','pipe','pipe']});
          child = build;
          build.stdout.on('data', b => output.append(b.toString()));
          build.stderr.on('data', b => output.append(b.toString()));
          build.on('error', reject);
          build.stdin.on('error', () => {});
          build.on('close', code => code === 0 ? resolve() : reject(new Error('コンパイルに失敗しました。Raymotion出力を確認してください。')));
          build.stdin.end(doc.getText());
        });
        if (token !== generation) return;
        child = spawn(path.join(temp, 'preview'), ['', ...options.map(String), 'mp4', '0'],
          {cwd:path.dirname(doc.fileName), detached:true, stdio:['pipe','pipe','pipe','pipe']});
        const running = child;
        running.stdin.on('error', () => {});
        running.stdout.on('data', b => output.append(b.toString()));
        running.stderr.on('data', b => output.append(b.toString()));
        const decoder = new FrameDecoder(frame => { pending = frame; present(); });
        running.stdio[3].on('data', b => {
          if (token !== generation) return;
          try { decoder.push(b); } catch (e) { fail(e); }
        });
        running.on('error', e => { if (token === generation) fail(e); });
        running.on('close', code => {
          if (token !== generation) return;
          stop();
          status(code === 0 ? 'stopped' : 'error', code === 0 ? '再生が終了しました。' : '描画に失敗しました。Raymotion出力を確認してください。');
        });
        status('playing', '最初のフレームを描画中…');
      } catch (e) { if (token === generation) fail(e); }
    }
    let waitingCredit = false;
    panel.webview.onDidReceiveMessage(message => {
      if (message.type === 'ready') status(state, '再生するとシーンをコンパイルして直接描画します。');
      if (message.type === 'stop') { waitingCredit = false; stop(); }
      if (message.type === 'toggle') {
        if (state === 'playing') { clearTimeout(timer); status('paused', '一時停止'); }
        else if (state === 'paused') {
          status('playing');
          if (pending) present();
          else if (waitingCredit) { waitingCredit = false; child?.stdin.write('\n'); }
        } else if (state !== 'compiling') { waitingCredit = false; start(); }
      }
      if (message.type === 'presented' && (state === 'playing' || state === 'paused')) {
        waitingCredit = true;
        clearTimeout(timer);
        timer = setTimeout(() => {
          if (state === 'playing' && waitingCredit) { waitingCredit = false; child?.stdin.write('\n'); }
        }, 1000 / fps);
      }
    });
    panel.onDidDispose(() => { disposed = true; stop(); });
    context.subscriptions.push({dispose: () => panel.dispose()});
    const nonce = crypto.randomBytes(18).toString('hex');
    const resource = name => panel.webview.asWebviewUri(vscode.Uri.joinPath(context.extensionUri, 'media', name));
    panel.webview.html = `<!doctype html><html lang="ja"><head><meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<meta http-equiv="Content-Security-Policy" content="default-src 'none'; style-src ${panel.webview.cspSource}; script-src 'nonce-${nonce}';">
<link rel="stylesheet" href="${resource('preview.css')}"></head><body>
<header><button id="play" type="button" aria-label="再生"><svg viewBox="0 0 24 24" aria-hidden="true"><path id="playIcon" d="M8 5v14l11-7z"/></svg><span id="playLabel">再生</span></button>
<button id="stop" type="button" disabled><svg viewBox="0 0 24 24" aria-hidden="true"><path d="M6 6h12v12H6z"/></svg>停止</button><span id="time">0.00 s</span></header>
<main><canvas id="canvas" aria-label="Raymotion描画プレビュー" hidden></canvas><p id="empty">再生してシーンをプレビュー</p></main>
<footer id="status" role="status" aria-live="polite"></footer>
<script nonce="${nonce}" src="${resource('preview.js')}"></script></body></html>`;
  }));
}
module.exports = {activate};
