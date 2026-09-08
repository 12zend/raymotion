const vscode = acquireVsCodeApi();
const canvas = document.getElementById('canvas');
const context = canvas.getContext('2d');
const play = document.getElementById('play');
const stop = document.getElementById('stop');
let image;
play.onclick = () => vscode.postMessage({type:'toggle'});
stop.onclick = () => vscode.postMessage({type:'stop'});
window.addEventListener('message', ({data}) => {
  if (data.type === 'state') {
    const playing = data.state === 'playing';
    play.disabled = data.state === 'compiling';
    stop.disabled = data.state === 'stopped' || data.state === 'error';
    const label = playing ? '一時停止' : data.state === 'paused' ? '再開' : '再生';
    document.getElementById('playLabel').textContent = label;
    play.setAttribute('aria-label', label);
    document.getElementById('playIcon').setAttribute('d', playing ? 'M6 5h4v14H6zM14 5h4v14h-4z' : 'M8 5v14l11-7z');
    document.getElementById('status').textContent = data.detail || '再生中';
  }
  if (data.type === 'frame') {
    if (!image || canvas.width !== data.w || canvas.height !== data.h) {
      canvas.width = data.w; canvas.height = data.h;
      image = context.createImageData(data.w, data.h);
      for (let j = 3; j < image.data.length; j += 4) image.data[j] = 255;
    }
    const rgb = atob(data.rgb);
    for (let i = 0, j = 0; i < rgb.length; i += 3, j += 4) {
      image.data[j] = rgb.charCodeAt(i); image.data[j+1] = rgb.charCodeAt(i+1);
      image.data[j+2] = rgb.charCodeAt(i+2);
    }
    context.putImageData(image, 0, 0);
    canvas.hidden = false; document.getElementById('empty').hidden = true;
    document.getElementById('time').textContent = 'Frame ' + (data.frame + 1);
    vscode.postMessage({type:'presented'});
  }
});
vscode.postMessage({type:'ready'});
