"""Shared .ray to C++ compiler; no CLI or export dependency."""
import re
import json

# Preserve strings/comments as opaque tokens; only rewrite actual code tokens.
TOKEN = re.compile(r'R"(?P<delimiter>[^ ()\\\t\r\n]{0,16})\(.*?\)(?P=delimiter)"|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|//[^\n]*|/\*.*?\*/|[A-Za-z_]\w*|\s+|.', re.S)

def transpile(source, preview=False, filename="main.ray"):
    tokens = [m.group() for m in TOKEN.finditer(source)]
    significant = [i for i,t in enumerate(tokens) if not t.isspace() and not t.startswith(('//','/*'))]
    for k, i in enumerate(significant):
        seq = [tokens[j] for j in significant[k:k+7]]
        if len(seq)==7 and seq[0]=='void' and re.fullmatch(r'[A-Za-z_]\w*',seq[1]) and seq[2:]==['=','object','.','init','(']:
            tokens[i]='auto'
    # Include directives belong outside the generated entry point.
    body=''.join(tokens)
    includes=[]
    lines=body.splitlines(keepends=True)
    for i,line in enumerate(lines):
        if re.match(r'^\s*#\s*include\b',line):
            includes.append(line); lines[i]='\n'
    generated = '''#include <raymotion/runtime.hpp>
#include <iostream>
#include <cmath>
'''+''.join(includes)+'''using namespace raymotion;
int main(int argc, char** argv) {
  try {
    if(argc!=8) throw std::runtime_error("run this program with raymotion export");
    const int width=std::stoi(argv[2]), height=std::stoi(argv[3]);
    const int sample=std::stoi(argv[4]), framerate=std::stoi(argv[5]);
    const bool video=std::string(argv[6])=="mp4";
    Objects object(argv[1],width,height,sample,video,framerate,std::stoi(argv[7]));
    auto& camera=object.camera;
    const auto& u_timer=object.u_timer;
    const auto& u_resolution=object.u_resolution;
    do {
    const double frame_start=u_timer;
#line 1 "main.ray"
'''+''.join(lines)+'''
    object.finish();
    if(u_timer==frame_start) throw std::runtime_error("no object.render() executed in this iteration");
    } while(video);
    return 0;
  } catch(const RenderComplete&) { return 0;
  } catch(const std::exception& e) { std::cerr << "raymotion: " << e.what() << "\\n"; return 1; }
}
'''

    generated = generated.replace('#line 1 "main.ray"', '#line 1 ' + json.dumps(filename))
    if preview:
        generated = generated.replace('#include <raymotion/runtime.hpp>', '#include <raymotion/preview.hpp>')
        generated = generated.replace('Objects object(argv[1],width,height,sample,video,framerate,std::stoi(argv[7]));',
            'PreviewStream stream;\n    Objects object(stream.sink(),width,height,sample,framerate);')
    return generated
