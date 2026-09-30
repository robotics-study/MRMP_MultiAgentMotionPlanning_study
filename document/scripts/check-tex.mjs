// TeX 체크어 — 모든 math 문자열을 KaTeX로 실제 렌더해 본다(throwOnError + strict warn도 실패).
// JSX attribute 문자열은 JS 이스케이프를 처리하지 않으므로 math="..."에는 역슬래시를 단일로
// 적어야 한다(\dots); 실수로 \\dots를 적으면 개시 매크로 + 리터럴 "dots"가 렌더된다 — 이 체커는
// 그 클래스를 strict warn으로 잡는다. cases의 줄바꿈 \\은 양쪽 모드에서 경고 없이 정상 렌더된다.
// math= attribute가 아닌 문자열(Terms key 등)은 JS 문자열 리터럴이므로 이스케이프가 풀린 값을
// 렌더한다 — 스캐너도 같은 방식으로 unescape한다.
import katex from 'katex';
import fs from 'fs'; import path from 'path';
const B = String.fromCharCode(92);
function walk(dir, acc = []) { for (const e of fs.readdirSync(dir, {withFileTypes: true})) { const p = path.join(dir, e.name); if (e.isDirectory()) walk(p, acc); else if (p.endsWith('.tsx')) acc.push(p);} return acc; }
function jsUnescape(s) { let out=''; for (let i=0;i<s.length;i++){ const c=s[i]; if(c!==B){out+=c;continue;} const n=s[++i]; switch(n){ case 't':out+='\t';break; case 'n':out+='\n';break; case 'r':out+='\r';break; case 'b':out+='\b';break; case 'f':out+='\f';break; case '0':out+='\0';break; case 'v':out+='\v';break; case B:out+=B;break; default: out+=n; } } return out; }
const files = walk('src').filter(f => !f.includes('CodeHighlight'));
let found = [];
for (const f of files) {
  const t = fs.readFileSync(f, 'utf8'); let line = 1; let i = 0;
  while (i < t.length) {
    const c = t[i]; if (c === '\n') line++;
    if (c !== '"' && c !== '`') { i++; continue; }
    const attr = t.slice(Math.max(0, i - 5), i) === 'math=';
    const sl = line; let s = ''; i++;
    while (i < t.length && t[i] !== c) { if (t[i] === B) { s += t[i] + t[i+1]; i += 2; } else { if (t[i]==='\n') line++; s += t[i++]; } }
    i++;
    if (!s.includes(B)) continue;
    const runtime = attr ? s : jsUnescape(s);
    found.push({f, line: sl, raw: s, runtime, attr});
  }
}
console.log('candidates:', found.length);
const bad = [];
for (const c of found) {
  if (!c.attr && !c.runtime.includes(B)) { bad.push({...c, why: 'TeX lost its backslash to JS escaping'}); continue; }
  let err = '';
  for (const dm of [false,true]) {
    const warns = [];
    const origWarn = console.warn;
    console.warn = (...a) => { warns.push(a.join(' ')); };
    try { katex.renderToString(c.runtime, {displayMode: dm, throwOnError: true}); } catch(e){ err = String(e.message||e); break; } finally { console.warn = origWarn; }
    if (warns.length) { err = 'strict warn: ' + warns.join(' | '); break; }
  }
  if (err) bad.push({...c, why: err.slice(0,120)});
}
console.log('BAD count:', bad.length);
for (const b of bad) console.log('-', b.f+':'+b.line, '|', JSON.stringify(b.raw.slice(0,70)), '|', b.why);
if (bad.length > 0) process.exit(1);
