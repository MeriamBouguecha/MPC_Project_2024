// Dependency-free checks for replay data and control handlers.
// This exercises JavaScript using a minimal canvas/DOM harness, not a browser.
import fs from 'node:fs';
import vm from 'node:vm';
import assert from 'node:assert/strict';
import path from 'node:path';
import {fileURLToPath} from 'node:url';
const root=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'..');
for(const folder of ['docs']) {
  const html=fs.readFileSync(path.join(root,folder,'replay.html'),'utf8');
  assert(!html.includes('__SIMULATION_DATA__'));
  const dataText=html.match(/<script id="simulation-data" type="application\/json">([\s\S]*?)<\/script>/)[1];
  const data=JSON.parse(dataText);
  for(const sim of data){
    assert.equal(sim.states.length,sim.t.length);
    assert.equal(sim.inputs.length,sim.t.length-1);
    assert.equal(sim.reference.length,sim.t.length);
    assert(sim.states.flat().every(Number.isFinite));
    assert(sim.inputs.flat().every(Number.isFinite));
    assert(sim.metrics.constraintViolation<=1e-5);
    for(let i=1;i<sim.t.length;i++)assert(sim.t[i]>sim.t[i-1]);
    if(sim.otherStates?.length){
      assert.equal(sim.otherStates.length,sim.t.length);
      for(let i=0;i<sim.t.length;i++){
        const a=sim.states[i],b=sim.otherStates[i];
        assert((a[0]-b[0])**2/144+(a[1]-b[1])**2/9>=1-1e-6);
      }
    }
  }
  let draws=0;
  const ctx=new Proxy({}, {get:(_,key)=> (...args)=>{
    draws++;
    for(const n of args.filter(x=>typeof x==='number'))assert(Number.isFinite(n),`Nonfinite ${key}`);
  },set:()=>true});
  const elements=new Map();
  const get=id=>{
    if(!elements.has(id))elements.set(id,{value:id==='rate'?'1':'0',textContent:'',tagName:'DIV',
      handlers:{},addEventListener(type,fn){this.handlers[type]=fn},
      append(){},getContext(){return ctx},getBoundingClientRect(){return {width:id==='road'?1200:350,height:id==='road'?280:165}}});
    return elements.get(id);
  };
  get('simulation-data').textContent=dataText;
  const document={getElementById:get,createElement:()=>({}),activeElement:{tagName:'BODY'},handlers:{},
    addEventListener(type,fn){this.handlers[type]=fn}};
  let raf;
  const sandbox={document,window:{devicePixelRatio:2,addEventListener(){}},requestAnimationFrame:fn=>{raf=fn},console};
  vm.createContext(sandbox);
  const script=html.match(/<script>\s*([\s\S]*?)<\/script>/)[1];
  vm.runInContext(script,sandbox);
  assert.equal(get('title').textContent,data[0].title);
  assert(draws>100);
  get('play').handlers.click();
  assert.equal(get('play').textContent,'Pause');
  raf(0);raf(1000);
  assert.equal(Number(get('scrub').value),1);
  get('scrub').handlers.input({target:{value:'5'}});
  assert.equal(get('play').textContent,'Play');
  assert(get('clock').textContent.startsWith('5.0'));
  document.handlers.keydown({code:'ArrowRight',preventDefault(){}});
  assert(get('clock').textContent.startsWith('6.0'));
  get('reset').handlers.click();
  assert(get('clock').textContent.startsWith('0.0'));
  for(let i=0;i<data.length;i++){
    get('scenario').value=String(i);get('scenario').handlers.change();
    assert.equal(get('title').textContent,data[i].title);
    get('scrub').handlers.input({target:{value:String(data[i].t.at(-1))}});
    assert(get('clock').textContent.startsWith(data[i].t.at(-1).toFixed(1)));
  }
  console.log(`${folder}: ${data.length} datasets and playback/seek/reset/scenario handlers passed`);
}
