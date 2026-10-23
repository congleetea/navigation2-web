const {test}=require('node:test');const assert=require('node:assert/strict');const L=require('../dist/engine.js');
for(const mode of ['nav-dijkstra','nav-astar','theta','smac','hybrid','lattice'])test(mode+' 每个更新携带真实启发值和 f=g+h',()=>{
 const m={w:20,h:15,walls:Array(300).fill(false)},r=L.plan(m,{x:3,y:7,theta:0},{x:15,y:7,theta:0},mode);
 const updates=r.events.flatMap(e=>e.updates||[]);assert.ok(updates.length);
 for(const u of updates){assert.ok(Number.isFinite(u.h));assert.ok(Math.abs(u.f-u.v-u.h)<1e-8);if(mode==='nav-dijkstra')assert.equal(u.h,0)}
 assert.equal(updates[0].v,0);assert.equal(updates[0].f,mode==='nav-dijkstra'?0:12);
});
