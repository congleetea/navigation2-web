const {test}=require('node:test');
const assert=require('node:assert/strict');
const L=require('../dist/engine.js');
for(const mode of ['nav-dijkstra','nav-astar','theta','smac','hybrid','lattice']){
 test(mode+' 每步代价回放包含起点零值',()=>{
  const m={w:20,h:15,walls:Array(300).fill(false)};
  const r=L.plan(m,{x:3,y:7,theta:0},{x:15,y:7,theta:0},mode);
  assert.equal(r.events[0].updates?.find(u=>u.id===143)?.v,0);
 });
}
test('Theta LOS 改善当前节点代价时，回放同步记录这个更低值',()=>{
 const m={w:20,h:15,walls:Array(300).fill(false)};
 const r=L.plan(m,{x:2,y:3},{x:16,y:11},'theta');
 const los=r.events.filter(e=>e.phase==='los'&&e.accepted);assert.ok(los.length>0);
 for(const e of los)assert.ok(e.updates?.some(u=>u.id===e.current.y*20+e.current.x&&Number.isFinite(u.v)));
});
