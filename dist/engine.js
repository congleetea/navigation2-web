/* Browser-local educational implementations, not the Nav2 runtime. */
(function(root){
'use strict';
const TAU=Math.PI*2, dirs4=[[1,0],[-1,0],[0,1],[0,-1]],dirs8=[...dirs4,[1,1],[1,-1],[-1,1],[-1,-1]];
const dist=(a,b)=>Math.hypot(a.x-b.x,a.y-b.y),angle=a=>Math.atan2(Math.sin(a),Math.cos(a));
class Heap{constructor(){this.a=[]}push(v){let i=this.a.length;this.a.push(v);while(i){let p=(i-1)>>1;if(this.a[p].f<=v.f)break;this.a[i]=this.a[p];i=p}this.a[i]=v}pop(){const a=this.a,r=a[0],v=a.pop();if(a.length){let i=0;while(i*2+1<a.length){let j=i*2+1;if(j+1<a.length&&a[j+1].f<a[j].f)j++;if(a[j].f>=v.f)break;a[i]=a[j];i=j}a[i]=v}return r}get length(){return this.a.length}}
function free(m,x,y){const ix=Math.floor(x+.5),iy=Math.floor(y+.5);return ix>=0&&iy>=0&&ix<m.w&&iy<m.h&&!m.walls[iy*m.w+ix]}
function prepare(input,weight=2){const m={w:input.w,h:input.h,walls:[...input.walls],weight};m.cost=Array(m.w*m.h).fill(0);const obs=[];m.walls.forEach((v,i)=>{if(v)obs.push([i%m.w,Math.floor(i/m.w)])});for(let y=0;y<m.h;y++)for(let x=0;x<m.w;x++){const i=y*m.w+x;if(m.walls[i]){m.cost[i]=254;continue}let d=Infinity;for(const [ox,oy]of obs)d=Math.min(d,Math.hypot(x-ox,y-oy));m.cost[i]=d<3.5?Math.round(180*Math.exp(-(d-1)*1.15)):0}return m}
function line(m,a,b){const n=Math.max(1,Math.ceil(dist(a,b)*10));let cost=0,prev=null;for(let j=0;j<=n;j++){const t=j/n,x=a.x+(b.x-a.x)*t,y=a.y+(b.y-a.y)*t;if(!free(m,x,y))return {ok:false,cost:Infinity};const ix=Math.floor(x+.5),iy=Math.floor(y+.5);if(prev&&ix!==prev[0]&&iy!==prev[1]&&(!free(m,ix,prev[1])||!free(m,prev[0],iy)))return {ok:false,cost:Infinity};prev=[ix,iy];cost+=(1+m.weight*m.cost[iy*m.w+ix]/252)*dist(a,b)/(n+1)}return {ok:true,cost}}
function poseFree(m,p){if(!free(m,p.x,p.y))return false;for(let i=0;i<8;i++)if(!free(m,p.x+.32*Math.cos(i*TAU/8),p.y+.32*Math.sin(i*TAU/8)))return false;return true}
function pathOK(m,p){return p.every((v,i)=>free(m,v.x,v.y)&&(!i||line(m,p[i-1],v).ok))}
function coordinate(m,id){return {x:id%m.w,y:Math.floor(id/m.w)}}
function plan(input,start,goal,mode,opt={}){
 const m=prepare(input,opt.weight??2),events=[],s={...start,theta:start.theta??0},t={...goal,theta:goal.theta??0};
 const result={map:m,events,path:[],raw:[],success:false,expanded:0,mode};
 const event=(phase,title,text,data={})=>events.push({phase,title,text,...data});
 const fail=message=>{event('failed','规划结束 · 未找到路径',message);result.message=message;return result};
 event('init','初始化代价地图','灰色 254 为障碍；蓝色数值低，橙红色数值高。A* 搜索默认显示 f，梯度提取使用 P，地图代价 C 可单独切换。');
 if(!free(m,s.x,s.y)||!free(m,t.x,t.y))return fail('起点或目标在障碍物内，或超出地图。请重新设置。');
 if(mode==='lattice'&&[s.theta,t.theta].some(a=>Math.abs(a/(Math.PI/4)-Math.round(a/(Math.PI/4)))>1e-6))return fail('Lattice 教学基元要求起终朝向为 45° 的整数倍，请调整朝向档位。');
 const startH=mode==='nav-dijkstra'?0:dist(s,t)+(['hybrid','lattice'].includes(mode)?.25*Math.abs(angle(s.theta-t.theta)):0);events[0].updates=[{id:Math.round(s.y)*m.w+Math.round(s.x),v:0,h:startH,f:startH}];
 if(dist(s,t)<.01&&(!['hybrid','lattice'].includes(mode)||Math.abs(angle(s.theta-t.theta))<.01)){result.path=[s];result.raw=[s];result.success=true;event('done','起终位姿重合','无需移动，返回一个有效位姿。',{path:[s]});return result}
 const done=()=>{result.success=true;event('done','规划完成','路径已恢复。点击时间线或上一步，可重新检查每次计算。',{path:result.path});return result};
 if(mode.startsWith('nav-')){
  const N=m.w*m.h,pot=Array(N).fill(Infinity),closed=new Set(),q=new Heap(),si=Math.round(s.y)*m.w+Math.round(s.x),ti=Math.round(t.y)*m.w+Math.round(t.x);pot[si]=0;events[0].updates=[{id:si,v:0,h:startH,f:startH}];q.push({id:si,g:0,f:0});
  const h=id=>mode==='nav-astar'?dist(coordinate(m,id),t):0;
  const at=(x,y)=>x>=0&&y>=0&&x<m.w&&y<m.h?pot[y*m.w+x]:Infinity;
  while(q.length&&result.expanded<12000){const n=q.pop();if(closed.has(n.id)||n.g>pot[n.id]+1e-7)continue;closed.add(n.id);result.expanded++;const p=coordinate(m,n.id),updates=[];
   for(const [dx,dy]of dirs4){const x=p.x+dx,y=p.y+dy;if(!free(m,x,y))continue;const id=y*m.w+x;if(closed.has(id))continue;let a=Math.min(at(x-1,y),at(x+1,y)),b=Math.min(at(x,y-1),at(x,y+1));const low=Math.min(a,b),delta=Math.abs(a-b),c=1+m.weight*m.cost[id]/252;const z=delta/c,v=delta>=c?low+c:low+c*(-.2301*z*z+.5307*z+.7040);if(v<pot[id]){pot[id]=v;updates.push({id,v,h:h(id),f:v+h(id)});q.push({id,g:v,f:v+h(id)})}}
   event('search','四邻域势值传播',`更新上下左右的势值。${mode==='nav-astar'?'启发距离只影响处理顺序，不写入势值。':'Dijkstra 按累计势值优先传播。'}`,{current:p,closed:n.id,updates,g:pot[n.id],h:h(n.id),f:pot[n.id]+h(n.id),neighbors:dirs4.map(([x,y])=>({x:p.x+x,y:p.y+y})),formula:'P = L + c · φ(|a−b| / c)'});
   if(n.id===ti)break;
  }
  result.potential=pot;if(!closed.has(ti))return fail('势传播无法到达目标：障碍阻断了可通行区域。');
  event('gradient','从势值计算下降方向','有效四邻域上：dx = P左 − P右，dy = P上 − P下；归一化得到下降方向。',{showGradient:true,formula:'d = normalize(−∇P)'});
  function grad(ix,iy){if(ix<0||iy<0||ix>=m.w||iy>=m.h||!Number.isFinite(at(ix,iy)))return {x:0,y:0};const c=at(ix,iy);let x=0,y=0;if(Number.isFinite(at(ix-1,iy)))x+=at(ix-1,iy)-c;if(Number.isFinite(at(ix+1,iy)))x+=c-at(ix+1,iy);if(Number.isFinite(at(ix,iy-1)))y+=at(ix,iy-1)-c;if(Number.isFinite(at(ix,iy+1)))y+=c-at(ix,iy+1);const n=Math.hypot(x,y)||1;return {x:x/n,y:y/n}}
  result.gradients=pot.map((v,id)=>Number.isFinite(v)?grad(id%m.w,Math.floor(id/m.w)):null);
  let p={x:t.x,y:t.y},rev=[p],seen=new Map();
  for(let k=0;k<N*5&&dist(p,s)>.4;k++){
   const ix=Math.floor(p.x),iy=Math.floor(p.y),fx=p.x-ix,fy=p.y-iy;let gx=0,gy=0;for(const [dx,dy]of [[0,0],[1,0],[0,1],[1,1]]){const g=grad(ix+dx,iy+dy),w=(dx?fx:1-fx)*(dy?fy:1-fy);gx+=g.x*w;gy+=g.y*w}const len=Math.hypot(gx,gy);let next={x:p.x+.32*gx/(len||1),y:p.y+.32*gy/(len||1)};
   const key=`${Math.round(p.x*3)},${Math.round(p.y*3)}`;seen.set(key,(seen.get(key)||0)+1);
   if(len<1e-5||!line(m,p,next).ok||seen.get(key)>3){const cx=Math.round(p.x),cy=Math.round(p.y);let best=null,bv=at(cx,cy);for(const [dx,dy]of dirs8){const v=at(cx+dx,cy+dy),b={x:cx+dx,y:cy+dy};if(v<bv&&line(m,p,b).ok){best=b;bv=v}}if(!best)break;next=best}
   rev.push(next);p=next;event('extract','沿负梯度提取路径','从实际目标沿势值下降方向走向零势值源；最后反转，得到起点到目标的路径。',{current:p,path:[...rev],formula:'p_next = p + step · normalize(d)'});
  }
  if(dist(p,s)>.8)return fail('势场已建立，但本教学提取器在局部无法继续下降；可调整端点或地图重试。');
  if(line(m,p,s).ok)rev.push({x:s.x,y:s.y});result.raw=rev.reverse();result.path=result.raw.map(p=>({...p}));return done();
 }
 if(mode==='smac'||mode==='theta'){
  const si=Math.round(s.y)*m.w+Math.round(s.x),ti=Math.round(t.y)*m.w+Math.round(t.x),nodes=new Map(),closed=new Set(),q=new Heap();nodes.set(si,{id:si,g:0,parent:null});q.push({id:si,g:0,f:dist(s,t)});
  while(q.length&&result.expanded<10000){const a=q.pop(),n=nodes.get(a.id);if(closed.has(a.id)||a.g>n.g+1e-7)continue;closed.add(a.id);result.expanded++;const p=coordinate(m,a.id),updates=[],edges=[];
   if(mode==='theta'&&n.parent!==null){const par=nodes.get(n.parent);if(par.parent!==null){const anc=coordinate(m,par.parent),l=line(m,anc,p),ng=nodes.get(par.parent).g+l.cost,accepted=l.ok&&ng<n.g-1e-7;if(accepted){n.g=ng;n.parent=par.parent}event('los','检查祖先与当前节点的视线',accepted?'无碰撞，而且代价更低：跳过中间父节点，连接祖先。':l.ok?'直线可通行，但没有更低代价，保留原父节点。':'直线穿过障碍，保留原父节点。',{current:p,los:{from:anc,to:p},accepted,updates:accepted?[{id:a.id,v:n.g,h:dist(p,t),f:n.g+dist(p,t)}]:[],formula:'g_candidate = g(ancestor) + cost(LOS)'})}}
   event('search','扩展八邻域节点','从优先队列取出 f = g + h 最小的节点，检查相邻格子并保存更低代价的父节点。',{current:p,closed:a.id,g:n.g,h:dist(p,t),f:n.g+dist(p,t),neighbors:dirs8.map(([x,y])=>({x:p.x+x,y:p.y+y}))});
   if(a.id===ti)break;
   for(const [dx,dy]of dirs8){const b={x:p.x+dx,y:p.y+dy};if(!free(m,b.x,b.y)||!line(m,p,b).ok)continue;const id=b.y*m.w+b.x;if(closed.has(id))continue;const c=mode==='theta'?line(m,p,b).cost:Math.hypot(dx,dy)*(1+m.weight*m.cost[id]/252),g=n.g+c,old=nodes.get(id);if(!old||g<old.g){nodes.set(id,{id,g,parent:a.id});q.push({id,g,f:g+dist(b,t)});updates.push({id,v:g,h:dist(b,t),f:g+dist(b,t)});edges.push({from:p,to:b})}}
   Object.assign(events.at(-1),{updates,edges});
  }
  if(!closed.has(ti))return fail('搜索队列耗尽，目标不可达。');let id=ti,back=[];while(id!==null){const p=coordinate(m,id);back.push(p);event('extract','沿父节点回溯','从目标追溯 parent，直到起点；随后反转路径。',{current:p,path:[...back]});id=nodes.get(id).parent}result.raw=back.reverse();result.path=result.raw.map(p=>({...p}));
  if(mode==='smac'&&result.path.length>=12){const original=result.raw;for(let it=0;it<22;it++){const next=result.path.map(p=>({...p}));for(let i=1;i<next.length-1;i++)for(const d of ['x','y'])next[i][d]+=.2*(original[i][d]-next[i][d])+.3*(next[i-1][d]+next[i+1][d]-2*next[i][d]);if(!pathOK(m,next)){event('smooth','平滑候选被拒绝','候选折线碰撞，保留上一轮安全路径。',{path:result.path,raw:result.raw});break}result.path=next;event('smooth',`位置平滑 · 第 ${it+1} 轮`,'原始路径吸引项限制偏移，相邻点平滑项减小折角，端点保持不动。',{path:next,raw:original,formula:'qᵢ += 0.2(pᵢ−qᵢ) + 0.3(qᵢ₋₁+qᵢ₊₁−2qᵢ)'})}}
  if(mode==='smac'){for(let i=0;i<result.path.length-1;i++)result.path[i].theta=Math.atan2(result.path[i+1].y-result.path[i].y,result.path[i+1].x-result.path[i].x);result.path.at(-1).theta=t.theta;event('orientation','单独生成有效朝向',result.path.length<12?'本地 Nav2 的短段会跳过位置平滑与朝向；本网页保留短段几何，但独立计算有效朝向。':'中间点朝向指向下一点，末点朝向采用设置的目标朝向。',{path:result.path,formula:'θᵢ = atan2(Δy, Δx)'})}else event('orientation','直线段输出','教学版本保留搜索终点，线段方向来自位置；没有把折角变成圆弧。',{path:result.path});return done();
 }
 if(!['hybrid','lattice'].includes(mode))return fail('未知的规划器模式。');
 if(!poseFree(m,s)||!poseFree(m,t))return fail('位姿演示使用半径 0.32 格的圆形足迹；端点离障碍太近。');
 const bins=mode==='hybrid'?16:8,da=TAU/bins;
 const key=p=>`${Math.round(p.x)},${Math.round(p.y)},${((Math.round(p.theta/da)%bins)+bins)%bins}`;
 const heur=p=>dist(p,t)+.25*Math.abs(angle(p.theta-t.theta));
 const nodes=new Map(),closed=new Set(),q=new Heap();let seq=0;const first={...s,g:0,parent:null,id:seq++,segment:[s]};nodes.set(key(s),first);q.push({node:first,f:heur(s)});let found=null;
 function motions(p){const out=[];if(mode==='hybrid'){for(const turn of [-1,0,1]){const seg=[],length=1.18,delta=turn*da,R=length/(delta||1),n=10;for(let i=1;i<=n;i++){const u=i/n,th=p.theta+delta*u;seg.push(turn?{x:p.x+R*(Math.sin(th)-Math.sin(p.theta)),y:p.y-R*(Math.cos(th)-Math.cos(p.theta)),theta:angle(th)}:{x:p.x+length*u*Math.cos(p.theta),y:p.y+length*u*Math.sin(p.theta),theta:p.theta})}out.push({seg,turn,length})}}
  else{const heading=Math.round(p.theta/da)*da;for(const turn of [-1,0,1]){const th=heading+turn*da,local={x:turn?2:2,y:turn},ex=Math.round(local.x*Math.cos(heading)-local.y*Math.sin(heading)),ey=Math.round(local.x*Math.sin(heading)+local.y*Math.cos(heading)),end={x:p.x+ex,y:p.y+ey,theta:angle(th)},len=Math.hypot(ex,ey),handle=len/3,seg=[];for(let j=1;j<=12;j++){const u=j/12,v=1-u,x=v*v*v*p.x+3*v*v*u*(p.x+handle*Math.cos(heading))+3*v*u*u*(end.x-handle*Math.cos(th))+u*u*u*end.x,y=v*v*v*p.y+3*v*v*u*(p.y+handle*Math.sin(heading))+3*v*u*u*(end.y-handle*Math.sin(th))+u*u*u*end.y;const dx=3*v*v*handle*Math.cos(heading)+6*v*u*((end.x-handle*Math.cos(th))-(p.x+handle*Math.cos(heading)))+3*u*u*handle*Math.cos(th),dy=3*v*v*handle*Math.sin(heading)+6*v*u*((end.y-handle*Math.sin(th))-(p.y+handle*Math.sin(heading)))+3*u*u*handle*Math.sin(th);seg.push({x,y,theta:Math.atan2(dy,dx)})}out.push({seg,turn,length:len})}for(const turn of [-1,1])out.push({seg:Array.from({length:6},(_,i)=>({x:p.x,y:p.y,theta:angle(p.theta+turn*da*(i+1)/6)})),turn,length:.9,rotate:true})}return out}
 const all=new Map([[first.id,first]]);
 while(q.length&&result.expanded<6500){const n=q.pop().node,k=key(n);if(nodes.get(k)!==n||closed.has(k))continue;closed.add(k);result.expanded++;const updates=[],candidates=[];
  if(dist(n,t)<.7&&Math.abs(angle(n.theta-t.theta))<da*.55){found=n;event('search','达到目标位姿容差','位置误差 < 0.7 格，角度误差 < 半个角度档位。',{current:n,g:n.g,h:heur(n),f:n.g+heur(n)});break}
  for(const a of motions(n)){let prev=n;const ok=a.seg.every(p=>{const valid=poseFree(m,p)&&line(m,prev,p).ok;prev=p;return valid});candidates.push({points:[n,...a.seg],ok,rotate:a.rotate});if(!ok)continue;const b=a.seg.at(-1),bk=key(b);if(closed.has(bk))continue;const avg=a.seg.reduce((v,p)=>v+m.cost[Math.round(p.y)*m.w+Math.round(p.x)],0)/a.seg.length,g=n.g+a.length*(1+m.weight*avg/252)*(a.turn?1.08:1);if(!nodes.has(bk)||g<nodes.get(bk).g){const child={...b,g,parent:n.id,id:seq++,segment:a.seg};nodes.set(bk,child);all.set(child.id,child);q.push({node:child,f:g+heur(child)});updates.push({id:Math.round(b.y)*m.w+Math.round(b.x),v:g,h:heur(child),f:g+heur(child)})}}
  event('search',mode==='hybrid'?'按转弯半径投影运动':'从内置状态格动作表扩展',mode==='hybrid'?'当前车头朝向决定直行、左弧和右弧；红色为被碰撞检查拒绝的动作。':'选择当前朝向的直行、转弯或原地旋转动作；展开基元中的采样位姿。',{current:n,closed:Math.round(n.y)*m.w+Math.round(n.x),updates,candidates,g:n.g,h:heur(n),f:n.g+heur(n),formula:mode==='hybrid'?'x′ = x + R(sin θ′ − sin θ)':'edge = primitive[heading][action]'})
 }
 if(!found)return fail(result.expanded>=6500?'达到教学搜索预算（6500 个状态）。可换起终点、减小障碍或选择 Lattice 原地旋转模型。':'当前运动模型未找到满足位置和朝向容差的路线。');
 const chain=[];let n=found;while(n){chain.push(n);n=n.parent===null?null:all.get(n.parent)}chain.reverse();let path=[s];for(let i=1;i<chain.length;i++){path=path.concat(chain[i].segment);event('extract','恢复运动序列','沿父状态恢复动作，并展开各动作的采样点。路径箭头表示车头朝向。',{path:[...path],current:chain[i]})}result.path=path;result.raw=path;return done();
}
const api={plan,prepare,free,line,poseFree,angle,dist};root.PlannerLab=api;if(typeof module!=='undefined')module.exports=api;
})(globalThis);
