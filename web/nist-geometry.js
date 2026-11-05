import * as THREE from 'three';
import { Octree } from 'three/addons/math/Octree.js';
import { Capsule } from 'three/addons/math/Capsule.js';

const SEGMENTS = 64, DEPTH = .25, INNER = .1, MOUTH = .13, WALL = .003;
const ned = (p) => new THREE.Vector3(p[1], -p[2], -p[0]);
const directions = [[0,0,-1],[-Math.SQRT1_2,0,-Math.SQRT1_2],[0,-Math.SQRT1_2,-Math.SQRT1_2],[Math.SQRT1_2,0,-Math.SQRT1_2],[0,Math.SQRT1_2,-Math.SQRT1_2]];
// Fabrication variant: tapered 25cm buckets; side bases 18.5cm from post axis, 13cm high.
const targets = [1,2,3].flatMap(station => directions.map((axisNED,i) => {
  const baseNED = [station*3 + (i ? axisNED[0]/Math.SQRT1_2*.185 : 0), i ? axisNED[1]/Math.SQRT1_2*.185 : 0, i ? -.13 : -.15];
  return Object.freeze({id:`${station}${i ? 'ABCD'[i-1] : ''}`,station,baseNED:Object.freeze(baseNED),axisNED:Object.freeze(axisNED),openingNED:Object.freeze(baseNED.map((v,j)=>v+DEPTH*axisNED[j])),depth:DEPTH,innerBottomRadius:INNER,innerOpeningRadius:MOUTH});
}));
export const NIST_LANE = Object.freeze({targets:Object.freeze(targets),length:15,launchNED:Object.freeze([0,0,0]),launchPadSize:1.2,launchCircleRadius:.3,variant:'25cm tapered bucket, 20cm inner bottom, 26cm mouth, 3mm wall; side base offset 18.5cm at 13cm; wooden mounting cleats altitude'});

function bucketGeometry() {
  const positions=[], groups=[];
  const point=(r,y,a)=>[r*Math.cos(a),y,r*Math.sin(a)];
  const tri=(a,b,c)=>positions.push(...a,...b,...c);
  const band=(r0,y0,r1,y1,material)=>{const start=positions.length/3;for(let i=0;i<SEGMENTS;i++){const a=i*2*Math.PI/SEGMENTS,b=(i+1)*2*Math.PI/SEGMENTS;const p=point(r0,y0,a),q=point(r0,y0,b),r=point(r1,y1,a),s=point(r1,y1,b);tri(p,r,q);tri(q,r,s);}groups.push([start,positions.length/3-start,material]);};
  // Ring and target meet at an edge on the actual inner bottom plane.
  band(0,0,.075,0,2); band(.075,0,INNER,0,1);
  band(INNER,0,MOUTH,DEPTH,0);
  band(MOUTH,DEPTH,MOUTH+WALL,DEPTH,0);
  band(MOUTH+WALL,DEPTH,INNER+WALL,-WALL,0);
  band(INNER+WALL,-WALL,0,-WALL,0);
  const g=new THREE.BufferGeometry();g.setAttribute('position',new THREE.Float32BufferAttribute(positions,3));for(const args of groups)g.addGroup(...args);g.computeVertexNormals();
  // Bottom target UVs remain independent of a browser texture factory.
  const uv=[];for(let i=0;i<positions.length;i+=3)uv.push(.5+positions[i]/.15,.5+positions[i+2]/.15);g.setAttribute('uv',new THREE.Float32BufferAttribute(uv,2));return g;
}
const bucket = bucketGeometry();
export function createNistLaneGroup({labelTexture}={}) {
  const group=new THREE.Group();group.name='NIST Level 1 lane';group.userData.cameraClass=5;
  const white=new THREE.MeshStandardMaterial({color:0xffffff,side:THREE.DoubleSide,roughness:.8});
  const green=new THREE.MeshStandardMaterial({color:0x00ad55,side:THREE.DoubleSide,roughness:.8});
  const wood=new THREE.MeshStandardMaterial({color:0xc6a46b,roughness:.95,side:THREE.DoubleSide});
  const add=(mesh)=>{mesh.userData.cameraClass=5;group.add(mesh);return mesh;};
  for(const t of targets){const texture=labelTexture?.(t);const label=texture?new THREE.MeshStandardMaterial({map:texture,side:THREE.DoubleSide}):white;const mesh=add(new THREE.Mesh(bucket,[white,green,label]));mesh.position.copy(ned(t.baseNED));mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0,1,0),ned(t.axisNED));mesh.userData.nistTarget=t.id;}
  for(const station of [1,2,3]){
    const post=add(new THREE.Mesh(new THREE.BoxGeometry(.1,.15,.1),wood));post.position.set(0,.075,-station*3);
    // Short mounting cleats join our tapered-bucket variant to the post.
    for(let i=0;i<4;i++){
      const back=.185-.003*Math.SQRT1_2, height=.13-.003*Math.SQRT1_2, half=.015*Math.SQRT1_2;
      const shape=new THREE.Shape();shape.moveTo(.04,.085);shape.lineTo(.04,.115);shape.lineTo(back-half,height+half);shape.lineTo(back+half,height-half);shape.closePath();
      const g=new THREE.ExtrudeGeometry(shape,{depth:.03,bevelEnabled:false});g.translate(0,0,-.015);g.rotateY(i*Math.PI/2);
      const cleat=add(new THREE.Mesh(g,wood));cleat.name='NIST mounting cleat';cleat.position.z=-station*3;
    }
    // A 30cm plank with 5cm thickness, 10cm width and a 45-degree inner end cut.
    const angle = Math.asin(.15 / Math.hypot(.25,.05)) - Math.atan2(.05,.25);
    for(let i=0;i<4;i++){
      const shape=new THREE.Shape();shape.moveTo(0,0);shape.lineTo(.05,.05);shape.lineTo(.30,.05);shape.lineTo(.30,0);shape.closePath();
      const g=new THREE.ExtrudeGeometry(shape,{depth:.10,bevelEnabled:false});
      g.translate(0,0,-.05);g.rotateZ(-angle);g.translate(.04,.30*Math.sin(angle),0);g.rotateY(i*Math.PI/2);
      const leg=add(new THREE.Mesh(g,wood));leg.position.z=-station*3;
    }
  }
  group.updateMatrixWorld(true);return group;
}

let collider;
export function createNistLaneCollider() {
  if(collider)return collider;
  const group=createNistLaneGroup(), tree=new Octree();
  // Octree rays cull backsides: insert both windings of the exact rendered triangles.
  group.traverse(mesh=>{if(!mesh.isMesh)return;const g=mesh.geometry.index?mesh.geometry.toNonIndexed():mesh.geometry;const p=g.getAttribute('position');for(let i=0;i<p.count;i+=3){const a=new THREE.Vector3().fromBufferAttribute(p,i).applyMatrix4(mesh.matrixWorld),b=new THREE.Vector3().fromBufferAttribute(p,i+1).applyMatrix4(mesh.matrixWorld),c=new THREE.Vector3().fromBufferAttribute(p,i+2).applyMatrix4(mesh.matrixWorld);if(new THREE.Triangle(a,b,c).getArea()<1e-14)continue;tree.addTriangle(new THREE.Triangle(a,b,c));tree.addTriangle(new THREE.Triangle(c,b,a));}});tree.build();
  const closest=new THREE.Vector3(), plane=new THREE.Plane(), crossing=new THREE.Vector3();
  function segmentTriangleDistanceSquared(start,end,triangle){
    const line=new THREE.Line3(start,end);triangle.getPlane(plane);
    if(plane.intersectLine(line,crossing)&&triangle.containsPoint(crossing))return 0;
    let best=Math.min(triangle.closestPointToPoint(start,closest).distanceToSquared(start),triangle.closestPointToPoint(end,closest).distanceToSquared(end));
    // Closest segment pair, including parallel and zero-length segments.
    for(const [a,b] of [[triangle.a,triangle.b],[triangle.b,triangle.c],[triangle.c,triangle.a]]){
      const u=end.clone().sub(start),v=b.clone().sub(a),w=start.clone().sub(a),A=u.dot(u),B=u.dot(v),C=v.dot(v),D=u.dot(w),E=v.dot(w),den=A*C-B*B;
      let s=den>1e-20?THREE.MathUtils.clamp((B*E-C*D)/den,0,1):0;
      let t=C?THREE.MathUtils.clamp((B*s+E)/C,0,1):0;
      s=A?THREE.MathUtils.clamp((B*t-D)/A,0,1):0;
      t=C?THREE.MathUtils.clamp((B*s+E)/C,0,1):0;
      best=Math.min(best,start.clone().addScaledVector(u,s).distanceToSquared(a.clone().addScaledVector(v,t)));
    }return best;
  }
  // r187 triangleCapsuleIntersect is one-sided and degenerates at zero length; keep its Octree/Capsule broadphase with exact distances.
  function intersects(a,b,radius){const capsule=new Capsule(a,b,radius+1e-9),triangles=[];tree.getCapsuleTriangles(capsule,triangles);return triangles.some(t=>segmentTriangleDistanceSquared(a,b,t)<=(radius+1e-9)**2);}
  collider={
    ray(origin,direction,maxDistance=Infinity){const result=tree.rayIntersect(new THREE.Ray(ned(origin),ned(direction)));return result&&result.distance<=maxDistance?result.distance:null;},
    overlaps(position,radius){const p=ned(position);return intersects(p,p,radius);},
    sweep(start,end,radius){const a=ned(start),b=ned(end);if(intersects(a,a,radius))return 0;if(!intersects(a,b,radius))return null;let low=0,high=1;for(let i=0;i<26;i++){const mid=(low+high)/2;if(intersects(a,a.clone().lerp(b,mid),radius))high=mid;else low=mid;}return high;}
  };return collider;
}
