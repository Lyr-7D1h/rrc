import { GUI } from "dat.gui";
import { OrbitControls } from "@three-ts/orbit-controls";
import { VertexNormalsHelper } from "three/addons/helpers/VertexNormalsHelper.js";
import {
  AxesHelper,
  BoxGeometry,
  BufferGeometry,
  DoubleSide,
  EdgesGeometry,
  Group,
  LineBasicMaterial,
  LineSegments,
  Matrix3,
  Mesh,
  MeshBasicMaterial,
  Object3D,
  PerspectiveCamera,
  Plane,
  PlaneGeometry,
  PlaneHelper,
  Scene,
  Triangle,
  Vector2,
  Vector3,
  WebGLRenderer,
  WireframeGeometry,
} from "three";
import Stats from "three/examples/jsm/libs/stats.module";
import { height } from "./geometry";
import { Connection, connect } from "./connection";

export class PositionalVector {
  point: Vector3;
  normal: Vector3;

  constructor(point?: Vector3, normal?: Vector3) {
    this.point = point || new Vector3(0,0,0);
    this.normal = normal ? normal.normalize() :new Vector3(0,0,1)
  }

  setLookAt(object: Object3D) {
    const o = new Vector3()
    object.getWorldPosition(o)
    o.add(this.normal);
    console.log("Looking at", o);
    object.lookAt(o);
  }
}

/** https://en.wikipedia.org/wiki/Linkage_(mechanical) */
class Link extends Mesh {
  private basic_material: MeshBasicMaterial
  constructor(width: number, height: number, depth: number) {
    const geometry = new BoxGeometry(width, height, depth);
    const material = new MeshBasicMaterial({
      color: 0x888888,
      wireframe: true,
    });
    super(geometry, material);
    this.basic_material = material
  }

  wireframe(value: boolean) {
    this.basic_material.wireframe = value
  }
}

class Joint extends Mesh {
  /** Join two 3d objects together with a joint using `relPos` of `base` */
  joinObjects(base: Mesh, attachment: Mesh, base_pos: PositionalVector, attachment_pos?: PositionalVector) {
    attachment_pos = attachment_pos || new PositionalVector()
    const jointHeight = height(this);
    const attachmentHeight = height(attachment);

    let p = base_pos.point;
    const sign = p.z >= 0 ? 1 : -1;
    // increased by jointHeight in positive or negative direction
    let z = Math.abs(p.z) + jointHeight / 2;
    this.position.set(p.x, p.y, sign * z);
    // add joint 
    base.add(this);
    base_pos.setLookAt(this);

    // add attachment on top of joint
    p = attachment_pos.point;
    z = Math.abs(p.z) + jointHeight / 2 + attachmentHeight / 2;
    attachment.position.set(p.x, p.y, sign * z);
    this.add(attachment);
  }
}
class SlidingJoint extends Joint {}
class RotationalJoint extends Joint {}

const robot = {
  joints: [
  ],
  links: {
    lift: {
      length: 10,
      position: [0,0,0],
      direction: [0,0,1]
    }
  }
}

class Robot extends Object3D {
  private links: Link[]
  private joints: (SlidingJoint | RotationalJoint)[]

  constructor() {
    super();
    const geometry = new PlaneGeometry(2, 2);
    const material = new MeshBasicMaterial({
      color: 0xffff00,
      side: DoubleSide,
      wireframe: true,
    });
    const base = new Mesh(geometry, material);
    this.add(base);
    // is a static base
    // base.matrixAutoUpdate = false;

    this.joints = []
    this.links = []

    const lift = new Link(1, 1, 10);
    let joint = new RotationalJoint(new BoxGeometry(1, 1, 0.5));
    let pos = new PositionalVector(new Vector3(0, 0, 0), new Vector3(0, 0, 1));
    joint.joinObjects(base, lift, pos);
    console.log(joint.position)
    console.log(joint.localToWorld(joint.position.clone()))

    this.joints.push(joint)
    this.links.push(lift)

    const arm = new Link(1, 1, 5);
    joint = new SlidingJoint(new BoxGeometry(1, 1, 1));
    pos = new PositionalVector(new Vector3(0, 0, 4), new Vector3(1, 0, 0));
    joint.joinObjects(lift, arm, pos);

    this.joints.push(joint)
    this.links.push(arm)

    const lower_arm = new Link(2.5, 1, 1);
    joint = new RotationalJoint(new BoxGeometry(1, 1, 0.5));
    pos = new PositionalVector(new Vector3(0.75, 0, 1.75), new Vector3(0, 0, -1));
    joint.joinObjects(arm, lower_arm, pos, new PositionalVector(new Vector3(0.75,0,0)));

    this.joints.push(joint)
    this.links.push(lower_arm)
  }

  buildGUI(gui: GUI): GUI {
    const folder = gui.addFolder('Robot')
    const jointFolder = folder.addFolder('joints')
    for (const i in this.joints) {
      const joint = this.joints[i]
      if (joint instanceof RotationalJoint) {
	jointFolder.add(joint.rotation, "z", -Math.PI*2, Math.PI * 2).name(`rot joint ${i}`)
      }
      if (joint instanceof SlidingJoint) {
	jointFolder.add(joint.position, "z", -4.5, 4.5).name(`sliding joint ${i}`)
      }
    }
    jointFolder.open()

    const debugFolder = folder.addFolder("debug")
    debugFolder.add({d: true}, "d").name("Wireframe").listen().onChange((b) => {
      this.links.forEach((l) => l.wireframe(b))
    })
    debugFolder.open()

    return folder
  }
}

export class Simulation {
  private gui: GUI;
  private stats: Stats;
  private scene: Scene;
  private camera: PerspectiveCamera;
  private renderer: WebGLRenderer;
  private robot: Robot;
  private controls: OrbitControls;

  constructor() {
    const socket = connect("ws://localhost:6543")

    this.scene = new Scene();

    this.camera = new PerspectiveCamera(
      75,
      window.innerWidth / window.innerHeight,
      0.1,
      2000,
    );
    this.camera.up.set(0, 0, 1);
    this.camera.position.x = 10;
    this.camera.position.y = 10;
    this.camera.position.z = 10;

    this.renderer = new WebGLRenderer();
    this.renderer.setSize(window.innerWidth, window.innerHeight);

    // https://threejs.org/docs/#examples/en/controls/OrbitControls
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    // TODO wasd controls

    this.gui = new GUI();

    const origin = new AxesHelper(2);
    this.scene.add(origin);

    this.robot = new Robot();
    this.scene.add(this.robot);
    this.robot.buildGUI(this.gui).open()

    // fps counter
    this.stats = new Stats();
  }

  show() {
    document.body.appendChild(this.stats.dom);
    document.body.appendChild(this.renderer.domElement);
  }

  draw() {
    window.requestAnimationFrame(() => this.draw());
    this.stats.update();
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }
}
