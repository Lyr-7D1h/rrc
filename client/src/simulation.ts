import { GUI } from "dat.gui";
import { OrbitControls } from "@three-ts/orbit-controls";
import { VertexNormalsHelper } from "three/addons/helpers/VertexNormalsHelper.js";
import {
  AxesHelper,
  BoxGeometry,
  BufferAttribute,
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
  PlaneGeometry,
  Points,
  PointsMaterial,
  Quaternion,
  Scene,
  Shape,
  ShapeGeometry,
  Vector3,
  WebGLRenderer,
} from "three";
import Stats from "three/examples/jsm/libs/stats.module";
import { Connection } from "./connection";
import {
  Pivot,
  PositionalVector,
  pivot,
  posvec3,
  quat,
  vec3,
} from "./geometry";

/** https://en.wikipedia.org/wiki/Linkage_(mechanical) */
class Link extends Mesh {
  private basicMaterial: MeshBasicMaterial;
  private midpoint?: Points;
  constructor(geometry: BufferGeometry) {
    const material = new MeshBasicMaterial({
      color: 0x888888,
      wireframe: true,
    });
    super(geometry, material);
    this.basicMaterial = material;
  }

  wireframe(value: boolean) {
    this.basicMaterial.wireframe = value;
  }

  toggleMidpoint() {
    if (this.midpoint) {
      this.remove(this.midpoint);
      return;
    }
    const geometry = new BufferGeometry();
    geometry.setAttribute(
      "position",
      new BufferAttribute(new Float32Array([0, 0, 0]), 3),
    );
    const dotMaterial = new PointsMaterial({ size: 50, color: 0x00ffff });
    const dot = new Points(geometry, dotMaterial);
    this.add(dot);
  }
}

class Joint extends Object3D {
  private dot?: Points;
  /** Join two 3d objects together with a joint using a relative `jointPos` on `base` with attachment oriented using `pivot` */
  constructor(
    base: Mesh,
    attachment: Mesh,
    jointPos?: PositionalVector,
    pivot?: Pivot,
  ) {
    super();

    jointPos = jointPos || new PositionalVector();
    pivot = pivot || new Pivot();

    // add joint
    jointPos.updateObject(this);
    base.add(this);

    // add attachment to joints
    pivot.updateObject(attachment);
    this.add(attachment);

    this.show();
  }

  show() {
    if (this.dot) return;
    const geometry = new BufferGeometry();
    geometry.setAttribute(
      "position",
      new BufferAttribute(new Float32Array([0, 0, 0]), 3),
    );
    const dotMaterial = new PointsMaterial({ size: 50, color: 0xffff00 });
    const dot = new Points(geometry, dotMaterial);
    this.add(dot);
    this.dot = dot;
  }
  hide() {
    if (this.dot) {
      this.remove(this.dot);
    }
  }
}
class SlidingJoint extends Joint {}
class RotationalJoint extends Joint {}
class StaticJoint extends Joint {}

class Robot extends Object3D {
  private links: Link[];
  private joints: (SlidingJoint | RotationalJoint)[];

  constructor() {
    super();
    const material = new MeshBasicMaterial({
      color: 0xffff00,
      side: DoubleSide,
      wireframe: true,
    });
    const base = new Mesh(new PlaneGeometry(200, 200), material);
    this.add(base);
    // is a static base
    // base.matrixAutoUpdate = false;

    this.joints = [];
    this.links = [];

    const width = 150;
    const lift = new Link(new BoxGeometry(width, width, 2000));
    let joint = new RotationalJoint(
      base,
      lift,
      posvec3(),
      pivot(vec3(0, 0, 1000)),
    );

    this.joints.push(joint);
    this.links.push(lift);

    const arm = new Link(new BoxGeometry(width, width, 700));
    // pos = new PositionalVector(new Vector3(-width/2, 0, 1000 - width / 2), new Vector3(1, 0, 0));
    let jointPos = posvec3(
      vec3(0, -width / 2, 1000 - width / 2),
      vec3(1, 0, 0),
    );
    let attachmentPivot = pivot(vec3(0, 0, 700 / 2));
    joint = new SlidingJoint(lift, arm, jointPos, attachmentPivot);

    this.joints.push(joint);
    this.links.push(arm);

    const lowerArm = new Link(new BoxGeometry(width, width, 750));
    jointPos = posvec3(vec3(0, -width / 2, 750 / 2 - width / 2), vec3(1, 0, 0));
    attachmentPivot = pivot(
      vec3(0, -width / 2, 750 / 2 - width / 2),
      quat(-1, 0, 0),
    );
    joint = new RotationalJoint(arm, lowerArm, jointPos, attachmentPivot);

    this.joints.push(joint);
    this.links.push(lowerArm);

    const geometry = new BoxGeometry(width, width, 300);
    const extension = new Link(geometry);
    joint = new RotationalJoint(
      lowerArm,
      extension,
      posvec3(vec3(0, -width / 2, 750 / 2 - width / 2), vec3(1, 0, 0)),
      pivot(vec3(0, 0, 300 / 2)),
    );

    this.joints.push(joint);
    this.links.push(extension);

    const extensionExtension = new Link(new BoxGeometry(width, width, 50));
    joint = new StaticJoint(
      extension,
      extensionExtension,
      posvec3(vec3(0, width / 2, 0), vec3(1, 0, 0)),
      pivot(vec3(0, -width / 2, 0), quat(1, 0, 0)),
    );

    this.joints.push(joint);
    this.links.push(extensionExtension);

    const gripper = new Link(new BoxGeometry(50, width, width));
    joint = new SlidingJoint(
      extensionExtension,
      gripper,
      posvec3(vec3(0, -(width / 2 - 50 / 2), -50 / 2), vec3(0, 0, -1)),
      pivot(vec3(0, 0, -50 / 2 - 50)),
    );

    this.joints.push(joint);
    this.links.push(gripper);
  }

  loadFromSpec(spec: Object) {
    console.log(spec);
    this.updateMatrix();
  }

  buildGUI(gui: GUI): GUI {
    const folder = gui.addFolder("Robot");
    const jointFolder = folder.addFolder("joints");
    for (const i in this.joints) {
      const joint = this.joints[i];
      if (joint instanceof RotationalJoint) {
        jointFolder
          .add(joint.rotation, "z", -Math.PI * 2, Math.PI * 2)
          .name(`rot joint ${i}`);
      }
      if (joint instanceof SlidingJoint) {
        jointFolder
          .add(joint.position, "z", -4.5, 4.5)
          .name(`sliding joint ${i}`);
      }
    }
    jointFolder.open();

    const debugFolder = folder.addFolder("debug");
    debugFolder
      .add({ d: true }, "d")
      .name("Wireframe")
      .listen()
      .onChange((b) => {
        this.links.forEach((l) => l.wireframe(b));
      });
    debugFolder
      .add({ d: true }, "d")
      .name("Show Joints")
      .listen()
      .onChange((b) => {
        if (b) {
          this.joints.forEach((j) => j.show());
        } else {
          this.joints.forEach((j) => j.hide());
        }
      });
    debugFolder
      .add({ d: false }, "d")
      .name("Show Midpoints")
      .listen()
      .onChange(() => {
        this.links.forEach((j) => j.toggleMidpoint());
      });
    debugFolder.open();

    return folder;
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
  private connection: Connection;

  constructor(connection: Connection) {
    if (connection.connected() === false) throw Error("not connected");

    this.connection = connection;

    this.scene = new Scene();

    this.camera = new PerspectiveCamera(
      40,
      window.innerWidth / window.innerHeight,
      0.1,
      200000,
    );
    this.camera.up.set(0, 0, 1);
    this.camera.position.x = 0;
    this.camera.position.y = 4000;
    this.camera.position.z = 4000;

    this.renderer = new WebGLRenderer();
    this.renderer.setSize(window.innerWidth, window.innerHeight);

    // https://threejs.org/docs/#examples/en/controls/OrbitControls
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;

    this.gui = new GUI();

    const origin = new AxesHelper(200);
    this.scene.add(origin);

    this.robot = new Robot();
    this.scene.add(this.robot);
    this.robot.buildGUI(this.gui).open();

    // fps counter
    this.stats = new Stats();
  }

  /** Build the initial state of the simulation */
  async init() {
    const spec = await this.connection.send({ type: "init" });
    this.robot.loadFromSpec(spec);

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
