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
  constructor(geometry: BufferGeometry, material?: MeshBasicMaterial) {
    let mat = material ? material : new MeshBasicMaterial({
      color: 0x888888,
    });
    mat.wireframe = true;
    super(geometry, mat);
    this.basicMaterial = mat;
  }

  wireframe(value: boolean) {
    this.basicMaterial.wireframe = value;
  }

  toggleMidpoint() {
    if (this.midpoint) {
      this.remove(this.midpoint);
      this.midpoint = undefined;
      return;
    }
    const geometry = new BufferGeometry();
    geometry.setAttribute(
      "position",
      new BufferAttribute(new Float32Array([0, 0, 0]), 3),
    );
    const dotMaterial = new PointsMaterial({ size: 50, color: 0x00ffff });
    this.midpoint = new Points(geometry, dotMaterial);
    this.add(this.midpoint);
  }
}

abstract class Joint extends Object3D {
  base: Mesh;
  attachment: Mesh;

  private dot?: Points;
  /** Join two 3d objects together with a joint using local space coordinates */
  constructor(
    name: string,
    base: Mesh,
    attachment: Mesh,
    /** position and direction in the local space of `base` */
    jointPos?: PositionalVector,
    /** position, direction and orientation in the local space of `attachment` */
    pivot?: Pivot,
  ) {
    super();

    jointPos = jointPos || new PositionalVector();
    pivot = pivot || new Pivot();

    // go to local position of base and put joint there
    jointPos.moveObject(this);
    base.add(this);

    // translate attachment
    pivot.point.negate()
    pivot.updateObject(attachment);
    this.add(attachment);

    this.togglePoint();

    this.name = name;
    this.base = base;
    this.attachment = attachment;
  }

  togglePoint() {
    if (this.dot) {
      this.remove(this.dot)
      this.dot = undefined
      return
    };
    const geometry = new BufferGeometry();
    geometry.setAttribute(
      "position",
      new BufferAttribute(new Float32Array([0, 0, 0]), 3),
    );
    const dotMaterial = new PointsMaterial({ size: 50, color: 0xffff00 });
    this.dot = new Points(geometry, dotMaterial);
    this.add(this.dot);
  }

  abstract contraints(): [number,number] | null

  abstract setContraints(min: number, max:number): void

  abstract update(value: number): void
}
class SlidingJoint extends Joint {
  override type: string = "sliding";
  update(value:number) {}
  contraints(): [number, number] | null {
      throw new Error("Method not implemented.");
  }
  setContraints(min: number, max: number): void {
      throw new Error("Method not implemented.");
  }
}
class RotationalJoint extends Joint {
  override type: string = "rotational";
  update(value:number) {}
  contraints(): [number, number] | null {
      throw new Error("Method not implemented.");
  }
  setContraints(min: number, max: number): void {
      throw new Error("Method not implemented.");
  }
}
class StaticJoint extends Joint {
  override type: string = "static";
  update(value:number) {}
  contraints(): [number, number] | null {
    return null
  }
  override setContraints(): void {}
}


class Robot extends Object3D {
  /** simple point without geometry at origin */
  private base: Link;
  private links: Link[];
  private joints: Joint[];

  constructor() {
    super();

    this.joints = [];
    this.links = [];

    // debug floor
    const material = new MeshBasicMaterial({
      color: 0xffff00,
      side: DoubleSide,
      wireframe: true
    });
    this.add(new Mesh(new PlaneGeometry(300, 300), material));

    this.base = new Link(new BufferGeometry())
    this.add(this.base)
    this.addLink(this.base)

    this.default()
  }
  
  default() {
    const width = 150;
    const lift = new Link(new BoxGeometry(width, width, 2000));
    this.addLink(lift)
    this.addJoint(new RotationalJoint(
      "swing",
      this.base,
      lift,
      posvec3(),
      pivot(vec3(0, 0, -1000)),
    ));

    const arm = new Link(new BoxGeometry(width, width, 700));
    this.addLink(arm)
    // pos = new PositionalVector(new Vector3(-width/2, 0, 1000 - width / 2), new Vector3(1, 0, 0));
    let jointPos = posvec3(
      vec3(0, -width / 2, 1000 - width / 2),
      vec3(1, 0, 0),
    );
    let attachmentPivot = pivot(vec3(0, 0, - (700 / 2)));
    this.addJoint(new SlidingJoint("lift", lift, arm, jointPos, attachmentPivot));
    
    const lowerArm = new Link(new BoxGeometry(width, width, 750));
    this.addLink(lowerArm)
    jointPos = posvec3(vec3(0, -width / 2, 750 / 2 - width / 2), vec3(1, 0, 0));
    attachmentPivot = pivot(
      vec3(0, width / 2, - 750 / 2 + width / 2),
      quat(-1, 0, 0),
    );
    this.addJoint(new RotationalJoint("elbow", arm, lowerArm, jointPos, attachmentPivot));
    
    const extension = new Link(new BoxGeometry(width, width, 300));
    this.addLink(extension)
    this.addJoint(new RotationalJoint(
      "wrist",
      lowerArm,
      extension,
      posvec3(vec3(0, -width / 2, 750 / 2 - width / 2), vec3(1, 0, 0)),
      pivot(vec3(0, 0, -300 / 2)),
    ));

    const extensionExtension = new Link(new BoxGeometry(width, width, 50));
    this.addLink(extensionExtension)
    this.addJoint(new StaticJoint(
      "",
      extension,
      extensionExtension,
      posvec3(vec3(0, width / 2, 0), vec3(1, 0, 0)),
      pivot(vec3(0, width / 2, 0), quat(1, 0, 0)),
    ));


    const gripper = new Link(new BoxGeometry(50, width, width-25));
    this.addLink(gripper)
    this.addJoint( new SlidingJoint(
      "gripper",
      extensionExtension,
      gripper,
      posvec3(vec3(0, -(width / 2 - 50 / 2), -50 / 2), vec3(0, 0, -1)),
      pivot(vec3(0, 0, 62.5)),
    ));
  }

  addLink(link: Link) { 
    link.name = this.links.length.toString()
    this.links.push(link)
  }

  addJoint(joint: Joint) {
    this.joints.push(joint)
  }

  specs() {
    return {
      // shape only
      links: this.links.map((link) => ({
        vertices: link.geometry.getAttribute("position") ? Array.from(link.geometry.getAttribute("position").array): [],
        indices: link.geometry.index ? Array.from(link.geometry.index.array) : []
      })),
      joints: this.joints.map((joint) => ({
        type: joint.type ,
        link1: {
          index: Number(joint.base.name),
          position: Array.from(joint.position),
          direction: Array.from(vec3(0,0,1).applyQuaternion(joint.quaternion))
        },
        link2: {
          index: Number(joint.attachment.name),
          position: Array.from(joint.attachment.position),
          quaternion: Array.from(joint.attachment.quaternion)
        }
      }))
    }
  }

  updateJoint(index: number, value: number) {
    this.joints[index]?.update(value)
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
      .onChange(() => {
        this.joints.forEach((j) => j.togglePoint());
      });
    debugFolder
      .add({ d: false }, "d")
      .name("Show Positions")
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
    this.connection.send({ type: "init", specs: this.robot.specs() });

    this.connection.on("message", (data) => {
      console.log(data)
    })

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
