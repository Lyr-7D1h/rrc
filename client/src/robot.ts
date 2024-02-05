import { GUI } from "dat.gui";
import {
  DoubleSide,
  Mesh,
  MeshBasicMaterial,
  Object3D,
  PlaneGeometry,
} from "three";
import { Link } from "./link";

import { pivot, posvec3, quat, vec3 } from "./geometry";
import { Joint, RotationalJoint, SlidingJoint, StaticJoint } from "./joint";
import { Connection } from "./connection";

export type State = number[];

export class RobotGUI {
  private folder: GUI;
  private real: State;
  /** allows for modifying values using the gui without any real changes happening directly */
  private virtual: State;

  constructor(gui: GUI, robot: Robot) {
    this.folder = gui.addFolder("Robot");
    const jointFolder = this.folder.addFolder("joints");

    this.real = robot.joints
      .map((j) => {
        if (j instanceof RotationalJoint || j instanceof SlidingJoint) {
          return j.value;
        }
        return null;
      })
      .filter((v) => v !== null) as number[];
    // deep copy
    this.virtual = Array.from(this.real);

    let index = 0;
    for (const joint of robot.joints) {
      if (joint instanceof RotationalJoint) {
        let name = joint.name.length > 0 ? joint.name : `rot joint ${index}`;
        let j = index;
        jointFolder
          .add(this.virtual, j, joint.min, joint.max)
          .name(name)
          .listen()
          .onFinishChange(() => {
            robot.move(this.virtual);
            this.virtual[j] = this.real[j];
          });
        index += 1;
      }
      if (joint instanceof SlidingJoint) {
        let name =
          joint.name.length > 0 ? joint.name : `sliding joint ${index}`;
        let j = index;
        jointFolder
          .add(this.virtual, index, joint.min, joint.max)
          .name(name)
          .listen()
          .onFinishChange(() => {
            robot.move(this.virtual);
            this.virtual[j] = this.real[j];
          });
        index += 1;
      }
    }
    jointFolder.open();

    const debugFolder = this.folder.addFolder("debug");
    debugFolder
      .add({ d: true }, "d")
      .name("Wireframe")
      .listen()
      .onChange((b) => {
        robot.links.forEach((l) => l.wireframe(b));
      });
    debugFolder
      .add({ d: true }, "d")
      .name("Show Joints")
      .listen()
      .onChange(() => {
        robot.joints.forEach((j) => j.togglePoint());
      });
    debugFolder
      .add({ d: false }, "d")
      .name("Show Positions")
      .listen()
      .onChange(() => {
        robot.links.forEach((j) => j.toggleMidpoint());
      });
    debugFolder.open();
  }

  update(state: State) {
    console.log(state)
    this.real = state;
    this.virtual = Array.from(state);
  }

  gui(): GUI {
    return this.folder;
  }
}

export class Robot extends Object3D {
  /** simple point without geometry at origin */
  private connection: Connection;
  private base: Link;
  links: Link[];
  joints: Joint[];
  private gui: RobotGUI | null;

  constructor(connection: Connection) {
    super();

    this.connection = connection;
    this.joints = [];
    this.links = [];
    this.gui = null;

    // debug floor
    const material = new MeshBasicMaterial({
      color: 0xffff00,
      side: DoubleSide,
      wireframe: true,
    });
    this.add(new Mesh(new PlaneGeometry(300, 300), material));

    // first link is just a point in space, this allows for starting with a joint
    this.base = new Link();
    this.add(this.base);
    this.addLink(this.base);

    this.default();
  }

  /** build the default robot */
  default() {
    const width = 150;
    const lift = new Link(width, width, 2000);
    this.addLink(lift);
    this.addJoint(
      new RotationalJoint(
        "swing",
        this.base,
        lift,
        posvec3(),
        pivot(vec3(0, 0, -1000)),
      ),
    );

    const arm = new Link(width, width, 700);
    this.addLink(arm);
    // pos = new PositionalVector(new Vector3(-width/2, 0, 1000 - width / 2), new Vector3(1, 0, 0));
    let jointPos = posvec3(
      vec3(0, -width / 2, 1000 - width / 2),
      vec3(1, 0, 0),
    );
    let attachmentPivot = pivot(vec3(0, 0, -(700 / 2)));
    this.addJoint(
      new SlidingJoint("lift", lift, arm, jointPos, attachmentPivot)
        .setAxis(vec3(0, -1, 0))
        .setContraints(0, 1500),
    );

    const lowerArm = new Link(width, width, 750);
    this.addLink(lowerArm);
    jointPos = posvec3(vec3(0, -width / 2, 750 / 2 - width / 2), vec3(1, 0, 0));
    attachmentPivot = pivot(
      vec3(0, width / 2, -750 / 2 + width / 2),
      quat(-1, 0, 0),
    );
    this.addJoint(
      new RotationalJoint(
        "elbow",
        arm,
        lowerArm,
        jointPos,
        attachmentPivot,
      ).setContraints(-Math.PI * 0.8, Math.PI * 0.8),
    );

    const extension = new Link(width, width, 300);
    this.addLink(extension);
    this.addJoint(
      new RotationalJoint(
        "wrist",
        lowerArm,
        extension,
        posvec3(vec3(0, -width / 2, 750 / 2 - width / 2), vec3(1, 0, 0)),
        pivot(vec3(0, 0, -300 / 2)),
      ),
    );

    const extensionExtension = new Link(width, width, 50);
    this.addLink(extensionExtension);
    this.addJoint(
      new StaticJoint(
        "",
        extension,
        extensionExtension,
        posvec3(vec3(0, width / 2, 0), vec3(1, 0, 0)),
        pivot(vec3(0, width / 2, 0), quat(1, 0, 0)),
      ),
    );

    const gripper = new Link(50, width, width - 25);
    this.addLink(gripper);
    this.addJoint(
      new SlidingJoint(
        "gripper",
        extensionExtension,
        gripper,
        posvec3(vec3(0, -(width / 2 - 50 / 2), -50 / 2), vec3(0, 0, -1)),
        pivot(vec3(0, 0, 62.5)),
      )
        .setAxis(vec3(0, 1, 0))
        .setContraints(0, 100),
    );
  }

  addLink(link: Link) {
    link.name = this.links.length.toString();
    this.links.push(link);
  }

  addJoint(joint: Joint) {
    this.joints.push(joint);
  }

  move(state: State) {
    this.connection.send({
      type: "move",
      state,
    });
  }

  /** update from state */
  update(state: State) {
    for (const i in state) {
      let value = state[i];
      this.joints[i].update(value);
    }
    if (this.gui) {
      this.gui.update(state);
    }
  }

  buildGUI(gui: GUI): GUI {
    if (this.gui === null) {
      this.gui = new RobotGUI(gui, this);
    }
    return this.gui.gui();
  }
}
