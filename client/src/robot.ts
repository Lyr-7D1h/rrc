import { GUI } from "dat.gui";
import {
  DoubleSide,
  Mesh,
  MeshBasicMaterial,
  Object3D,
  PlaneGeometry,
} from "three";
import { Link } from "./link";

import { transform, quat, vec3 } from "./geometry";
import { Joint, RotationalJoint, SlidingJoint, StaticJoint } from "./joint";
import { Connection } from "./connection";

/** The current state of the joints, always in degrees and mm's */
export type State = number[];

export class RobotGUI {
  private folder: GUI;
  private real: State;
  /** allows for modifying values using the gui without any real changes happening directly */
  private virtual: State;

  constructor(gui: GUI, robot: Robot) {
    this.folder = gui.addFolder("Robot");
    const jointFolder = this.folder.addFolder("joints");

    this.real = robot.state;
    // deep copy
    this.virtual = Array.from(this.real);

    let index = 0;
    for (let i = 0; i < robot.joints.length; i += 1) {
      const joint = robot.joints[i];
      if (joint instanceof RotationalJoint) {
        const name = joint.name.length > 0 ? joint.name : `rot joint ${i}`;
        const j = index;
        jointFolder
          .add(this.virtual, j, joint.min, joint.max)
          .name(name)
          .listen()
          .onChange(() => {
            robot.move(this.virtual);
          })
          .onFinishChange(() => {
            this.virtual[j] = this.real[j];
          });
        index += 1;
      }
      if (joint instanceof SlidingJoint) {
        const name = joint.name.length > 0 ? joint.name : `sliding joint ${i}`;
        const j = index;
        jointFolder
          .add(this.virtual, j, joint.min, joint.max)
          .name(name)
          .listen()
          .onChange(() => {
            robot.move(this.virtual);
          })
          .onFinishChange(() => {
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
    if (state.length == this.real.length && state[0] !== null) {
      this.real = state;
      for (const i in state) {
        this.virtual[i] = state[i];
      }
    }
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
  state: State;

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
    // initialize state as zero
    this.state = this.joints
      .map((j) => {
        if (j instanceof SlidingJoint || j instanceof RotationalJoint) {
          return 0;
        }
        return null;
      })
      .filter((j) => j !== null) as State;
  }

  /** build a default robot */
  default() {
    const width = 150;
    const lift = new Link(width, width, 2000);
    this.addLink(lift);
    this.addJoint(
      new RotationalJoint(
        "swing",
        this.base,
        lift,
        transform(),
        transform(vec3(0, 0, 1000)),
      ),
    );

    const arm = new Link(width, width, 700);
    this.addLink(arm);
    this.addJoint(
      new SlidingJoint(
        "lift",
        lift,
        arm,
        transform(vec3(0, width / 2, 1000 - width / 2), quat(-1, 0, 0)),
        transform(vec3(0, 0, 700 / 2)),
      )
        .setAxis(vec3(0, 1, 0))
        .setContraints(0, 1500),
    );

    const lowerArm = new Link(width, width, 750);
    this.addLink(lowerArm);
    this.addJoint(
      new RotationalJoint(
        "elbow",
        arm,
        lowerArm,
        transform(vec3(0, width / 2, 750 / 2 - width / 2), quat(-1, 0, 0)),
        transform(vec3(0, -750 / 2 + width / 2, width / 2), quat(1, 0, 0)),
      ).setContraints(-150, 150),
    );

    const extension = new Link(width, width, 300);
    this.addLink(extension);
    this.addJoint(
      new RotationalJoint(
        "wrist",
        lowerArm,
        extension,
        transform(vec3(0, width / 2, 750 / 2 - width / 2), quat(-1, 0, 0)),
        transform(vec3(0, 0, 300 / 2)),
      ),
    );

    const extensionExtension = new Link(width, 50, width);
    this.addLink(extensionExtension);
    this.addJoint(
      new StaticJoint(
        "",
        extension,
        extensionExtension,
        transform(vec3(0, -width / 2, 0), quat(1, 0, 0)),
        transform(vec3(0, 0, width / 2)),
      ),
    );

    const gripper = new Link(width, 50, width - 25);
    this.addLink(gripper);
    this.addJoint(
      new SlidingJoint(
        "gripper",
        extensionExtension,
        gripper,
        transform(vec3(0, 50 / 2, width / 2 - 50 / 2), quat(-1, 0, 0)),
        transform(vec3(0, 0, 62.5)),
      )
        .setAxis(vec3(0, 1, 0))
        .setContraints(0, 100),
    );
  }

  limits() {
    return this.joints
      .map((j, i) => {
        if (j instanceof SlidingJoint || j instanceof RotationalJoint) {
          return {
            index: i,
            acceleration: j.maxAcceleration,
            velocity: j.maxVelocity,
            min: j.min,
            max: j.max,
          };
        }
        return null;
      })
      .filter((l) => l !== null);
  }

  addLink(link: Link) {
    link.name = this.links.length.toString();
    this.links.push(link);
  }

  addJoint(joint: Joint) {
    this.joints.push(joint);
  }

  move(state: State) {
    console.log(state)
    this.connection.send({
      type: "move",
      state,
    });
  }

  /** update from state */
  update(state: State) {
    if (this.gui) {
      this.gui.update(state);
    }

    // ignore static joints
    let i = 0;
    for (const j of this.joints) {
      if (j instanceof SlidingJoint || j instanceof RotationalJoint) {
        j.update(state[i] as number);
        i += 1;
      }
    }
    this.state = state;
  }

  buildGUI(gui: GUI): GUI {
    if (this.gui === null) {
      this.gui = new RobotGUI(gui, this);
    }
    return this.gui.gui();
  }
}
