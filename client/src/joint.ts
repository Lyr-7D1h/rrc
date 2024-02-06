import {
  AxesHelper,
  BufferAttribute,
  BufferGeometry,
  Mesh,
  Object3D,
  Points,
  PointsMaterial,
  Vector3,
} from "three";

import { Transform, PositionalVector, vec3 } from "./geometry";

export abstract class Joint extends Object3D {
  base: Mesh;
  attachment: Mesh;
  jointPos: Transform;
  pivot: Transform;

  private dot?: Points;
  /** Join two 3d objects together with a joint using local space coordinates */
  constructor(
    name: string,
    base: Mesh,
    attachment: Mesh,
    /** position and direction in the local space of `base` */
    jointPos?: Transform,
    /** position, direction and orientation in the local space of `attachment` */
    pivot?: Transform,
  ) {
    super();

    jointPos = jointPos || new Transform();
    // jointPos = jointPos || new PositionalVector();
    pivot = pivot || new Transform();

    // go to local position of base and put joint there
    jointPos.updateObject(this);
    // jointPos.moveObject(this);
    base.add(this);

    // translate attachment
    // pivot.point.negate();
    pivot.updateObject(attachment);
    this.add(attachment);

    this.togglePoint();

    this.name = name;
    this.base = base;
    this.attachment = attachment;
    this.jointPos = jointPos;
    this.pivot = pivot;
  }

  togglePoint() {
    if (this.dot) {
      this.remove(this.dot);
      this.dot = undefined;
      return;
    }
    const geometry = new BufferGeometry();
    geometry.setAttribute(
      "position",
      new BufferAttribute(new Float32Array([0, 0, 0]), 3),
    );
    const dotMaterial = new PointsMaterial({ size: 50, color: 0xffff00 });
    this.dot = new Points(geometry, dotMaterial);
    this.dot.add(new AxesHelper(100));
    this.add(this.dot);
  }

  abstract update(value: number): void;
}
export class SlidingJoint extends Joint {
  override type: string = "sliding";
  origin: Vector3;
  oldValue: number;
  value: number;
  min: number;
  max: number;
  maxVelocity: number;
  axis: Vector3;
  constructor(
    name: string,
    base: Mesh,
    attachment: Mesh,
    /** position and direction in the local space of `base` */
    jointPos?: Transform,
    /** position, direction and orientation in the local space of `attachment` */
    pivot?: Transform,
  ) {
    super(name, base, attachment, jointPos, pivot);
    this.origin = this.position.clone();
    this.oldValue = 0;
    this.value = 0;
    this.axis = vec3(0, 0, 1).applyQuaternion(this.quaternion);
    this.min = -100;
    this.max = 100;
    this.maxVelocity = 100;
  }
  setAxis(axis: Vector3): SlidingJoint {
    this.axis = axis;
    return this;
  }
  update(value: number) {
    this.translateOnAxis(this.axis, value - this.oldValue);
    this.oldValue = value;
  }
  contraints(): [number, number] | null {
    return [this.min, this.max];
  }
  setContraints(min: number, max: number): SlidingJoint {
    this.min = min;
    this.max = max;
    return this;
  }
}
export class RotationalJoint extends Joint {
  override type: string = "rotational";
  oldValue: number;
  value: number;
  min: number;
  max: number;
  maxVelocity: number;
  axis: Vector3;
  constructor(
    name: string,
    base: Mesh,
    attachment: Mesh,
    /** position and direction in the local space of `base` */
    jointPos?: Transform,
    /** position, direction and orientation in the local space of `attachment` */
    pivot?: Transform,
  ) {
    super(name, base, attachment, jointPos, pivot);
    this.oldValue = 0;
    this.value = 0;
    this.axis = vec3(0, 0, 1);
    this.min = -Math.PI;
    this.max = Math.PI;
    this.maxVelocity = 0.2;
  }
  setAxis(axis: Vector3): RotationalJoint {
    this.axis = axis;
    return this;
  }
  update(value: number) {
    this.rotateOnAxis(this.axis, value - this.oldValue);
    this.value = value;
  }
  contraints(): [number, number] | null {
    return [this.min, this.max];
  }
  setContraints(min: number, max: number): RotationalJoint {
    this.min = min;
    this.max = max;
    return this;
  }
}
export class StaticJoint extends Joint {
  override type: string = "static";
  update() {}
  contraints(): [number, number] | null {
    return null;
  }
  setContraints(): void {}
}
