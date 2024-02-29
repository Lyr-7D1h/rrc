import {
  AxesHelper,
  BufferAttribute,
  BufferGeometry,
  type Mesh,
  Object3D,
  Points,
  PointsMaterial,
  Quaternion,
  type Vector3,
} from 'three'

import { Transform, vec3 } from './geometry'

export abstract class Joint extends Object3D {
  base: Mesh
  attachment: Mesh
  jointPos: Transform
  pivot: Transform

  private dot?: Points
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
    super()

    jointPos = jointPos ?? new Transform()
    pivot = pivot ?? new Transform()

    // go to local position of base and put joint there
    jointPos.updateObject(this)
    base.add(this)

    // translate attachment
    pivot.updateObject(attachment)
    this.add(attachment)

    this.togglePoint()

    this.name = name
    this.base = base
    this.attachment = attachment
    this.jointPos = jointPos
    this.pivot = pivot
  }

  togglePoint(): void {
    if (typeof this.dot !== 'undefined') {
      this.remove(this.dot)
      this.dot = undefined
      return
    }
    const geometry = new BufferGeometry()
    geometry.setAttribute(
      'position',
      new BufferAttribute(new Float32Array([0, 0, 0]), 3),
    )
    const dotMaterial = new PointsMaterial({ size: 50, color: 0xffff00 })
    this.dot = new Points(geometry, dotMaterial)
    this.dot.add(new AxesHelper(100))
    this.add(this.dot)
  }

  abstract update(value: number): void
}
export class PrismaticJoint extends Joint {
  override type: string = 'prismatic'
  origin: Vector3
  value: number

  // limits in mm
  min: number
  max: number
  maxVelocity: number
  maxAcceleration: number

  axis: Vector3
  constructor(
    name: string,
    base: Mesh,
    attachment: Mesh,
    /** position and direction in the local space of `base` */
    jointPos?: Transform,
    /** position, direction and orientation in the local space of `attachment` */
    pivot?: Transform,
  ) {
    super(name, base, attachment, jointPos, pivot)
    this.origin = this.position.clone()
    this.value = 0
    this.axis = vec3(0, 0, 1).applyQuaternion(this.quaternion)
    this.min = -100
    this.max = 100
    this.maxVelocity = 200
    this.maxAcceleration = 50
  }

  setAxis(axis: Vector3): this {
    this.axis = axis
    return this
  }

  update(value: number): void {
    this.translateOnAxis(this.axis, value - this.value)
    this.value = value
  }

  contraints(): [number, number] | null {
    return [this.min, this.max]
  }

  setContraints(min: number, max: number): this {
    this.min = min
    this.max = max
    return this
  }
}
export class RevoluteJoint extends Joint {
  override type: string = 'revolute'
  oldValue: number
  value: number

  // limits in degrees
  min: number
  max: number
  maxVelocity: number
  maxAcceleration: number

  origin: Quaternion

  axis: Vector3
  constructor(
    name: string,
    base: Mesh,
    attachment: Mesh,
    /** position and direction in the local space of `base` */
    jointPos?: Transform,
    /** position, direction and orientation in the local space of `attachment` */
    pivot?: Transform,
  ) {
    super(name, base, attachment, jointPos, pivot)
    this.oldValue = 0
    this.value = 0
    this.axis = vec3(0, 0, 1)
    this.min = -180
    this.max = 180
    this.maxVelocity = 100
    this.maxAcceleration = 20
    this.origin = this.quaternion.clone()
  }

  setAxis(axis: Vector3): this {
    this.axis = axis
    return this
  }

  // will rotate clockwise around an axis
  update(value: number): void {
    value = -(value * Math.PI) / 180
    const quat = this.origin
      .clone()
      .multiply(new Quaternion().setFromAxisAngle(this.axis, value).normalize())
    this.quaternion.copy(quat)
  }

  contraints(): [number, number] | null {
    return [this.min, this.max]
  }

  setContraints(min: number, max: number): this {
    this.min = min
    this.max = max
    return this
  }
}
export class FixedJoint extends Joint {
  override type: string = 'fixed'
  update(): void {}
  contraints(): [number, number] | null {
    return null
  }

  setContraints(): void {}
}
