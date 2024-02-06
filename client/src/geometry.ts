import {
  DoubleSide,
  PlaneGeometry,
  Vector3,
  AxesHelper,
  BoxGeometry,
  BufferGeometry,
  EdgesGeometry,
  Group,
  LineBasicMaterial,
  LineSegments,
  Matrix3,
  Mesh,
  MeshBasicMaterial,
  Object3D,
  PerspectiveCamera,
  Quaternion,
  Scene,
  Vector4,
} from "three";

export function height(mesh: Mesh): number {
  // if box geometry it is in params
  const geometry = mesh.geometry;
  if (
    "parameters" in geometry &&
    geometry.parameters instanceof Object &&
    "depth" in geometry.parameters &&
    typeof geometry.parameters.depth === "number"
  ) {
    return geometry.parameters.depth;
  }
  if (mesh.geometry.boundingBox === null) {
    mesh.geometry.computeBoundingBox();
  }
  return mesh.geometry.boundingBox?.max.z - mesh.geometry.boundingBox.min.z;
}

export function vec3(x?: number, y?: number, z?: number): Vector3 {
  return new Vector3(x, y, z);
}

export function quat(
  x?: number,
  y?: number,
  z?: number,
  w?: number,
): Quaternion {
  return new Quaternion(x, y, z, w);
}

export function posvec3(point?: Vector3, normal?: Vector3): PositionalVector {
  return new PositionalVector(point, normal);
}
export class PositionalVector {
  point: Vector3;
  normal: Vector3;

  constructor(point?: Vector3, normal?: Vector3) {
    this.point = point || new Vector3(0, 0, 0);
    this.normal = normal ? normal.normalize() : new Vector3(0, 0, 1);
  }

  /** update 3d object so up is same direction as `this.normal` and located at `this.point` */
  moveObject(object: Object3D) {
    object.position.copy(this.point);

    console.log(this.normal, object.quaternion);
    let n = this.normal;
    this.normal.copy().cross(vec3(0, 0, 1));
    Vector4;
    object.quaternion.set(n.x, n.y, n.z, 1).normalize();
    console.log(object.quaternion);
    // object.lookAt(this.normal.x, this.normal.y, this.normal.z);
    // object.up.copy(this.normal);
  }
}
export function transform(point?: Vector3, quaternion?: Quaternion): Transform {
  return new Transform(point, quaternion);
}
/** A point in space with a direction and orientation */
export class Transform {
  point: Vector3;
  quaternion: Quaternion;

  constructor(point?: Vector3, quaternion?: Quaternion) {
    this.quaternion = quaternion ? quaternion.normalize() : new Quaternion();
    this.point = point || new Vector3(0, 0, 0);
    // this.point.applyQuaternion(this.quaternion);
  }

  /** update 3d object */
  updateObject(object: Object3D) {
    object.quaternion.copy(this.quaternion);
    object.position.copy(this.point);
  }
}
