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
} from "three";

export function plane(axis: PositionalVector): Mesh {
  const geometry = new PlaneGeometry(3, 3);
  const material = new MeshBasicMaterial({ color: 0x002288, side: DoubleSide });
  const plane = new Mesh(geometry, material);
  axis.setLookAt(plane);
  plane.position.copy(axis.point);
  return plane;
}

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
  const q = new Quaternion();
  q.setFromAxisAngle(vec3(x, y, z), w || Math.PI / 2);
  return q;
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

  /** update 3d object so up is same direction as `this.normal` and located at `this.point` in the direction of the normal */
  moveObject(object: Object3D) {
    object.position.copy(this.point);
    object.quaternion.setFromAxisAngle(this.normal, Math.PI / 2);
  }
}
export function pivot(point?: Vector3, quaternion?: Quaternion): Pivot {
  return new Pivot(point, quaternion);
}
/** A point in space with a direction and orientation */
export class Pivot {
  point: Vector3;
  quaternion: Quaternion;

  constructor(point?: Vector3, quaternion?: Quaternion) {
    this.quaternion = quaternion ? quaternion.normalize() : new Quaternion();
    this.point = point || new Vector3(0, 0, 0);
    this.point.applyQuaternion(this.quaternion);
  }

  /** update 3d object */
  updateObject(object: Object3D) {
    object.quaternion.copy(this.quaternion);
    object.position.copy(this.point);
  }
}
