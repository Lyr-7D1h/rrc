import { Vector3, Mesh, Object3D, Quaternion } from "three";

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
