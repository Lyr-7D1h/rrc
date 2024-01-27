import {
  DoubleSide,
  Mesh,
  MeshBasicMaterial,
  PlaneGeometry,
  Vector3,
} from "three";
import { Axis } from "./renderer";

export function plane(axis: Axis): Mesh {
  const geometry = new PlaneGeometry(3, 3);
  const material = new MeshBasicMaterial({ color: 0x002288, side: DoubleSide });
  const plane = new Mesh(geometry, material);
  axis.setLookAt(plane);
  plane.position.copy(axis.point);
  return plane;
}

export function height(mesh: Mesh): number {
  // if box geometry it is in params
  if (
    "parameters" in mesh.geometry &&
    typeof mesh.geometry.parameters.depth === "number"
  ) {
    return mesh.geometry.parameters.depth;
  }
  if (mesh.geometry.boundingBox === null) {
    mesh.geometry.computeBoundingBox();
  }
  return mesh.geometry.boundingBox?.max.z - mesh.geometry.boundingBox.min.z;
}

export function warn(...args) {
  console.warn(args);
}
export function error(...args) {
  console.error(args);
}
