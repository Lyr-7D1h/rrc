import {
  DoubleSide,
  Mesh,
  MeshBasicMaterial,
  PlaneGeometry,
} from "three";
import { PositionalVector } from "./renderer";

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
