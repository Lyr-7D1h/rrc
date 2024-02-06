import { GUI } from "dat.gui";
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

/** https://en.wikipedia.org/wiki/Linkage_(mechanical) */
export class Link extends Mesh {
  private basicMaterial: MeshBasicMaterial;
  private midpoint?: Points;
  width?: number;
  height?: number;
  depth?: number;

  // FIXME: allow for different geometries
  constructor(
    width?: number,
    height?: number,
    depth?: number,
    material?: MeshBasicMaterial,
  ) {
    let mat = material
      ? material
      : new MeshBasicMaterial({
          color: 0x888888,
        });
    mat.wireframe = true;
    super(new BoxGeometry(width, height, depth), mat);
    this.width = width;
    this.height = height;
    this.depth = depth;
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
    this.midpoint.add(new AxesHelper(100));
    this.add(this.midpoint);
  }
}
