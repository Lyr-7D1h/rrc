import { GUI } from 'dat.gui'
import { OrbitControls } from '@three-ts/orbit-controls';
import { AxesHelper, BoxGeometry, EdgesGeometry, LineBasicMaterial, LineSegments, Mesh, MeshBasicMaterial, Object3D, PerspectiveCamera, Scene, Triangle, Vector2, Vector3, WebGLRenderer, WireframeGeometry } from "three";
import Stats from 'three/examples/jsm/libs/stats.module'

/** https://en.wikipedia.org/wiki/Linkage_(mechanical) */
class Link extends Mesh {
	constructor(width: number, height: number, depth: number) {
		const geometry = new BoxGeometry(width, height, depth);
		geometry.normalizeNormals()
		const material = new MeshBasicMaterial({ color: 0x888888, wireframe: true });
		super(geometry, material)
	}

	addLink(faceIndex: number, relJointPos: Vector2, joint: Joint, link: Link) {
	}
}

class Joint {
	constructor(width: number, height: number) {
	}
}
class SlidingJoint extends Joint {
}
class RotationalJoint extends Joint {
}

class Crane extends Object3D {
	constructor() {
		super()

		const base = new Link(1, 1, 1)
		// put base just above z axis
		base.position.set(0, 0, 0.5)
		this.add(base)

		// const lift = new Link(1, 10, 1)
		// base.addLink(new SlidingJoint(), new Vector3())

		// const [armWidth, armHeight] = [1, 1]
		// geometry = new BoxGeometry();
		// material = new MeshBasicMaterial({ color: 0x00ff00 });
		// const arm = new Mesh(geometry, material)
		// arm.position.set(0, 0,)
	}
}

export class Simulation {
	private gui: GUI;
	private stats: Stats;
	private scene: Scene;
	private camera: PerspectiveCamera;
	private renderer: WebGLRenderer;
	private crane: Crane;
	private controls: OrbitControls;

	constructor() {
		this.scene = new Scene();

		this.camera = new PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 2000);
		this.camera.position.x = 5;
		this.camera.position.y = 5;
		this.camera.position.z = 5;
		// this.camera.rotateX(135)
		// this.camera.rotateY(135)

		this.renderer = new WebGLRenderer();
		this.renderer.setSize(window.innerWidth, window.innerHeight);

		// https://threejs.org/docs/#examples/en/controls/OrbitControls
		this.controls = new OrbitControls(this.camera, this.renderer.domElement);
		this.controls.enableDamping = true;
		// TODO wasd controls

		this.crane = new Crane()
		this.scene.add(this.crane)

		const origin = new AxesHelper(2);
		this.scene.add(origin);

		this.gui = new GUI()
		const cameraFolder = this.gui.addFolder('Camera')
		cameraFolder.add(this.camera.position, 'z', 0, 10)
		cameraFolder.add(this.camera.position, 'x', 0, 10)
		cameraFolder.add(this.camera.position, 'y', 0, 10)
		cameraFolder.open()
		this.stats = new Stats()

	}

	show() {
		document.body.appendChild(this.stats.dom)
		document.body.appendChild(this.renderer.domElement);
	}

	draw() {
		window.requestAnimationFrame(() => this.draw());
		this.stats.update()
		this.controls.update()
		// this.crane.rotation.x += 0.01;
		// this.crane.rotation.y += 0.01;
		this.renderer.render(this.scene, this.camera);
	}
}
