import { GUI } from 'dat.gui'
import { OrbitControls } from '@three-ts/orbit-controls'
import {
  BufferAttribute,
  BufferGeometry,
  PerspectiveCamera,
  Points,
  PointsMaterial,
  Scene,
  WebGLRenderer,
} from 'three'
import Stats from 'three/examples/jsm/libs/stats.module'
import { type Connection } from './connection'
import { robotToUrdf } from './urdf'
import { Robot, type State } from './robot'

/** responsible for building and rendering the entire simulated world */
export class Simulation {
  private readonly gui: GUI
  private readonly stats: Stats
  private readonly scene: Scene
  private readonly camera: PerspectiveCamera
  private readonly renderer: WebGLRenderer
  private readonly robot: Robot
  private readonly controls: OrbitControls
  private readonly connection: Connection

  constructor(connection: Connection) {
    if (!connection.connected()) throw Error('not connected')

    this.connection = connection

    this.scene = new Scene()

    this.camera = new PerspectiveCamera(
      40,
      window.innerWidth / window.innerHeight,
      0.1,
      200000,
    )
    this.camera.up.set(0, 0, 1)
    this.camera.position.x = 4000
    this.camera.position.y = 4000
    this.camera.position.z = 4000

    this.renderer = new WebGLRenderer()
    this.renderer.setSize(window.innerWidth, window.innerHeight)

    // https://threejs.org/docs/#examples/en/controls/OrbitControls
    this.controls = new OrbitControls(this.camera, this.renderer.domElement)
    this.controls.enableDamping = true

    this.gui = new GUI()

    // const origin = new AxesHelper(200);
    // this.scene.add(origin);

    this.robot = new Robot(connection)
    this.scene.add(this.robot)
    this.robot.buildGUI(this.gui).open()

    // fps counter
    this.stats = new Stats()
  }

  async addControls(): Promise<void> {
    const geometry = new BufferGeometry()
    geometry.setAttribute(
      'position',
      new BufferAttribute(new Float32Array([0, 0, 0]), 3),
    )
    const dotMaterial = new PointsMaterial({ size: 50, color: 0xff0000 })
    const dot = new Points(geometry, dotMaterial)
    this.scene.add(dot)

    const controls = this.gui.addFolder('Controls')
    const position = dot.position
    controls.add(position, 'x').onChange((_) => {
      this.connection.send({
        type: 'ikmove',
        position: [position.x, position.y, position.z],
      })
    })
    controls.add(position, 'y').onChange((_) => {
      this.connection.send({
        type: 'ikmove',
        position: [position.x, position.y, position.z],
      })
    })
    controls.add(position, 'z').onChange((_) => {
      this.connection.send({
        type: 'ikmove',
        position: [position.x, position.y, position.z],
      })
    })
    controls.open()
  }

  /** Build the initial state of the simulation */
  async init(): Promise<void> {
    await this.addControls()

    const urdf = robotToUrdf(this.robot)
    const limits = this.robot.limits()
    this.connection.send({
      type: 'init',
      urdf,
      limits,
      state: this.robot.state,
    })

    this.connection.on('message', (state) => {
      // before init it will send empty states
      if ((state as State).length === 0) {
        return
      }
      this.robot.update(state as State)
    })

    document.body.appendChild(this.stats.dom)
    document.body.appendChild(this.renderer.domElement)
  }

  draw(): void {
    window.requestAnimationFrame(() => {
      this.draw()
    })
    this.stats.update()
    this.controls.update()
    this.renderer.render(this.scene, this.camera)
  }
}
