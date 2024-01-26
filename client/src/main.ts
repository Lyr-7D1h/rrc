import WebGL from 'three/addons/capabilities/WebGL.js';
import { Simulation } from './renderer';

if (WebGL.isWebGLAvailable()) {

	// Initiate function or other initializations here
	const renderer = new Simulation()
	renderer.show()
	renderer.draw()
} else {
	const warning = WebGL.getWebGLErrorMessage();
	// document.getElementById( 'container' ).appendChild( warning );
	// TODO handle warning
}
