import WebGL from "three/addons/capabilities/WebGL.js";
import { Simulation } from "./renderer";
import { connect } from "./connection";
import { error, info } from "./util";

const URL = "ws://localhost:6543";

(async () => {
  if (WebGL.isWebGLAvailable()) {
    const infoBlock = info(`Connecting to ${URL}`, true);
    try {
      // const connection = await connect(URL);
      // console.log("A")
      // infoBlock.remove();
      const renderer = new Simulation();
      renderer.show();
      renderer.draw();
    } catch (e) {
      infoBlock.remove();
      error(e, true);
    }
  } else {
    const error = WebGL.getWebGLErrorMessage();
    error(error);
  }
})()
