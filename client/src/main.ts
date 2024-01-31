import WebGL from "three/addons/capabilities/WebGL.js";
import { Simulation } from "./simulation";
import { connect } from "./connection";
import { error, info } from "./util";

const URL = "localhost:6543";

(async () => {
  if (WebGL.isWebGLAvailable()) {
    const infoBlock = info(`Connecting to ${URL}`, true);
    const connection = await connect(URL).catch((e) => {
      infoBlock.remove();
      error(e, true);
    });
    if (connection) {
      infoBlock.remove();
      const simulation = new Simulation(connection);
      await simulation.init();
      simulation.draw();
    }
  } else {
    const error = WebGL.getWebGLErrorMessage();
    error(error);
  }
})();
