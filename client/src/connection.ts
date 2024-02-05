import { State } from "./robot";
import { error } from "./util";

/** Connect to socket in a blocking manner erroring in case of timeout or error */
export async function connect(addr: string): Promise<Connection> {
  return new Promise((resolve, reject) => {
    const connection = new Connection(addr);

    let returned = false;
    connection.on("error", () => {
      returned = true;
      reject(`connection to ${addr} failed`);
    });

    connection.on("open", () => {
      returned = true;
      resolve(connection);
    });

    setTimeout(() => {
      if (returned === false) {
        return reject(`connection timed out`);
      }
    }, 8000);
  });
}

export type Command =
  | {
      type: "init";
      specs: unknown;
    }
  | { type: "move"; state: State }
  | { type: "ikmove"; position: number[] };

export class Connection {
  private socket: WebSocket;

  constructor(addr: string) {
    this.socket = new WebSocket(`ws://${addr}`);
  }

  on(type: "message", cb: (data: Object) => void): void;
  on(type: "message" | "open" | "error", cb: (event: Event) => void) {
    switch (type) {
      case "message":
        this.socket.addEventListener("message", (e) => cb(JSON.parse(e.data)));
        break;
      case "open":
      case "error":
        this.socket.addEventListener(type, (e) => cb(e));
    }
  }

  send(cmd: Command) {
    this.socket.send(JSON.stringify(cmd));
  }

  init() {
    this.socket.send(JSON.stringify({}));
  }

  connected() {
    return this.socket.readyState == 1;
  }
}
