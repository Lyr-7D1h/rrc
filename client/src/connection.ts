/** Connect to socket in a blocking manner erroring in case of timeout or error */
export async function connect(addr: string): Promise<Connection> {
  return new Promise((resolve, reject) => {
    let timeout;
    const connection = new Connection(socket)

    // TODO: use something like protobuf for sending messages
    // socket.onmessage = (e) => console.log(e);

    socket.onerror = () => {
      reject(`connection to ${addr} failed`);
    };

    let tries = 0;
    let attempt = () => {
      if (connection.connected) {
        return resolve();
      }
      if (tries === 16) {
        return reject(`connection timed out`);
      }
      tries += 1;
      timeout = setTimeout(attempt, 500);
    };
  });
}

export class Connection {
  private socket: WebSocket

  constructor(addr: string) {
    this.socket = new WebSocket(addr);
    this.socket.onerror = (e) => this.errorEvents.push(e);

    this.events = []
    this.errorEvents = []
  }

  onMessage(cb: (object: Object) => {}) {
    this.socket.addEventListener("message", (e) => cb(JSON.parse(e.data)))
  }

  onOpen(cb: (object: Object) => {}) {
    this.socket.addEventListener("open", (e) => cb(JSON.parse(e.data)))
  }

  connected() {
      return this.socket.readyState == 1;
  }
}
