import { error } from "./util";

export class Connection {
  private socket: WebSocket;
  constructor() {
    this.socket = new WebSocket("ws://localhost:6543");
    this.socket.onerror = (e) => error(e)
    // TODO protobuf
    this.socket.onmessage = (e) => console.log(e)
  }

  send() {
    this.socket.send("a")
  }

  close() {

  }
}
