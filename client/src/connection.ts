import { error } from "./util";

/** Connect to socket in a blocking manner erroring in case of timeout or error */ 
export function connect(addr: string): WebSocket | null{
  const socket = new WebSocket(addr);
  socket.onerror = (e) => error(e)
  // TODO use something like protobuf for sending messages
  socket.onmessage = (e) => console.log(e)
  let tries = 0;
  while (socket.readyState != 0) {
    if (tries === 3) {
      error(`failed to connect to ${addr}`) 
      return null
    }
    setTimeout(() => {}, 2000);
    tries += 1;
  }

  return socket
}
