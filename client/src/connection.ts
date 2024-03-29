import { type State } from './robot'

/** Connect to socket in a blocking manner erroring in case of timeout or error */
export async function connect(addr: string): Promise<Connection> {
  return await new Promise((resolve, reject) => {
    const connection = new Connection(addr)

    let returned = false
    connection.on('error', () => {
      returned = true
      reject(new Error(`connection to ${addr} failed`))
    })

    connection.on('open', () => {
      returned = true
      resolve(connection)
    })

    setTimeout(() => {
      if (!returned) {
        reject(new Error('connection timed out'))
      }
    }, 8000)
  })
}

export type Command =
  | {
      type: 'init'
      urdf: unknown
      limits: unknown
      state: State
    }
  | { type: 'move'; state: State }
  | { type: 'ikmove'; position: number[] }

export class Connection {
  private readonly socket: WebSocket

  constructor(addr: string) {
    this.socket = new WebSocket(`ws://${addr}`)
  }

  on(type: 'message', cb: (data: unknown) => void): void
  on(type: 'message' | 'open' | 'error', cb: (event: Event) => void): void {
    switch (type) {
      case 'message':
        this.socket.addEventListener('message', (e) => {
          cb(JSON.parse(e.data))
        })
        break
      case 'open':
      case 'error':
        this.socket.addEventListener(type, (e) => {
          cb(e)
        })
    }
  }

  send(cmd: Command) {
    this.socket.send(JSON.stringify(cmd))
  }

  init() {
    this.socket.send(JSON.stringify({}))
  }

  connected() {
    return this.socket.readyState === 1
  }
}
