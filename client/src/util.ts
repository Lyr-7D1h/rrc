import { message } from './message'

export function info(msg: any, permanent?: boolean): HTMLElement {
  if (typeof msg !== 'string') {
    msg = JSON.stringify(msg)
  }
  console.info(msg)
  return message('info', msg as string, permanent === true ? undefined : 3)
}

export function warn(msg: any, permanent?: boolean): HTMLElement {
  if (typeof msg !== 'string') {
    msg = JSON.stringify(msg)
  }
  console.warn(msg)
  return message('warn', msg as string, permanent === true ? undefined : 3)
}

export function error(msg: any, permanent?: boolean): HTMLElement {
  if (typeof msg !== 'string') {
    msg = JSON.stringify(msg)
  }
  console.error(msg)
  return message('error', msg as string, permanent === true ? undefined : 3)
}

export function toNumber(n: string): number | null {
  const num = Number(n)
  if (typeof num === 'number') {
    return num
  }
  return null
}
