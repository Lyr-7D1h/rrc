import { message } from "./message";

export function info(msg: any, permanent?: boolean): HTMLElement {
  if (typeof msg !== "string") {
    msg = JSON.stringify(msg);
  }
  console.info(msg);
  return message("info", msg, permanent ? undefined : 3);
}

export function warn(msg: any, permanent?: boolean): HTMLElement {
  if (typeof msg !== "string") {
    msg = JSON.stringify(msg);
  }
  console.warn(msg);
  return message("warn", msg, permanent ? undefined : 3);
}

export function error(msg: any, permanent?: boolean): HTMLElement {
  if (typeof msg !== "string") {
    msg = JSON.stringify(msg);
  }
  console.error(msg);
  return message("error", msg, permanent === true ? undefined : 3);
}
