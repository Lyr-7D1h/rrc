import { message } from "./message";

export function warn(msg?: any, ...optionalParams: any[]) {
  if (typeof msg !== "string") {
    msg = JSON.stringify(msg)
  }
  console.warn(String(msg), ...optionalParams);
  message("warn", msg, 3)
}

export function error(msg?: any, ...optionalParams: any[]) {
  if (typeof msg !== "string") {
    msg = JSON.stringify(msg)
  }
  console.error(String(msg), ...optionalParams);
  message("error", msg, 3)
}
