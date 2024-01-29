export function message(type: "error" | "info" | "warn", message: string, timeout?: number) {
  console.log("creating msg")
  const messageElement = document.createElement("div")
  messageElement.classList.add(`message-${type}`, "message")
  messageElement.textContent = message

  console.log("A")
  const html = document.getElementById("messages")
  html?.appendChild(messageElement)
  

  // if (timeout !== undefined) {
  //   setTimeout(() => {
  //     messageElement.remove()
  //   }, timeout)
  // }
}
