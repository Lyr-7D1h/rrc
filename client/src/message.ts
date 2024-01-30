export function message(
  type: "error" | "info" | "warn",
  message: string,
  timeout?: number,
): HTMLElement {
  const messageElement = document.createElement("div");
  messageElement.classList.add(`message-${type}`, "message");
  messageElement.textContent = message;

  const html = document.getElementById("messages");
  html?.appendChild(messageElement);

  if (timeout !== undefined) {
    setTimeout(() => {
      messageElement.remove();
    }, timeout * 1000);
  }

  return messageElement;
}
