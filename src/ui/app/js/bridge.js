export const webviewBridge = {
  available() {
    const webview = window.chrome?.webview;
    return !!webview && typeof webview.postMessage === "function" && typeof webview.addEventListener === "function";
  },
  postMessage(message) { window.chrome.webview.postMessage(message); },
  addMessageListener(handler) { window.chrome?.webview?.addEventListener?.("message", handler); }
};
