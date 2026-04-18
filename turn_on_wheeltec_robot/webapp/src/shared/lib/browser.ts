export function isBrowser() {
  return typeof window !== "undefined";
}

function trimTrailingSlash(value: string) {
  return value.replace(/\/+$/, "");
}

function getEnvValue(key: "VITE_ROSBRIDGE_URL" | "VITE_API_BASE") {
  return typeof import.meta !== "undefined" ? import.meta.env[key] : undefined;
}

export function buildRosbridgeUrl() {
  const configured = getEnvValue("VITE_ROSBRIDGE_URL");
  if (configured) {
    return trimTrailingSlash(configured);
  }

  if (!isBrowser()) {
    return "ws://127.0.0.1:9090";
  }
  return `ws://${window.location.hostname || "127.0.0.1"}:9090`;
}

export function buildApiBase(rosbridgeUrl?: string) {
  const configured = getEnvValue("VITE_API_BASE");
  if (configured) {
    return trimTrailingSlash(configured);
  }

  const candidateUrl = rosbridgeUrl ?? buildRosbridgeUrl();
  try {
    const parsed = new URL(candidateUrl);
    const protocol = parsed.protocol === "wss:" ? "https" : "http";
    return `${protocol}://${parsed.hostname || "127.0.0.1"}:8000`;
  } catch {
    // Fall through to browser-derived default when the URL is incomplete.
  }

  if (!isBrowser()) {
    return "http://127.0.0.1:8000";
  }
  return `http://${window.location.hostname || "127.0.0.1"}:8000`;
}

export function getSystemTheme() {
  if (!isBrowser()) {
    return "dark" as const;
  }
  return window.matchMedia("(prefers-color-scheme: dark)").matches ? "dark" : "light";
}
