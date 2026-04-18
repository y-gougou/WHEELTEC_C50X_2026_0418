import { useRosConnectStore } from "@/features/ros-connect/model/ros-connect-store";
import type { RecordedFile } from "@/shared/types/recorder";

export async function listRecorderFiles() {
  const response = await fetch(`${useRosConnectStore.getState().apiBase}/api/data/list`);
  if (!response.ok) {
    throw new Error(`加载录制文件失败: ${response.status}`);
  }
  const payload = (await response.json()) as { files?: RecordedFile[] };
  return payload.files ?? [];
}

export function buildRecorderDownloadUrl(name: string) {
  return `${useRosConnectStore.getState().apiBase}/api/data/download/${encodeURIComponent(name)}`;
}
