import { create } from "zustand";

import { robotConfig } from "@/shared/config/robot";
import { buildApiBase } from "@/shared/lib/browser";
import type { RosConnectionSnapshot } from "@/shared/types/ros";

interface RosConnectState extends RosConnectionSnapshot {
  draftUrl: string;
  apiBase: string;
  manualDisconnect: boolean;
  setDraftUrl: (url: string) => void;
  applyDraftUrl: () => void;
  setSnapshot: (snapshot: RosConnectionSnapshot) => void;
  setManualDisconnect: (manualDisconnect: boolean) => void;
}

export const useRosConnectStore = create<RosConnectState>((set) => ({
  status: "disconnected",
  url: robotConfig.rosbridgeUrl,
  draftUrl: robotConfig.rosbridgeUrl,
  apiBase: robotConfig.apiBase,
  error: undefined,
  manualDisconnect: false,
  setDraftUrl: (draftUrl) => set({ draftUrl }),
  applyDraftUrl: () =>
    set((state) => ({
      url: state.draftUrl,
      apiBase: buildApiBase(state.draftUrl),
    })),
  setSnapshot: (snapshot) => set(snapshot),
  setManualDisconnect: (manualDisconnect) => set({ manualDisconnect }),
}));
