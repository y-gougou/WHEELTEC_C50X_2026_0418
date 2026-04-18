import { useEffect } from "react";

import { useRobotStore } from "@/entities/robot/model/robot-store";
import { registerRosTopics } from "@/features/ros-connect/lib/register-ros-topics";
import { useRosConnectStore } from "@/features/ros-connect/model/ros-connect-store";
import { rosClient } from "@/shared/lib/ros/client";

export function useRosRuntime() {
  const setSnapshot = useRosConnectStore((state) => state.setSnapshot);
  const appendLog = useRobotStore((state) => state.appendLog);

  useEffect(() => {
    const unsubscribeState = rosClient.onStateChange((snapshot) => {
      setSnapshot(snapshot);
    });
    const unsubscribeLog = rosClient.onLog((message) => {
      appendLog({ level: "info", message });
    });
    const unregisterTopics = registerRosTopics();

    const { manualDisconnect, url } = useRosConnectStore.getState();
    if (!manualDisconnect) {
      rosClient.connect(url);
    }

    return () => {
      unsubscribeState();
      unsubscribeLog();
      unregisterTopics();
      rosClient.disconnect();
    };
  }, [appendLog, setSnapshot]);
}
