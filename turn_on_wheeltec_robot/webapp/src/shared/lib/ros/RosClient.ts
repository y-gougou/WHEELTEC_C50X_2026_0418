import ROSLIB from "roslib";

import type { RosConnectionSnapshot, RosConnectionStatus } from "@/shared/types/ros";

type SubscriptionListener<T = unknown> = (message: T) => void;
type StateListener = (snapshot: RosConnectionSnapshot) => void;
type LogListener = (message: string) => void;

interface TopicSubscription {
  name: string;
  messageType: string;
  listeners: Set<SubscriptionListener>;
  topic?: ROSLIB.Topic;
}

interface TopicPublisher {
  name: string;
  messageType: string;
  topic?: ROSLIB.Topic;
}

const initialSnapshot: RosConnectionSnapshot = {
  status: "disconnected",
  url: "",
};

export class RosClient {
  private ros?: ROSLIB.Ros;
  private desiredConnection = false;
  private reconnectTimer?: number;
  private reconnectAttempts = 0;
  private snapshot: RosConnectionSnapshot = initialSnapshot;
  private readonly subscriptions = new Map<string, TopicSubscription>();
  private readonly publishers = new Map<string, TopicPublisher>();
  private readonly stateListeners = new Set<StateListener>();
  private readonly logListeners = new Set<LogListener>();

  connect(url: string) {
    this.desiredConnection = true;
    this.clearReconnectTimer();
    this.teardownActiveRos();
    this.setSnapshot("connecting", url);
    this.openConnection(url);
  }

  disconnect() {
    this.desiredConnection = false;
    this.clearReconnectTimer();
    this.teardownActiveRos();
    this.setSnapshot("disconnected", this.snapshot.url);
  }

  publish<T extends object>(topicName: string, messageType: string, message: T) {
    const publisher = this.ensurePublisher(topicName, messageType);
    if (!publisher.topic && this.ros) {
      publisher.topic = this.createTopic(topicName, messageType);
    }
    if (!publisher.topic) {
      return;
    }
    publisher.topic.publish(new ROSLIB.Message(message));
  }

  subscribe<T>(topicName: string, messageType: string, listener: SubscriptionListener<T>) {
    const key = `${topicName}:${messageType}`;
    let entry = this.subscriptions.get(key);
    if (!entry) {
      entry = {
        name: topicName,
        messageType,
        listeners: new Set(),
      };
      this.subscriptions.set(key, entry);
    }

    entry.listeners.add(listener as SubscriptionListener);
    if (this.ros && this.snapshot.status === "connected" && !entry.topic) {
      entry.topic = this.attachTopic(entry);
    }

    return () => {
      const current = this.subscriptions.get(key);
      if (!current) {
        return;
      }
      current.listeners.delete(listener as SubscriptionListener);
      if (current.listeners.size === 0) {
        if (current.topic) {
          current.topic.unsubscribe();
        }
        this.subscriptions.delete(key);
      }
    };
  }

  onStateChange(listener: StateListener) {
    this.stateListeners.add(listener);
    listener(this.snapshot);
    return () => this.stateListeners.delete(listener);
  }

  onLog(listener: LogListener) {
    this.logListeners.add(listener);
    return () => this.logListeners.delete(listener);
  }

  getSnapshot() {
    return this.snapshot;
  }

  private openConnection(url: string) {
    const ros = new ROSLIB.Ros({ url });
    this.ros = ros;

    ros.on("connection", () => {
      if (this.ros !== ros) {
        return;
      }
      this.reconnectAttempts = 0;
      this.setSnapshot("connected", url);
      this.log(`Connected to ${url}`);
      this.rebuildTopics();
    });

    ros.on("error", (error: unknown) => {
      if (this.ros !== ros) {
        return;
      }
      const message = error instanceof Error ? error.message : "Unknown rosbridge error";
      this.setSnapshot("error", url, message);
      this.log(`ROS error: ${message}`);
    });

    ros.on("close", () => {
      if (this.ros !== ros) {
        return;
      }
      this.resetTopicHandles();
      if (!this.desiredConnection) {
        this.setSnapshot("disconnected", url);
        this.log("ROS disconnected");
        return;
      }
      this.setSnapshot("reconnecting", url);
      this.scheduleReconnect(url);
    });
  }

  private scheduleReconnect(url: string) {
    this.clearReconnectTimer();
    this.reconnectAttempts += 1;
    const delay = Math.min(1000 * 2 ** (this.reconnectAttempts - 1), 16000);
    this.log(`ROS reconnect in ${(delay / 1000).toFixed(0)}s`);
    this.reconnectTimer = window.setTimeout(() => this.openConnection(url), delay);
  }

  private clearReconnectTimer() {
    if (this.reconnectTimer) {
      window.clearTimeout(this.reconnectTimer);
      this.reconnectTimer = undefined;
    }
  }

  private teardownActiveRos() {
    const activeRos = this.ros;
    this.ros = undefined;
    this.resetTopicHandles();

    if (!activeRos) {
      return;
    }

    try {
      activeRos.close();
    } catch {
      // Ignore close races from browser unload/reconnect.
    }
  }

  private rebuildTopics() {
    this.publishers.forEach((publisher) => {
      publisher.topic = this.createTopic(publisher.name, publisher.messageType);
    });

    this.subscriptions.forEach((entry) => {
      entry.topic = this.attachTopic(entry);
    });
  }

  private resetTopicHandles() {
    this.publishers.forEach((publisher) => {
      publisher.topic = undefined;
    });
    this.subscriptions.forEach((entry) => {
      entry.topic = undefined;
    });
  }

  private attachTopic(entry: TopicSubscription) {
    const topic = this.createTopic(entry.name, entry.messageType);
    topic.subscribe((message) => {
      entry.listeners.forEach((listener) => listener(message));
    });
    return topic;
  }

  private ensurePublisher(topicName: string, messageType: string) {
    const key = `${topicName}:${messageType}`;
    let publisher = this.publishers.get(key);
    if (!publisher) {
      publisher = {
        name: topicName,
        messageType,
      };
      this.publishers.set(key, publisher);
    }
    return publisher;
  }

  private createTopic(topicName: string, messageType: string) {
    if (!this.ros) {
      throw new Error("ROS client not connected");
    }

    return new ROSLIB.Topic({
      ros: this.ros,
      name: topicName,
      messageType,
      queue_length: 1,
    });
  }

  private setSnapshot(status: RosConnectionStatus, url: string, error?: string) {
    this.snapshot = {
      status,
      url,
      error,
    };
    this.stateListeners.forEach((listener) => listener(this.snapshot));
  }

  private log(message: string) {
    this.logListeners.forEach((listener) => listener(message));
  }
}
