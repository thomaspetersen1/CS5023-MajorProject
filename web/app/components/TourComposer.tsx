"use client";

import { useEffect, useMemo, useRef, useState } from "react";
import * as ROSLIB from "roslib";
import type { Landmark } from "@/lib/landmarks";

type TourStatus = {
  state: "IDLE" | "PLANNING" | "NAVIGATING";
  current_target: string | null;
  remaining: string[];
  visited: string[];
  last_event: string;
  timestamp: number;
};

type ConnectionState = "connecting" | "connected" | "disconnected" | "error";

const DEFAULT_ROSBRIDGE_URL = "ws://localhost:9090";

export default function TourComposer({ landmarks }: { landmarks: Landmark[] }) {
  const [selected, setSelected] = useState<Set<string>>(new Set());
  const [status, setStatus] = useState<TourStatus | null>(null);
  const [connection, setConnection] = useState<ConnectionState>("connecting");
  const tourConfigPubRef = useRef<ROSLIB.Topic | null>(null);

  useEffect(() => {
    const url =
      process.env.NEXT_PUBLIC_ROSBRIDGE_URL ?? DEFAULT_ROSBRIDGE_URL;
    const ros = new ROSLIB.Ros({ url });

    ros.on("connection", () => setConnection("connected"));
    ros.on("error", () => setConnection("error"));
    ros.on("close", () => setConnection("disconnected"));

    const tourConfigPub = new ROSLIB.Topic({
      ros,
      name: "/tour_config",
      messageType: "std_msgs/String",
    });
    tourConfigPubRef.current = tourConfigPub;

    const tourStatusSub = new ROSLIB.Topic({
      ros,
      name: "/tour_status",
      messageType: "std_msgs/String",
    });
    tourStatusSub.subscribe((msg) => {
      const data = (msg as { data: string }).data;
      try {
        setStatus(JSON.parse(data) as TourStatus);
      } catch (e) {
        console.warn("Bad tour_status payload", e);
      }
    });

    return () => {
      tourStatusSub.unsubscribe();
      ros.close();
    };
  }, []);

  const toggle = (name: string) => {
    setSelected((prev) => {
      const next = new Set(prev);
      if (next.has(name)) next.delete(name);
      else next.add(name);
      return next;
    });
  };

  const submitTour = () => {
    tourConfigPubRef.current?.publish({
      data: JSON.stringify({ landmarks: Array.from(selected) }),
    });
  };

  const clearTour = () => {
    tourConfigPubRef.current?.publish({
      data: JSON.stringify({ landmarks: [] }),
    });
  };

  const isRunning =
    status?.state === "PLANNING" || status?.state === "NAVIGATING";
  const canSubmit = connection === "connected" && selected.size > 0;
  const submitLabel = isRunning ? "Update Tour" : "Start Tour";

  const visitedSet = useMemo(
    () => new Set(status?.visited ?? []),
    [status?.visited],
  );
  const remainingSet = useMemo(
    () => new Set(status?.remaining ?? []),
    [status?.remaining],
  );

  return (
    <div className="mx-auto w-full max-w-md space-y-4 p-4 pb-24">
      <header className="flex items-center justify-between">
        <h1 className="text-xl font-semibold tracking-tight">Tour Composer</h1>
        <ConnectionBadge state={connection} />
      </header>

      <section className="rounded-2xl border border-zinc-200 bg-white p-4 shadow-sm dark:border-zinc-800 dark:bg-zinc-950">
        <div className="mb-3 flex items-center justify-between">
          <h2 className="text-sm font-medium text-zinc-500 dark:text-zinc-400">
            Pick your stops
          </h2>
          <span className="text-sm tabular-nums text-zinc-500 dark:text-zinc-400">
            {selected.size} / {landmarks.length}
          </span>
        </div>
        <ul className="space-y-2">
          {landmarks.map((lm) => {
            const isSelected = selected.has(lm.name);
            const isCurrent = status?.current_target === lm.name;
            const isVisited = visitedSet.has(lm.name);
            const isUpcoming = remainingSet.has(lm.name);
            return (
              <li key={lm.name}>
                <button
                  type="button"
                  onClick={() => toggle(lm.name)}
                  className={`flex w-full items-center justify-between rounded-xl border px-4 py-3 text-left transition active:scale-[0.99] ${
                    isSelected
                      ? "border-emerald-500 bg-emerald-50 dark:border-emerald-500/60 dark:bg-emerald-500/10"
                      : "border-zinc-200 bg-white hover:border-zinc-300 dark:border-zinc-800 dark:bg-zinc-950 dark:hover:border-zinc-700"
                  }`}
                >
                  <div>
                    <div className="font-medium">{lm.name}</div>
                    <div className="text-xs tabular-nums text-zinc-500 dark:text-zinc-400">
                      ({lm.x.toFixed(2)}, {lm.y.toFixed(2)})
                    </div>
                  </div>
                  <div className="flex items-center gap-2">
                    {isCurrent && (
                      <span className="rounded-full bg-blue-500/15 px-2 py-0.5 text-xs font-medium text-blue-600 dark:text-blue-300">
                        now
                      </span>
                    )}
                    {!isCurrent && isUpcoming && (
                      <span className="rounded-full bg-zinc-200/80 px-2 py-0.5 text-xs font-medium text-zinc-600 dark:bg-zinc-800 dark:text-zinc-300">
                        next
                      </span>
                    )}
                    {isVisited && (
                      <span className="rounded-full bg-emerald-500/15 px-2 py-0.5 text-xs font-medium text-emerald-600 dark:text-emerald-300">
                        done
                      </span>
                    )}
                    <span
                      aria-hidden
                      className={`flex h-5 w-5 items-center justify-center rounded-full border ${
                        isSelected
                          ? "border-emerald-500 bg-emerald-500 text-white"
                          : "border-zinc-300 dark:border-zinc-700"
                      }`}
                    >
                      {isSelected && (
                        <svg
                          viewBox="0 0 12 12"
                          className="h-3 w-3"
                          fill="none"
                          stroke="currentColor"
                          strokeWidth="2"
                        >
                          <path d="M2.5 6.5L5 9l4.5-5.5" />
                        </svg>
                      )}
                    </span>
                  </div>
                </button>
              </li>
            );
          })}
        </ul>
      </section>

      {status && (
        <section className="rounded-2xl border border-zinc-200 bg-white p-4 shadow-sm dark:border-zinc-800 dark:bg-zinc-950">
          <div className="mb-2 flex items-center justify-between">
            <h2 className="text-sm font-medium text-zinc-500 dark:text-zinc-400">
              Live status
            </h2>
            <StateBadge state={status.state} />
          </div>
          <div className="space-y-1 text-sm">
            <div>
              <span className="text-zinc-500 dark:text-zinc-400">Now: </span>
              <span className="font-medium">
                {status.current_target ?? "—"}
              </span>
            </div>
            <div className="text-zinc-600 dark:text-zinc-300">
              {status.last_event}
            </div>
            <div className="pt-1 text-xs tabular-nums text-zinc-500 dark:text-zinc-400">
              {status.visited.length} done · {status.remaining.length} to go
            </div>
          </div>
        </section>
      )}

      <div className="fixed inset-x-0 bottom-0 border-t border-zinc-200 bg-white/90 p-4 backdrop-blur dark:border-zinc-800 dark:bg-black/80">
        <div className="mx-auto flex w-full max-w-md gap-2">
          <button
            type="button"
            onClick={clearTour}
            disabled={connection !== "connected"}
            className="flex-1 rounded-full border border-zinc-300 px-4 py-3 text-sm font-medium text-zinc-700 transition active:scale-[0.99] disabled:opacity-40 dark:border-zinc-700 dark:text-zinc-200"
          >
            Stop
          </button>
          <button
            type="button"
            onClick={submitTour}
            disabled={!canSubmit}
            className="flex-[2] rounded-full bg-emerald-600 px-4 py-3 text-sm font-semibold text-white transition active:scale-[0.99] disabled:opacity-40"
          >
            {submitLabel}
          </button>
        </div>
      </div>
    </div>
  );
}

function ConnectionBadge({ state }: { state: ConnectionState }) {
  const label =
    state === "connected"
      ? "Connected"
      : state === "connecting"
        ? "Connecting"
        : state === "error"
          ? "Error"
          : "Offline";
  const dot =
    state === "connected"
      ? "bg-emerald-500"
      : state === "connecting"
        ? "bg-amber-500 animate-pulse"
        : "bg-rose-500";
  return (
    <span className="flex items-center gap-2 text-xs text-zinc-500 dark:text-zinc-400">
      <span className={`h-2 w-2 rounded-full ${dot}`} />
      {label}
    </span>
  );
}

function StateBadge({ state }: { state: TourStatus["state"] }) {
  const styles =
    state === "NAVIGATING"
      ? "bg-blue-500/15 text-blue-600 dark:text-blue-300"
      : state === "PLANNING"
        ? "bg-amber-500/15 text-amber-600 dark:text-amber-300"
        : "bg-zinc-200 text-zinc-600 dark:bg-zinc-800 dark:text-zinc-300";
  return (
    <span
      className={`rounded-full px-2 py-0.5 text-xs font-medium ${styles}`}
    >
      {state}
    </span>
  );
}
