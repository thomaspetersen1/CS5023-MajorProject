import "server-only";
import fs from "node:fs/promises";
import path from "node:path";
import yaml from "js-yaml";

export type Landmark = {
  name: string;
  x: number;
  y: number;
  yaw: number;
};

type LandmarksFile = {
  landmarks?: Record<string, { x: number; y: number; yaw?: number }>;
};

export async function loadLandmarks(): Promise<Landmark[]> {
  const yamlPath = path.resolve(
    process.cwd(),
    "..",
    "src",
    "tour_guide",
    "config",
    "landmarks.yaml",
  );
  const raw = await fs.readFile(yamlPath, "utf8");
  const parsed = yaml.load(raw) as LandmarksFile | null;
  const entries = parsed?.landmarks ?? {};
  return Object.entries(entries).map(([name, data]) => ({
    name,
    x: data.x,
    y: data.y,
    yaw: data.yaw ?? 0,
  }));
}
