import TourComposer from "./components/TourComposer";
import { loadLandmarks } from "@/lib/landmarks";

export default async function Home() {
  const landmarks = await loadLandmarks();
  return (
    <div className="min-h-dvh bg-zinc-50 text-zinc-900 antialiased dark:bg-black dark:text-zinc-100">
      <TourComposer landmarks={landmarks} />
    </div>
  );
}
