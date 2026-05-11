import type { NextConfig } from "next";

const nextConfig: NextConfig = {
  // Static export so the production build is just plain HTML/JS/CSS and
  // we can serve it with `python3 -m http.server` or anything similar.
  // We don't use server actions or route handlers, so this is fine.
  output: "export",

  // Lets phones and laptops on the same wifi load the dev server by LAN
  // IP without Next.js blocking dev-only assets like HMR or font fetches.
  allowedDevOrigins: ["10.*", "172.*", "192.168.*"],
};

export default nextConfig;
