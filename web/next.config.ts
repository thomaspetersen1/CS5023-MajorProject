import type { NextConfig } from "next";

const nextConfig: NextConfig = {
  // Static export so the production bundle is a folder of HTML/JS/CSS
  // that any plain web server (including `python3 -m http.server`) can
  // serve. We have no server actions, route handlers, or dynamic params,
  // so static export is sound.
  output: "export",

  // Allow phones/laptops on the same wifi to load the dev server by LAN IP
  // (e.g. http://10.x.x.x:3000) without Next.js blocking dev-only resources
  // like HMR and font fetches.
  allowedDevOrigins: ["10.*", "172.*", "192.168.*"],
};

export default nextConfig;
