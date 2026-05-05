import type { NextConfig } from "next";

const nextConfig: NextConfig = {
  // Allow phones/laptops on the same wifi to load the dev server by LAN IP
  // (e.g. http://10.x.x.x:3000) without Next.js blocking dev-only resources
  // like HMR and font fetches.
  allowedDevOrigins: ["10.*", "172.*", "192.168.*"],
};

export default nextConfig;
