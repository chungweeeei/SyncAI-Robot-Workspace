import type { NextConfig } from "next";

const nextConfig: NextConfig = {
  // Production Dockerfile ships only .next/standalone (+ static assets); the
  // dev flow (`npm run dev` against the mounted workspace) is unaffected.
  output: "standalone",
};

export default nextConfig;
