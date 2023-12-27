import { defineConfig } from "vite";
import reactRefresh from "@vitejs/plugin-react-refresh";
import path from "path";
import { nodeResolve } from "@rollup/plugin-node-resolve";
import commonjs from "@rollup/plugin-commonjs";
import copy from "rollup-plugin-copy-assets";

console.error(__dirname);
console.error(path.resolve(__dirname, "src/utils"));
console.error(process.env.REACT_APP_COMMIT_HASH);

// https://vitejs.dev/config/
export default defineConfig({
  plugins: [
    reactRefresh(),
    // copy({
    //   assets: ["src/proto/detection.proto"],
    // }),
    // nodeResolve(),
    // commonjs({
    //   dynamicRequireTargets: [
    //     "node_modules/protobufjs/**/*.js",
    //     "**/proto/detection.js",
    //   ],
    // }),
  ],
  resolve: {
    alias: {
      "@plugins": path.resolve(__dirname, "src/plugins"),
      "@components": path.resolve(__dirname, "src/components"),
      "@rpc": path.resolve(__dirname, "src/rpc"),
      "@proto": path.resolve(__dirname, "src/proto"),
      "@assets": path.resolve(__dirname, "src/assets"),
      "@utils": path.resolve(__dirname, "src/utils"),
      "@hooks": path.resolve(__dirname, "src/hooks"),
    },
  },
  server: {
    fs: {
      // Allow serving files from one level up to the project root
      strict: false,
    },
  },
  esbuild: {
    // jsxInject: `import React from 'react'`,
    // jsxInject: `import * as log from 'loglevel'`,
  },
  define: {
    COMMIT_HASH: JSON.stringify(process.env.REACT_APP_COMMIT_HASH),
  },
  assetsInclude: [/\.glb(\?.*)?$/],
  build: {
    rollupOptions: {
      external: ["protobufjs"],
      output: {
        paths: {
          protobufjs: "/protobuf.min.js",
        },
        // manualChunks: {
        //   react: ["react", "react-dom"],
        //   three: ["three", "@react-three/fiber"],
        // },
      },
    },
  },
  // build: {
  //   commonjsOptions: {
  //     include: "*/proto/*.js",
  //     transformMixedEsModules: true,
  //     esmExternals: true,
  //   },
  // },
});
