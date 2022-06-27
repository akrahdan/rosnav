// Contents of the file /rollup.config.js
import typescript from "@rollup/plugin-typescript";
import commonjs from "rollup-plugin-commonjs";
import dts from "rollup-plugin-dts";
import json from "@rollup/plugin-json";
import replace from "@rollup/plugin-replace";
import { nodeResolve as resolve } from "@rollup/plugin-node-resolve";
const config = [
  {
    input: "build/dist/index.js",
    output: {
      file: "rosnav.js",
      format: "cjs",
      sourcemap: true,
    },
    plugins: [
      replace({
        "process.env.NODE_ENV": JSON.stringify("development"),
        preventAssignment: true
      }),
      commonjs(),
      typescript(),
    //   resolve(),
     
      json(),
    ],
  },
  {
    input: "build/dist/index.d.ts",
    output: {
      file: "rosnav.d.ts",
      format: "es",
    },
    plugins: [dts(), json()],
  },
];
export default config;
