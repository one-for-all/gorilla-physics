const { rspack } = require("@rspack/core");
const WasmPackPlugin = require("@wasm-tool/wasm-pack-plugin");
const path = require("path");

const isDev = process.env.NODE_ENV === "development";
const dist = path.resolve(__dirname, "dist");

const featureGPU = process.env.FEATURE_GPU === "1";

module.exports = (env) => {
  const entryName = env.entry || "demo";

  return {
    mode: isDev ? "development" : "production",
    entry: `./src/${entryName}.ts`,
    devtool: isDev ? "inline-source-map" : false,
    output: {
      path: dist,
      filename: "index.js",
    },
    resolve: {
      extensions: [".ts", ".js"],
    },
    experiments: {
      asyncWebAssembly: true,
      syncWebAssembly: true,
    },
    module: {
      rules: [
        {
          test: /\.ts$/,
          loader: "builtin:swc-loader",
        },
      ],
    },
    plugins: [
      new rspack.CopyRspackPlugin({
        patterns: [{ from: "static", to: dist }],
      }),

      new WasmPackPlugin({
        crateDirectory: "../",
        extraArgs: featureGPU ? "--features gpu" : "",
      }),
    ],
    // To disable warning on screen
    stats: {
      warnings: false,
    },
    performance: {
      hints: false,
    },
  };
};
