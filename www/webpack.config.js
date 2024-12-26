const webpack = require("webpack");
const CopyPlugin = require("copy-webpack-plugin");
const WasmPackPlugin = require("@wasm-tool/wasm-pack-plugin");
const path = require("path");

const isDev = process.env.NODE_ENV === "development";
const dist = path.resolve(__dirname, "dist");

/**
 * @type {import('webpack').Configuration}
 */
const webpackConfig = {
  mode: isDev ? "development" : "production",
  entry: "./src/index.ts",
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
        use: "ts-loader",
      },
    ],
  },
  plugins: [
    new CopyPlugin({
      patterns: [{ from: "static", to: dist }],
    }),

    new WasmPackPlugin({
      crateDirectory: "../",
    }),
  ],
};

module.exports = webpackConfig;
