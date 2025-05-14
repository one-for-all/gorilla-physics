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
  mode: "production", //  isDev ? "development" : "production", // TODO: Call stack size overflow if not built for production
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
      {
        test: /\.js$/,
        resolve: {
          fullySpecified: false,
        },
      },
    ],
  },
  plugins: [
    new CopyPlugin({
      patterns: [{ from: "static", to: dist }],
    }),

    new WasmPackPlugin({
      crateDirectory: "../",
      extraArgs: "--target web",
    }),
  ],
  devServer: {
    headers: {
      "Cross-Origin-Embedder-Policy": "require-corp",
      "Cross-Origin-Opener-Policy": "same-origin",
    },
    static: {
      directory: path.join(__dirname, "dist"),
    },
    compress: true,
    port: 8080,
  },
};

module.exports = webpackConfig;
