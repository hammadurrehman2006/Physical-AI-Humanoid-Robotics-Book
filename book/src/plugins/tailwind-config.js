module.exports = function tailwindPlugin(context, options) {
  return {
    name: "tailwind-plugin",
    configurePostCss(postcssOptions) {
      // Add Tailwind
      postcssOptions.plugins.push(require("@tailwindcss/postcss"));
      // Add preset-env to transpile Tailwind's modern output
      postcssOptions.plugins.push(require("postcss-preset-env")({
        autoprefixer: { flexbox: 'no-2009' },
        stage: 3,
      }));
      return postcssOptions;
    },
  };
};