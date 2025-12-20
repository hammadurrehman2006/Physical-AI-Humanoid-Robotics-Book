module.exports = function tailwindPlugin(context, options) {
  return {
    name: "tailwind-plugin",
    configurePostCss(postcssOptions) {
      // Appending Tailwind instead of overwriting all plugins
      postcssOptions.plugins.push(require("@tailwindcss/postcss"));
      return postcssOptions;
    },
  };
};
