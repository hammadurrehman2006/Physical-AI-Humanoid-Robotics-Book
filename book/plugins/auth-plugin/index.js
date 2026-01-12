// book/plugins/auth-plugin/index.js
module.exports = function (context, options) {
  return {
    name: 'auth-plugin',
    async loadContent() {
      // This plugin doesn't load any content, but the function is required
    },
    configureWebpack(config, isServer, utils) {
      // Configure webpack if needed
      return {};
    },
    routes: {
      routes: [
        {
          path: '/api/auth',
          component: '@docusaurus/theme-common/Noop',
          modules: [],
          exact: false,
        },
      ],
    },
  };
};