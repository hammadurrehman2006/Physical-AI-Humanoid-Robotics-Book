import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';


// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Bridging Digital AI with Embodied Intelligence',
  favicon: 'img/logo.svg',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://your-docusaurus-site.example.com',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'facebook', // Usually your GitHub org/user name.
  projectName: 'docusaurus', // Usually your repo name.

  onBrokenLinks: 'warn',

  headTags: [
    // Preconnect to Google Fonts for performance optimization
    {
      tagName: 'link',
      attributes: {
        rel: 'preconnect',
        href: 'https://fonts.googleapis.com',
      },
    },
    {
      tagName: 'link',
      attributes: {
        rel: 'preconnect',
        href: 'https://fonts.gstatic.com',
        crossOrigin: 'anonymous',
      },
    },
    // Load Urdu fonts from Google Fonts
    {
      tagName: 'link',
      attributes: {
        rel: 'stylesheet',
        href: 'https://fonts.googleapis.com/css2?family=Jameel+Noori+Nastaleeq:wght@400&display=swap',
      },
    },
  ],

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'],
    localeConfigs: {
      en: {
        label: 'English',
        direction: 'ltr',
        htmlLang: 'en-US',
        path: 'en',
      },
              ur: {
                label: 'اردو',
                direction: 'rtl',
                htmlLang: 'ur-PK',
                path: 'ur',
              },    },
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          sidebarCollapsible: true,
        },
        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ['rss', 'atom'],
            xslt: true,
          },
          // Useful options to enforce blogging best practices
          onInlineTags: 'warn',
          onInlineAuthors: 'warn',
          onUntruncatedBlogPosts: 'warn',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  plugins: [
    './src/plugins/tailwind-config.js',
    // Plugin for 3D/simulation embeds
    [
      '@docusaurus/plugin-content-docs',
      {
        id: 'simulations',
        path: 'simulations_content',
        routeBasePath: 'simulations',
        sidebarPath: require.resolve('./sidebarsSimulations.js'),
      },
    ],
    // Plugin for additional markdown features
    [
      '@docusaurus/plugin-client-redirects',
      {
        redirects: [
          {
            to: '/docs/module-2/introduction/prerequisites', // new location
            from: ['/docs/gazebo', '/docs/unity'], // old locations
          },
        ],
      },
    ],
    [
      require.resolve("@easyops-cn/docusaurus-search-local"),
      ({
        hashed: true,
      }),
    ],
  ],
  themes: [
    '@docusaurus/theme-mermaid',
  ],
  themeConfig: {
    // Replace with your project's social card
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      logo: {
        alt: 'Physical AI & Humanoid Robotics Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Start Reading',
        },
        {
          type: 'localeDropdown',
          position: 'right',
        },
        {
          href: 'https://github.com/hammadurrehman2006/Physical-AI-Humanoid-Robotics-Book',
          position: 'right',
          className: 'header-github-link', // Add a class for potential styling or targeting
          html: `<svg viewBox="0 0 16 16" fill="currentColor" aria-hidden="true" width="24" height="24">
                  <path d="M8 0C3.58 0 0 3.58 0 8c0 3.54 2.29 6.53 5.47 7.59.4.07.55-.17.55-.38 0-.19-.01-.82-.01-1.49-2.01.37-2.53-.49-2.53-.49-.4-.94-.95-1.19-.95-1.19-.65-.45.07-.44.07-.44.73.05 1.12.75 1.12.75.64 1.09 1.78.78 2.21.6.06-.47.28-.78.51-.96-1.68-.18-3.46-.84-3.46-3.73 0-.82.29-1.49.79-2.01-.08-.18-.36-.95.07-1.99 0 0 .64-.2 2.1.79.6-.16 1.25-.26 1.9-.26.65 0 1.3.09 1.9.26 1.46-1 2.1-.79 2.1-.79.43 1.04.15 1.81.07 1.99.5.52.79 1.2.79 2.01 0 2.9-1.79 3.54-3.47 3.72.29.25.54.73.54 1.48 0 1.07-.01 1.93-.01 2.22 0 .2.15.46.55.38A8.013 8.013 0 0016 8c0-4.42-3.58-8-8-8z"></path>
                  </svg>`,
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Book Content',
          items: [
            {
              label: 'Introduction',
              to: '/docs/intro',
            },
            {
              label: 'Module 1: ROS 2',
              to: '/docs/module-1',
            },
            {
              label: 'Module 2: Digital Twin (Gazebo & Unity)',
              to: '/docs/module-2',
            },
            {
              label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
              to: '/docs/module-3',
            },
            {
              label: 'Module 4: Vision-Language-Action (VLA)',
              to: '/docs/module-4',
            },
          ],
        },
        {
          title: 'Resources',
          items: [
            {
              label: 'ROS 2 Documentation',
              href: 'https://docs.ros.org/en/humble/',
            },
            {
              label: 'Docusaurus',
              href: 'https://docusaurus.io',
            },
          ],
        },
        {
          title: 'More',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/hammadurrehman2006/Physical-AI-Humanoid-Robotics-Book',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Book. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
