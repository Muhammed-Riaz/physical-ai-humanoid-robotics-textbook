// @ts-check
// `@type` JSDoc annotations allow IDEs and type checkers to type-check this file
// even if it's using plain JavaScript.

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics Textbook',
  tagline: 'Bridging digital AI with physical bodies - An AI-Native Textbook',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://Muhammed-Riaz.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/physical-ai-humanoid-robotics-textbook/',

  // GitHub pages deployment config.
  organizationName: 'Muhammed-Riaz', // Usually your GitHub org/user name.
  projectName: 'physical-ai-humanoid-robotics-textbook', // Usually your repo name.

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  deploymentBranch: 'gh-pages',
  trailingSlash: false,

  // Custom fields for the chatbot API
  customFields: {
    chatbotApiUrl: process.env.CHATBOT_API_URL || 'https://riaz110-text-book.hf.space/api',
  },

 


  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'],
  },
  

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          // Path to the docs folder relative to the site directory
          path: './docs', // Point to the docs directory where content is located
          // Route base path for the docs section
          routeBasePath: '/', // Serve the docs at the site's root - keeping this to serve docs at root under the baseUrl
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/Muhammed-Riaz/physical-ai-humanoid-robotics-textbook/edit/main/docs/',
          // Configure docs to work with multiple locales
          showLastUpdateAuthor: false,
          showLastUpdateTime: false,
        },
        blog: false, // Disable blog if not needed
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg',
      navbar: {
        title: 'Physical AI Textbook',
        logo: {
          alt: 'Physical AI & Humanoid Robotics Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            to: '/tutorial-intro',
            label: 'Tutorial',
            position: 'left',
          },
          {
            type: 'localeDropdown',
            position: 'right',
          },
          {
            href: 'https://github.com/Muhammed-Riaz/physical-ai-humanoid-robotics-textbook',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Docs',
            items: [
              {
                label: 'Textbook',
                to: '/',
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'Discord',
                href: 'https://discordapp.com/invite/docusaurus',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/Muhammed-Riaz/physical-ai-humanoid-robotics-textbook',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Panaversity. Built with Docusaurus.`,
      },
      prism: {
        theme: require('prism-react-renderer').themes.github,
        darkTheme: require('prism-react-renderer').themes.dracula,
      },
    }),
};

module.exports = config;