// @ts-check
// Note: type annotations allow type checking and IDEs autocompletion

import {themes as prismThemes} from 'prism-react-renderer';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics Textbook',
  tagline: 'Comprehensive Guide to Physical AI, ROS 2, Gazebo, and NVIDIA Isaac',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://sheikhsamra.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is '/<projectName>/'
  baseUrl: '/GIAIC_Hackathon_Book_Creation_Project/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'sheikhsamra', // Usually your GitHub org/user name.
  projectName: 'GIAIC_Hackathon_Book_Creation_Project', // Usually your repo name.

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',
  trailingSlash: false,

  // Even if you don't use internalization, you can use this field to set useful
  // metadata like html lang. For example, if your site is Chinese, you may want
  // to replace "en" with "zh-Hans".
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
          path: 'docs',
          routeBasePath: 'docs',
          sidebarPath: require.resolve('./sidebars.js'),
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/sheikhsamra/GIAIC_Hackathon_Book_Creation_Project',
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
          alt: 'Physical AI Textbook Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'textbookSidebar',
            position: 'left',
            label: 'Textbook',
          },
          {
            type: 'localeDropdown',
            position: 'right',
          },
          {
            href: 'https://github.com/sheikhsamra/GIAIC_Hackathon_Book_Creation_Project',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Modules',
            items: [
              {
                label: 'Introduction to Physical AI',
                to: '/docs/module-1',
              },
              {
                label: 'ROS 2 Fundamentals',
                to: '/docs/module-2',
              },
              {
                label: 'Gazebo Simulation',
                to: '/docs/module-3',
              },
              {
                label: 'NVIDIA Isaac Platform',
                to: '/docs/module-4',
              },
              {
                label: 'Humanoid Robotics',
                to: '/docs/module-5',
              },
              {
                label: 'Vision-Language-Action Systems',
                to: '/docs/module-6',
              },
              {
                label: 'Capstone Project',
                to: '/docs/capstone',
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'Stack Overflow',
                href: 'https://stackoverflow.com/questions/tagged/ros2',
              },
              {
                label: 'Gazebo Community',
                href: 'https://community.gazebosim.org/',
              },
              {
                label: 'NVIDIA Isaac Forum',
                href: 'https://forums.developer.nvidia.com/c/isaac/280',
              },
              {
                label: 'ROS Answers',
                href: 'https://answers.ros.org/questions/',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/sheikhsamra/GIAIC_Hackathon_Book_Creation_Project',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Textbook. Built with Docusaurus.`,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
        additionalLanguages: ['python', 'bash', 'yaml', 'markup', 'json', 'cpp', 'docker', 'cmake', 'shell-session'],
      },
    }),
};

export default config;