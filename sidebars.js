// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  textbookSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      items: ['module-1/index', 'module-1/learning-objectives', 'module-1/theory'],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'ROS 2 Fundamentals',
      items: ['module-2/index', 'module-2/learning-objectives', 'module-2/theory', 'module-2/code-examples'],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Gazebo Simulation',
      items: ['module-3/index', 'module-3/learning-objectives', 'module-3/theory', 'module-3/code-examples'],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'NVIDIA Isaac Platform',
      items: ['module-4/index', 'module-4/learning-objectives', 'module-4/theory', 'module-4/code-examples'],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Humanoid Robotics',
      items: ['module-5/index', 'module-5/learning-objectives', 'module-5/theory', 'module-5/code-examples'],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Vision-Language-Action',
      items: ['module-6/index', 'module-6/learning-objectives', 'module-6/theory', 'module-6/code-examples'],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Capstone Project',
      items: ['capstone/index', 'capstone/project-description', 'capstone/architecture-diagrams'],
      collapsed: false,
    },
    {
      type: 'link',
      label: 'Quiz Center',
      href: '/quiz',
    },
    {
      type: 'link',
      label: 'Lab Exercises',
      href: '/labs',
    },
    {
      type: 'link',
      label: 'RAG Chatbot',
      href: '/chat',
    },
  ],
};

module.exports = sidebars;