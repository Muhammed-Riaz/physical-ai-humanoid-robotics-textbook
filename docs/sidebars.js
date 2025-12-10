// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Physical AI Tutorial',
      items: [
        'intro',
        'tutorial-intro',
        {
          type: 'category',
          label: 'Frontmatter',
          items: [
            'frontmatter/index',
          ],
        },
        {
          type: 'category',
          label: 'Modules',
          items: [
            {
              type: 'category',
              label: 'Module 1: ROS 2',
              items: [
                'modules/ros2/index',
                'modules/ros2/installation',
              ],
            },
            {
              type: 'category',
              label: 'Module 2: Digital Twin',
              items: [
                'modules/digital-twin/index',
                'modules/digital-twin/gazebo_sim',
              ],
            },
            {
              type: 'category',
              label: 'Module 3: AI-Robot Brain',
              items: [
                'modules/isaac/index',
                'modules/isaac/isaac_sim',
                'modules/isaac/vslam_jetson',
                'modules/isaac/nav2_jetson',
              ],
            },
            {
              type: 'category',
              label: 'Module 4: Vision-Language-Action',
              items: [
                'modules/vla/index',
              ],
            },
          ],
        },
        {
          type: 'category',
          label: 'Weeks',
          items: [
            {
              type: 'category',
              label: 'Weeks 1-2: Introduction',
              items: [
                'weeks/week-01_02/index',
              ],
            },
            {
              type: 'category',
              label: 'Weeks 11-12: Humanoid Development',
              items: [
                'weeks/week-11_12/index',
              ],
            },
          ],
        },
        {
          type: 'category',
          label: 'Capstone Project',
          items: [
            'capstone/index',
            'capstone/tasks',
          ],
        },
        {
          type: 'category',
          label: 'Hardware Guide',
          items: [
            'hardware-guide/index',
          ],
        },
        {
          type: 'category',
          label: 'Gallery',
          items: [
            'gallery/index',
          ],
        },
        {
          type: 'category',
          label: 'Contributors',
          items: [
            'contributors/index',
          ],
        },
      ],
    },
  ],
};

module.exports = sidebars;