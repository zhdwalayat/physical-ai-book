import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  bookSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System',
      link: {
        type: 'generated-index',
        description: 'ROS 2 middleware for robot control - Weeks 1-5',
      },
      items: [
        'module-1-ros2/week-1-foundations',
        'module-1-ros2/week-2-embodied-intelligence',
        'module-1-ros2/week-3-ros2-architecture',
        'module-1-ros2/week-4-nodes-topics-services',
        'module-1-ros2/week-5-python-packages',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin',
      link: {
        type: 'generated-index',
        description: 'Gazebo & Unity simulation - Weeks 6-7',
      },
      items: [
        'module-2-digital-twin/week-6-gazebo-setup',
        'module-2-digital-twin/week-7-physics-sensors',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain',
      link: {
        type: 'generated-index',
        description: 'NVIDIA Isaac platform - Weeks 8-10',
      },
      items: [
        'module-3-isaac/week-8-isaac-sim',
        'module-3-isaac/week-9-perception-vslam',
        'module-3-isaac/week-10-sim-to-real',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action',
      link: {
        type: 'generated-index',
        description: 'LLMs meet Robotics - Weeks 11-13',
      },
      items: [
        'module-4-vla/week-11-humanoid-kinematics',
        'module-4-vla/week-12-locomotion-manipulation',
        'module-4-vla/week-13-conversational-robotics',
      ],
    },
    {
      type: 'category',
      label: 'Assessments',
      link: {
        type: 'generated-index',
        description: 'Projects and Capstone - Week 14',
      },
      items: [
        'assessments/week-14-capstone',
      ],
    },
  ],
};

export default sidebars;
