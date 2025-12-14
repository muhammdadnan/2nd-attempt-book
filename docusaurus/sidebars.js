// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'modules/module-1/introduction',
        'modules/module-1/ros2-architecture',
        'modules/module-1/nodes',
        'modules/module-1/topics',
        'modules/module-1/services-actions',
        'modules/module-1/parameters-launch',
        'modules/module-1/rclpy-ai',
        'modules/module-1/urdf-fundamentals',
        'modules/module-1/humanoid-urdf',
        'modules/module-1/rviz-visualization',
        'modules/module-1/conclusion'
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'modules/module-2/introduction',
        'modules/module-2/gazebo-physics',
        'modules/module-2/urdf-gazebo',
        'modules/module-2/physics-tuning',
        'modules/module-2/sensor-simulation',
        'modules/module-2/sensor-ros',
        'modules/module-2/gazebo-worlds',
        'modules/module-2/unity-robotics',
        'modules/module-2/unity-hdrp',
        'modules/module-2/ros-unity-bridge',
        'modules/module-2/conclusion'
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'modules/module-3/introduction',
        'modules/module-3/isaac-installation',
        'modules/module-3/urdf-isaac',
        'modules/module-3/isaac-integration',
        'modules/module-3/isaac-navigation',
        'modules/module-3/isaac-perception',
        'modules/module-3/isaac-manipulation',
        'modules/module-3/isaac-simulation-manipulation',
        'modules/module-3/vla-integration',
        'modules/module-3/photorealistic-rendering',
        'modules/module-3/synthetic-datasets',
        'modules/module-3/isaac-ros-architecture',
        'modules/module-3/vslam-pipeline',
        'modules/module-3/perception-nodes',
        'modules/module-3/navigation-stack',
        'modules/module-3/nav2-stack',
        'modules/module-3/nav2-isaac',
        'modules/module-3/conclusion'
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'modules/module-4/introduction',
        'modules/module-4/vla-pipeline',
        'modules/module-4/whisper-integration',
        'modules/module-4/nlp-parser',
        'modules/module-4/llm-planner',
        'modules/module-4/safety-guardrails',
        'modules/module-4/perception-selection',
        'modules/module-4/nav2-integration',
        'modules/module-4/manipulation-tasks',
        'modules/module-4/vla-agent',
        'modules/module-4/conclusion'
      ],
    },
    'capstone'
  ],
};

module.exports = sidebars;