import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // Book sidebar structure
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      items: [
        'intro/foundations-of-physical-ai/index',
        'intro/digital-ai-transition/index',
        'intro/humanoid-landscape/index',
        'intro/why-physical-ai/index',
        'intro/sensor-systems-overview/index',
        'intro/prerequisites-setup/index'
      ],
      link: {
        type: 'doc',
        id: 'intro/overview',
      },
    },
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System - ROS 2',
      items: [
        'module-1/ros2-architecture/index',
        'module-1/nodes-topics-services/index',
        'module-1/actions-robotic-systems/index',
        'module-1/python-rclpy-bridge/index',
        'module-1/building-ros2-packages/index',
        'module-1/launch-files-params/index',
        'module-1/urdf-humanoids/index',
        'module-1/assessment/index',
        'module-1/supplementary/index'
      ],
      link: {
        type: 'doc',
        id: 'module-1/index',
      },
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'module-2/introduction/prerequisites',
        'module-2/introduction/setup-gazebo-environment',
        'module-2/introduction/basic-robot-spawning',
        'module-2/introduction/troubleshooting-guide',
        {
          type: 'category',
          label: 'URDF/SDF Formats',
          items: [
            'module-2/urdf-sdf-formats/urdf-basics',
            'module-2/urdf-sdf-formats/sdf-advanced',
            'module-2/urdf-sdf-formats/creating-robot-models',
            'module-2/urdf-sdf-formats/conversion-guide',
            'module-2/urdf-sdf-formats/practical-urdf-sdf-examples'
          ]
        },
        {
          type: 'category',
          label: 'Physics Simulation',
          items: [
            'module-2/physics-simulation/gravity-and-collisions',
            'module-2/physics-simulation/material-properties',
            'module-2/physics-simulation/environment-modeling',
            'module-2/physics-simulation/physics-debugging-validation'
          ]
        },
        {
          type: 'category',
          label: 'Sensor Simulation',
          items: [
            'module-2/sensor-simulation/lidar-simulation',
            'module-2/sensor-simulation/camera-simulation',
            'module-2/sensor-simulation/imu-simulation',
            'module-2/sensor-simulation/sensor-fusion',
            'module-2/sensor-simulation/sensor-data-validation'
          ]
        },
        {
          type: 'category',
          label: 'Unity Integration',
          items: [
            'module-2/unity-integration/unity-setup',
            'module-2/unity-integration/ros2-unity-bridge',
            'module-2/unity-integration/visualization-techniques',
            'module-2/unity-integration/unity-troubleshooting'
          ]
        },
        {
          type: 'category',
          label: 'Assessment Project',
          items: [
            'module-2/assessment-project/project-overview',
            'module-2/assessment-project/requirements',
            'module-2/assessment-project/evaluation-criteria'
          ]
        },
        {
          type: 'category',
          label: 'Module 2 Assessment Project',
          items: [
            'projects/module-2-assessment/project-scaffolding',
            'projects/module-2-assessment/step-by-step-instructions',
            'projects/module-2-assessment/solution-examples'
          ]
        }




      ],
      link: {
        type: 'doc',
        id: 'module-2/index',
      },
    }
  ],
};

export default sidebars;
