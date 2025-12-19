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
    }
  ],
};

export default sidebars;
