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
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'module-3/introduction/index',
        'module-3/introduction/isaac-sim-overview',
        'module-3/introduction/prerequisites',
        'module-3/introduction/validation-framework',
        {
          type: 'category',
          label: 'Phase 1: Installation & Setup',
          items: [
            'module-3/phase-1-installation/isaac-sim-installation',
            'module-3/phase-1-installation/hardware-requirements',
            'module-3/phase-1-installation/gpu-requirements',
            'module-3/phase-1-installation/jetson-agx-orin-platform',
            'module-3/phase-1-installation/omniverse-compatibility',
            'module-3/phase-1-installation/sdk-integration',
            'module-3/phase-1-installation/environment-setup'
          ]
        },
        {
          type: 'category',
          label: 'Phase 2: Scene Creation',
          items: [
            'module-3/phase-2-scene-creation/index',
            'module-3/phase-2-scene-creation/photorealistic-scenes',
            'module-3/phase-2-scene-creation/physics-configuration',
            'module-3/phase-2-scene-creation/robot-integration'
          ]
        },
        {
          type: 'category',
          label: 'Phase 3: Sensor Simulation',
          items: [
            'module-3/phase-3-sensor-simulation/sensor-configuration',
            'module-3/phase-3-sensor-simulation/data-generation',
            'module-3/phase-3-sensor-simulation/domain-randomization'
          ]
        },
        {
          type: 'category',
          label: 'Phase 4: RL Infrastructure',
          items: [
            'module-3/phase-4-rl-infrastructure/rl-setup',
            'module-3/phase-4-rl-infrastructure/training-environments',
            'module-3/phase-4-rl-infrastructure/performance-optimization'
          ]
        },
        {
          type: 'category',
          label: 'Phase 5: Visualization & Debugging Tools',
          items: [
            'module-3/phase-11-visualization-debugging-tools/visualization-and-debugging-tools'
          ]
        },
        {
          type: 'category',
          label: 'Phase 6: Performance Optimization',
          items: [
            'module-3/phase-12-performance-optimization/index',
            'module-3/phase-12-performance-optimization/isaac-sim-performance-optimization',
            'module-3/phase-12-performance-optimization/quick-reference',
            'module-3/phase-12-performance-optimization/practical-exercises'
          ]
        },
        {
          type: 'category',
          label: 'Module 3 Assessment Project',
          items: [
            'module-3/assessment/final-project'
          ]
        }
      ],
      link: {
        type: 'doc',
        id: 'module-3/index',
      },
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'module-4/diagrams/vla-architecture-documentation',
        {
          type: 'category',
          label: 'Voice Processing Setup',
          items: [
            'module-4/voice-processing/index',
            'module-4/voice-processing/setup-tutorial',
            'module-4/voice-processing/examples',
            {
              type: 'category',
              label: 'Exercises',
              items: [
                'module-4/voice-processing/exercise-1-voice-recognition-testing',
                'module-4/voice-processing/exercise-2-whisper-optimization',
                'module-4/voice-processing/exercise-3-troubleshooting',
                'module-4/voice-processing/exercises/audio-input-setup',
                'module-4/voice-processing/exercises/audio-preprocessing',
                'module-4/voice-processing/exercises/speech-recognition',
                'module-4/voice-processing/exercises/voice-command-parsing',
              ]
            }
          ]
        },
        {
          type: 'category',
          label: 'Language Understanding',
          items: [
            'module-4/language-understanding/index',
            'module-4/language-understanding/vision-examples',
          ]
        },
        {
          type: 'category',
          label: 'Computer Vision Integration',
          items: [
            'module-4/computer-vision/index',
            'module-4/computer-vision/exercises/object-detection',
          ]
        },
        {
          type: 'category',
          label: 'Action Execution',
          items: [
            'module-4/action-execution/index',
            'module-4/action-execution/exercises/action-planning',
            'module-4/action-execution/exercises/robot-control',
          ]
        },
        {
          type: 'category',
          label: 'Multi-Modal Fusion',
          items: [
            'module-4/multi-modal-fusion/index',
            'module-4/multi-modal-fusion/exercises/cross-modal-attention',
          ]
        },
        {
          type: 'category',
          label: 'Integration and Testing',
          items: [
            'module-4/integration/index',
          ]
        },
        {
          type: 'category',
          label: 'Performance Optimization',
          items: [
            'module-4/optimization/index',
          ]
        },
        {
          type: 'category',
          label: 'Advanced Applications and Deployment',
          items: [
            'module-4/applications/index',
            'module-4/applications/ENHANCEMENT_SUMMARY',
          ]
        },
        {
          type: 'category',
          label: 'Module 4 Assessment Project',
          items: [
            'module-4/assessment/final-project'
          ]
        }
      ],
      link: {
        type: 'doc',
        id: 'module-4/index',
      },
    }
  ],
};

export default sidebars;
