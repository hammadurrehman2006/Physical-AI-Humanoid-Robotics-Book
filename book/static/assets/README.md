# Content Asset Management System

## Overview

This directory contains all content assets for the Physical AI & Humanoid Robotics book, organized by type and purpose.

## Directory Structure

```
static/assets/
├── code-examples/          # All code examples used in lessons
│   ├── python-examples/    # Python code examples
│   │   └── ros2-basics/    # Basic ROS 2 Python examples
│   ├── cpp-examples/       # C++ code examples
│   └── launch-files/       # ROS 2 launch files
├── images/                 # Static images and screenshots
└── diagrams/               # Technical diagrams and illustrations
```

## Asset Guidelines

### Code Examples
- All Python examples should be compatible with Python 3.10+ and ROS 2 Humble Hawksbill
- Include comments explaining key concepts
- Provide expected output where applicable
- Name files descriptively (e.g., `publisher_subscriber_example.py`)

### Images
- Use appropriate file formats (PNG for diagrams, JPEG for photos)
- Optimize for web delivery
- Include alt text descriptions
- Maintain consistent sizing when possible

### Diagrams
- Use vector formats when possible (SVG)
- Include both source files and rendered versions
- Use consistent color schemes and styling

## File Naming Convention

- Use kebab-case for file names: `my-descriptive-name.extension`
- Include version information when relevant: `my-file-v1.0.extension`
- Use descriptive names that indicate content and purpose