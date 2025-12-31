# Contributing to Isaac AI Robot Brain

Thank you for your interest in contributing to the Isaac AI Robot Brain project! This document provides guidelines and information to help you contribute effectively.

## Code of Conduct

Please follow our Code of Conduct to keep our community approachable and respectable.

## Getting Started

### Prerequisites

- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill
- Isaac Sim 2023.1+
- Python 3.10+
- C++ compiler with C++17 support

### Setting up the Development Environment

1. Fork the repository
2. Clone your fork:
   ```bash
   git clone https://github.com/your-username/isaac-robot-brain.git
   ```
3. Set up the development environment:
   ```bash
   cd isaac-robot-brain
   ./scripts/setup_dev_environment.sh
   ```

## How to Contribute

### Reporting Bugs

- Use the issue tracker to report bugs
- Describe the issue in detail
- Include steps to reproduce
- Mention your environment (OS, ROS version, Isaac Sim version)

### Suggesting Features

- Use the issue tracker to suggest features
- Explain the use case
- Describe the desired behavior
- Consider implementation complexity

### Pull Requests

1. Create a feature branch from the `develop` branch
2. Make your changes
3. Add tests if applicable
4. Update documentation if needed
5. Ensure all tests pass
6. Submit a pull request to the `develop` branch

## Coding Guidelines

### Python

- Follow PEP 8 style guide
- Use type hints where possible
- Write docstrings for all public functions
- Keep functions focused and small

### C++

- Follow ROS 2 C++ style guide
- Use smart pointers for memory management
- Write unit tests for all functionality
- Use const correctness

### ROS 2

- Follow ROS 2 package conventions
- Use standard message types when possible
- Implement proper lifecycle management
- Handle errors gracefully

## Development Workflow

1. Create an issue for significant changes
2. Fork the repository
3. Create a feature branch
4. Make changes
5. Add/update tests
6. Update documentation
7. Submit pull request

## Testing

- Write unit tests for all new functionality
- Ensure all existing tests continue to pass
- Test integration with Isaac Sim when applicable
- Verify performance requirements are met

## Documentation

- Update API documentation
- Add user guides for new features
- Update README if necessary
- Document configuration options

## Questions

If you have questions about contributing, please open an issue in the repository.

Thank you for contributing to the Isaac AI Robot Brain project!