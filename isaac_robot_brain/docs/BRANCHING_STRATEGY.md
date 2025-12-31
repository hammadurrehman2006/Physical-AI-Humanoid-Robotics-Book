# Branching Strategy for Isaac AI Robot Brain

## Overview

This document outlines the branching strategy for the Isaac AI Robot Brain project to ensure a consistent and efficient development workflow.

## Branches

### Main Branches

- `main`: Production-ready code that has passed all tests and reviews
- `develop`: Integration branch for features being developed

### Supporting Branches

- `feature/*`: Feature development branches (e.g., `feature/vslam-improvements`)
- `release/*`: Release preparation branches (e.g., `release/v0.1.0`)
- `hotfix/*`: Urgent fixes for production issues (e.g., `hotfix/critical-bug-fix`)

## Workflow

### Feature Development

1. Create a feature branch from `develop`:
   ```bash
   git checkout develop
   git pull origin develop
   git checkout -b feature/feature-name
   ```

2. Develop the feature with regular commits:
   ```bash
   git add .
   git commit -m "Add feature: description of change"
   ```

3. Push the feature branch:
   ```bash
   git push origin feature/feature-name
   ```

4. Create a pull request from `feature/feature-name` to `develop`

5. After review and approval, merge the feature branch into `develop`

### Release Process

1. Create a release branch from `develop`:
   ```bash
   git checkout develop
   git pull origin develop
   git checkout -b release/vX.Y.Z
   ```

2. Update version numbers and finalize release notes

3. Create pull request from `release/vX.Y.Z` to `main`

4. After merging to `main`, create a tag:
   ```bash
   git tag -a vX.Y.Z -m "Release version X.Y.Z"
   git push origin vX.Y.Z
   ```

5. Create pull request from `release/vX.Y.Z` to `develop` to merge back changes

### Hotfix Process

1. Create hotfix branch from `main`:
   ```bash
   git checkout main
   git pull origin main
   git checkout -b hotfix/hotfix-name
   ```

2. Implement the fix

3. Create pull request from `hotfix/hotfix-name` to `main`

4. After merging to `main`, create a tag:
   ```bash
   git tag -a vX.Y.Z -m "Hotfix version X.Y.Z"
   git push origin vX.Y.Z
   ```

5. Create pull request from `hotfix/hotfix-name` to `develop` to merge back changes

## Commit Messages

Follow the conventional commits format:
- `feat: Add new feature`
- `fix: Fix a bug`
- `docs: Documentation changes`
- `style: Code style changes`
- `refactor: Code refactoring`
- `test: Add or update tests`
- `chore: Other changes`

## Pull Requests

- Keep pull requests focused on a single feature or fix
- Include a clear description of the changes
- Reference related issues
- Ensure all tests pass
- Get code review approval before merging

## Tags

- Use semantic versioning (vX.Y.Z)
- Tag releases after merging to `main`
- Include release notes with each tag