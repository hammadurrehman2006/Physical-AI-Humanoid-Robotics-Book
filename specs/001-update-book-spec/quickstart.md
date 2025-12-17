# Quickstart Guide: Physical AI & Humanoid Robotics Online Book - Introduction and Module 1

## Prerequisites

- Node.js 18+ installed
- Git for version control
- Python 3.10+ for ROS 2 Humble Hawksbill compatibility
- Basic knowledge of command line tools
- Docker (optional, for isolated ROS 2 environments)

## Setup Development Environment

1. **Clone the repository**
   ```bash
   git clone <repository-url>
   cd <repository-name>
   ```

2. **Navigate to the book directory**
   ```bash
   cd book
   ```

3. **Install dependencies**
   ```bash
   npm install
   ```

4. **Install ROS 2 Humble Hawksbill** (for testing examples)
   - Follow the official installation guide: https://docs.ros.org/en/humble/Installation.html
   - Or use the Docker setup provided in the repository

5. **Start the development server**
   ```bash
   npm start
   ```

6. **Open your browser**
   - The site will be available at `http://localhost:3000`
   - Auto-reload is enabled during development

## Content Development

1. **Create a new lesson**
   - Navigate to the appropriate section directory in `book/docs/`
   - Create a new Markdown file with the lesson content
   - Update `_category_.json` if adding a new chapter

2. **Add hands-on exercises (required for every lesson)**
   - Include practical examples in every lesson as per specification
   - Place code files in `book/docs/assets/code-samples/`
   - Test all examples in ROS 2 Humble Hawksbill environment

3. **Add interactive components**
   - Use custom Docusaurus components like CodeRunner, SimulationViewer, and Quiz
   - Place interactive components in `book/src/components/`
   - Reference them in lessons using Docusaurus MDX syntax

4. **Add images/videos**
   - Place media files in `book/static/img/` or `book/static/files/`
   - Reference them in lessons using standard Markdown syntax

## Enhanced UI/UX Components

1. **Interactive Demos**
   - Use the InteractiveDemo component for step-by-step visualizations
   - Example: `{<InteractiveDemo config={demoConfig} />}`

2. **Code Execution Environments**
   - Use the CodeRunner component for embedded Python/ROS examples
   - Example: `{<CodeRunner code={pythonCode} environment="ros2-humble" />}`

3. **Simulation Viewers**
   - Use the SimulationViewer component for embedded ROS 2 simulations
   - Example: `{<SimulationViewer simulation="basic-ros-node" />}`

4. **Interactive Quizzes**
   - Use the Quiz component for knowledge checks
   - Example: `{<Quiz questions={quizQuestions} />}`

## Running Tests

1. **Content validation**
   ```bash
   npm run build
   ```

2. **Automated UI tests** (with Playwright)
   ```bash
   npx playwright test
   ```

3. **ROS 2 example validation**
   ```bash
   # Run from within a ROS 2 environment
   source /opt/ros/humble/setup.bash
   # Execute the test scripts in the testing directory
   ```

## Building for Production

```bash
npm run build
```

The static site will be generated in the `build/` directory and ready for deployment.

## Deployment

The site is configured for deployment to GitHub Pages or similar static hosting services. See the `.github/workflows/deploy.yml` file for CI/CD configuration details.

## Docker Environment (Optional)

For consistent ROS 2 development environment:
```bash
# Build the development container
docker build -f Dockerfile.dev -t ros-book-dev .

# Run the development server in container
docker run -p 3000:3000 -v $(pwd):/app ros-book-dev
```