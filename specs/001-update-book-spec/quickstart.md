# Quickstart Guide: Physical AI & Humanoid Robotics Online Book

## Prerequisites

- Node.js 18+ installed
- Git for version control
- Basic knowledge of command line tools
- Python 3.8+ for robotics examples (optional for viewing content)

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

4. **Start the development server**
   ```bash
   npm start
   ```

5. **Open your browser**
   - The site will be available at `http://localhost:3000`
   - Auto-reload is enabled during development

## Content Development

1. **Create a new lesson**
   - Navigate to the appropriate module and chapter directory in `book/docs/`
   - Create a new Markdown file with the lesson content
   - Update `_category_.json` if adding a new chapter

2. **Add code examples**
   - Place code files in `book/docs/assets/code-samples/`
   - Reference them in lessons using Docusaurus code block syntax

3. **Add images/videos**
   - Place media files in `book/static/img/` or `book/static/files/`
   - Reference them in lessons using standard Markdown syntax

## Running Tests

1. **Content validation**
   ```bash
   npm run build
   ```

2. **Automated UI tests** (if Playwright is configured)
   ```bash
   npx playwright test
   ```

## Building for Production

```bash
npm run build
```

The static site will be generated in the `build/` directory and ready for deployment.

## Deployment

The site is configured for deployment to GitHub Pages or similar static hosting services. See the `.github/workflows/deploy.yml` file for CI/CD configuration details.