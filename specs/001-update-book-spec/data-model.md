# Data Model: Physical AI & Humanoid Robotics Online Book - Introduction and Module 1

## Entities

### Book Section
- **Description**: A major section of the curriculum focusing on a specific aspect of Physical AI and Humanoid Robotics
- **Fields**:
  - id: unique identifier for the section
  - title: display title of the section
  - description: brief overview of the section content
  - learningObjectives: list of key learning outcomes
  - prerequisites: list of required knowledge/skills
  - assessmentProject: associated project to evaluate understanding (if applicable)
  - order: sequential position in the curriculum
  - type: "introduction" or "module"
- **Relationships**: Contains multiple Chapters; belongs to the book
- **Validation**: Must have unique title and non-empty content

### Book Chapter
- **Description**: A subsection within a section containing multiple related lessons on a specific topic
- **Fields**:
  - id: unique identifier for the chapter
  - title: display title of the chapter
  - description: brief overview of the chapter content
  - section: reference to the parent section
  - learningObjectives: list of key learning outcomes for the chapter
  - estimatedDuration: time needed to complete the chapter (hours)
  - order: sequential position within the section
- **Relationships**: Contains multiple Lessons; belongs to one Section
- **Validation**: Must have unique title within the section

### Lesson
- **Description**: A structured learning unit within a chapter containing theory, code examples, hands-on exercises, and checkpoints
- **Fields**:
  - id: unique identifier for the lesson
  - title: display title of the lesson
  - content: main content body in Markdown format
  - chapter: reference to the parent chapter
  - learningObjectives: list of key learning outcomes for the lesson
  - prerequisites: list of required knowledge/skills for the lesson
  - codeExamples: list of associated code snippets/files
  - handsOnExercises: list of practical exercises (required for every lesson)
  - checkpoints: list of self-assessment questions
  - troubleshooting: common issues and solutions
  - resources: additional reading materials and links
  - estimatedDuration: time needed to complete the lesson (minutes)
  - order: sequential position within the chapter
- **Relationships**: Belongs to one Chapter; may reference multiple Code Examples and Assets
- **Validation**: Must have non-empty content, unique title within the chapter, and at least one hands-on exercise

### Assessment Project
- **Description**: A comprehensive project that evaluates user understanding of a section's concepts and skills
- **Fields**:
  - id: unique identifier for the project
  - title: display title of the project
  - section: reference to the associated section (for Module 1)
  - description: detailed project requirements
  - learningObjectives: list of skills to be demonstrated
  - requirements: specific deliverables and criteria
  - resources: needed tools, libraries, or datasets
  - estimatedDuration: time needed to complete the project (hours/days)
  - evaluationCriteria: how the project will be assessed
- **Relationships**: Associated with one Section; may reference multiple Lessons
- **Validation**: Must have clear requirements and evaluation criteria

### Content Asset
- **Description**: Media, code examples, diagrams, or other materials that support learning objectives
- **Fields**:
  - id: unique identifier for the asset
  - title: display title of the asset
  - type: category of asset (image, video, code, document, interactive-demo, etc.)
  - filePath: location of the asset file
  - description: brief explanation of the asset's purpose
  - associatedLessons: list of lessons that use this asset
  - size: file size for download considerations
  - createdAt: timestamp of asset creation
- **Relationships**: May be referenced by multiple Lessons
- **Validation**: Must have valid file path and appropriate file type

### Simulation Environment
- **Description**: Technical framework (ROS 2 Humble Hawksbill) used for hands-on learning experiences
- **Fields**:
  - id: unique identifier for the environment
  - name: display name of the environment
  - version: specific version of the framework (Humble Hawksbill)
  - description: brief overview of the environment's capabilities
  - setupInstructions: steps to configure the environment
  - compatibility: supported operating systems and hardware
  - associatedLessons: list of lessons that use this environment
  - requirements: system requirements for the environment
- **Relationships**: Referenced by multiple Lessons requiring hands-on exercises
- **Validation**: Must have working setup instructions and compatibility information

### Interactive Component
- **Description**: Custom UI components that enhance the learning experience with interactive features
- **Fields**:
  - id: unique identifier for the component
  - name: display name of the component (e.g., "CodeRunner", "SimulationViewer", "Quiz")
  - description: brief explanation of the component's functionality
  - configuration: settings and parameters for the component
  - associatedLessons: list of lessons that use this component
  - props: component-specific properties and options
- **Relationships**: Embedded in multiple Lessons
- **Validation**: Must have valid configuration and render properly in Docusaurus environment