# Vision-Language-Action (VLA) Architecture Diagrams

This directory contains architecture diagrams for the Vision-Language-Action module of the humanoid robotics system. These diagrams illustrate the system architecture, component relationships, data flows, and processing pipelines.

## Diagrams Included

### 1. High-Level System Architecture (`vla-high-level-architecture.mmd`)
- Shows the overall flow from voice input to action execution
- Illustrates the main system components and their interactions
- Highlights external systems and data stores

### 2. Component Diagram (`vla-component-diagram.mmd`)
- Details the relationships between voice processing, language understanding, computer vision, and action execution modules
- Shows interfaces and data exchange between components
- Includes notes on functionality of each component

### 3. Sequence Diagram (`vla-sequence-diagram.mmd`)
- Demonstrates the step-by-step flow of a voice command through the system
- Shows interactions between all modules during command processing
- Illustrates the timing and order of operations

### 4. Data Flow Diagram (`vla-data-flow-diagram.mmd`)
- Visualizes how information moves between different modules
- Shows input/output relationships and data transformations
- Highlights data stores and their roles in the system

### 5. Comprehensive Architecture Overview (`vla-comprehensive-architecture.mmd`)
- Detailed view of all components and their relationships
- Shows layered architecture with specific processing steps
- Includes styling to distinguish different types of components

### 6. Architecture Documentation (`vla-architecture-documentation.md`)
- Complete documentation file with all diagrams embedded
- Detailed explanations of each component and its function
- Suitable for inclusion in Docusaurus documentation

## How to Use These Diagrams

### In Docusaurus Documentation
1. The `.mmd` files can be rendered using the Mermaid plugin for Docusaurus
2. The `vla-architecture-documentation.md` file can be directly included in your documentation
3. Add the following to your Docusaurus config to enable Mermaid:
   ```js
   module.exports = {
     plugins: [
       [
         require.resolve("@cmfcmf/docusaurus-magic-details"),
         {
           markdownType: "mdx",
         },
       ],
     ],
     themes: [
       [
         "mdx",
         {
           beforeDefaultRehypePlugins: [
             require("@mdx-js/mdx/plugins/remark-gfm"),
             require("remark-mermaid"),
           ],
         },
       ],
     ],
   };
   ```

### For Development
- Use these diagrams as reference when implementing VLA components
- Ensure new features align with the architectural patterns shown
- Update diagrams when making architectural changes

### For Understanding
- Start with the high-level architecture to understand the overall system
- Use the sequence diagram to understand the command processing flow
- Refer to the component diagram for detailed module relationships
- Use the data flow diagram to understand information movement

## Architecture Principles

The VLA system follows these key architectural principles:

1. **Modularity**: Components are designed to be independent and replaceable
2. **Scalability**: Architecture supports addition of new capabilities
3. **Real-time Processing**: Designed for real-time response requirements
4. **Safety**: Built-in feedback and validation mechanisms
5. **Extensibility**: Easy to integrate new AI models and capabilities

## Updating Diagrams

When making changes to the VLA system:

1. Update the relevant diagram files
2. Update the documentation file to reflect changes
3. Ensure all diagrams remain consistent with each other
4. Validate that the sequence flows remain accurate
5. Update this README if new diagrams are added

## File Structure

```
diagrams/
├── vla-high-level-architecture.mmd      # High-level system view
├── vla-component-diagram.mmd            # Component relationships
├── vla-sequence-diagram.mmd             # Command processing sequence
├── vla-data-flow-diagram.mmd            # Information flow
├── vla-comprehensive-architecture.mmd   # Detailed architecture overview
├── vla-architecture-documentation.md    # Complete documentation with diagrams
└── README.md                           # This file
```

## Dependencies

These diagrams are created using Mermaid syntax, which requires:
- Mermaid.js for rendering
- Docusaurus with Mermaid plugin for documentation integration
- A modern browser for visualization