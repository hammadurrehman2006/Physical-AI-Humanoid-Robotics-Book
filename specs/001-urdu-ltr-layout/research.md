# Research: Urdu LTR Layout Integration

## Overview
This research document addresses the requirements for integrating Urdu translations while maintaining a Left-to-Right (LTR) design in the Docusaurus documentation site. The plan sequences work across multiple team roles: Documentation Engineer, Technical Writer, and Code Developer, with specialized phases for baseline capture and deployment strategy.

## Decision: Team Work Sequence
**Rationale**: The implementation requires specialized skills in sequence to ensure proper setup, content migration, and technical implementation.
- Phase 1: Documentation Engineer initializes locales and i18n configuration
- Phase 2: Technical Writer handles content migration and translation
- Phase 3: Code Developer applies CSS locks and technical fixes

## Decision: Playwright Baseline Capture
**Rationale**: Capturing baseline LTR screenshots of the English site provides a visual template for the Urdu version, ensuring consistent layout and design.
- Use Playwright MCP to capture screenshots of key pages in English
- Compare with Urdu implementation to ensure layout consistency
- Identify any layout overflows or issues early in the process

## Decision: Deployment Strategy for /ur/ Sub-paths
**Rationale**: Docusaurus i18n supports locale-specific paths, allowing for clean URL structure.
- Configure Docusaurus to serve Urdu content at /ur/ sub-paths
- Implement proper hreflang tags for SEO
- Ensure proper language detection and routing

## Decision: Rollback Plan for Font Rendering Issues
**Rationale**: Urdu fonts may cause layout overflows or rendering issues that need quick resolution.
- Maintain CSS overrides to revert to basic font stack if needed
- Keep English content as fallback option
- Implement font loading strategies with fallback mechanisms

## Alternatives Considered
- Alternative 1: Using right-to-left (RTL) layout for Urdu - Rejected because requirement specifically states LTR layout must be maintained
- Alternative 2: Using image-based text rendering - Rejected because it would harm accessibility and SEO
- Alternative 3: Server-side language detection instead of path-based routing - Rejected because path-based routing provides better SEO and user control

## Technology Research Findings

### Docusaurus i18n Configuration
- Docusaurus supports custom locale configuration with direction override
- Locale path can be set to '/ur' for Urdu content
- HTML lang attribute and direction can be controlled per locale

### Urdu Font Options
- Jameel Noori Nastaleeq Kasheeda: Excellent for Urdu text, available via Google Fonts
- Noto Nastaliq Urdu: Good alternative with good browser support
- Custom web fonts can be loaded via CSS @font-face

### CSS Direction Overrides
- Use `direction: ltr` to force left-to-right layout
- Override any automatic RTL behavior with specific selectors
- Ensure text alignment remains left-aligned for Urdu content
- Prevent flex/grid layout reversal that occurs with RTL languages

### Playwright Testing Approach
- Capture screenshots of key pages before and after Urdu implementation
- Compare layout metrics to ensure consistency
- Test responsive behavior across different screen sizes
- Verify font rendering and text display

## Implementation Risks and Mitigation
- Risk: Font loading performance impact - Mitigation: Optimize font loading with preconnect and font-display: swap
- Risk: Layout overflows with Urdu text - Mitigation: Implement responsive design and flexible containers
- Risk: SEO impact from language changes - Mitigation: Proper hreflang tags and canonical URLs
- Risk: Accessibility issues - Mitigation: Proper lang attributes and screen reader testing