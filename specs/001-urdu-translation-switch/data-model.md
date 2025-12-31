# Data Model: Urdu Translation Switch

## Entities

### Translation Content
- **Description**: Represents the translated content for each page in the book
- **Fields**:
  - `id` (string): Unique identifier for the content piece
  - `pageId` (string): Reference to the original page
  - `language` (string): Language code ('en' or 'ur')
  - `title` (string): Translated title of the content
  - `body` (string): Translated body content in Markdown format
  - `createdAt` (date): Creation timestamp
  - `updatedAt` (date): Last update timestamp
  - `version` (string): Version of the translation
- **Relationships**: One-to-many with Language Preference (via pageId)

### Language Preference
- **Description**: Represents the user's selected language setting that persists across sessions
- **Fields**:
  - `userId` (string): User identifier (or session identifier)
  - `selectedLanguage` (string): Currently selected language ('en' or 'ur')
  - `lastUpdated` (date): Timestamp of last preference change
  - `preferences` (object): Additional display preferences (font size, etc.)
- **Relationships**: Many-to-one with Translation Content (via user's content access)

### Font Configuration
- **Description**: Represents the Google font settings required for proper Urdu text rendering
- **Fields**:
  - `fontFamily` (string): Primary font family for Urdu text (e.g., 'Jameel Noori Nastaleeq Kasheeda', 'Gulzar', 'Noto Nastaliq Urdu')
  - `fontUrl` (string): URL to the Google Font resource
  - `fallbackFonts` (array): Array of fallback font families
  - `cssProperties` (object): Additional CSS properties for proper rendering
  - `languageCode` (string): Associated language ('ur')
- **Relationships**: One-to-one with Translation Content (for rendering context)

## State Transitions

### Language State
- **Initial State**: Default language (English)
- **Transition 1**: User selects Urdu → State becomes 'urdu-active'
  - Trigger: User clicks language switcher
  - Action: Load Urdu fonts and content
  - Guard: Urdu translation exists for current page
- **Transition 2**: User selects English → State becomes 'english-active'
  - Trigger: User clicks language switcher
  - Action: Switch back to default fonts and content
  - Guard: Always available

## Validation Rules

### Translation Content Validation
- `language` must be one of ['en', 'ur']
- `body` must not be empty when status is 'published'
- `pageId` must reference an existing page in the system
- `version` must follow semantic versioning

### Language Preference Validation
- `selectedLanguage` must be one of ['en', 'ur']
- `userId` must be a valid identifier format
- Preferences must be a valid JSON object

### Font Configuration Validation
- `fontFamily` must be a valid CSS font family string
- `fontUrl` must be a valid HTTPS URL
- `languageCode` must correspond to supported languages