# Quickstart: Urdu LTR Layout Implementation

## Overview
This guide provides a quick start to implementing Urdu language support with Left-to-Right (LTR) layout in the Docusaurus documentation site.

## Prerequisites
- Node.js 18+ installed
- Docusaurus project set up
- Access to Urdu content files
- Basic knowledge of Docusaurus i18n configuration

## Step 1: Configure Urdu Locale in Docusaurus

Update your `docusaurus.config.ts` to include Urdu as a supported locale with LTR direction:

```typescript
i18n: {
  defaultLocale: 'en',
  locales: ['en', 'ur'], // Add 'ur' to supported locales
  localeConfigs: {
    en: {
      label: 'English',
      direction: 'ltr',
      htmlLang: 'en-US',
      path: 'en',
    },
    ur: {
      label: 'اردو',           // Urdu label
      direction: 'ltr',        // CRITICAL: Force LTR despite Urdu being RTL
      htmlLang: 'ur-PK',       // Proper language tag for Urdu
      path: 'ur',              // URL path prefix
    },
  },
},
```

## Step 2: Create Urdu Content Directory Structure

Create the necessary directory structure in `book/i18n/ur/`:

```
book/i18n/ur/
├── code.json                    # UI translations
├── docusaurus-plugin-content-docs/
│   └── current/                 # Current version of docs
│       ├── intro.json
│       └── module-*.json
├── docusaurus-plugin-content-blog/
│   └── current/
├── docusaurus-theme-classic/
│   └── navbar.json
└── docusaurus-theme-classic/
    └── footer.json
```

## Step 3: Add Urdu Font Support

In your CSS (e.g., `src/css/custom.css`), add Urdu font support:

```css
/* Urdu-specific font stack */
.urdu-text,
html[lang="ur"] {
  font-family: 'Jameel Noori Nastaleeq Kasheeda', 'Noto Nastaliq Urdu', 'Jameel Noori Nastaleeq', 'Urdu Typesetting', serif;
}

/* Force LTR layout for Urdu content */
html[lang="ur"] {
  direction: ltr !important;
}

body[dir="rtl"] {
  direction: ltr !important;
}
```

## Step 4: Update Language Switcher Component

Ensure your language switcher properly handles the Urdu locale:

```jsx
// In your LanguageSwitcher component
const handleLanguageChange = (language) => {
  if (language !== currentLanguage) {
    let newUrl;
    const currentPath = window.location.pathname;

    if (language === 'ur') {
      if (currentPath.startsWith('/ur/')) {
        return; // Already on Urdu locale
      } else {
        newUrl = `/ur${currentPath}`;
      }
    } else {
      // Switching to English, remove /ur/ prefix if present
      if (currentPath.startsWith('/ur/')) {
        newUrl = currentPath.replace(/^\/ur\//, '/');
        if (newUrl === '/') {
          newUrl = '/';
        }
      } else {
        return; // Already on English locale
      }
    }

    window.location.href = newUrl;
  }
};
```

## Step 5: Test Implementation

1. Build the site with i18n support:
   ```bash
   npm run build
   ```

2. Start the development server:
   ```bash
   npm run start -- --locale ur
   ```

3. Verify that:
   - Urdu content displays at `/ur/` paths
   - Layout remains LTR despite Urdu language
   - Fonts render correctly
   - Language switcher works properly
   - SEO metadata is correct

## Common Issues and Solutions

### Issue: RTL layout still appearing
**Solution**: Ensure CSS direction override is applied with `!important`:
```css
html[lang="ur"] {
  direction: ltr !important;
}
```

### Issue: Urdu fonts not loading
**Solution**: Add font preconnect in headTags:
```js
headTags: [
  {
    tagName: 'link',
    attributes: {
      rel: 'preconnect',
      href: 'https://fonts.googleapis.com',
    },
  },
  {
    tagName: 'link',
    attributes: {
      rel: 'preconnect',
      href: 'https://fonts.gstatic.com',
      crossOrigin: 'anonymous',
    },
  },
],
```

### Issue: Content not appearing in Urdu
**Solution**: Verify that content files exist in the correct i18n/ur/ directories and match the English content structure.

## Next Steps
- Add complete Urdu translations to the i18n/ur/ directories
- Test responsive behavior across different screen sizes
- Verify SEO metadata with hreflang tags
- Run accessibility tests with screen readers