---
sidebar_position: 2
---

# اپنی سائٹ کا ترجمہ کریں

آئیے `docs/intro.md` کا فرانسیسی میں ترجمہ کریں۔

## i18n کنفیگر کریں

`fr` لوکل کی سپورٹ شامل کرنے کے لیے `docusaurus.config.js` میں ترمیم کریں:

```js title="docusaurus.config.js"
export default {
  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'fr'],
  },
};
```

## دستاویز کا ترجمہ کریں

`docs/intro.md` فائل کو `i18n/fr` فولڈر میں کاپی کریں:

```bash
mkdir -p i18n/fr/docusaurus-plugin-content-docs/current/

cp docs/intro.md i18n/fr/docusaurus-plugin-content-docs/current/intro.md
```

`i18n/fr/docusaurus-plugin-content-docs/current/intro.md` کا فرانسیسی میں ترجمہ کریں۔

## اپنی لوکلائزڈ سائٹ شروع کریں

فرانسیسی لوکل پر اپنی سائٹ شروع کریں:

```bash
npm run start -- --locale fr
```

آپ کی لوکلائزڈ سائٹ [http://localhost:3000/fr/](http://localhost:3000/fr/) پر قابل رسائی ہے اور `Getting Started` صفحہ ترجمہ شدہ ہے۔

:::caution

ڈویلپمنٹ میں، آپ ایک وقت میں صرف ایک لوکل استعمال کر سکتے ہیں۔

:::

## لوکل ڈراپ ڈاؤن شامل کریں

زبانوں میں بغیر کسی رکاوٹ کے نیویگیٹ کرنے کے لیے، لوکل ڈراپ ڈاؤن شامل کریں۔

`docusaurus.config.js` فائل میں ترمیم کریں:

```js title="docusaurus.config.js"
export default {
  themeConfig: {
    navbar: {
      items: [
        // highlight-start
        {
          type: 'localeDropdown',
        },
        // highlight-end
      ],
    },
  },
};
```

لوکل ڈراپ ڈاؤن اب آپ کے نیویگیشن بار میں ظاہر ہوتا ہے:

![Locale Dropdown](./img/localeDropdown.png)

## اپنی لوکلائزڈ سائٹ بنائیں

کسی مخصوص لوکل کے لیے اپنی سائٹ بنائیں:

```bash
npm run build -- --locale fr
```

یا تمام لوکلز کو ایک ساتھ شامل کرنے کے لیے اپنی سائٹ بنائیں:

```bash
npm run build
```