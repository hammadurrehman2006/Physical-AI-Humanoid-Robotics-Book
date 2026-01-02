---
sidebar_position: 2
---

# دستاویز بنائیں

دستاویزات **صفحات کے گروپس** ہیں جو ان کے ذریعے جڑے ہوئے ہیں:

- ایک **سائیڈ بار**
- **پچھلا/اگلا نیویگیشن**
- **ورژننگ**

## اپنی پہلی دستاویز بنائیں

`docs/hello.md` پر ایک مارک ڈاؤن فائل بنائیں:

```md title="docs/hello.md"
# Hello

This is my **first Docusaurus document**!
```

ایک نئی دستاویز اب [http://localhost:3000/docs/hello](http://localhost:3000/docs/hello) پر دستیاب ہے۔

## سائیڈ بار کو ترتیب دیں

Docusaurus خود بخود `docs` فولڈر سے **ایک سائیڈ بار بناتا ہے**۔

سائیڈ بار لیبل اور پوزیشن کو اپنی مرضی کے مطابق بنانے کے لیے میٹا ڈیٹا شامل کریں:

```md title="docs/hello.md" {1-4}
---
sidebar_label: 'Hi!'
sidebar_position: 3
---

# Hello

This is my **first Docusaurus document**!
```

`sidebars.js` میں واضح طور پر اپنا سائیڈ بار بنانا بھی ممکن ہے:

```js title="sidebars.js"
export default {
  tutorialSidebar: [
    'intro',
    // highlight-next-line
    'hello',
    {
      type: 'category',
      label: 'Tutorial',
      items: ['tutorial-basics/create-a-document'],
    },
  ],
};
```